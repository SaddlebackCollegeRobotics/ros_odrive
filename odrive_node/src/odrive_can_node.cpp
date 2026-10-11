#include "odrive_can_node.hpp"
#include "odrive_enums.h"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

enum CmdId : uint32_t {
    kHeartbeat = 0x001,            // ControllerStatus  - publisher
    kGetError = 0x003,             // SystemStatus      - publisher
    kSetAxisState = 0x007,         // SetAxisState      - service
    kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
    kSetInputPos,                  // ControlMessage    - subscriber
    kSetInputVel,                  // ControlMessage    - subscriber
    kSetInputTorque,               // ControlMessage    - subscriber
    kGetIq = 0x014,                // ControllerStatus  - publisher
    kGetTemp,                      // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
    kClearErrors = 0x018,          // ClearErrors       - service
    kGetTorques = 0x01c,           // ControllerStatus  - publisher
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);
    rclcpp::Node::declare_parameter<bool>("axis_idle_on_shutdown", false);

    // SCR patch (2026-08-17): KeepLast, not KeepAll. With a RELIABLE KEEP_ALL
    // history, one reliable subscriber vanishing ungracefully (a laptop
    // dropping off the network mid-drive) fills the history with
    // unacknowledged samples until DDS write() fails, rclcpp throws, and the
    // uncaught exception aborts the node -- all four axis nodes died this way
    // ('failed to publish message: cannot publish data', exit -6). KEEP_LAST
    // lets the writer discard stale unacked telemetry instead; readers only
    // care about the newest sample. Reliability is unchanged, so existing
    // reliable subscribers still match.
    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepLast(10));
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);

    rclcpp::QoS odrv_stat_qos(rclcpp::KeepLast(10));
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});

#if RCLCPP_VERSION_MAJOR >= 18 
    // For ros2 jazzy and above. 
    // PR about deprecation of get_rmw_qos_profile: 
    //  - https://github.com/ros2/rclcpp/pull/713
    //  - https://github.com/ros2/rclcpp/pull/1969
    auto srv_qos_profile = srv_qos;
#else
    auto srv_qos_profile = srv_qos.get_rmw_qos_profile();
#endif

    service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::service_callback, this, _1, _2), srv_qos_profile);
    service_clear_errors_ = rclcpp::Node::create_service<Empty>("clear_errors", std::bind(&ODriveCanNode::service_clear_errors_callback, this, _1, _2), srv_qos_profile);
}

ODriveCanNode::~ODriveCanNode() {
    deinit();
}

void ODriveCanNode::run_can_event_loop() {
    try {
        if (event_loop_->run_until_empty()) return;
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "CAN event loop failed");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "CAN event loop failed: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "CAN event loop failed with an unknown exception");
    }
    stopping_.store(true, std::memory_order_relaxed);
    fresh_heartbeat_.notify_all();
    axis_state_complete_.notify_all();
    clear_errors_complete_.notify_all();
    rclcpp::shutdown(get_node_base_interface()->get_context(), "CAN event loop failed");
}

void ODriveCanNode::deinit() {
    stopping_.store(true, std::memory_order_relaxed);
    fresh_heartbeat_.notify_all();
    axis_state_complete_.notify_all();
    clear_errors_complete_.notify_all();
    if (event_loop_) event_loop_->request_stop();
    if (can_event_loop_.joinable()) can_event_loop_.join();

    // No epoll callback can access the socket or node state beyond this point.
    if (initialized_ && axis_idle_on_shutdown_) {
        struct can_frame frame = {};
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
        frame.can_dlc = 4;
        can_intf_.send_can_frame(frame);
    }

    sub_evt_.deinit();
    srv_evt_.deinit();
    srv_clear_errors_evt_.deinit();
    can_intf_.deinit();
    initialized_ = false;
    event_loop_ = nullptr;
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {
    if (event_loop_ || !event_loop) return false;
    event_loop_ = event_loop;

    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    axis_idle_on_shutdown_ = rclcpp::Node::get_parameter("axis_idle_on_shutdown").as_bool();
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (!can_intf_.init(interface, event_loop, [this](const can_frame& frame) { recv_callback(frame); })) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        deinit();
        return false;
    }
    if (!sub_evt_.init(event_loop, [this](uint32_t) { ctrl_msg_callback(); })) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        deinit();
        return false;
    }
    if (!srv_evt_.init(event_loop, [this](uint32_t) { request_state_callback(); })) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
        deinit();
        return false;
    }
    if (!srv_clear_errors_evt_.init(event_loop, [this](uint32_t) { request_clear_errors_callback(); })) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
        deinit();
        return false;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    initialized_ = true;
    stopping_.store(false, std::memory_order_relaxed);
    try {
        can_event_loop_ = std::thread(&ODriveCanNode::run_can_event_loop, this);
    } catch (...) {
        deinit();
        throw;
    }
    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {
    if (stopping_.load(std::memory_order_relaxed)) return;

    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    switch(frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors    = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            ++heartbeat_sequence_;
            ctrl_pub_flag_ |= 0b0001;
            fresh_heartbeat_.notify_one();
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000; 
            break;
        }
        case CmdId::kSetAxisState:
        case CmdId::kSetControllerMode:
        case CmdId::kSetInputPos:
        case CmdId::kSetInputVel:
        case CmdId::kSetInputTorque:
        case CmdId::kClearErrors: {
            break; // Ignore commands coming from another master/host on the bus
        }
        default: {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }
    
    // SCR patch (2026-08-17): a failed telemetry publish must never abort the
    // process that drives a motor. Drop the sample, log throttled, keep going.
    if (ctrl_pub_flag_ == 0b1111) {
        try {
            ctrl_publisher_->publish(ctrl_stat_);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(rclcpp::Node::get_logger(), *rclcpp::Node::get_clock(), 1000,
                "dropping controller_status publish: %s", e.what());
        }
        ctrl_pub_flag_ = 0;
    }

    if (odrv_pub_flag_ == 0b111) {
        try {
            odrv_publisher_->publish(odrv_stat_);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(rclcpp::Node::get_logger(), *rclcpp::Node::get_clock(), 1000,
                "dropping odrive_status publish: %s", e.what());
        }
        odrv_pub_flag_ = 0;
    }
}

void ODriveCanNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
    if (stopping_.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

void ODriveCanNode::service_callback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<AxisState::Request> request) {
    const auto context = get_node_base_interface()->get_context();
    if (stopping_.load(std::memory_order_relaxed) || !rclcpp::ok(context)) return;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request->axis_requested_state;
        axis_state_pending_ = true;
        axis_state_sent_ = false;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
        if (!srv_evt_.set()) {
            axis_state_pending_ = false;
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to queue axis state request");
            return;
        }
        while (axis_state_pending_ && !stopping_.load(std::memory_order_relaxed) &&
               rclcpp::ok(context)) {
            axis_state_complete_.wait_for(guard, std::chrono::milliseconds(100));
        }
        if (stopping_.load(std::memory_order_relaxed) || !rclcpp::ok(context)) return;
        if (!axis_state_sent_) {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to send axis state request; no success response sent");
            return;
        }
    }

    // Wait for at least 1 second for a new heartbeat to arrive.
    // If the requested state is something other than CLOSED_LOOP_CONTROL, also
    // wait for the procedure to complete (procedure_result != BUSY).
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    auto call_time = std::chrono::steady_clock::now();
    auto heartbeat_sequence = heartbeat_sequence_;
    while (!stopping_.load(std::memory_order_relaxed) &&
           rclcpp::ok(context)) {
        if (heartbeat_sequence_ != heartbeat_sequence) {
            heartbeat_sequence = heartbeat_sequence_;
            bool is_busy = ctrl_stat_.procedure_result == ODriveProcedureResult::PROCEDURE_RESULT_BUSY;
            bool requested_closed_loop = request->axis_requested_state == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
            bool minimum_time_passed = (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1));
            if ((requested_closed_loop || !is_busy) && minimum_time_passed) break;
        }
        // The ROS signal handler invalidates the context without sending a
        // heartbeat. Recheck it even when this axis is disconnected.
        fresh_heartbeat_.wait_for(guard, std::chrono::milliseconds(100));
    }
    
    if (stopping_.load(std::memory_order_relaxed) || !rclcpp::ok(context)) return;
    AxisState::Response response;
    response.axis_state = ctrl_stat_.axis_state;
    response.active_errors = ctrl_stat_.active_errors;
    response.procedure_result = ctrl_stat_.procedure_result;
    guard.unlock();
    try {
        service_->send_response(*request_header, response);
    } catch (const rclcpp::exceptions::RCLError&) {
        // A signal can invalidate the context after the validity check.
        if (rclcpp::ok(context)) throw;
    }
}

void ODriveCanNode::service_clear_errors_callback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<Empty::Request> /*request*/) {
    const auto context = get_node_base_interface()->get_context();
    if (stopping_.load(std::memory_order_relaxed) || !rclcpp::ok(context)) return;
    RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
    std::unique_lock<std::mutex> guard(clear_errors_mutex_);
    clear_errors_pending_ = true;
    clear_errors_sent_ = false;
    if (!srv_clear_errors_evt_.set()) {
        clear_errors_pending_ = false;
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to queue clear errors request");
        return;
    }
    while (clear_errors_pending_ && !stopping_.load(std::memory_order_relaxed) &&
           rclcpp::ok(context)) {
        clear_errors_complete_.wait_for(guard, std::chrono::milliseconds(100));
    }
    if (stopping_.load(std::memory_order_relaxed) || !rclcpp::ok(context)) return;
    if (!clear_errors_sent_) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to send clear errors request; no success response sent");
        return;
    }
    guard.unlock();
    Empty::Response response;
    try {
        service_clear_errors_->send_response(*request_header, response);
    } catch (const rclcpp::exceptions::RCLError&) {
        if (rclcpp::ok(context)) throw;
    }
}

void ODriveCanNode::request_state_callback() {
    if (stopping_.load(std::memory_order_relaxed)) return;
    uint32_t axis_state;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state = axis_state_;
    }

    struct can_frame frame = {};
    bool sent = true;

    if (axis_state != ODriveAxisState::AXIS_STATE_IDLE) {
        // Clear errors if requested state is not IDLE
        frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
        write_le<uint8_t>(0, frame.data);
        frame.can_dlc = 1;
        sent = can_intf_.send_can_frame(frame);
    }

    if (sent) {
        // Do not enter the requested state if its prerequisite clear failed.
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(axis_state, frame.data);
        frame.can_dlc = 4;
        sent = can_intf_.send_can_frame(frame);
    }
    {
        std::lock_guard<std::mutex> guard(axis_state_mutex_);
        axis_state_sent_ = sent;
        axis_state_pending_ = false;
    }
    axis_state_complete_.notify_one();
}

void ODriveCanNode::request_clear_errors_callback() {
    if (stopping_.load(std::memory_order_relaxed)) return;
    struct can_frame frame = {};
    frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
    write_le<uint8_t>(0, frame.data);
    frame.can_dlc = 1;
    bool sent = can_intf_.send_can_frame(frame);
    {
        std::lock_guard<std::mutex> guard(clear_errors_mutex_);
        clear_errors_sent_ = sent;
        clear_errors_pending_ = false;
    }
    clear_errors_complete_.notify_one();
}

void ODriveCanNode::ctrl_msg_callback() {
    if (stopping_.load(std::memory_order_relaxed)) return;

    uint32_t control_mode;
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
    
    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,       frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }    
        default: 
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
            return;
    }

    can_intf_.send_can_frame(frame);
}

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}
