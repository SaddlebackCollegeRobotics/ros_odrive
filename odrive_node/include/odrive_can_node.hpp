#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <array>
#include <atomic>
#include <thread>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

using AxisState = odrive_can::srv::AxisState;
using Empty = std_srvs::srv::Empty;

class ODriveCanNode : public rclcpp::Node {
public:
    ODriveCanNode(const std::string& node_name);
    ~ODriveCanNode() override;
    bool init(EpollEventLoop* event_loop); 
    void deinit();
private:
    void run_can_event_loop();
    void recv_callback(const can_frame& frame);
    void subscriber_callback(const ControlMessage::SharedPtr msg);
    void service_callback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<AxisState::Request> request);
    void service_clear_errors_callback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<Empty::Request> request);
    void request_state_callback();
    void request_clear_errors_callback();
    void ctrl_msg_callback();
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);
    
    uint16_t node_id_ = 0;
    bool axis_idle_on_shutdown_ = false;
    bool initialized_ = false;
    std::atomic<bool> stopping_{true};
    EpollEventLoop* event_loop_ = nullptr;
    SocketCanIntf can_intf_;
    
    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    uint64_t heartbeat_sequence_ = 0;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;
    
    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;

    EpollEvent srv_evt_;
    uint32_t axis_state_;
    std::mutex axis_state_mutex_;
    std::condition_variable axis_state_complete_;
    bool axis_state_pending_ = false;
    bool axis_state_sent_ = false;
    std::condition_variable fresh_heartbeat_;
    rclcpp::Service<AxisState>::SharedPtr service_;

    EpollEvent srv_clear_errors_evt_;
    rclcpp::Service<Empty>::SharedPtr service_clear_errors_;
    std::mutex clear_errors_mutex_;
    std::condition_variable clear_errors_complete_;
    bool clear_errors_pending_ = false;
    bool clear_errors_sent_ = false;

    // Joined by deinit() before any callback targets or descriptors are destroyed.
    std::thread can_event_loop_;

};

#endif // ODRIVE_CAN_NODE_HPP
