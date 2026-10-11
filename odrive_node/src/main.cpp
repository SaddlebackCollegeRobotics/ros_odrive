#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include <cstdio>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        // The loop outlives the node. The node joins its worker before removing
        // registrations, closing descriptors, or destroying callback targets.
        EpollEventLoop event_loop;
        auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");
        if (can_node->init(&event_loop)) {
            rclcpp::spin(can_node);
        } else {
            result = 1;
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ODrive CAN node failed: %s\n", e.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
