#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

#include "epoll_event_loop.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <functional>

using FrameProcessor = std::function<void(const can_frame&)>;

class SocketCanIntf {
public:
    SocketCanIntf() = default;
    ~SocketCanIntf();
    SocketCanIntf(const SocketCanIntf&) = delete;
    SocketCanIntf& operator=(const SocketCanIntf&) = delete;
    bool init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor);
    void deinit();
    bool send_can_frame(const can_frame& frame);

    bool read_nonblocking();
private:
    int socket_id_ = -1;
    EpollEventLoop* event_loop_ = nullptr;
    EpollEventLoop::EventContext socket_evt_;
    FrameProcessor frame_processor_;

    void on_socket_event(uint32_t mask);
    void process_can_frame(const can_frame& frame) {
        frame_processor_(frame);
    }
};

#endif  // SOCKET_CAN_HPP
