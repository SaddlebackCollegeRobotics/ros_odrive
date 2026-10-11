#include "socket_can.hpp"
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>
#include <net/if.h>
#include <sys/ioctl.h>

SocketCanIntf::~SocketCanIntf() {
    deinit();
}

bool SocketCanIntf::init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor) {
    if (socket_id_ >= 0 || !event_loop) return false;
    event_loop_ = event_loop;
    frame_processor_ = std::move(frame_processor);
    socket_id_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (socket_id_ == -1) {
        event_loop_ = nullptr;
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }

    struct ifreq ifr = {};
    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(socket_id_, SIOCGIFINDEX, &ifr) == -1) {
        std::cerr << "Failed to get interface index" << std::endl;
        deinit();
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_id_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        std::cerr << "Failed to bind socket" << std::endl;
        deinit();
        return false;
    }

    struct msghdr message = {
        .msg_name = nullptr,
        .msg_namelen = 0,
        .msg_iov = nullptr,
        .msg_iovlen = 0,
        .msg_control = nullptr,
        .msg_controllen = 0,
        .msg_flags = 0
    };

    int retcode = recvmsg(socket_id_, &message, 0);
    if (retcode < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        deinit();
        return false;
    }

    if (!event_loop_->register_event(socket_evt_, socket_id_, EPOLLIN, [this](uint32_t mask) { on_socket_event(mask); })) {
        std::cerr << "Failed to register socket with event loop" << std::endl;
        deinit();
        return false;
    }

    return true;
}

void SocketCanIntf::deinit() {
    if (event_loop_) event_loop_->deregister_event(socket_evt_);
    if (socket_id_ >= 0) close(socket_id_);
    socket_id_ = -1;
    event_loop_ = nullptr;
}

bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    ssize_t nbytes;
    do {
        nbytes = write(socket_id_, &frame, sizeof(frame));
    } while (nbytes < 0 && errno == EINTR);
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        const int error = errno;
        std::cerr << "Failed to send CAN frame";
        if (nbytes < 0) std::cerr << ": " << std::strerror(error);
        else std::cerr << ": short write " << nbytes;
        std::cerr << std::endl;
        return false;
    }

    return true;
}

void SocketCanIntf::on_socket_event(uint32_t mask) {
    if (mask & EPOLLIN) {
        while (socket_id_ >= 0 && !event_loop_->stop_requested() && read_nonblocking());
    }
    if (mask & EPOLLERR) {
        std::cerr << "interface disappeared" << std::endl;
        deinit();
        return;
    }
    if (mask & ~(EPOLLIN | EPOLLERR)) {
        std::cerr << "unexpected event " << mask << std::endl;
        deinit();
        return;
    }
    return;
}

bool SocketCanIntf::read_nonblocking() {
    if (socket_id_ < 0) return false;
    struct can_frame frame;
    struct cmsghdr ctrlmsg;

    struct iovec vec = {.iov_base = &frame, .iov_len = sizeof(frame)};
    struct msghdr message = {
        .msg_name = nullptr,
        .msg_namelen = 0,
        .msg_iov = &vec, 
        .msg_iovlen = 1,
        .msg_control = &ctrlmsg,
        .msg_controllen = sizeof(ctrlmsg),
        .msg_flags = 0
        };

    ssize_t n_received = recvmsg(socket_id_, &message, MSG_DONTWAIT);
    if (n_received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // std::cerr << "no message received" << std::endl;
            return false;
        } else {
            std::cerr << "Socket read failed: " << std::endl;
            return false;
        }
    }

    if (n_received < static_cast<ssize_t>(sizeof(struct can_frame))) {
        std::cerr << "invalid message length " << n_received << std::endl;
        return true;
    }

    process_can_frame(frame);
    return true;
}
