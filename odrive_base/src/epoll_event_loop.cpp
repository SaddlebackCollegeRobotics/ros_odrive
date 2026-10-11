#include "epoll_event_loop.hpp"
#include <cerrno>
#include <utility>

EpollEventLoop::EpollEventLoop() {
    epollfd = epoll_create1(EPOLL_CLOEXEC);
    if (epollfd < 0) return;

    stopfd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    struct epoll_event ev = {
        .events = EPOLLIN,
        .data = { .ptr = nullptr }
    };
    if (stopfd_ < 0 || epoll_ctl(epollfd, EPOLL_CTL_ADD, stopfd_, &ev) == -1) {
        if (stopfd_ >= 0) close(stopfd_);
        close(epollfd);
        stopfd_ = -1;
        epollfd = -1;
    }
}

EpollEventLoop::~EpollEventLoop() {
    if (stopfd_ >= 0) close(stopfd_);
    if (epollfd >= 0) close(epollfd);
}

bool EpollEventLoop::register_event(EventContext& evt, int fd, uint32_t events, Callback callback) {
    if (epollfd < 0 || stop_requested() || evt.fd >= 0) return false;
    evt.callback = std::move(callback);
    struct epoll_event ev = {
        .events = events,
        .data = { .ptr = &evt }
    };
    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev) == -1) return false;

    evt.fd = fd;
    ++n_events_;
    return true;
}

bool EpollEventLoop::deregister_event(EventContext& evt) {
    if (evt.fd < 0) return true;
    bool removed = epoll_ctl(epollfd, EPOLL_CTL_DEL, evt.fd, nullptr) != -1;
    for (int i = 0; i < n_triggered_events_; ++i) {
        if (triggered_events_[i].data.ptr == &evt) {
            triggered_events_[i].data.ptr = nullptr;
        }
    }
    evt.fd = -1;
    --n_events_;
    return removed;
}

bool EpollEventLoop::run_until_empty() {
    if (epollfd < 0) return false;
    while (n_events_ && !stop_requested()) {
        n_triggered_events_ = epoll_wait(epollfd, triggered_events_, kMaxEventsPerIteration, -1);
        if (n_triggered_events_ == -1) {
            n_triggered_events_ = 0;
            if (errno == EINTR) continue;
            return false;
        }
        for (int i = 0; i < n_triggered_events_ && !stop_requested(); ++i) {
            auto* handler = static_cast<EventContext*>(triggered_events_[i].data.ptr);
            if (handler) handler->callback(triggered_events_[i].events);
        }
        n_triggered_events_ = 0;
    }
    return true;
}

void EpollEventLoop::request_stop() {
    stop_requested_.store(true, std::memory_order_relaxed);
    if (stopfd_ < 0) return;
    const uint64_t val = 1;
    ssize_t written;
    do {
        written = write(stopfd_, &val, sizeof(val));
    } while (written < 0 && errno == EINTR);
    // EAGAIN means the wake-up is already pending.
}

EpollEvent::~EpollEvent() {
    deinit();
}

bool EpollEvent::init(EpollEventLoop* event_loop, const Callback& callback) {
    if (fd_ >= 0 || !event_loop) return false;
    event_loop_ = event_loop;
    callback_ = callback;

    fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (fd_ < 0) {
        event_loop_ = nullptr;
        return false;
    }

    if (!event_loop_->register_event(evt_, fd_, EPOLLIN, [this](uint32_t event_id) { on_trigger(event_id); })) {
        deinit();
        std::cerr << "Failed to register event" << std::endl;
        return false;
    }

    return true;
}

void EpollEvent::deinit() {
    if (event_loop_) event_loop_->deregister_event(evt_);
    if (fd_ >= 0) close(fd_);
    fd_ = -1;
    event_loop_ = nullptr;
}

bool EpollEvent::set() {
    const uint64_t val = 1;
    ssize_t written;
    do {
        written = write(fd_, &val, sizeof(val));
    } while (written < 0 && errno == EINTR);
    return written == static_cast<ssize_t>(sizeof(val)) || (written < 0 && errno == EAGAIN);
}

void EpollEvent::on_trigger(uint32_t event_id) {
    uint64_t val;
    ssize_t received;
    do {
        received = read(fd_, &val, sizeof(val));
    } while (received < 0 && errno == EINTR);
    if (received != static_cast<ssize_t>(sizeof(val))) {
        if (received < 0 && errno == EAGAIN) return;
        std::cerr << "Failed to read eventfd" << std::endl;
        return;
    }

    callback_(event_id);
}
