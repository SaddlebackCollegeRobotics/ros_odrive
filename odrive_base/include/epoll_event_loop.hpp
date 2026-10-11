#ifndef EPOLL_EVENT_LOOP_HPP
#define EPOLL_EVENT_LOOP_HPP

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <iostream>
#include <functional>
#include <atomic>
#include <unistd.h>

using std::placeholders::_1;
using Callback = std::function<void(uint32_t)>;

class EpollEventLoop {
public:
    struct EventContext {
        int fd = -1;
        Callback callback;
    };


    EpollEventLoop();

    ~EpollEventLoop();
    EpollEventLoop(const EpollEventLoop&) = delete;
    EpollEventLoop& operator=(const EpollEventLoop&) = delete;

    // Registration and removal run on the loop thread, or while it is stopped.
    // EventContext and callback owners must outlive run_until_empty().
    bool register_event(EventContext& evt, int fd, uint32_t events, Callback callback);

    bool deregister_event(EventContext& evt);

    bool run_until_empty();

    // The only operation permitted concurrently with run_until_empty().
    void request_stop();
    bool stop_requested() const { return stop_requested_.load(std::memory_order_relaxed); }

private:
    static constexpr size_t kMaxEventsPerIteration = 16;
    int epollfd = -1;
    int stopfd_ = -1;
    std::atomic<bool> stop_requested_{false};
    size_t n_events_ = 0;
    int n_triggered_events_ = 0;
    struct epoll_event triggered_events_[kMaxEventsPerIteration];
};

class EpollEvent {
public:
    bool init(EpollEventLoop* event_loop, const Callback& callback);
    EpollEvent() = default;
    ~EpollEvent();
    EpollEvent(const EpollEvent&) = delete;
    EpollEvent& operator=(const EpollEvent&) = delete;
    void deinit();

    bool set();

private:
    void on_trigger(uint32_t event_id);

    EpollEventLoop* event_loop_ = nullptr;
    int fd_ = -1;
    EpollEventLoop::EventContext evt_;
    Callback callback_;
};


#endif // EPOLL_EVENT_LOOP_HPP
