#pragma once

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <vector>

class ThreadPool {
public:
    explicit ThreadPool(std::size_t n_workers);
    ~ThreadPool();

    template<typename F, typename... Args>
    auto submit(F &&f, Args &&... args)
        -> std::future<typename std::result_of<F(Args...)>::type>;

private:
    void worker_loop();

    std::vector<std::thread>            workers_;
    std::queue<std::function<void()>>   tasks_;
    std::mutex                          mtx_;
    std::condition_variable             cv_;
    bool                                stop_ = false;
};

// ===== Template implementation =====

inline ThreadPool::ThreadPool(std::size_t n_workers) {
    for (std::size_t i = 0; i < n_workers; ++i) {
        workers_.emplace_back([this]{ worker_loop(); });
    }
}

inline ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto &t : workers_) {
        if (t.joinable()) t.join();
    }
}

inline void ThreadPool::worker_loop() {
    while (true) {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this]{ return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty()) return;
            job = std::move(tasks_.front());
            tasks_.pop();
        }
        job();
    }
}

template<typename F, typename... Args>
auto ThreadPool::submit(F &&f, Args &&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using R = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<R()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    {
        std::lock_guard<std::mutex> lk(mtx_);
        tasks_.emplace([task]{ (*task)(); });
    }
    cv_.notify_one();
    return task->get_future();
}
