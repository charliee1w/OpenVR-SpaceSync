// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

// One owned work item, including its unconsumed completion. A stalled solver
// cannot accumulate memory or hold the submitting thread's state lock. The
// owner decides when a completion is still relevant and may publish it.
template<class Work> class BoundedWorker
{
public:
    using Solver = std::function<void(Work&)>;
    struct Completion
    {
        std::unique_ptr<Work> work;
        std::exception_ptr error;
        explicit operator bool() const { return bool(work); }
    };

    explicit BoundedWorker(Solver solver) : solver(std::move(solver)) { }
    ~BoundedWorker() { Stop(); }
    BoundedWorker(const BoundedWorker&) = delete;
    BoundedWorker& operator=(const BoundedWorker&) = delete;

    // Explicit startup keeps DLL static initialization free of worker threads.
    bool Start()
    {
        std::lock_guard stopLock(stopMutex);
        std::lock_guard lock(mutex);
        if (stopped) return false;
        if (thread.joinable()) return true;
        accepting = true;
        try { thread = std::thread([this] { Run(); }); }
        catch (...) { accepting = false; return false; }
        return true;
    }

    bool Submit(std::unique_ptr<Work> work)
    {
        std::lock_guard lock(mutex);
        if (!accepting || pending || running || completed || !work) return false;
        pending = std::move(work);
        ready.notify_one();
        return true;
    }

    Completion Take()
    {
        std::lock_guard lock(mutex);
        return std::move(completed);
    }

    void Stop()
    {
        // Concurrent shutdown callers may not join the same std::thread.
        std::lock_guard stopLock(stopMutex);
        {
            std::lock_guard lock(mutex);
            accepting = false;
            stopped = true;
            pending.reset();
            ready.notify_all();
        }
        if (thread.joinable()) thread.join();
        std::lock_guard lock(mutex);
        completed = {};
    }

private:
    void Run()
    {
        std::unique_lock lock(mutex);
        for (;;)
        {
            ready.wait(lock, [this] { return pending || !accepting; });
            if (!accepting) return;
            auto work = std::move(pending);
            running = true;
            lock.unlock();
            std::exception_ptr error;
            try { solver(*work); }
            catch (...) { error = std::current_exception(); }
            lock.lock();
            running = false;
            if (accepting) completed = {std::move(work), error};
        }
    }

    Solver solver;
    std::mutex mutex, stopMutex;
    std::condition_variable ready;
    bool accepting = false, running = false, stopped = false;
    std::unique_ptr<Work> pending;
    Completion completed;
    // Last: all state above exists before the worker can start.
    std::thread thread;
};
