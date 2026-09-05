// SPDX-License-Identifier: AGPL-3.0-only
// Added by charliee1w, 2026-09-04. See NOTICE.md.
#pragma once

#include <condition_variable>
#include <mutex>
#include <utility>

// Owns detour execution through the call to the original function. No lock is
// held across runtime calls, which can re-enter another detour on another thread.
class HookRundown
{
public:
    class Lease
    {
    public:
        explicit Lease(HookRundown& owner) : owner(owner)
        {
            std::lock_guard<std::mutex> lock(owner.mutex);
            admitted = !owner.stopping;
            targetRestored = owner.detached;
            // Denied calls can still be using a retained trampoline after a
            // failed detach; include them if detachment is retried later.
            ++owner.active;
        }
        ~Lease()
        {
            std::lock_guard<std::mutex> lock(owner.mutex);
            if (--owner.active == 0) owner.drained.notify_all();
        }
        Lease(const Lease&) = delete;
        Lease& operator=(const Lease&) = delete;
        explicit operator bool() const { return admitted; }
        bool UseRestoredTarget() const { return targetRestored; }
    private:
        HookRundown& owner;
        bool admitted;
        bool targetRestored;
    };

    template<class Register>
    void RegisterWhileOpen(Register&& registerHook)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!stopping) std::forward<Register>(registerHook)();
    }

    template<class Disable>
    bool StopAndDrain(Disable&& disable)
    {
        std::unique_lock<std::mutex> lock(mutex);
        stopping = true;
        // Serialize detachment with hook creation and lease admission. A thread
        // redirected before detachment can arrive late: it uses the restored
        // entry point, never a trampoline which may already have been released.
        detached = std::forward<Disable>(disable)();
        drained.wait(lock, [this] { return active == 0; });
        return detached;
    }

private:
    std::mutex mutex;
    std::condition_variable drained;
    size_t active = 0;
    bool stopping = false;
    bool detached = false;
};
