// SPDX-License-Identifier: AGPL-3.0-only
#include "HookRundown.h"
#include <atomic>
#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>
#include <cstdio>

using namespace std::chrono_literals;
static void check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }

int main()
{
    try {
        HookRundown rundown;
        std::promise<void> entered, release, detached;
        auto releaseFuture = release.get_future();
        std::atomic<bool> registered = false;
        std::thread callback([&] {
            HookRundown::Lease lease(rundown);
            check(static_cast<bool>(lease), "callback admission");
            entered.set_value();
            releaseFuture.wait(); // Represents both provider work AND original call.
            rundown.RegisterWhileOpen([&] { registered = true; });
        });
        entered.get_future().wait();
        auto shutdown = std::async(std::launch::async, [&] {
            return rundown.StopAndDrain([&] { detached.set_value(); return true; });
        });
        detached.get_future().wait();
        const bool blocked = shutdown.wait_for(25ms) == std::future_status::timeout;
        bool lateSafe;
        {
            HookRundown::Lease late(rundown);
            lateSafe = !late && late.UseRestoredTarget();
        }
        release.set_value();
        callback.join();
        check(shutdown.get(), "detach success");
        check(blocked, "trampolines released with callback in flight");
        check(lateSafe, "late detour must bypass released trampoline");
        check(!registered, "hook creation raced shutdown");

        HookRundown failure;
        check(!failure.StopAndDrain([] { return false; }), "detach failure reported");
        std::promise<void> retryDetached;
        std::future<bool> retry;
        bool retryBlocked;
        {
            HookRundown::Lease failedLate(failure);
            check(!failedLate && !failedLate.UseRestoredTarget(), "failed detach must retain trampoline");
            retry = std::async(std::launch::async, [&] {
                return failure.StopAndDrain([&] { retryDetached.set_value(); return true; });
            });
            retryDetached.get_future().wait();
            retryBlocked = retry.wait_for(25ms) == std::future_status::timeout;
        }
        check(retry.get() && retryBlocked, "detach retry freed a trampoline used by a denied call");
        printf("PASS hook shutdown, late entry, registration exclusion, detach failure\n");
    } catch (const std::exception& error) {
        fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
