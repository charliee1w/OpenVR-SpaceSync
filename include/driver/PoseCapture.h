// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <openvr_driver.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

namespace spacesync {

// Diagnostic capture never waits for disk or queue space on a pose callback.
// Start/Stop belong to the lifecycle owner; TryRecord may run concurrently.
class PoseCapture {
public:
    enum class Role : uint32_t { Hmd = 0, Tracker = 1 };
    struct Limits { uint64_t records = 100000; double seconds = 120.0; };
    struct Stats { uint64_t accepted = 0, written = 0, dropped = 0; bool ioFailed = false; };
    class Sink {
    public:
        virtual ~Sink() = default;
        virtual bool Write(std::string_view data) noexcept = 0;
        // Thread-safe, nonblocking cancellation; any outstanding Write must
        // relinquish its buffers before returning. Future writes must fail.
        virtual void Cancel() noexcept = 0;
    };

    PoseCapture() = default;
    ~PoseCapture();
    PoseCapture(const PoseCapture&) = delete;
    PoseCapture& operator=(const PoseCapture&) = delete;
    bool Start(const std::filesystem::path& path);
    bool Start(std::unique_ptr<Sink> sink, Limits limits);
    void Stop() noexcept;
    bool TryRecord(uint32_t device, Role role, uint64_t generation,
        double arrivalSeconds, const vr::DriverPose_t& pose) noexcept;
    Stats GetStats() const noexcept;

private:
    static constexpr size_t Capacity = 512;
    struct Record {
        uint32_t device;
        Role role;
        uint64_t generation;
        double arrival;
        vr::DriverPose_t pose;
    };
    void Run() noexcept;
    std::atomic<bool> enabled{false};
    std::atomic<uint64_t> accepted{0}, written{0}, dropped{0};
    std::atomic<bool> ioFailed{false};
    std::mutex mutex;
    std::condition_variable ready;
    std::thread worker;
    std::unique_ptr<Record[]> queue;
    std::unique_ptr<Sink> sink;
    size_t head = 0, count = 0;
    bool stopping = false, hasFirst = false, finished = false;
    double firstArrival = 0;
    Limits limits;
};
}
