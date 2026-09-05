// SPDX-License-Identifier: AGPL-3.0-only
#include "PoseCapture.h"
#include <Windows.h>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace spacesync {
namespace {
constexpr std::string_view Header =
    "# spacesync_pose_capture,1\n"
    "type,arrival,device,role,generation,valid,connected,result,offset,"
    "qw,qx,qy,qz,px,py,pz,vx,vy,vz,wx,wy,wz,ax,ay,az,"
    "world_qw,world_qx,world_qy,world_qz,world_x,world_y,world_z,"
    "head_qw,head_qx,head_qy,head_qz,head_x,head_y,head_z\n";

class FileSink final : public PoseCapture::Sink {
    HANDLE file;
public:
    explicit FileSink(const std::filesystem::path& path)
        : file(CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr,
            CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr)) {}
    ~FileSink() override { if (file != INVALID_HANDLE_VALUE) CloseHandle(file); }
    bool Valid() const { return file != INVALID_HANDLE_VALUE; }
    bool Write(std::string_view data) noexcept override {
        while (!data.empty()) {
            DWORD bytes = 0;
            const DWORD chunk = static_cast<DWORD>((std::min)(data.size(), size_t(1 << 20)));
            if (!WriteFile(file, data.data(), chunk, &bytes, nullptr) || bytes == 0) return false;
            data.remove_prefix(bytes);
        }
        return true;
    }
};
}

PoseCapture::~PoseCapture() { Stop(); }

bool PoseCapture::Start(const std::filesystem::path& path) {
    // CREATE_NEW deliberately refuses an existing file, including after restart.
    try {
        if (worker.joinable()) return false;
        auto file = std::make_unique<FileSink>(path);
        if (!file->Valid()) return false;
        return Start(std::move(file), Limits{});
    } catch (...) { return false; }
}

bool PoseCapture::Start(std::unique_ptr<Sink> destination, Limits requested) {
    if (worker.joinable() || !destination || requested.records == 0 || requested.records > 100000
        || !std::isfinite(requested.seconds) || requested.seconds <= 0 || requested.seconds > 120)
        return false;
    try {
        // Lifecycle calls are serialized by their owner. Header I/O precedes admission.
        if (!destination->Write(Header)) return false;
        queue = std::make_unique<Record[]>(Capacity);
        sink = std::move(destination);
        limits = requested;
        head = count = 0;
        stopping = hasFirst = false;
        accepted = written = dropped = 0;
        ioFailed = false;
        worker = std::thread(&PoseCapture::Run, this);
        enabled.store(true, std::memory_order_release);
        return true;
    } catch (...) {
        queue.reset(); sink.reset(); return false;
    }
}

void PoseCapture::Stop() noexcept {
    enabled.store(false, std::memory_order_release);
    {
        std::lock_guard lock(mutex);
        stopping = true;
    }
    ready.notify_one();
    if (worker.joinable()) worker.join();
    queue.reset();
    sink.reset();
}

bool PoseCapture::TryRecord(uint32_t device, Role role, uint64_t generation,
    double arrivalSeconds, const vr::DriverPose_t& pose) noexcept {
    if (!enabled.load(std::memory_order_acquire)) return false;
    if (!std::isfinite(arrivalSeconds) || arrivalSeconds < 0
        || device >= vr::k_unMaxTrackedDeviceCount || (role != Role::Hmd && role != Role::Tracker)) {
        ++dropped; return false;
    }
    std::unique_lock lock(mutex, std::try_to_lock);
    if (!lock.owns_lock()) { ++dropped; return false; }
    if (!enabled.load(std::memory_order_relaxed) || stopping) return false;
    if (!hasFirst) { firstArrival = arrivalSeconds; hasFirst = true; }
    if (accepted.load(std::memory_order_relaxed) >= limits.records
        || arrivalSeconds - firstArrival > limits.seconds) {
        enabled.store(false, std::memory_order_release);
        stopping = true;
        ready.notify_one();
        return false;
    }
    if (count == Capacity) { ++dropped; return false; }
    queue[(head + count) % Capacity] = {device, role, generation, arrivalSeconds, pose};
    ++count; ++accepted;
    lock.unlock();
    ready.notify_one();
    return true;
}

PoseCapture::Stats PoseCapture::GetStats() const noexcept {
    return {accepted.load(), written.load(), dropped.load(), ioFailed.load()};
}

void PoseCapture::Run() noexcept {
    try {
        for (;;) {
            Record record;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock, [&] { return stopping || count != 0; });
                if (count == 0) break;
                record = queue[head];
                head = (head + 1) % Capacity; --count;
            }
            // Preserve invalid poses too: their flags/NaNs mark a diagnostic gap.
            // Formatting and file I/O only happen on this worker.
            std::ostringstream row;
            row.imbue(std::locale::classic());
            row << std::setprecision(17) << "sample," << record.arrival << ',' << record.device << ','
                << static_cast<uint32_t>(record.role) << ',' << record.generation << ','
                << record.pose.poseIsValid << ',' << record.pose.deviceIsConnected << ','
                << static_cast<int>(record.pose.result) << ',' << record.pose.poseTimeOffset;
            auto q = [&](const vr::HmdQuaternion_t& value) {
                row << ',' << value.w << ',' << value.x << ',' << value.y << ',' << value.z;
            };
            auto v = [&](const double (&value)[3]) {
                row << ',' << value[0] << ',' << value[1] << ',' << value[2];
            };
            q(record.pose.qRotation); v(record.pose.vecPosition);
            v(record.pose.vecVelocity); v(record.pose.vecAngularVelocity); v(record.pose.vecAcceleration);
            q(record.pose.qWorldFromDriverRotation); v(record.pose.vecWorldFromDriverTranslation);
            q(record.pose.qDriverFromHeadRotation); v(record.pose.vecDriverFromHeadTranslation);
            row << '\n';
            if (!sink->Write(row.str())) throw std::runtime_error("capture write failed");
            ++written;
        }
        const auto stats = GetStats();
        const auto footer = "# end,accepted=" + std::to_string(stats.accepted)
            + ",written=" + std::to_string(stats.written) + ",dropped=" + std::to_string(stats.dropped) + "\n";
        if (!sink->Write(footer)) throw std::runtime_error("capture footer failed");
    } catch (...) {
        enabled.store(false, std::memory_order_release);
        ioFailed = true;
        std::lock_guard lock(mutex);
        dropped.fetch_add(count + 1);
        count = 0; stopping = true;
    }
}
}
