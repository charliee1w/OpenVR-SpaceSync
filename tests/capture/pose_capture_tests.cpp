// SPDX-License-Identifier: AGPL-3.0-only
#include "PoseCapture.h"
#include <chrono>
#include <cstdio>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <Windows.h>

using spacesync::PoseCapture;
static void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
struct State {
    std::string data;
    std::atomic<bool> block{false}, entered{false}, fail{false};
};
class MemorySink final : public PoseCapture::Sink {
    std::shared_ptr<State> state;
public:
    explicit MemorySink(std::shared_ptr<State> state) : state(std::move(state)) {}
    bool Write(std::string_view data) noexcept override {
        if (data.starts_with("sample,")) {
            state->entered = true;
            while (state->block.load()) std::this_thread::yield();
            if (state->fail) return false;
        }
        try { state->data.append(data); return true; } catch (...) { return false; }
    }
};
static vr::DriverPose_t pose() {
    vr::DriverPose_t p{};
    p.qRotation = p.qWorldFromDriverRotation = p.qDriverFromHeadRotation = {1, 0, 0, 0};
    p.poseIsValid = p.deviceIsConnected = true;
    p.result = vr::TrackingResult_Running_OK;
    p.vecPosition[0] = 1.25; p.poseTimeOffset = -.02;
    return p;
}
static bool record(PoseCapture& c, double t = 10) {
    return c.TryRecord(0, PoseCapture::Role::Hmd, 7, t, pose());
}
static void until(const std::function<bool()>& predicate) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate() && std::chrono::steady_clock::now() < end) std::this_thread::yield();
    require(predicate(), "worker did not reach expected state");
}
static void lifecycle() {
    PoseCapture c;
    require(!record(c), "disabled capture must do nothing");
    auto s = std::make_shared<State>();
    require(c.Start(std::make_unique<MemorySink>(s), {}), "capture must start");
    require(record(c), "valid capture must accept pose");
    auto bad = pose(); bad.poseIsValid = false; bad.vecPosition[2] = std::numeric_limits<double>::quiet_NaN();
    require(c.TryRecord(1, PoseCapture::Role::Tracker, 8, 10.01, bad), "invalid tracking must remain visible in diagnostic capture");
    c.Stop(); c.Stop();
    require(c.GetStats().written == 2, "stop must drain accepted records");
    require(s->data.find("# spacesync_pose_capture,1") == 0, "versioned header missing");
    require(s->data.find("sample,10,0,0,7,") != std::string::npos, "arrival/identity/epoch metadata missing");
    require(!record(c), "stopped capture accepted pose");
    s = std::make_shared<State>();
    require(c.Start(std::make_unique<MemorySink>(s), {}), "capture restart must work");
    require(record(c), "restarted capture must accept pose");
    c.Stop(); require(c.GetStats().written == 1, "restart stats must reset");
}
static void bounds() {
    PoseCapture c; auto s = std::make_shared<State>();
    require(c.Start(std::make_unique<MemorySink>(s), {2, 1.0}), "bounded capture start");
    require(!record(c, std::numeric_limits<double>::infinity()), "nonfinite arrival accepted");
    require(record(c, 100), "first bounded record");
    require(record(c, 100.5), "second bounded record");
    require(!record(c, 100.6), "record limit exceeded");
    c.Stop(); require(c.GetStats().written == 2, "wrong bounded file length");
    require(c.Start(std::make_unique<MemorySink>(s), {100, .5}), "duration start");
    require(record(c, 100), "duration first record");
    require(!record(c, 100.501), "duration limit exceeded");
    c.Stop(); require(c.GetStats().written == 1, "duration accepted late sample");
    require(!c.Start(std::make_unique<MemorySink>(s), {0, 1}), "zero limit accepted");
    require(!c.Start(std::make_unique<MemorySink>(s), {100001, 1}), "oversized record limit accepted");
    require(!c.Start(std::make_unique<MemorySink>(s), {1, 121}), "oversized duration accepted");
}
static void saturation() {
    PoseCapture c; auto s = std::make_shared<State>(); s->block = true;
    require(c.Start(std::make_unique<MemorySink>(s), {}), "saturation start");
    require(record(c), "saturation first record"); until([&] { return s->entered.load(); });
    auto producer = std::async(std::launch::async, [&] {
        for (int i = 0; i < 2000; ++i) record(c, 10 + i * .0001);
    });
    const bool completed = producer.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    s->block = false; producer.get(); c.Stop();
    require(completed, "callback blocked on slow capture sink");
    require(c.GetStats().accepted <= 513, "capture queue exceeded fixed capacity");
    require(c.GetStats().dropped > 0, "capture saturation was silent");
}
static void failure() {
    PoseCapture c; auto s = std::make_shared<State>(); s->fail = true;
    require(c.Start(std::make_unique<MemorySink>(s), {}), "failure start");
    require(record(c), "failure first record"); until([&] { return c.GetStats().ioFailed; });
    require(!record(c), "failed writer must close admission"); c.Stop();
    require(c.GetStats().written == 0, "failed write reported success");
    auto path = std::filesystem::temp_directory_path() / ("spacesync-capture-" + std::to_string(GetCurrentProcessId()) + ".csv");
    require(c.Start(path), "file capture start"); require(record(c), "file first record"); c.Stop();
    require(!c.Start(path), "existing capture must not be overwritten");
    std::filesystem::remove(path);
    require(!c.Start(path / "missing" / "capture.csv"), "missing directory reported successful start");
}
int main(int argc, char** argv) {
    if (argc == 3 && std::string_view(argv[1]) == "--write-fixture") {
        PoseCapture c;
        if (!c.Start(std::filesystem::path(argv[2]))) return 1;
        if (!record(c, 20)) return 1;
        c.Stop();
        return c.GetStats().written == 1 && !c.GetStats().ioFailed ? 0 : 1;
    }
    try { lifecycle(); bounds(); saturation(); failure(); std::puts("PASS: bounded pose capture"); return 0; }
    catch (const std::exception& e) { std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
}
