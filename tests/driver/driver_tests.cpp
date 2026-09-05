// SPDX-License-Identifier: AGPL-3.0-only
#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <limits>
#include <functional>
#include <future>
#include <stdexcept>
#include <thread>

struct DriverStateTestAccess
{
    static int trackerHistory(const ServerTrackedDeviceProvider& p) { return p.clock.tracker.count; }
    static int hmdHistory(const ServerTrackedDeviceProvider& p) { return p.clock.hmd.count; }
    static bool trackerPrimed(const ServerTrackedDeviceProvider& p) { return p.frames.primed; }
    static double lastClockTrackerX(const ServerTrackedDeviceProvider& p)
    {
        return p.clock.trackerPoses.poses[(p.clock.trackerPoses.next + 15) % 16].position.v[0];
    }
    static void primeClock(ServerTrackedDeviceProvider& p)
    {
        LARGE_INTEGER now, frequency; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&frequency);
        const double seconds = double(now.QuadPart) / frequency.QuadPart;
        for (int i = 0; i < 900; ++i)
        {
            const double t = seconds - (900 - i) / 180.0;
            const double rotationSpeed = 1 + .4 * std::sin(i * .05);
            const double translationSpeed = .4 + .2 * std::sin(i * .067);
            p.clock.hmd.add(t, rotationSpeed, translationSpeed);
            p.clock.tracker.add(t, rotationSpeed, translationSpeed);
        }
    }
    static double lastClockSolve(const ServerTrackedDeviceProvider& p) { return p.clock.lastSolve; }
};

static void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
static void checkNear(double got, double want, const char* message)
{
    if (std::abs(got - want) > 1e-8 || !std::isfinite(got)) throw std::runtime_error(message);
}
static vr::DriverPose_t pose(double x = 0, double y = 0, double z = 0)
{
    vr::DriverPose_t p{};
    p.qWorldFromDriverRotation = p.qDriverFromHeadRotation = p.qRotation = {1, 0, 0, 0};
    p.vecPosition[0] = x; p.vecPosition[1] = y; p.vecPosition[2] = z;
    p.deviceIsConnected = p.poseIsValid = true;
    p.result = vr::TrackingResult_Running_OK;
    return p;
}
static protocol::SetHmdTracker config()
{
    protocol::SetHmdTracker c{};
    c.hmdID = 0; c.trackerID = 1; c.enabled = true;
    c.slamFallback = true; c.followSlamHmd = true; c.predictionTime = 1;
    c.offsetRotation = c.calibrationRotation = {1, 0, 0, 0};
    c.calibrationScale = c.hmdScale = 1;
    return c;
}
static void disable(ServerTrackedDeviceProvider& p)
{
    auto c = config(); c.enabled = false; p.SetHmdTracker(c);
}
static vr::HmdVector3d_t worldPosition(const vr::DriverPose_t& p)
{
    auto local = vecAdd(vecFromArray(p.vecPosition), quaternionRotateVector(p.qRotation, p.vecDriverFromHeadTranslation));
    return vecAdd(quaternionRotateVector(p.qWorldFromDriverRotation, local), vecFromArray(p.vecWorldFromDriverTranslation));
}
static void scalePrediction()
{
    ServerTrackedDeviceProvider provider; disable(provider);
    const auto yaw = quaternionFromRotationVector({0, POSE_PI / 2, 0});
    provider.SetDeviceTransform({2, true, {3, 4, 5}, yaw, .5});
    auto p = pose(2, 4, 6); p.vecVelocity[0] = 3; p.vecVelocity[1] = -2;
    provider.HandleDevicePoseUpdated(2, p);
    auto future = p;
    for (int i = 0; i < 3; ++i) future.vecPosition[i] += future.vecVelocity[i] * .02;
    auto w = worldPosition(future);
    checkNear(w.v[0], 6, "scaled/rotated prediction x");
    checkNear(w.v[1], 5.98, "linear velocity must use position scale");
    checkNear(w.v[2], 3.97, "scaled/rotated prediction z");
}
static void referenceScale()
{
    ServerTrackedDeviceProvider provider; disable(provider);
    auto c = config(); c.calibrationScale = c.hmdScale = .5;
    c.calibrationRotation = quaternionFromRotationVector({0, POSE_PI / 2, 0});
    c.calibrationTranslation = {3, 0, 0}; provider.SetHmdTracker(c);
    auto tracker = pose(1, 0, 0); provider.HandleDevicePoseUpdated(1, tracker);
    auto hmd = pose(3, 0, -.5); hmd.qRotation = c.calibrationRotation;
    provider.HandleDevicePoseUpdated(0, hmd);
    provider.SetDeviceTransform({2, true, c.calibrationTranslation, c.calibrationRotation, .5});
    auto body = pose(2, 0, 0); provider.HandleDevicePoseUpdated(2, body);
    auto w = worldPosition(body);
    checkNear(w.v[0], 3, "reference scale x"); checkNear(w.v[2], -1, "head reference must use calibration scale");
}
static void invalidTrackerBoundary()
{
    for (int failure = 0; failure != 8; ++failure)
    {
        ServerTrackedDeviceProvider provider; disable(provider); provider.SetHmdTracker(config());
        for (int i = 0; i != 12; ++i)
        {
            auto p = pose(.01 * i, 0, 0); provider.HandleDevicePoseUpdated(1, p);
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        require(DriverStateTestAccess::trackerHistory(provider) > 0, "valid timing history must accumulate");
        auto bad = pose();
        if (failure == 0) bad.poseIsValid = false;
        if (failure == 1) bad.deviceIsConnected = false;
        if (failure == 2) bad.result = vr::TrackingResult_Running_OutOfRange;
        if (failure == 3) bad.vecPosition[0] = std::numeric_limits<double>::quiet_NaN();
        if (failure == 4) bad.qRotation = {0, 0, 0, 0};
        if (failure == 5) bad.poseTimeOffset = std::numeric_limits<double>::infinity();
        if (failure == 6) bad.vecVelocity[2] = std::numeric_limits<double>::quiet_NaN();
        if (failure == 7) bad.vecAcceleration[1] = std::numeric_limits<double>::infinity();
        provider.HandleDevicePoseUpdated(1, bad);
        require(DriverStateTestAccess::trackerHistory(provider) == 0, "invalid tracker interval must clear timing history");
        require(!DriverStateTestAccess::trackerPrimed(provider), "invalid tracker interval must break differencing");
        auto good = pose(); provider.HandleDevicePoseUpdated(1, good);
        require(DriverStateTestAccess::trackerHistory(provider) == 0, "first recovery pose must not difference across loss");
    }
}
static void transformPublication()
{
    ServerTrackedDeviceProvider provider; disable(provider);
    const auto identity = vr::HmdQuaternion_t{1, 0, 0, 0};
    provider.SetDeviceTransform({2, true, {1, 2, 3}, identity, 1});
    std::atomic<bool> start{false};
    std::thread writer([&] {
        while (!start.load()) std::this_thread::yield();
        for (int i = 0; i != 100000; ++i)
            provider.SetDeviceTransform({2, true, i % 2 ? vr::HmdVector3d_t{1, 2, 3} : vr::HmdVector3d_t{4, 5, 6}, identity, i % 2 ? 1.0 : 2.0});
    });
    bool mixed = false; start = true;
    for (int i = 0; i != 100000; ++i)
    {
        auto p = pose(1, 1, 1); provider.HandleDevicePoseUpdated(2, p);
        const auto w = worldPosition(p);
        const bool a = w.v[0] == 2 && w.v[1] == 3 && w.v[2] == 4;
        const bool b = w.v[0] == 6 && w.v[1] == 7 && w.v[2] == 8;
        if (!a && !b) mixed = true;
    }
    writer.join(); require(!mixed, "callback consumed mixed configuration generations");
}

static void driftScalePrediction()
{
    ServerTrackedDeviceProvider provider; disable(provider);
    auto c = config(); c.trackerID = vr::k_unTrackedDeviceIndexInvalid; c.hmdScale = .5;
    provider.SetHmdTracker(c);
    const auto yaw = quaternionFromRotationVector({0, POSE_PI / 2, 0});
    provider.SetDeviceTransform({2, true, {3, 4, 5}, yaw, .75});
    auto p = pose(1, 2, 3); p.vecVelocity[0] = 8;
    provider.HandleDevicePoseUpdated(2, p);
    checkNear(p.vecVelocity[0], 3, "inverse drift must scale linear velocity after calibration");
    c.followSlamHmd = false; provider.SetHmdTracker(c);
    provider.SetDeviceTransform({2, false}); provider.SetSlamSync({2, true});
    p = pose(1, 2, 3); p.vecVelocity[0] = 8;
    provider.HandleDevicePoseUpdated(2, p);
    checkNear(p.vecVelocity[0], 16, "forward drift must scale linear velocity");
}

class TestPoseSource final : public DriverPoseSource
{
public:
    std::function<void()> duringRead;
    float trackerX = 0;
    vr::TrackedDevicePose_t ReadTrackerPose(uint32_t, uint32_t, float) override
    {
        if (duringRead) duringRead();
        vr::TrackedDevicePose_t p{};
        p.bDeviceIsConnected = p.bPoseIsValid = true; p.eTrackingResult = vr::TrackingResult_Running_OK;
        p.mDeviceToAbsoluteTracking.m[0][0] = p.mDeviceToAbsoluteTracking.m[1][1] = p.mDeviceToAbsoluteTracking.m[2][2] = 1;
        p.mDeviceToAbsoluteTracking.m[0][3] = trackerX;
        return p;
    }
};

static void poseAcquisitionTransactions()
{
    TestPoseSource source;
    ServerTrackedDeviceProvider provider(&source);
    auto c = config(); c.followSlamHmd = false; provider.SetHmdTracker(c);
    source.duringRead = [&] { provider.SetHmdTracker(c); };
    auto p = pose();
    require(provider.HandleDevicePoseUpdated(0, p), "equivalent periodic resend must not discard a pose");
    source.duringRead = [&] { auto disabled = c; disabled.enabled = false; provider.SetHmdTracker(disabled); };
    p = pose(1, 2, 3);
    require(!provider.HandleDevicePoseUpdated(0, p), "changed configuration must discard an old runtime acquisition");
    checkNear(p.vecPosition[0], 1, "discarded pose must not have partial transforms");
}

static void shutdownWaitsForAcquisition()
{
    TestPoseSource source;
    ServerTrackedDeviceProvider provider(&source);
    auto c = config(); c.followSlamHmd = false; provider.SetHmdTracker(c);
    std::promise<void> entered, release;
    auto released = release.get_future().share();
    source.duringRead = [&] { entered.set_value(); released.wait(); };
    auto callback = std::async(std::launch::async, [&] { auto p = pose(); return provider.HandleDevicePoseUpdated(0, p); });
    entered.get_future().wait();
    auto shutdown = std::async(std::launch::async, [&] { provider.ShutdownPoseUpdates(); });
    const bool waited = shutdown.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout;
    release.set_value(); callback.get(); shutdown.get();
    require(waited, "shutdown must drain external acquisitions before releasing the driver context");
    auto p = pose(); require(!provider.HandleDevicePoseUpdated(0, p), "shutdown must close new callback admission");
    provider.ShutdownPoseUpdates();
}

static void invalidHmdBoundary()
{
    ServerTrackedDeviceProvider provider; disable(provider); provider.SetHmdTracker(config());
    for (int i = 0; i != 12; ++i)
    {
        auto p = pose(.01 * i, 0, 0); provider.HandleDevicePoseUpdated(0, p);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    require(DriverStateTestAccess::hmdHistory(provider) > 0, "valid HMD history must accumulate");
    auto bad = pose(); bad.poseIsValid = false; provider.HandleDevicePoseUpdated(0, bad);
    require(DriverStateTestAccess::hmdHistory(provider) == 0, "invalid HMD interval must clear timing history");
    auto good = pose(); provider.HandleDevicePoseUpdated(0, good);
    require(DriverStateTestAccess::hmdHistory(provider) == 0, "HMD recovery must not difference across loss");
}

static void trackerSelectionBreaksHistory()
{
    ServerTrackedDeviceProvider provider; disable(provider); auto c = config(); provider.SetHmdTracker(c);
    for (int i = 0; i != 12; ++i)
    {
        auto p = pose(.01 * i, 0, 0); provider.HandleDevicePoseUpdated(1, p);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    require(DriverStateTestAccess::trackerHistory(provider) > 0, "timing history precondition");
    c.trackerID = 2; provider.SetHmdTracker(c);
    require(DriverStateTestAccess::trackerHistory(provider) == 0, "new tracker must not inherit old serial timing history");
    require(!DriverStateTestAccess::trackerPrimed(provider), "new tracker must not difference against old serial");
}

static void calibratedMountClockPoint()
{
    ServerTrackedDeviceProvider provider; disable(provider); auto c = config();
    c.calibrationScale = .5; c.offsetTranslation = {.2, 0, 0}; provider.SetHmdTracker(c);
    auto p = pose(1, 0, 0); provider.HandleDevicePoseUpdated(1, p);
    checkNear(DriverStateTestAccess::lastClockTrackerX(provider), 1.4, "clock mount offset must be converted to raw tracker units");
}

static void overrideReferenceScale()
{
    TestPoseSource source; source.trackerX = 1;
    ServerTrackedDeviceProvider provider(&source); auto c = config();
    c.followSlamHmd = false; c.calibrationScale = .5;
    c.calibrationRotation = quaternionFromRotationVector({0, POSE_PI / 2, 0});
    c.calibrationTranslation = {3, 0, 0}; provider.SetHmdTracker(c);
    auto p = pose(); provider.HandleDevicePoseUpdated(0, p);
    checkNear(p.vecPosition[0], 3, "override scale x");
    checkNear(p.vecPosition[2], -.5, "override reference must use calibration scale");
}

static void trackerGapBoundary()
{
    ServerTrackedDeviceProvider provider; disable(provider); provider.SetHmdTracker(config());
    for (int i = 0; i != 12; ++i)
    {
        auto p = pose(.01 * i, 0, 0); provider.HandleDevicePoseUpdated(1, p);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    require(DriverStateTestAccess::trackerHistory(provider) > 0, "gap history precondition");
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    auto p = pose(); provider.HandleDevicePoseUpdated(1, p);
    require(DriverStateTestAccess::trackerHistory(provider) == 0, "tracking gap must not leave interpolation history");
    p.poseTimeOffset = -1; provider.HandleDevicePoseUpdated(1, p);
    require(DriverStateTestAccess::trackerHistory(provider) == 0, "backward timestamp must break differencing");
}

static void resetPublication()
{
    ServerTrackedDeviceProvider provider;
    auto c = config(); c.trackerID = vr::k_unTrackedDeviceIndexInvalid;
    provider.SetHmdTracker(c);
    provider.SetDeviceTransform({2, true, {0, 0, 0}, {1, 0, 0, 0}, 1});
    std::atomic<bool> start{false};
    std::thread writer([&] {
        while (!start.load()) std::this_thread::yield();
        for (int i = 0; i != 10000; ++i)
        {
            auto next = c; next.enabled = false; provider.SetHmdTracker(next);
            next.enabled = true; next.hmdScale = i % 2 ? 1 : 2; provider.SetHmdTracker(next);
        }
    });
    bool mixed = false; start = true;
    for (int i = 0; i != 20000; ++i)
    {
        auto p = pose(1, 1, 1); provider.HandleDevicePoseUpdated(2, p);
        const auto w = worldPosition(p);
        if (!((w.v[0] == 1 || w.v[0] == 2) && w.v[0] == w.v[1] && w.v[1] == w.v[2])) mixed = true;
        protocol::DriverStatus status{}; provider.GetStatus(status);
        if (!std::isfinite(status.hmdScale)) mixed = true;
    }
    writer.join(); require(!mixed, "disable/reset cannot publish partial state to a pose or status reader");
}

static void clockCallbackCost()
{
    ServerTrackedDeviceProvider provider; disable(provider); provider.SetHmdTracker(config());
    DriverStateTestAccess::primeClock(provider);
    auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
    auto hmd = pose(); const auto start = std::chrono::steady_clock::now();
    provider.HandleDevicePoseUpdated(0, hmd);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    require(DriverStateTestAccess::lastClockSolve(provider) > 0, "cost fixture must execute the real clock solve");
    std::printf("MEASURE populated clock solve callback %.3f ms (900 samples)\n", ms);
}
int main()
{
    LogFile = tmpfile(); if (!LogFile) return 2;
    int failures = 0;
    const auto run = [&](const char* name, auto test) {
        try { test(); std::printf("PASS %s\n", name); }
        catch (const std::exception& e) { ++failures; std::printf("FAIL %s: %s\n", name, e.what()); }
    };
    run("scale_prediction", scalePrediction);
    run("follow_reference_scale", referenceScale);
    run("invalid_tracker_boundary", invalidTrackerBoundary);
    run("transform_publication", transformPublication);
    run("drift_scale_prediction", driftScalePrediction);
    run("pose_acquisition_transactions", poseAcquisitionTransactions);
    run("shutdown_waits_for_acquisition", shutdownWaitsForAcquisition);
    run("invalid_hmd_boundary", invalidHmdBoundary);
    run("tracker_selection_breaks_history", trackerSelectionBreaksHistory);
    run("calibrated_mount_clock_point", calibratedMountClockPoint);
    run("override_reference_scale", overrideReferenceScale);
    run("tracker_gap_boundary", trackerGapBoundary);
    run("reset_publication", resetPublication);
    run("clock_callback_cost", clockCallbackCost);
    fclose(LogFile); LogFile = nullptr;
    return failures ? 1 : 0;
}
