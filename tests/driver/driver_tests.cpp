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
#include <cstring>

struct DriverStateTestAccess
{
    static std::unique_lock<std::mutex> holdState(ServerTrackedDeviceProvider& p) { return std::unique_lock(p.stateMutex); }
    static bool callbackEntered(ServerTrackedDeviceProvider& p)
    {
        std::lock_guard lock(p.callbackMutex); return p.activePoseUpdates != 0;
    }
    static double trackerTime(const ServerTrackedDeviceProvider& p)
    {
        LARGE_INTEGER frequency; QueryPerformanceFrequency(&frequency);
        return double(p.trackerSample.time.QuadPart) / frequency.QuadPart;
    }
    static double hmdTime(const ServerTrackedDeviceProvider& p) { return p.lastHmdTime; }
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
            p.clock.tracker.add(t + .025, rotationSpeed, translationSpeed);
        }
    }
    static double lastClockSolve(const ServerTrackedDeviceProvider& p) { return p.clock.lastSolve; }
    static bool outstanding(ServerTrackedDeviceProvider& p) { std::lock_guard lock(p.stateMutex); return p.solveOutstanding; }
    static void primeRefinement(ServerTrackedDeviceProvider& p)
    {
        for (int i = 0; i != 3; ++i)
            p.refine.add({1, 0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, 0, 1, 20);
    }
    static unsigned refinementSolves(const ServerTrackedDeviceProvider& p) { return p.refine.solves; }
    static void primeTranslationRefinement(ServerTrackedDeviceProvider& p)
    {
        const auto fill = [&] {
            for (int i = 0; i < 120; ++i)
            {
                const double angle = 2 * POSE_PI * i / 120;
                const auto q = quaternionFromRotationVector({.7071067811865475244 * angle, .7071067811865475244 * angle, 0});
                p.refine.add(q, quaternionRotateVector(q, vr::HmdVector3d_t{0, 0, .012}), q, {0, 0, 0}, 0, 1, .5);
            }
        };
        fill(); align::MountRefiner::Delta primer;
        if (!p.refine.evaluate(primer)) throw std::runtime_error("reference refinement block not ready");
        fill();
    }
    static uint64_t epoch(const ServerTrackedDeviceProvider& p) { return p.alignmentEpoch; }
    static void setKnownLag(ServerTrackedDeviceProvider& p, double lag)
    {
        p.clock.rot.tau = p.clock.pos.tau = lag;
        p.clock.rot.primed = p.clock.pos.primed = true;
        p.nextSolveTime = 1e100;
    }
    static void wrapSolver(ServerTrackedDeviceProvider& p, std::function<void()> before, std::function<void()> after)
    {
        p.alignmentWorker->Stop();
        p.alignmentWorker = std::make_unique<ServerTrackedDeviceProvider::AlignmentWorker>(
            [before, after](auto& work) { before(); ServerTrackedDeviceProvider::SolveAlignment(work); after(); });
        p.alignmentWorker->Start();
    }
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
    for (int i = 0; i != 7; ++i)
    {
        tracker = pose(1, 0, 0); provider.HandleDevicePoseUpdated(1, tracker);
        provider.HandleDevicePoseUpdated(0, hmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    protocol::DriverStatus status{}; provider.GetStatus(status);
    require(status.driftValid, "reference scale check must execute the live inverse-drift path");
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
    double targetTime = 0;
    bool invalidTiming = false, invalidPosition = false;
    TimedTrackerPose ReadTrackerPose(uint32_t, uint32_t, float predictionFrames) override
    {
        LARGE_INTEGER now, frequency; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&frequency);
        targetTime = double(now.QuadPart) / frequency.QuadPart + predictionFrames / 90.0;
        if (duringRead) duringRead();
        vr::TrackedDevicePose_t p{};
        p.bDeviceIsConnected = p.bPoseIsValid = true; p.eTrackingResult = vr::TrackingResult_Running_OK;
        p.mDeviceToAbsoluteTracking.m[0][0] = p.mDeviceToAbsoluteTracking.m[1][1] = p.mDeviceToAbsoluteTracking.m[2][2] = 1;
        p.mDeviceToAbsoluteTracking.m[0][3] = trackerX;
        if (invalidPosition) p.mDeviceToAbsoluteTracking.m[0][3] = std::numeric_limits<float>::quiet_NaN();
        return {p, invalidTiming ? std::numeric_limits<double>::quiet_NaN() : targetTime};
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
    require(DriverStateTestAccess::lastClockSolve(provider) == 0, "callback must not synchronously execute or publish the clock solve");
    std::printf("MEASURE populated clock solve callback %.3f ms (900 samples)\n", ms);
}

static void callbackArrivalPrecedesStateWait()
{
    for (uint32_t id : {0u, 1u})
    {
        ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
        auto lock = DriverStateTestAccess::holdState(provider);
        auto update = std::async(std::launch::async, [&] { auto p = pose(); provider.HandleDevicePoseUpdated(id, p); });
        while (!DriverStateTestAccess::callbackEntered(provider)) std::this_thread::yield();
        LARGE_INTEGER before, frequency; QueryPerformanceCounter(&before); QueryPerformanceFrequency(&frequency);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        lock.unlock(); update.get();
        const double sample = id == 0 ? DriverStateTestAccess::hmdTime(provider) : DriverStateTestAccess::trackerTime(provider);
        require(sample <= double(before.QuadPart) / frequency.QuadPart + .005,
            "pose arrival timestamp must not include the wait for stateMutex");
    }
}

static double nowSeconds()
{
    LARGE_INTEGER now, frequency; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&frequency);
    return double(now.QuadPart) / frequency.QuadPart;
}

static void predictionTimestampSurvivesQueryWait()
{
    TestPoseSource source; ServerTrackedDeviceProvider provider(&source);
    auto c = config(); c.followSlamHmd = false; c.predictionTime = 3;
    provider.SetHmdTracker(c);
    source.duringRead = [] { std::this_thread::sleep_for(std::chrono::milliseconds(40)); };
    auto p = pose(); provider.HandleDevicePoseUpdated(0, p);
    require(p.poseTimeOffset < 0, "already predicted pose becomes historical while query is stalled");
    require(std::abs(nowSeconds() + p.poseTimeOffset - source.targetTime) < .003,
        "output timestamp must identify the queried target, not callback entry or processing time");
    for (int bad = 0; bad != 3; ++bad)
    {
        source.invalidTiming = bad == 0; source.invalidPosition = bad == 1;
        source.duringRead = bad == 2 ? std::function<void()>([] { std::this_thread::sleep_for(std::chrono::milliseconds(150)); }) : std::function<void()>{};
        c.slamFallback = false; provider.SetHmdTracker(c);
        p = pose(); provider.HandleDevicePoseUpdated(0, p);
        require(!p.poseIsValid && std::isfinite(p.vecPosition[0]), "malformed or expired predicted samples cannot replace HMD");
    }
}

static void nativeFollowAndRecovery()
{
    ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
    provider.SetDeviceTransform({0, true, {100, 100, 100}, {1, 0, 0, 0}, 2});
    for (int i = 0; i != 7; ++i)
    {
        auto tracker = pose(2); provider.HandleDevicePoseUpdated(1, tracker);
        auto hmd = pose(1); const auto original = hmd;
        provider.HandleDevicePoseUpdated(0, hmd);
        require(hmd.poseTimeOffset <= original.poseTimeOffset && hmd.poseTimeOffset > -.01,
            "Follow preserves the represented timestamp while accounting for callback processing");
        hmd.poseTimeOffset = original.poseTimeOffset;
        require(std::memcmp(&hmd, &original, sizeof hmd) == 0, "Follow must preserve native HMD geometry and derivatives");
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    protocol::DriverStatus status{}; provider.GetStatus(status);
    require(status.driftValid, "bracketed historical pairs must eventually initialize alignment");
    provider.SetDeviceTransform({2, true, {0, 0, 0}, {1, 0, 0, 0}, 1});
    auto body = pose(3); provider.HandleDevicePoseUpdated(2, body);
    checkNear(worldPosition(body).v[0], 2, "historical pair must align body to native head");
    auto invalid = pose(); invalid.poseIsValid = false; provider.HandleDevicePoseUpdated(1, invalid);
    for (int i = 0; i != 2; ++i)
    {
        auto tracker = pose(4); provider.HandleDevicePoseUpdated(1, tracker);
        auto hmd = pose(1); provider.HandleDevicePoseUpdated(0, hmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    body = pose(3); provider.HandleDevicePoseUpdated(2, body);
    checkNear(worldPosition(body).v[0], 2, "a short post-loss burst must retain the supported mapping");
}

static void boundedWorkerLifetime()
{
    std::promise<void> entered, release;
    auto released = release.get_future().share();
    BoundedWorker<int> worker([&](int& value) { entered.set_value(); released.wait(); ++value; });
    require(worker.Start(), "explicit worker startup");
    require(worker.Submit(std::make_unique<int>(7)), "first work admitted");
    entered.get_future().wait();
    unsigned accepted = 0;
    for (int i = 0; i < 10000; ++i) accepted += worker.Submit(std::make_unique<int>(i));
    auto stop = std::async(std::launch::async, [&] { worker.Stop(); });
    const bool waited = stop.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout;
    release.set_value(); stop.get();
    require(accepted == 0, "a blocked solver must not accumulate pending work");
    require(waited, "stop must join the running solver");
    require(!worker.Take() && !worker.Submit(std::make_unique<int>(1)), "stopped worker cannot publish or accept work");
}

static void workerFailureRecovery()
{
    BoundedWorker<int> worker([](int& value) { if (value == 0) throw std::runtime_error("solver failure"); ++value; });
    require(worker.Start(), "explicit worker startup");
    for (int input : {0, 8})
    {
        require(worker.Submit(std::make_unique<int>(input)), "completed job releases capacity");
        BoundedWorker<int>::Completion result;
        while (!(result = worker.Take())) std::this_thread::yield();
        require(bool(result.error) == (input == 0), "failure must reach the owner without killing worker");
        if (input) require(*result.work == 9, "worker recovers after failed solve");
    }
}

static void solveStallAndStalePublication()
{
    for (int change = 0; change != 5; ++change)
    {
        std::promise<void> entered, release, solved;
        auto released = release.get_future().share();
        std::atomic<bool> first{true};
        ServerTrackedDeviceProvider provider; auto c = config(); provider.SetHmdTracker(c);
        DriverStateTestAccess::primeClock(provider);
        DriverStateTestAccess::wrapSolver(provider, [&] { if (first.exchange(false)) { entered.set_value(); released.wait(); } }, [&] { try { solved.set_value(); } catch (...) { } });
        auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
        auto hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
        entered.get_future().wait();
        auto callback = std::async(std::launch::async, [&] {
            auto body = pose(); provider.HandleDevicePoseUpdated(2, body);
            protocol::DriverStatus status{}; provider.GetStatus(status);
            if (change == 1) { c.calibrationTranslation.v[0] = 1; provider.SetHmdTracker(c); }
            if (change == 2) { auto bad = pose(); bad.poseIsValid = false; provider.HandleDevicePoseUpdated(1, bad); }
            if (change == 3) { auto gap = pose(); gap.poseTimeOffset = .2; provider.HandleDevicePoseUpdated(1, gap); }
            if (change == 4) { auto jump = pose(); jump.vecWorldFromDriverTranslation[0] = .1; provider.HandleDevicePoseUpdated(0, jump); }
        });
        const bool responsive = callback.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready;
        release.set_value(); callback.get(); solved.get_future().wait();
        // Consume the completed result through the real HMD callback. Keep the
        // new frame/config intact and supply valid post-discontinuity tracking.
        for (int i = 0; i < 100 && DriverStateTestAccess::outstanding(provider); ++i)
        {
            tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
            hmd = pose(); if (change == 4) hmd.vecWorldFromDriverTranslation[0] = .1;
            provider.HandleDevicePoseUpdated(0, hmd);
            if (DriverStateTestAccess::lastClockSolve(provider) > 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(responsive, "clock/refinement worker stall must not block pose/config/status transactions");
        if (change == 0) require(DriverStateTestAccess::lastClockSolve(provider) > 0, "unchanged generation must publish a real clock solve");
        else
        {
            protocol::DriverStatus status{}; provider.GetStatus(status);
            require(status.latencyMs == 0 && status.latencyPosMs == 0, "old-generation latency votes must not survive invalidation");
        }
    }
}

static void invalidConfigurationIsTransactional()
{
    ServerTrackedDeviceProvider provider; disable(provider);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int bad = 0; bad != 7; ++bad)
    {
        provider.SetDeviceTransform({2, true, {1, 0, 0}, {1, 0, 0, 0}, 2});
        auto update = protocol::SetDeviceTransform{2, true, {8, 0, 0}, {1, 0, 0, 0}, 3};
        if (bad == 0) update.translation.v[0] = nan;
        if (bad == 1) update.rotation = {0, 0, 0, 0};
        if (bad == 2) update.scale = nan;
        if (bad == 3) update.scale = 0;
        if (bad == 4) update.scale = 1e308;
        if (bad == 5) update.scale = 1e-308;
        if (bad == 6) update.translation.v[0] = 1e308;
        provider.SetDeviceTransform(update);
        auto p = pose(1); provider.HandleDevicePoseUpdated(2, p);
        checkNear(worldPosition(p).v[0], 3, "invalid transform must preserve the complete previous transform");
    }
    auto c = config(); c.trackerID = vr::k_unTrackedDeviceIndexInvalid; provider.SetHmdTracker(c);
    c.hmdScale = nan; c.offsetTranslation.v[0] = nan; provider.SetHmdTracker(c);
    protocol::DriverStatus status{}; provider.GetStatus(status);
    require(std::isfinite(status.offsetTranslation.v[0]) && status.hmdScale == 1, "invalid HMD configuration cannot poison published offsets");

    TestPoseSource source; ServerTrackedDeviceProvider filtered(&source);
    c = config(); c.followSlamHmd = false; filtered.SetHmdTracker(c);
    protocol::SetOneEuro filter{}; filter.headEnabled = true; filter.head = {5, .8, 1};
    filtered.SetOneEuro(filter); auto p = pose(); filtered.HandleDevicePoseUpdated(0, p);
    filter.head.beta = nan; filtered.SetOneEuro(filter);
    source.trackerX = 1; p = pose(); filtered.HandleDevicePoseUpdated(0, p);
    require(p.poseIsValid && std::isfinite(p.vecPosition[0]), "invalid filter parameters cannot poison replacement HMD output");
}

static void trackerReferenceChangeBreaksHistory()
{
    ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
    for (int i = 0; i != 9; ++i)
    {
        auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    require(DriverStateTestAccess::trackerHistory(provider) > 0, "tracker frame test needs continuous pre-change history");
    const auto before = DriverStateTestAccess::epoch(provider);
    auto tracker = pose(); tracker.vecWorldFromDriverTranslation[0] = .001;
    provider.HandleDevicePoseUpdated(1, tracker);
    require(DriverStateTestAccess::epoch(provider) != before && DriverStateTestAccess::trackerHistory(provider) == 0,
        "explicit Lighthouse reference changes must split history even below kinematic jump thresholds");

    for (int i = 0; i != 9; ++i)
    {
        auto hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    require(DriverStateTestAccess::hmdHistory(provider) > 0, "HMD frame test needs continuous pre-change history");
    const auto beforeTilt = DriverStateTestAccess::epoch(provider);
    auto tilted = pose(); tilted.qWorldFromDriverRotation = quaternionFromRotationVector({.02, 0, 0});
    provider.HandleDevicePoseUpdated(0, tilted);
    require(DriverStateTestAccess::epoch(provider) != beforeTilt && DriverStateTestAccess::hmdHistory(provider) == 0,
        "a frame tilt outside the yaw-rebase model must still invalidate measurement history");
}

static void refinementContinuityUsesCurrentHmd()
{
    ServerTrackedDeviceProvider provider;
    auto c = config(); c.trackerID = vr::k_unTrackedDeviceIndexInvalid; provider.SetHmdTracker(c);
    c.trackerID = 1; provider.SetHmdTracker(c);
    DriverStateTestAccess::setKnownLag(provider, 0);
    DriverStateTestAccess::primeTranslationRefinement(provider);
    provider.SetDeviceTransform({2, true, {0, 0, 0}, {1, 0, 0, 0}, 1});
    std::promise<void> solved;
    DriverStateTestAccess::wrapSolver(provider, []{}, [&] { try { solved.set_value(); } catch (...) {} });
    auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
    auto hmd = pose(); hmd.qRotation = quaternionFromYaw(POSE_PI / 2); provider.HandleDevicePoseUpdated(0, hmd);
    solved.get_future().wait();
    for (int i = 0; i < 100 && DriverStateTestAccess::outstanding(provider); ++i)
    {
        hmd = pose(); hmd.qRotation = quaternionFromYaw(POSE_PI / 2); provider.HandleDevicePoseUpdated(0, hmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    protocol::DriverStatus status{}; provider.GetStatus(status);
    require(status.refinementSolves > 0 && status.offsetTranslation.v[2] < -.005,
        "continuity fixture must publish a real accepted mount translation solve");
    auto body = pose(); provider.HandleDevicePoseUpdated(2, body);
    const auto world = worldPosition(body);
    require(world.v[0] > .005 && std::fabs(world.v[2]) < .0001,
        "mount correction must use current HMD axes, not the stale tracker's orientation");
}

static void passthroughTargetSurvivesStateWait()
{
    for (uint32_t id : {0u, 2u})
    {
        ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
        auto lock = DriverStateTestAccess::holdState(provider);
        vr::DriverPose_t output = pose(1, 2, 3); output.poseTimeOffset = .02;
        auto callback = std::async(std::launch::async, [&] { provider.HandleDevicePoseUpdated(id, output); });
        while (!DriverStateTestAccess::callbackEntered(provider)) std::this_thread::yield();
        const double arrivedBy = nowSeconds();
        std::this_thread::sleep_for(std::chrono::milliseconds(35));
        lock.unlock(); callback.get();
        require(output.poseTimeOffset < 0, "passthrough pose must not be relabeled as newly predicted after lock wait");
        require(nowSeconds() + output.poseTimeOffset <= arrivedBy + .025, "passthrough absolute target must precede state-lock delay");
        checkNear(output.vecPosition[0], 1, "passthrough timing compensation cannot alter geometry");
    }
}

static void publicationChecksActualAge()
{
    ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
    DriverStateTestAccess::primeClock(provider);
    std::promise<void> solved;
    DriverStateTestAccess::wrapSolver(provider, []{}, [&] { try { solved.set_value(); } catch (...) { } });
    auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
    auto hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
    solved.get_future().wait();
    auto lock = DriverStateTestAccess::holdState(provider);
    auto callback = std::async(std::launch::async, [&] { auto p = pose(); provider.HandleDevicePoseUpdated(0, p); });
    while (!DriverStateTestAccess::callbackEntered(provider)) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(280));
    lock.unlock(); callback.get();
    protocol::DriverStatus status{}; provider.GetStatus(status);
    require(status.latencyMs == 0 && status.latencyPosMs == 0, "expired solve must not publish using a pre-wait callback timestamp");
}

static void providerShutdownJoinsSolve()
{
    ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
    std::promise<void> entered, release; auto released = release.get_future().share();
    DriverStateTestAccess::wrapSolver(provider, [&] { entered.set_value(); released.wait(); }, []{});
    auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
    auto hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
    entered.get_future().wait();
    auto shutdown = std::async(std::launch::async, [&] { provider.ShutdownPoseUpdates(); });
    const bool waited = shutdown.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout;
    const bool rejected = !provider.HandleDevicePoseUpdated(0, hmd);
    release.set_value(); shutdown.get();
    require(waited && rejected, "provider shutdown must close admission then join the blocked solve");
}

static void mountSolvePublication()
{
    for (bool reset : {false, true})
    {
        ServerTrackedDeviceProvider provider; auto c = config(); provider.SetHmdTracker(c);
        DriverStateTestAccess::primeRefinement(provider);
        std::promise<void> entered, release, solved; auto released = release.get_future().share();
        std::atomic<bool> first{true};
        DriverStateTestAccess::wrapSolver(provider, [&] { if (first.exchange(false)) { entered.set_value(); released.wait(); } }, [&] { try { solved.set_value(); } catch (...) {} });
        auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
        auto hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
        entered.get_future().wait();
        if (reset) { c.offsetTranslation.v[0] = .01; provider.SetHmdTracker(c); }
        release.set_value(); solved.get_future().wait();
        for (int i = 0; i < 100 && DriverStateTestAccess::outstanding(provider); ++i)
        {
            hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(DriverStateTestAccess::refinementSolves(provider) == (reset ? 0u : 1u),
            "real mount block solves publish only to their originating configuration");
    }
}

static void changedClockInvalidatesMountBlock()
{
    ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
    DriverStateTestAccess::primeClock(provider); DriverStateTestAccess::primeRefinement(provider);
    std::promise<void> solved;
    DriverStateTestAccess::wrapSolver(provider, []{}, [&] { try { solved.set_value(); } catch (...) {} });
    auto tracker = pose(); provider.HandleDevicePoseUpdated(1, tracker);
    auto hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
    solved.get_future().wait();
    for (int i = 0; i < 100 && DriverStateTestAccess::outstanding(provider); ++i)
    {
        hmd = pose(); provider.HandleDevicePoseUpdated(0, hmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    protocol::DriverStatus status{}; provider.GetStatus(status);
    require(status.latencyMs > 20, "fixture must learn the injected positive clock offset");
    require(DriverStateTestAccess::refinementSolves(provider) == 0,
        "mount block collected with old latency must not publish after a large clock correction");
}

static void asynchronousHistoricalBodyAlignment()
{
    ServerTrackedDeviceProvider provider; provider.SetHmdTracker(config());
    DriverStateTestAccess::setKnownLag(provider, .025);
    provider.SetDeviceTransform({2, true, {0, 0, 0}, {1, 0, 0, 0}, 1});
    const double epoch = nowSeconds();
    double trackerTime = 0, hmdTime = .003, maximumError = 0;
    int measured = 0;
    const auto motion = [](double t) { return .08 * std::sin(8 * t) + .03 * std::sin(17 * t); };
    // Synthetic asynchronous producers provide independent position histories.
    // Deliberately uninformative reported velocity must not replace observed
    // history with a latest-sample extrapolation during accelerations/reversals.
    while (hmdTime < 1.0)
    {
        if (trackerTime <= hmdTime)
        {
            auto p = pose(1 + motion(trackerTime));
            p.poseTimeOffset = epoch + trackerTime + .025 - nowSeconds();
            provider.HandleDevicePoseUpdated(1, p); trackerTime += .011;
        }
        else
        {
            auto p = pose(motion(hmdTime));
            p.poseTimeOffset = epoch + hmdTime - nowSeconds();
            provider.HandleDevicePoseUpdated(0, p);
            protocol::DriverStatus status{}; provider.GetStatus(status);
            if (hmdTime > .1 && status.driftValid)
            {
                auto body = pose(2); provider.HandleDevicePoseUpdated(2, body);
                maximumError = std::fmax(maximumError, std::abs(worldPosition(body).v[0] - 1)); ++measured;
            }
            hmdTime += .007;
        }
    }
    require(measured > 100 && maximumError < .0008,
        "historical alignment must keep constant body/head registration through asynchronous acceleration and reversals");
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
    run("callback_arrival_precedes_state_wait", callbackArrivalPrecedesStateWait);
    run("prediction_timestamp_survives_query_wait", predictionTimestampSurvivesQueryWait);
    run("native_follow_and_recovery", nativeFollowAndRecovery);
    run("bounded_worker_lifetime", boundedWorkerLifetime);
    run("worker_failure_recovery", workerFailureRecovery);
    run("solve_stall_and_stale_publication", solveStallAndStalePublication);
    run("invalid_configuration_transaction", invalidConfigurationIsTransactional);
    run("passthrough_target_survives_state_wait", passthroughTargetSurvivesStateWait);
    run("publication_checks_actual_age", publicationChecksActualAge);
    run("provider_shutdown_joins_solve", providerShutdownJoinsSolve);
    run("mount_solve_publication", mountSolvePublication);
    run("changed_clock_invalidates_mount_block", changedClockInvalidatesMountBlock);
    run("asynchronous_historical_body_alignment", asynchronousHistoricalBodyAlignment);
    run("tracker_reference_change_breaks_history", trackerReferenceChangeBreaksHistory);
    run("refinement_continuity_uses_current_hmd", refinementContinuityUsesCurrentHmd);
    fclose(LogFile); LogFile = nullptr;
    return failures ? 1 : 0;
}
