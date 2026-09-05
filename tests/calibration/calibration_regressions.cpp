// Tests compile the shipped implementation. Only OS/VR/IPC boundaries are replaced;
// no registry, SteamVR runtime, or live named pipe is touched.
#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"
#include <Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "../../src/overlay/Configuration.cpp"

namespace test {
struct System {
    std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> poses{};
    std::array<vr::ETrackedDeviceClass, vr::k_unMaxTrackedDeviceCount> classes{};
    std::array<std::string, vr::k_unMaxTrackedDeviceCount> serials{};
    vr::ETrackedDeviceClass GetTrackedDeviceClass(uint32_t id) { return classes.at(id); }
    uint32_t GetStringTrackedDeviceProperty(uint32_t id, vr::ETrackedDeviceProperty prop,
        char* out, uint32_t size, vr::ETrackedPropertyError* error = nullptr) {
        std::string value = prop == vr::Prop_SerialNumber_String ? serials.at(id) :
            prop == vr::Prop_TrackingSystemName_String ? (id == 0 ? "slam" : "lighthouse") : "vive tracker 3.0 mv";
        if (error) *error = vr::TrackedProp_Success;
        if (size) { std::strncpy(out, value.c_str(), size); out[size - 1] = 0; }
        return static_cast<uint32_t>(value.size() + 1);
    }
    void GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin, float, vr::TrackedDevicePose_t* out, uint32_t count) {
        std::copy_n(poses.begin(), count, out);
    }
} system;
struct Chaperone {
    void RevertWorkingCopy() {}
    bool GetLiveCollisionBoundsInfo(vr::HmdQuad_t*, uint32_t* n) { *n = 0; return true; }
    bool GetWorkingStandingZeroPoseToRawTrackingPose(vr::HmdMatrix34_t*) { return true; }
    bool GetWorkingPlayAreaSize(float*, float*) { return true; }
    void SetWorkingCollisionBoundsInfo(vr::HmdQuad_t*, uint32_t) {}
    void SetWorkingStandingZeroPoseToRawTrackingPose(vr::HmdMatrix34_t*) {}
    void SetWorkingPlayAreaSize(float, float) {}
    bool CommitWorkingCopy(vr::EChaperoneConfigFile) { return true; }
} chaperone;
struct Notifications { void RemoveNotification(vr::VRNotificationId) {} } notifications;
std::string saved;
int saves = 0;
bool saveFailure = false;
std::vector<protocol::Request> requests;
void Require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
}
namespace vr {
test::System* TestSystem() { return &test::system; }
test::Chaperone* TestChaperone() { return &test::chaperone; }
test::Notifications* TestNotifications() { return &test::notifications; }
}
void TestLoadProfile(CalibrationContext& ctx) {
    if (test::saved.empty()) { ctx.Clear(); return; }
    std::istringstream input(test::saved);
    ParseProfile(ctx, input);
}
bool TestSaveProfile(CalibrationContext& ctx) {
    if (test::saveFailure) return false;
    std::ostringstream output;
    WriteProfile(ctx, output);
    test::saved = output.str();
    ++test::saves;
    return true;
}
IPCClient::~IPCClient() = default;
void IPCClient::Connect() {}
protocol::Response IPCClient::SendBlocking(const protocol::Request& request) {
    test::requests.push_back(request);
    return protocol::Response(protocol::ResponseSuccess);
}
namespace sound { void Stop() {} void Play(const char*) {} }

#define VRSystem TestSystem
#define VRChaperoneSetup TestChaperone
#define VRNotifications TestNotifications
#define LoadProfile TestLoadProfile
#define SaveProfile TestSaveProfile
#include "../../src/overlay/Calibration.cpp"
#undef VRSystem
#undef VRChaperoneSetup
#undef VRNotifications
#undef LoadProfile
#undef SaveProfile

namespace test {
vr::TrackedDevicePose_t DevicePose(const Eigen::Matrix3d& rotation, Eigen::Vector3d p) {
    vr::TrackedDevicePose_t pose{};
    pose.bPoseIsValid = pose.bDeviceIsConnected = true;
    pose.eTrackingResult = vr::TrackingResult_Running_OK;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) pose.mDeviceToAbsoluteTracking.m[row][col] = static_cast<float>(rotation(row, col));
        pose.mDeviceToAbsoluteTracking.m[row][3] = static_cast<float>(p[row]);
    }
    return pose;
}
void SetMotion(int sample, bool twoAxes) {
    const double angle = 0.52 * std::sin(sample * 0.12);
    auto axis = twoAxes && (sample / 60) % 2 ? Eigen::Vector3d::UnitX().eval() : Eigen::Vector3d::UnitY().eval();
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    Eigen::Vector3d tracker(0.2, 1.7, 0.1);
    system.poses[1] = DevicePose(rotation, tracker);
    system.poses[0] = DevicePose(rotation, tracker + rotation * Eigen::Vector3d(0.02, -0.08, 0.06));
}
void Reset() {
    CalCtx = CalibrationContext{};
    CalCtx.calibratedRotation.setZero();
    CalCtx.calibratedTranslation.setZero();
    CalCtx.targetID = 1;
    CalCtx.validProfile = CalCtx.validRelativeOffset = true;
    CalCtx.trackerSerial = "A";
    CalCtx.hmdSerial = "HMD";
    CalCtx.targetTrackingSystem = "lighthouse";
    CalCtx.relativeTranslation = { 0.01, 0.02, 0.03 };
    CalCtx.chaperone.autoApply = false;
    CalCtx.chaperone.valid = false;
    system = System{};
    system.classes[0] = vr::TrackedDeviceClass_HMD;
    system.classes[1] = vr::TrackedDeviceClass_GenericTracker;
    system.serials[0] = "HMD";
    system.serials[1] = "B";
    SetMotion(0, true);
    saves = 0;
    saveFailure = false;
    TestSaveProfile(CalCtx);
    saves = 0;
    requests.clear();
    StartCalibration();
    CalibrationTick(0.1); // actual Begin -> Sampling, selects B
}
void Tick(double time, int i, bool twoAxes = true) { SetMotion(i, twoAxes); CalibrationTick(time); }

picojson::object GoodProfile() {
    CalibrationContext ctx{};
    ctx.calibratedRotation.setZero(); ctx.calibratedTranslation.setZero();
    ctx.targetTrackingSystem = "lighthouse"; ctx.trackerSerial = "A"; ctx.hmdSerial = "HMD";
    ctx.validProfile = ctx.validRelativeOffset = true;
    std::ostringstream output; WriteProfile(ctx, output);
    picojson::value value; picojson::parse(value, output.str());
    return value.get<picojson::array>()[0].get<picojson::object>();
}
void Parse(CalibrationContext& ctx, const picojson::object& profile) {
    picojson::array profiles{picojson::value(profile)};
    std::istringstream input(picojson::value(profiles).serialize());
    ParseProfile(ctx, input);
}
void ExpectRejected(picojson::object profile) {
    CalibrationContext ctx{};
    ctx.trackerSerial = "prior"; ctx.calibratedTranslation = Eigen::Vector3d(3, 4, 5);
    ctx.validProfile = true;
    bool rejected = false;
    try { Parse(ctx, profile); } catch (const std::runtime_error&) { rejected = true; }
    Require(rejected, "malformed profile was accepted");
    Require(ctx.trackerSerial == "prior" && ctx.calibratedTranslation.x() == 3 && ctx.validProfile,
        "failed parse partially mutated the prior profile");
}

void TransientLoss() {
    Reset();
    system.poses[1].bPoseIsValid = false;
    CalibrationTick(0.2);
    Require(CalCtx.state == CalibrationState::Sampling, "one invalid sample aborted calibration");
    Tick(0.3, 1);
    Require(CalCtx.state == CalibrationState::Sampling, "tracking recovery did not resume sampling");
    CancelCalibration();
    Require(CalCtx.trackerSerial == "A" && CalCtx.relativeTranslation.v[0] == 0.01,
        "cancel failed to restore tracker A and its mount atomically");
}
void LossTimeoutRollback() {
    Reset();
    // The rollback must use the start-of-run snapshot, not a subsequently replaced registry profile.
    saved.clear();
    for (int i = 1; i <= 120; ++i) {
        system.poses[1].bPoseIsValid = false;
        CalibrationTick(0.1 + i * 0.06);
    }
    Require(CalCtx.state == CalibrationState::None, "tracking-loss timeout did not terminate");
    Require(CalCtx.trackerSerial == "A" && CalCtx.validRelativeOffset && CalCtx.relativeTranslation.v[0] == 0.01,
        "invalid sample after tracker swap left old offset attached to B");
    Require(saves == 0, "failed calibration overwrote persisted calibration");
}
void TwoAxesComplete() {
    Reset();
    for (int i = 1; i <= 420; ++i) Tick(0.1 + i * 0.06, i);
    Require(CalCtx.lastCalibrationOk && CalCtx.state == CalibrationState::None,
        "observable +/-30 degree two-axis calibration was rejected");
    Require(CalCtx.trackerSerial == "B" && saves == 1, "successful calibration was not committed once");
    Require(CalCtx.calibrationCheck.passed && !CalCtx.calibrationCheck.scaleMeasured && CalCtx.calibrationCheck.scale == 1.0,
        "head rotations falsely reported independently measured scale");
    Require(CalCtx.calibrationCheck.positionRmsMm < 0.01 && CalCtx.calibrationCheck.angularRmsDegrees < 0.01,
        "exact held-out rigid motion did not report its measured consistency");
    CalibrationContext loaded = CalCtx; TestLoadProfile(loaded);
    Require(!loaded.calibrationCheck.passed, "profile reload retained a check not present in the saved profile");
}
void FitRemainsPrivate() {
    Reset();
    for (int i = 1; i <= 205; ++i) Tick(0.1 + i * 0.06, i);
    Require(CalCtx.state == CalibrationState::Sampling && !CalCtx.lastCalibrationOk && saves == 0,
        "training fit was published before independent validation");
    Require(CalCtx.trackerSerial == "A" && CalCtx.relativeTranslation.v[0] == 0.01,
        "unvalidated fit replaced the active profile");
    TestSaveProfile(CalCtx);
    CalibrationContext loaded{}; TestLoadProfile(loaded);
    Require(loaded.trackerSerial == "A" && loaded.relativeTranslation.v[0] == 0.01,
        "shutdown during validation persisted the unvalidated candidate");
    CancelCalibration();
}
void HeldOutBadPosition() {
    Reset();
    for (int i = 1; i <= 205; ++i) Tick(0.1 + i * 0.06, i);
    for (int i = 206; i <= 420; ++i) {
        SetMotion(i, true);
        system.poses[0].mDeviceToAbsoluteTracking.m[0][3] += 0.15f;
        CalibrationTick(0.1 + i * 0.06);
    }
    Require(!CalCtx.lastCalibrationOk && CalCtx.trackerSerial == "A" && saves == 0,
        "good training fit accepted an inconsistent held-out position segment");
}
void HeldOutBadAngle() {
    Reset();
    for (int i = 1; i <= 205; ++i) Tick(0.1 + i * 0.06, i);
    for (int i = 206; i <= 420; ++i) {
        SetMotion(i, true);
        const auto& original = system.poses[0].mDeviceToAbsoluteTracking;
        Pose pose(original);
        system.poses[0] = DevicePose(pose.rot * Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix(), pose.trans);
        CalibrationTick(0.1 + i * 0.06);
    }
    Require(!CalCtx.lastCalibrationOk && CalCtx.trackerSerial == "A" && saves == 0,
        "good training fit accepted inconsistent held-out orientation");
}
int FitCandidate() {
    int i = 0;
    while (!CalCtx.validating && i < 205) { ++i; Tick(0.1 + i * 0.06, i); }
    Require(CalCtx.validating && saves == 0, "fit did not enter private validation");
    return i;
}
void HeldOutCoverageTimeout() {
    Reset();
    int i = FitCandidate();
    for (++i; i <= 850 && CalCtx.state == CalibrationState::Sampling; ++i) Tick(0.1 + i * 0.06, i, false);
    Require(CalCtx.state == CalibrationState::None && !CalCtx.lastCalibrationOk && saves == 0 && CalCtx.trackerSerial == "A",
        "training motion substituted for missing independent validation axes");
}
void HeldOutCoverageRecovery() {
    Reset();
    int i = FitCandidate();
    const int fitEnd = i;
    for (++i; i <= fitEnd + 215; ++i) Tick(0.1 + i * 0.06, i, false);
    Require(CalCtx.validating && saves == 0, "insufficient validation did not provide a new complete sequence");
    for (; i <= fitEnd + 440 && CalCtx.state == CalibrationState::Sampling; ++i) Tick(0.1 + i * 0.06, i);
    Require(CalCtx.lastCalibrationOk && CalCtx.calibrationCheck.passed && saves == 1,
        "eventual complete independent validation failed");
}
void HeldOutErrorIsNotRetried() {
    Reset();
    int i = FitCandidate();
    const int end = i + 210;
    for (++i; i <= end && CalCtx.state == CalibrationState::Sampling; ++i) {
        SetMotion(i, false); // inadequate axes, but the observed positions already disagree
        system.poses[0].mDeviceToAbsoluteTracking.m[0][3] += 0.15f;
        CalibrationTick(0.1 + i * 0.06);
    }
    Require(CalCtx.state == CalibrationState::None && !CalCtx.lastCalibrationOk && saves == 0,
        "coverage retry discarded contrary held-out evidence and allowed another chance to pass");
}
void HeldOutTrackingLoss() {
    Reset();
    int i = FitCandidate();
    system.poses[1].bPoseIsValid = false;
    CalibrationTick(0.1 + ++i * 0.06);
    Require(CalCtx.validating && saves == 0, "brief invalid held-out sample aborted or committed the fit");
    Tick(0.1 + ++i * 0.06, i);
    Require(CalCtx.validating, "validation did not resume after brief loss");
    for (int end = i + 45; i < end; ++i) {
        system.poses[1].mDeviceToAbsoluteTracking.m[0][3] = std::numeric_limits<float>::quiet_NaN();
        CalibrationTick(0.1 + (i + 1) * 0.06);
    }
    Require(CalCtx.state == CalibrationState::None && CalCtx.trackerSerial == "A" && saves == 0,
        "invalid held-out poses did not time out and restore the prior profile");
}
void HeldOutSegmentFailure() {
    Reset();
    int i = FitCandidate();
    const int end = i + 220;
    for (++i; i <= end && CalCtx.state == CalibrationState::Sampling; ++i) {
        SetMotion(i, true);
        // A 4 cm error for one 1.5 s segment passes the aggregate 3 cm RMS /
        // 5 cm p95 / 10 cm peak gates, but must fail that segment's RMS gate.
        const double elapsed = (0.1 + i * 0.06) - CalCtx.sequenceStart;
        if (elapsed >= 6.0 && elapsed < 7.5) system.poses[0].mDeviceToAbsoluteTracking.m[0][3] += 0.04f;
        CalibrationTick(0.1 + i * 0.06);
    }
    Require(!CalCtx.lastCalibrationOk && CalCtx.state == CalibrationState::None && CalCtx.trackerSerial == "A" && saves == 0,
        "a bad held-out direction was averaged away by good segments");
}
void HeldOutCancel() {
    Reset(); FitCandidate();
    CancelCalibration();
    Require(CalCtx.state == CalibrationState::None && !CalCtx.validating && !CalCtx.lastCalibrationOk
        && CalCtx.trackerSerial == "A" && CalCtx.relativeTranslation.v[0] == 0.01 && saves == 0,
        "cancelling independent validation failed to restore the prior profile");
}
void HeldOutMissingSegment() {
    Reset();
    int i = FitCandidate();
    for (++i; i <= 850 && CalCtx.state == CalibrationState::Sampling; ++i) {
        SetMotion(i, true);
        const double elapsed = (0.1 + i * 0.06) - CalCtx.sequenceStart;
        if (elapsed >= 3.0 && elapsed < 4.5) system.poses[1].bPoseIsValid = false;
        CalibrationTick(0.1 + i * 0.06);
    }
    Require(CalCtx.state == CalibrationState::None && CalCtx.trackerSerial == "A" && saves == 0,
        "partial validation sequences were accepted without coverage of every segment");
}
void HeldOutSparseContradiction() {
    for (int validCount = 1; validCount < 10; ++validCount) {
        Reset();
        int i = FitCandidate(), segmentSamples = 0;
        const int end = i + 210;
        for (++i; i <= end && CalCtx.state == CalibrationState::Sampling; ++i) {
            SetMotion(i, true);
            const double elapsed = (0.1 + i * 0.06) - CalCtx.sequenceStart;
            if (elapsed >= 6.0 && elapsed < 7.5) {
                if (segmentSamples++ < validCount)
                    system.poses[0].mDeviceToAbsoluteTracking.m[0][3] += 0.04f;
                else system.poses[1].bPoseIsValid = false;
            }
            CalibrationTick(0.1 + i * 0.06);
        }
        Require(CalCtx.state == CalibrationState::None && !CalCtx.lastCalibrationOk && saves == 0
            && CalCtx.trackerSerial == "A" && CalCtx.relativeTranslation.v[0] == 0.01,
            "a bad segment with 1-9 valid poses was discarded as insufficient coverage");
    }
}
void RetryWaitAndRecover() {
    Reset();
    for (int i = 1; i <= 215; ++i) Tick(0.1 + i * 0.06, i, false);
    Require(CalCtx.state == CalibrationState::Sampling && !CalCtx.lastCalibrationOk,
        "retry window aborted before new motion could be collected");
    for (int i = 216; i <= 700 && CalCtx.state == CalibrationState::Sampling; ++i) Tick(0.1 + i * 0.06, i, true);
    Require(CalCtx.lastCalibrationOk, "additional observable motion failed to complete calibration");
}
void SingleAxisTimeout() {
    Reset();
    for (int i = 1; i <= 2200 && CalCtx.state == CalibrationState::Sampling; ++i) Tick(0.1 + i * 0.06, i, false);
    Require(CalCtx.state == CalibrationState::None && !CalCtx.lastCalibrationOk, "single-axis sampling did not terminate");
    Require(CalCtx.trackerSerial == "A" && saves == 0, "unobservable run failed to restore prior profile");
}
void RetryCancel() {
    Reset();
    for (int i = 1; i <= 205; ++i) Tick(0.1 + i * 0.06, i, false);
    Require(CalCtx.state == CalibrationState::Sampling, "no meaningful retry window before cancellation");
    CancelCalibration();
    Require(CalCtx.state == CalibrationState::None && CalCtx.trackerSerial == "A", "retry cancel did not roll back");
}
void SaveFailureRollback() {
    Reset();
    saveFailure = true;
    for (int i = 1; i <= 420; ++i) Tick(0.1 + i * 0.06, i);
    Require(!CalCtx.lastCalibrationOk && CalCtx.trackerSerial == "A" && CalCtx.relativeTranslation.v[0] == 0.01,
        "failed persistence committed the replacement calibration");
    Require(saves == 0, "failed save replaced the persisted profile");
    saveFailure = false;
}
void PendingIdentityIsNotSaved() {
    Reset();
    TestSaveProfile(CalCtx); // e.g. application shutdown during sampling
    CalibrationContext loaded{}; TestLoadProfile(loaded);
    Require(loaded.trackerSerial == "A" && loaded.relativeTranslation.v[0] == 0.01,
        "saving during sampling paired the candidate tracker with the old mount");
    CancelCalibration();
}
void UnsavedSnapshotRestored() {
    Reset(); CancelCalibration();
    CalCtx.relativeTranslation.v[0] = 0.07; // newer than the persisted A profile
    StartCalibration(); CalibrationTick(0.2); CancelCalibration();
    Require(CalCtx.trackerSerial == "A" && CalCtx.relativeTranslation.v[0] == 0.07,
        "rollback discarded the newer in-memory mount");
}
void PoseIngressValidation() {
    Reset();
    CalCtx.devicePoses[0] = system.poses[0]; CalCtx.devicePoses[1] = system.poses[1];
    Require(CollectSample(CalCtx).valid, "valid connected poses rejected");
    CalCtx.devicePoses[1].eTrackingResult = vr::TrackingResult_Running_OutOfRange;
    Require(!CollectSample(CalCtx).valid, "out-of-range pose accepted");
    CalCtx.devicePoses[1] = system.poses[1]; CalCtx.devicePoses[1].bDeviceIsConnected = false;
    Require(!CollectSample(CalCtx).valid, "disconnected pose accepted");
    CalCtx.devicePoses[1] = system.poses[1]; CalCtx.devicePoses[1].mDeviceToAbsoluteTracking.m[0][3] = std::numeric_limits<float>::quiet_NaN();
    Require(!CollectSample(CalCtx).valid, "non-finite pose accepted");
    CalCtx.devicePoses[1] = system.poses[1]; CalCtx.devicePoses[1].mDeviceToAbsoluteTracking.m[0][0] = 3;
    Require(!CollectSample(CalCtx).valid, "non-rigid pose accepted");
    CalCtx.devicePoses[1] = system.poses[1]; CalCtx.devicePoses[1].mDeviceToAbsoluteTracking.m[0][3] = 1e20f;
    Require(!CollectSample(CalCtx).valid, "finite but impossible calibration position accepted");
    CalCtx.targetID = vr::k_unTrackedDeviceIndexInvalid;
    Require(!CollectSample(CalCtx).valid, "invalid device index accepted");
    CancelCalibration();
}
void ProfileGeometryBounds() {
    auto profile = GoodProfile();
    picojson::object chaperone;
    chaperone["auto_apply"] = picojson::value(true);
    chaperone["play_space_size"] = picojson::value(picojson::array(2, picojson::value(2.0)));
    picojson::array center(12, picojson::value(0.0)); center[0] = center[5] = center[10] = picojson::value(1.0);
    chaperone["standing_center"] = picojson::value(center);
    for (int count : {1, 11, 13, 23, 12 * 4097}) {
        chaperone["geometry"] = picojson::value(picojson::array(count, picojson::value(0.0)));
        profile["chaperone"] = picojson::value(chaperone);
        ExpectRejected(profile);
    }
    for (int count : {0, 12, 24}) {
        chaperone["geometry"] = picojson::value(picojson::array(count, picojson::value(0.0)));
        profile["chaperone"] = picojson::value(chaperone);
        CalibrationContext ctx{}; Parse(ctx, profile);
        Require(ctx.validProfile && ctx.chaperone.geometry.size() == count / 12, "complete quad geometry rejected");
    }
    for (const picojson::value& invalid : {picojson::value("bad"), picojson::value(1e300)}) {
        picojson::array geometry(12, picojson::value(0.0)); geometry[11] = invalid;
        chaperone["geometry"] = picojson::value(geometry);
        profile["chaperone"] = picojson::value(chaperone);
        ExpectRejected(profile);
    }
}
void ProfileTypeValidation() {
    auto p = GoodProfile(); p["roll"] = picojson::value("bad"); ExpectRejected(p);
    p = GoodProfile(); p["scale"] = picojson::value(0.0); ExpectRejected(p);
    p = GoodProfile(); p["rel_qw"] = picojson::value(0.0); ExpectRejected(p);
    p = GoodProfile(); p.erase("rel_qx"); ExpectRejected(p);
    p = GoodProfile(); p["calibration_speed"] = picojson::value(99.0); ExpectRejected(p);
    p = GoodProfile(); p["predictionTime"] = picojson::value(-1.0); ExpectRejected(p);
    p = GoodProfile(); p["targetModelScale"] = picojson::value(1e300); ExpectRejected(p);
}
void ParseFailureIsTransactional() {
    auto p = GoodProfile();
    picojson::object chaperone;
    chaperone["auto_apply"] = picojson::value(true);
    chaperone["play_space_size"] = picojson::value("invalid");
    p["chaperone"] = picojson::value(chaperone);
    ExpectRejected(p);
}
void ProfileRootsAndLegacyDefaults() {
    for (const char* json : {"{}", "null", "[]", "[null]", "[{}]", "[true]"}) {
        CalibrationContext ctx{}; ctx.trackerSerial = "prior";
        bool rejected = false;
        try { std::istringstream input(json); ParseProfile(ctx, input); }
        catch (const std::runtime_error&) { rejected = true; }
        Require(rejected && ctx.trackerSerial == "prior", "invalid root/schema mutated or replaced profile");
    }
    auto profile = GoodProfile();
    for (const char* key : {"scale", "targetModelScale", "hmdScale", "rel_qw", "rel_qx", "rel_qy", "rel_qz",
        "rel_tx", "rel_ty", "rel_tz", "headFilter", "driftFilter", "followSlam", "continuousSync", "hideHeadTracker"}) profile.erase(key);
    CalibrationContext ctx{}; ctx.chaperone.valid = true; ctx.chaperone.geometry.resize(3);
    ctx.validRelativeOffset = true;
    Parse(ctx, profile);
    Require(ctx.validProfile && !ctx.validRelativeOffset && !ctx.chaperone.valid && ctx.chaperone.geometry.empty(),
        "legacy defaults retained a stale mount or chaperone");
    Require(ctx.calibratedScale == 1 && ctx.hmdScale == 1 && !ctx.followSlamHmd && ctx.continuousSync,
        "legacy profile defaults changed");
}
void ProfileOptionalTypesAndBounds() {
    for (const char* key : {"hmd_serial", "tracker_serial", "followSlam", "continuousSync", "noHeadTracker",
        "hideHeadTracker", "headFilterEnabled", "headFilter", "driftFilter", "hmdScale", "uiScale"}) {
        auto p = GoodProfile(); p[key] = picojson::value(picojson::array{}); ExpectRejected(p);
    }
    for (double value : {-1.0, 0.0, 1e300}) {
        auto p = GoodProfile(); p["scale"] = picojson::value(value); ExpectRejected(p);
    }
    auto p = GoodProfile();
    picojson::object filter; filter["minCutoff"] = picojson::value(-1.0);
    p["headFilter"] = picojson::value(filter); ExpectRejected(p);
    p = GoodProfile(); p["rel_qw"] = picojson::value(2.0); ExpectRejected(p);
    p = GoodProfile(); p["rel_qw"] = picojson::value(1.00001);
    CalibrationContext ctx{}; Parse(ctx, p);
    Require(std::abs(ctx.relativeRotation.w - 1.0) < 1e-12, "quaternion rounding drift was not normalized");
    std::string malformed = picojson::value(picojson::array{picojson::value(GoodProfile())}).serialize();
    auto where = malformed.find("\"scale\":1");
    Require(where != std::string::npos, "test JSON did not contain scale");
    malformed.replace(where, std::strlen("\"scale\":1"), "\"scale\":1e309");
    bool rejected = false;
    try { std::istringstream input(malformed); ParseProfile(ctx, input); }
    catch (const std::runtime_error&) { rejected = true; }
    Require(rejected, "overflowing JSON number accepted");
}
void ScaleNeedsIndependentTranslation() {
    std::vector<Sample> samples;
    for (int i = 0; i < 120; ++i) {
        Eigen::Matrix3d rotation = (Eigen::AngleAxisd(0.5 * std::sin(i * 0.15), Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.5 * std::cos(i * 0.13), Eigen::Vector3d::UnitY())).toRotationMatrix();
        // Large spread that is entirely explained by rotation of a lever arm:
        // position = rotation * mount, so scale and mount cannot be separated.
        Pose pose(DevicePose(rotation, rotation * Eigen::Vector3d(10, 0, 0)).mDeviceToAbsoluteTracking);
        samples.emplace_back(pose, pose);
    }
    Require(std::abs(EstimateHmdSpaceScale(samples, Eigen::Matrix3d::Identity(), 1.0) - 1.0) < 1e-12,
        "unobservable scale/mount coupling produced a fitted scale");
}
void MeasuredScaleAndFallback() {
    std::vector<Sample> samples;
    constexpr double scale = 1.012;
    const Eigen::Vector3d mount(0.02, -0.08, 0.06);
    for (int i = 0; i < 120; ++i) {
        Eigen::Matrix3d rotation = (Eigen::AngleAxisd(0.5 * std::sin(i * 0.15), Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.5 * std::cos(i * 0.13), Eigen::Vector3d::UnitY())).toRotationMatrix();
        Eigen::Vector3d position(0.9 * std::sin(i * 0.071), 1.7 + 0.5 * std::cos(i * 0.093), 0.8 * std::cos(i * 0.051));
        samples.emplace_back(Pose(DevicePose(rotation, scale * (position + rotation * mount)).mDeviceToAbsoluteTracking),
            Pose(DevicePose(rotation, position).mDeviceToAbsoluteTracking));
    }
    bool measured = false;
    const double fitted = EstimateHmdSpaceScale(samples, Eigen::Matrix3d::Identity(), 1.0, &measured);
    Require(measured && std::abs(fitted - scale) < 1e-5, "independent translation failed to identify measured scale");
    CalibrationContext candidate{};
    candidate.hmdScale = fitted; candidate.validRelativeOffset = true;
    candidate.relativeTranslation = {mount.x(), mount.y(), mount.z()};
    Require(CheckAlignment(candidate, samples).passed, "validation compared raw and scale-corrected positions in different units");
    for (auto &sample : samples) sample.ref.trans *= 1.1;
    Require(EstimateHmdSpaceScale(samples, Eigen::Matrix3d::Identity(), 1.0, &measured) == 1.0 && !measured,
        "out-of-range scale was reported as measured");
    measured = true;
    Require(EstimateHmdSpaceScale({}, Eigen::Matrix3d::Identity(), 1.0, &measured) == 1.0 && !measured,
        "empty scale data retained a measured status");
}
void AlignmentBoundsAndTailErrors() {
    CalibrationContext candidate{}; candidate.validRelativeOffset = true;
    Pose pose(DevicePose(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()).mDeviceToAbsoluteTracking);
    std::vector<Sample> samples(80, Sample(pose, pose));
    Require(CheckAlignment(candidate, samples).passed, "exact alignment control failed");
    for (int i = 0; i < 10; ++i) samples[i].ref.trans.x() = 0.04;
    Require(CheckAlignment(candidate, samples).passed, "aggregate control for the segment failure test was not below limits");
    for (int i = 0; i < 10; ++i) samples[i].ref.trans.x() = 0.06;
    Require(!CheckAlignment(candidate, samples).passed, "position p95 failure was hidden by good RMS");
    samples.assign(80, Sample(pose, pose)); samples[0].ref.trans.x() = 0.11;
    Require(!CheckAlignment(candidate, samples).passed, "single large position excursion was ignored");
    samples.assign(80, Sample(pose, pose));
    for (int i = 0; i < 10; ++i) samples[i].ref.rot = Eigen::AngleAxisd(3.5 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Require(!CheckAlignment(candidate, samples).passed, "angular p95 failure was hidden by good RMS");
    samples.assign(80, Sample(pose, pose));
    samples[0].ref.rot = Eigen::AngleAxisd(7.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Require(!CheckAlignment(candidate, samples).passed, "single large angular excursion was ignored");
    samples.assign(80, Sample(pose, pose)); samples[0].valid = false;
    Require(!CheckAlignment(candidate, samples).passed, "invalid sample entered the agreement check");
    samples[0].valid = true; samples[0].ref.trans.x() = std::numeric_limits<double>::quiet_NaN();
    Require(!CheckAlignment(candidate, samples).passed, "non-finite check input passed");
    samples.assign(80, Sample(pose, pose));
    for (double scale : {0.0, 2.0, std::numeric_limits<double>::infinity()}) {
        candidate.hmdScale = scale;
        Require(!CheckAlignment(candidate, samples).passed, "invalid candidate scale passed");
    }
    candidate.hmdScale = 1.0; candidate.relativeTranslation.v[0] = 10.0;
    Require(!FiniteCandidate(candidate), "impossible mount was accepted");
    candidate.relativeTranslation.v[0] = 0.0; candidate.relativeRotation.w = 0.0;
    Require(!FiniteCandidate(candidate), "zero quaternion candidate was accepted");
}
void AdaptiveMotionGuidance() {
    std::vector<Sample> samples;
    for (int i = 0; i < 100; ++i) {
        SetMotion(i, false);
        samples.emplace_back(Pose(system.poses[0].mDeviceToAbsoluteTracking), Pose(system.poses[1].mDeviceToAbsoluteTracking));
    }
    Require(MissingMotionGuidance(samples).find("nods") != std::string::npos, "yaw-only motion did not request nods");
    for (auto &sample : samples) {
        const double angle = RotationVector(sample.ref.rot).y();
        sample.ref.rot = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }
    Require(MissingMotionGuidance(samples).find("turns") != std::string::npos, "pitch-only motion did not request turns");
}
}

int main(int argc, char** argv) {
    const std::pair<const char*, std::function<void()>> tests[] = {
        {"transient_loss", test::TransientLoss}, {"loss_timeout_rollback", test::LossTimeoutRollback},
        {"two_axes_complete", test::TwoAxesComplete}, {"retry_wait_recover", test::RetryWaitAndRecover},
        {"single_axis_timeout", test::SingleAxisTimeout}, {"retry_cancel", test::RetryCancel},
        {"save_failure_rollback", test::SaveFailureRollback}, {"pending_identity", test::PendingIdentityIsNotSaved},
        {"unsaved_snapshot", test::UnsavedSnapshotRestored}, {"pose_ingress", test::PoseIngressValidation},
        {"parse_transaction", test::ParseFailureIsTransactional}, {"profile_types", test::ProfileTypeValidation},
        {"geometry_bounds", test::ProfileGeometryBounds},
        {"profile_roots_legacy", test::ProfileRootsAndLegacyDefaults}, {"profile_optional_bounds", test::ProfileOptionalTypesAndBounds},
        {"scale_observability", test::ScaleNeedsIndependentTranslation},
        {"fit_remains_private", test::FitRemainsPrivate},
        {"heldout_bad_position", test::HeldOutBadPosition}, {"heldout_bad_angle", test::HeldOutBadAngle},
        {"heldout_coverage_timeout", test::HeldOutCoverageTimeout}, {"heldout_coverage_recovery", test::HeldOutCoverageRecovery},
        {"heldout_error_not_retried", test::HeldOutErrorIsNotRetried},
        {"heldout_tracking_loss", test::HeldOutTrackingLoss}, {"heldout_segment_failure", test::HeldOutSegmentFailure},
        {"heldout_cancel", test::HeldOutCancel}, {"heldout_missing_segment", test::HeldOutMissingSegment},
        {"heldout_sparse_contradiction", test::HeldOutSparseContradiction},
        {"scale_measured_fallback", test::MeasuredScaleAndFallback}, {"alignment_bounds_tails", test::AlignmentBoundsAndTailErrors},
        {"adaptive_guidance", test::AdaptiveMotionGuidance},
    };
    int failures = 0;
    for (const auto& [name, run] : tests) {
        if (argc > 1 && std::string(argv[1]) != name) continue;
        try { run(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& e) { std::cout << "FAIL " << name << ": " << e.what() << '\n'; ++failures; }
    }
    return failures ? 1 : 0;
}
