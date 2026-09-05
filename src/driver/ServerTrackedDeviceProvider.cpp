// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include "Version.h"

#include <cmath>
#include <filesystem>
#include <string>

namespace
{
	void StartConfiguredPoseCapture(spacesync::PoseCapture& capture) noexcept
	{
		// Diagnostics are opt-in and must never prevent a usable driver startup.
		try
		{
			auto* settings = vr::VRSettings();
			vr::EVRSettingsError error = vr::VRSettingsError_None;
			if (!settings || !settings->GetBool("driver_spacesync", "capturePoses", &error)
				|| error != vr::VRSettingsError_None) return;
			const DWORD capacity = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
			if (capacity == 0) { LOG("Pose capture unavailable: LOCALAPPDATA is missing%s", ""); return; }
			std::wstring localAppData(capacity, L'\0');
			const DWORD length = GetEnvironmentVariableW(L"LOCALAPPDATA", localAppData.data(), capacity);
			if (length == 0 || length >= capacity)
			{ LOG("Pose capture unavailable: LOCALAPPDATA changed during lookup%s", ""); return; }
			localAppData.resize(length);
			const auto directory = std::filesystem::path(localAppData) / L"SpaceSync" / L"captures";
			std::filesystem::create_directories(directory);
			SYSTEMTIME utc;
			GetSystemTime(&utc);
			LARGE_INTEGER counter;
			QueryPerformanceCounter(&counter);
			wchar_t filename[128];
			swprintf_s(filename, L"poses_%04u%02u%02u_%02u%02u%02u_%lu_%lld.csv",
				utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute, utc.wSecond,
				GetCurrentProcessId(), counter.QuadPart);
			const auto path = directory / filename;
			if (capture.Start(path)) LOG("Pose capture enabled: %ls", path.c_str());
			else LOG("Pose capture could not start: %ls", path.c_str());
		}
		catch (const std::exception& error) { LOG("Pose capture unavailable: %s", error.what()); }
		catch (...) { LOG("Pose capture unavailable: unexpected startup failure%s", ""); }
	}

	double QpcNowSeconds()
	{
		LARGE_INTEGER time, frequency;
		QueryPerformanceCounter(&time);
		QueryPerformanceFrequency(&frequency);
		return double(time.QuadPart) / frequency.QuadPart;
	}

	class SteamVrPoseSource final : public DriverPoseSource
	{
	public:
		TimedTrackerPose ReadTrackerPose(uint32_t hmdID, uint32_t trackerID, float predictionFrames) override
		{
			if (trackerID >= vr::k_unMaxTrackedDeviceCount || !std::isfinite(predictionFrames))
				return {};
			auto* properties = vr::VRProperties();
			auto* host = vr::VRServerDriverHost();
			if (!properties || !host)
				return {};
			const auto container = properties->TrackedDeviceToPropertyContainer(hmdID);
			float frequency = properties->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
			if (!std::isfinite(frequency) || frequency <= 0.0f) frequency = 90.0f;
			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount] = {};
			const double targetTime = QpcNowSeconds() + predictionFrames / frequency;
			host->GetRawTrackedDevicePoses(predictionFrames / frequency, poses, vr::k_unMaxTrackedDeviceCount);
			return {poses[trackerID], targetTime};
		}
	};

	bool FiniteVector(const double (&v)[3])
	{
		return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
	}

	bool ValidQuaternion(const vr::HmdQuaternion_t& q)
	{
		const double normSquared = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
		return std::isfinite(normSquared) && normSquared > 1e-12;
	}

	bool ConfigurationTranslationValid(const vr::HmdVector3d_t& value)
	{
		// Far beyond a room-scale calibration, but keeps finite IPC payloads
		// near DBL_MAX from overflowing downstream transform composition.
		return FiniteVector(value.v) && std::fabs(value.v[0]) <= 1e6
			&& std::fabs(value.v[1]) <= 1e6 && std::fabs(value.v[2]) <= 1e6;
	}

	bool AlignmentPoseValid(const vr::DriverPose_t& pose)
	{
		return pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK
			&& std::isfinite(pose.poseTimeOffset)
			&& ValidQuaternion(pose.qRotation) && ValidQuaternion(pose.qWorldFromDriverRotation)
			&& ValidQuaternion(pose.qDriverFromHeadRotation)
			&& FiniteVector(pose.vecPosition) && FiniteVector(pose.vecVelocity)
			&& FiniteVector(pose.vecAngularVelocity) && FiniteVector(pose.vecAcceleration)
			&& FiniteVector(pose.vecWorldFromDriverTranslation) && FiniteVector(pose.vecDriverFromHeadTranslation);
	}

	bool RuntimePoseFinite(const TimedTrackerPose& sample)
	{
		if (!std::isfinite(sample.targetTime)) return false;
		for (const auto& row : sample.pose.mDeviceToAbsoluteTracking.m)
			for (float value : row) if (!std::isfinite(value)) return false;
		for (float value : sample.pose.vVelocity.v) if (!std::isfinite(value)) return false;
		for (float value : sample.pose.vAngularVelocity.v) if (!std::isfinite(value)) return false;
		return true;
	}

	// Preserve the absolute time of replacement output, including a runtime
	// query's prediction horizon, across processing and lock delays.
	struct OutputPoseTime
	{
		vr::DriverPose_t& pose;
		double target = std::numeric_limits<double>::quiet_NaN();
		~OutputPoseTime()
		{
			if (std::isfinite(target)) pose.poseTimeOffset = target - QpcNowSeconds();
		}
	};
}

static double QpcSeconds(const LARGE_INTEGER& t)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return t.QuadPart / (double)freq.QuadPart;
}

static double FilterStep(double& lastUpdate, bool primed, double sampleTime)
{
	double dt = primed ? sampleTime - lastUpdate : 0.0;
	lastUpdate = sampleTime;
	if (dt <= 0.0 || isnan(dt)) dt = 1.0 / 90.0;
	if (dt > 0.1) dt = 0.1;
	return dt;
}

ServerTrackedDeviceProvider::ServerTrackedDeviceProvider(DriverPoseSource* source)
	: poseSource(source), alignmentWorker(std::make_unique<AlignmentWorker>(SolveAlignment)), server(this) { }

ServerTrackedDeviceProvider::~ServerTrackedDeviceProvider()
{
	ShutdownPoseUpdates();
}

void ServerTrackedDeviceProvider::InvalidateAlignmentHistory(bool resetPoses)
{
	++alignmentEpoch;
	if (resetPoses) pairHistory.reset();
	clock.noteHmdDiscontinuity();
	clock.noteTrackerDiscontinuity();
	refine.clearSums();
	refinePrimed = false;
	lastPairTime = -1.0;
	recoveryStarted = -1.0;
	recoveryPairs = 0;
	nextSolveTime = 0.0;
	residualDiag.reset();
	drift.estimator.resetDetectors();
	drift.lastUpdate = -1.0;
}

void ServerTrackedDeviceProvider::SolveAlignment(AlignmentWork& work)
{
	// Pure snapshot work: no provider, OpenVR, logging or callback locks.
	if (work.solveClock) work.clockUpdated = work.clock.solve(work.time);
	if (work.solveRefine) work.refineUpdated = work.refine.evaluate(work.applied);
}

void ServerTrackedDeviceProvider::ScheduleAlignment(double time)
{
	if (solveOutstanding) return;
	const bool solveClock = time >= nextSolveTime && clock.due(time);
	const bool solveRefine = refine.weight() >= refine.blockSeconds;
	if (!solveClock && !solveRefine) return;
	auto work = std::make_unique<AlignmentWork>();
	work->configuration = stateGeneration;
	work->epoch = alignmentEpoch;
	work->history = pairHistory.generation();
	work->time = time;
	work->solveClock = solveClock;
	work->solveRefine = solveRefine;
	if (solveClock) work->clock = clock;
	if (solveRefine) work->refine = refine;
	if (alignmentWorker->Submit(std::move(work)))
	{
		solveOutstanding = true;
		refineOutstanding = solveRefine;
		if (solveClock) nextSolveTime = time + clock.solveInterval;
	}
}

void ServerTrackedDeviceProvider::PublishAlignment(double now, const vr::HmdQuaternion_t& rawRotation,
	const vr::HmdVector3d_t& rawPosition, const vr::HmdQuaternion_t& headRotationBase)
{
	auto completion = alignmentWorker->Take();
	if (!completion) return;
	solveOutstanding = false;
	refineOutstanding = false;
	const auto& work = *completion.work;
	// History/config invalidation never waits for a solver. Its old completion
	// is discarded here, and observations collected since it began stay intact.
	if (completion.error || work.configuration != stateGeneration || work.epoch != alignmentEpoch
		|| work.history != pairHistory.generation() || QpcNowSeconds() - work.time > .25 || now < work.time)
		return;
	if (work.solveClock)
	{
		clock.rot = work.clock.rot;
		clock.pos = work.clock.pos;
		clock.lastSolve = work.clock.lastSolve;
		if (work.clockUpdated && now - clockLogTime > 10.0)
		{
			clockLogTime = now;
			LOG("Clock alignment: rot %.1f ms (corr %.2f), pos %.1f ms (corr %.2f)",
				clock.tauRot() * 1000.0, clock.rot.lastCorrelation, clock.tauPos() * 1000.0, clock.pos.lastCorrelation);
		}
	}
	if (work.refineUpdated)
	{
		// A joint snapshot may discover a different delay while evaluating a
		// block collected with the old one. Never apply that block first and
		// only clear it on the next observation.
		const double rotationTime = clock.tauRot() + tauRotTrim;
		const double positionTime = clock.pos.primed ? clock.tauPos() : rotationTime;
		if (std::fabs(rotationTime - refineTauRot) > .015 || std::fabs(positionTime - refineTauPos) > .015)
		{
			refine.clearSums();
			refinePrimed = false;
			++alignmentEpoch;
			return;
		}
		const double scaleBefore = SlamToCorrectedScale();
		refine = work.refine; // Accumulation was paused for this completed block.
		const auto& applied = work.applied;
		if (vecNorm(applied.rotation) > 0.0 || vecNorm(applied.translation) > 0.0 || applied.scale != 0.0)
		{
			drift.estimator.absorbRefinement(applied.rotation, applied.translation, applied.scale,
				rawRotation, rawPosition, headRotationBase, SlamToCorrectedScaleBase(), scaleBefore);
			UpdateEffectiveOffsets();
			drift.rotation = drift.estimator.rotation();
			drift.translation = drift.estimator.translation;
		}
		mount.suspected = refine.suspected;
		mount.tiltDeg = refine.rotationAngle() * 180.0 / POSE_PI;
		mount.translationDeviation = refine.translationDistance();
		LOG("Mount refinement block %u: reference=%d translation=%d rotation=%d, rms %.1f -> %.1f mm, rotation %.2f -> %.2f deg",
			refine.solves, (int)refine.lastHadReference, (int)refine.lastAcceptedTranslation, (int)refine.lastAcceptedRotation,
			refine.lastRmsBefore * 1000.0, refine.lastRmsAfter * 1000.0,
			refine.lastRotBefore * 180.0 / POSE_PI, refine.lastRotAfter * 180.0 / POSE_PI);
	}
}

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	OpenLogFile();
	LOG("SpaceSync driver " SPACECAL_VERSION_STRING " loaded");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);

	headFilter.rotationFilter.params = { 5.0, 0.8, 1.0 };
	headFilter.translationFilter.params = { 5.0, 0.8, 1.0 };
	headVel.filter.params = { 8.0, 1.0, 1.0 };

	trackerFilter.translation.SetQ(2.5e-7);
  	trackerFilter.translation.SetR(1.0e-5);
	trackerFilter.translation.SetAdaptiveGain(4.0);

	if (!alignmentWorker->Start() || !InjectHooks(pDriverContext) || !server.Run())
	{
		LOG("Hook or IPC startup failed; unloading driver%s", "");
		DisableHooks();
		ShutdownPoseUpdates();
		CloseLogFile();
		VR_CLEANUP_SERVER_DRIVER_CONTEXT();
		return vr::VRInitError_Driver_Failed;
	}

	StartConfiguredPoseCapture(poseCapture);
	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	ShutdownPoseUpdates();
	poseCapture.Stop();
	const auto captureStats = poseCapture.GetStats();
	if (captureStats.accepted || captureStats.dropped || captureStats.ioFailed)
		LOG("Pose capture stopped: accepted=%llu written=%llu dropped=%llu ioFailed=%d",
			static_cast<unsigned long long>(captureStats.accepted),
			static_cast<unsigned long long>(captureStats.written),
			static_cast<unsigned long long>(captureStats.dropped), captureStats.ioFailed ? 1 : 0);
	// IPC and complete detour calls have relinquished all logging/context use.
	LOG("SpaceSync driver unloaded");
	CloseLogFile();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;
	if ((newTransform.updateTranslation && !ConfigurationTranslationValid(newTransform.translation))
		|| (newTransform.updateRotation && !ValidQuaternion(newTransform.rotation))
		// Model ratios can span .01*.01/100 through 100*100/.01.
		|| (newTransform.updateScale && (!std::isfinite(newTransform.scale) || newTransform.scale < 1e-6 || newTransform.scale > 1e6)))
		return;

	std::lock_guard<std::mutex> lock(stateMutex);
	auto next = transforms[newTransform.openVRID];

	if (newTransform.updateTranslation)
		next.translation = newTransform.translation;

	if (newTransform.updateRotation)
		next.rotation = quaternionNormalize(newTransform.rotation);

	if (newTransform.updateScale)
		next.scale = newTransform.scale;

	next.enabled = newTransform.enabled;
	auto& tf = transforms[newTransform.openVRID];
	if (tf.enabled != next.enabled || !OffsetsEqual(tf.rotation, tf.translation, tf.scale, next.rotation, next.translation, next.scale))
	{
		tf = next;
		++stateGeneration;
	}
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& supplied)
{
	auto cmd = supplied;
	if (cmd.enabled)
	{
		if (cmd.hmdID >= vr::k_unMaxTrackedDeviceCount
			|| (cmd.trackerID != vr::k_unTrackedDeviceIndexInvalid && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
			|| !ValidQuaternion(cmd.offsetRotation) || !ValidQuaternion(cmd.calibrationRotation)
			|| !ConfigurationTranslationValid(cmd.offsetTranslation) || !ConfigurationTranslationValid(cmd.calibrationTranslation)
			|| !std::isfinite(cmd.hmdScale) || cmd.hmdScale < .01 || cmd.hmdScale > 100
			|| !std::isfinite(cmd.calibrationScale) || cmd.calibrationScale < .01 || cmd.calibrationScale > 100
			|| !std::isfinite(cmd.predictionTime) || cmd.predictionTime < 0 || cmd.predictionTime > 10) return;
		cmd.offsetRotation = quaternionNormalize(cmd.offsetRotation);
		cmd.calibrationRotation = quaternionNormalize(cmd.calibrationRotation);
		// Init starts the production worker; this also supports independent
		// providers configured outside a SteamVR driver context.
		if (!alignmentWorker->Start()) return;
	}
	std::lock_guard<std::mutex> lock(stateMutex);
	const bool wasEnabled = hmdTracker.enabled;

	double cmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;
	bool keepOffsets = false;
	if (cmd.enabled && wasEnabled)
	{
		bool sameBase = OffsetsEqual(cmd.offsetRotation, cmd.offsetTranslation, cmdScale, hmdTracker.offsetRotation, hmdTracker.offsetTranslation, hmdTracker.hmdScale);
		bool samePublished = publishedValid && OffsetsEqual(cmd.offsetRotation, cmd.offsetTranslation, cmdScale, published.rotation, published.translation, published.hmdScale);
		keepOffsets = sameBase || samePublished;
	}
	// The overlay resends this command every scan, including published refinement
	// echoes. An equivalent resend must not invalidate an in-flight runtime query.
	const double calibrationScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	const bool sameOffsets = keepOffsets || OffsetsEqual(cmd.offsetRotation, cmd.offsetTranslation, cmdScale,
		hmdTracker.offsetRotation, hmdTracker.offsetTranslation, hmdTracker.hmdScale);
	if (wasEnabled == cmd.enabled && sameOffsets && hmdTracker.followSlam == cmd.followSlamHmd
		&& hmdTracker.hideHeadTracker == (cmd.hideHeadTracker && cmd.followSlamHmd)
		&& hmdTracker.slamFallback == cmd.slamFallback && hmdTracker.enableAngularVelocity == cmd.enableAngularVelocity
		&& hmdTracker.predictionTime == cmd.predictionTime && hmdTracker.hmdID == cmd.hmdID && hmdTracker.trackerID == cmd.trackerID
		&& OffsetsEqual(cmd.calibrationRotation, cmd.calibrationTranslation, calibrationScale,
			hmdTracker.calibrationRotation, hmdTracker.calibrationTranslation, hmdTracker.calibrationScale))
		return;
	++stateGeneration;
	InvalidateAlignmentHistory();
	trackerFrame.reset();
	hmdTracker.enabled = cmd.enabled;
	if (hmdTracker.hmdID != cmd.hmdID || hmdTracker.trackerID != cmd.trackerID || hmdTracker.followSlam != cmd.followSlamHmd)
	{
		// A new producer or mode cannot reuse finite differences, latency samples,
		// or filter state from the previous source. The last alignment remains a
		// fallback until the new source supplies a valid observation.
		trackerSample.valid = false;
		frames.reset();
		clock.reset();
		lastHmdTime = -1.0;
		tauRotTrim = 0.0;
		residualDiag.reset();
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		keepOffsets = false;
	}

	vr::HmdQuaternion_t baseRotation = hmdTracker.offsetRotation;
	vr::HmdVector3d_t baseTranslation = hmdTracker.offsetTranslation;
	double baseScale = hmdTracker.hmdScale;

	if (hmdTracker.followSlam != cmd.followSlamHmd)
		hmdFrame.reset();

	hmdTracker.followSlam = cmd.followSlamHmd;
	hmdTracker.hideHeadTracker = cmd.hideHeadTracker && cmd.followSlamHmd;
	hmdTracker.slamFallback = cmd.slamFallback;
	hmdTracker.enableAngularVelocity = cmd.enableAngularVelocity;
	hmdTracker.predictionTime = cmd.predictionTime;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.trackerID;
	hmdTracker.offsetRotation = cmd.offsetRotation;
	hmdTracker.offsetTranslation = cmd.offsetTranslation;
	hmdTracker.calibrationRotation = cmd.calibrationRotation;
	hmdTracker.calibrationTranslation = cmd.calibrationTranslation;
	hmdTracker.calibrationScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	hmdTracker.hmdScale = cmdScale;

	if (keepOffsets)
	{
		hmdTracker.offsetRotation = baseRotation;
		hmdTracker.offsetTranslation = baseTranslation;
		hmdTracker.hmdScale = baseScale;
	}

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.estimator.reset();
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		trackerState.reset();
		driftLog.reset();
		clock.reset();
		lastHmdTime = -1.0;
		tauRotTrim = 0.0;
		frames.reset();
		hmdFrame.reset();
		residualDiag.reset();
		mount.reset();
		refine.reset();
		refinePrimed = false;
		trackerSample.valid = false;
		effectiveSharedValid = false;
		publishedValid = false;
		memset(slamSync, 0, sizeof slamSync);
	}
	else
	{
		if (!wasEnabled || !keepOffsets)
		{
			refine.reset();
			mount.reset();
			refinePrimed = false;
			UpdateEffectiveOffsets();
			publishedValid = false;
			if (wasEnabled)
				LOG("Head tracker offsets changed while running, mount refinement restarted");
			if (cmd.followSlamHmd && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
			{
				drift.estimator.reset();
				drift.estimator.valid = true;
				drift.estimator.yaw = 0.0;
				drift.estimator.translation = { 0, 0, 0 };
				drift.rotation = { 1, 0, 0, 0 };
				drift.translation = { 0, 0, 0 };
				drift.valid = true;
				LOG("Follow mode without head tracker: static alignment from calibration, headset recenters are carried over");
			}
		}
		hmdTracker.enabled = true;
	}
}

void ServerTrackedDeviceProvider::UpdateEffectiveOffsets()
{
	effective.rotation = refine.effectiveRotation(hmdTracker.offsetRotation);
	effective.translation = refine.effectiveTranslation(hmdTracker.offsetRotation, hmdTracker.offsetTranslation);
	effective.hmdScale = hmdTracker.hmdScale / (1.0 + refine.scale);
	effectiveShared = effective;
	effectiveSharedValid = true;
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	std::lock_guard<std::mutex> lock(stateMutex);
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount && slamSync[cmd.openVRID] != cmd.enabled)
	{
		slamSync[cmd.openVRID] = cmd.enabled;
		++stateGeneration;
	}
}

void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	std::lock_guard<std::mutex> lock(stateMutex);
	if (!cmd.headEnabled)
	{
		headFilter.reset();
		headFilter.enabled = false;
		return;
	}
	if (!std::isfinite(cmd.head.minCutoff) || cmd.head.minCutoff <= 0.0
		|| !std::isfinite(cmd.head.beta) || cmd.head.beta < 0.0
		|| !std::isfinite(cmd.head.dCutoff) || cmd.head.dCutoff <= 0.0) return;
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = p.minCutoff < 0.01 ? 0.01 : p.minCutoff;
		out.beta = p.beta < 0.0 ? 0.0 : p.beta;
		out.dCutoff = p.dCutoff < 0.01 ? 0.01 : p.dCutoff;
		return out;
	};

	headFilter.rotationFilter.params = toParams(cmd.head);
	headFilter.translationFilter.params = toParams(cmd.head);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3], double confidence, double sampleTime)
{
	vr::HmdQuaternion_t relation = quaternionNormalize(correctedRotation * quaternionConjugate(rawRotation));
	vr::HmdQuaternion_t instRot = quaternionProjectYaw(relation);

	double dt = FilterStep(drift.lastUpdate, drift.valid && drift.lastUpdate >= 0.0, sampleTime);
	double slamScale = SlamToCorrectedScale();

	auto distance = [](const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b) {
		return vecNorm(vecSub(a, b));
	};

	auto& est = drift.estimator;
	double yawBefore = est.valid ? est.yaw : 0.0;
	bool snapped = est.update(quaternionYawRad(instRot), vecFromArray(correctedPosition), vecFromArray(rawPosition), rawRotation, slamScale, confidence, dt);
	drift.rotation = est.rotation();
	drift.translation = est.translation;
	drift.valid = true;

	if (snapped)
	{
		++alignmentEpoch;
		LOG("Drift jump compensated: yaw %.2f -> %.2f deg, translation delta %.1f cm, %d frames, sigma %.2f deg / %.1f mm",
			yawBefore * 180.0 / POSE_PI, est.yaw * 180.0 / POSE_PI, vecNorm(est.lastJumpTranslation) * 100.0, est.lastJumpFrames,
			est.sigmaYaw() * 180.0 / POSE_PI, est.sigmaTranslation() * 1000.0);
		refine.shift(est.lastJumpYaw, est.lastJumpTranslation);
		driftLog.yawDeg = quaternionYawDeg(drift.rotation);
		driftLog.translation = drift.translation;
		return;
	}

	double yawDeg = quaternionYawDeg(drift.rotation);
	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);
	double tauMs = clock.tauRot() * 1000.0;
	if (!driftLog.primed)
	{
		driftLog.primed = true;
		driftLog.yawDeg = yawDeg;
		driftLog.translation = drift.translation;
		driftLog.tauMs = tauMs;
		driftLog.lastLog = now;
		LOG("Drift (SLAM->tracker) initialised: yaw %.2f deg, translation (%.3f, %.3f, %.3f) m, latency offset rot %.1f ms / pos %.1f ms",
			yawDeg, drift.translation.v[0], drift.translation.v[1], drift.translation.v[2], tauMs, clock.tauPos() * 1000.0);
	}
	else
	{
		double dYaw = wrapDeg(yawDeg - driftLog.yawDeg);
		double dTrans = distance(drift.translation, driftLog.translation);
		double dTau = tauMs - driftLog.tauMs;
		double sinceLog = (now.QuadPart - driftLog.lastLog.QuadPart) / (double)freq.QuadPart;

		if ((std::fabs(dYaw) > 1.0 || dTrans > 0.05 || std::fabs(dTau) > 5.0) && sinceLog > 1.0)
		{
			LOG("Drift (SLAM->tracker) changed: yaw %.2f -> %.2f deg (delta %.2f), translation (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f) m (delta %.1f cm), latency offset rot %.1f -> %.1f ms, pos %.1f ms",
				driftLog.yawDeg, yawDeg, dYaw,
				driftLog.translation.v[0], driftLog.translation.v[1], driftLog.translation.v[2],
				drift.translation.v[0], drift.translation.v[1], drift.translation.v[2], dTrans * 100.0,
				driftLog.tauMs, tauMs, clock.tauPos() * 1000.0);
			driftLog.yawDeg = yawDeg;
			driftLog.translation = drift.translation;
			driftLog.tauMs = tauMs;
			driftLog.lastLog = now;
		}
	}
}

void ServerTrackedDeviceProvider::GetStatus(protocol::DriverStatus& status)
{
	std::lock_guard<std::mutex> lock(stateMutex);
	status.driftValid = drift.valid;
	status.mountShiftSuspected = mount.suspected;
	status.tiltDeg = mount.tiltDeg;
	status.translationDeviationM = mount.translationDeviation;
	status.latencyMs = clock.tauRot() * 1000.0;
	status.latencyPosMs = clock.tauPos() * 1000.0;
	status.jumpsCompensated = drift.estimator.jumps;
	status.sigmaYawDeg = drift.estimator.sigmaYaw() * 180.0 / POSE_PI;
	status.sigmaTranslationM = drift.estimator.sigmaTranslation();
	status.calmSeconds = refine.weight();
	status.refinementSolves = refine.applied;

	status.refinementValid = effectiveSharedValid && hmdTracker.enabled;
	status.offsetRotation = effectiveShared.rotation;
	status.offsetTranslation = effectiveShared.translation;
	status.hmdScale = effectiveShared.hmdScale;
	if (status.refinementValid)
	{
		published = effectiveShared;
		publishedValid = true;
	}
}

void ServerTrackedDeviceProvider::ApplyDrift(vr::DriverPose_t& pose) const
{
	double slamScale = SlamToCorrectedScale();

	pose.qWorldFromDriverRotation = quaternionNormalize(drift.rotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;
	for (double& velocity : pose.vecVelocity) velocity *= slamScale;

	double scaledTranslation[3] = {
		pose.vecWorldFromDriverTranslation[0] * slamScale,
		pose.vecWorldFromDriverTranslation[1] * slamScale,
		pose.vecWorldFromDriverTranslation[2] * slamScale
	};
	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(drift.rotation, scaledTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + drift.translation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + drift.translation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + drift.translation.v[2];
}

// Follow mode: exact inverse of ApplyDrift, moves a calibrated lighthouse pose into SLAM space.
void ServerTrackedDeviceProvider::ApplyInverseDrift(vr::DriverPose_t& pose) const
{
	double slamScale = SlamToCorrectedScale();
	double invScale = slamScale > 0.0 ? 1.0 / slamScale : 1.0;

	vr::HmdQuaternion_t invRotation = quaternionConjugate(drift.rotation);

	pose.qWorldFromDriverRotation = quaternionNormalize(invRotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= invScale;
	pose.vecPosition[1] *= invScale;
	pose.vecPosition[2] *= invScale;
	for (double& velocity : pose.vecVelocity) velocity *= invScale;

	double shifted[3] = {
		(pose.vecWorldFromDriverTranslation[0] - drift.translation.v[0]) * invScale,
		(pose.vecWorldFromDriverTranslation[1] - drift.translation.v[1]) * invScale,
		(pose.vecWorldFromDriverTranslation[2] - drift.translation.v[2]) * invScale
	};
	vr::HmdVector3d_t rotated = quaternionRotateVector(invRotation, shifted);
	pose.vecWorldFromDriverTranslation[0] = rotated.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotated.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotated.v[2];
}

bool ServerTrackedDeviceProvider::DetectHmdFrameJump(const vr::DriverPose_t& pose, double& jumpYaw, vr::HmdVector3d_t& jumpTranslation, bool& frameChanged)
{
	frameChanged = false;
	vr::HmdQuaternion_t rotation = quaternionNormalize(pose.qWorldFromDriverRotation);
	vr::HmdVector3d_t translation = vecFromArray(pose.vecWorldFromDriverTranslation);

	auto& w = hmdFrame;
	if (!w.primed)
	{
		w.primed = true;
		w.rotation = rotation;
		w.translation = translation;
		return false;
	}

	vr::HmdQuaternion_t step = quaternionNormalize(rotation * quaternionConjugate(w.rotation));
	vr::HmdVector3d_t stepTranslation = vecSub(translation, quaternionRotateVector(step, w.translation));
	double angle = quaternionAngleRad(step);
	double distance = vecNorm(stepTranslation);
	frameChanged = angle > 1e-6 || distance > 1e-6;

	w.rotation = rotation;
	w.translation = translation;

	if (angle < 0.25 * POSE_PI / 180.0 && distance < 0.003)
		return false;

	double tilt = quaternionAngleRad(quaternionNormalize(step * quaternionConjugate(quaternionProjectYaw(step))));
	if (tilt > 0.5 * POSE_PI / 180.0 || distance > 10.0)
		return false;

	jumpYaw = quaternionYawRad(quaternionProjectYaw(step));
	jumpTranslation = stepTranslation;
	w.jumps++;
	return true;
}

void ServerTrackedDeviceProvider::StoreTrackerSample(const vr::DriverPose_t& pose, const LARGE_INTEGER& arrival)
{
	TrackerSample s;
	s.valid = true;
	s.poseIsValid = pose.poseIsValid;
	s.deviceIsConnected = pose.deviceIsConnected;
	s.result = pose.result;
	if (!AlignmentPoseValid(pose))
	{
		// Preserve the tracking flags for status/fallback, but never put invalid
		// coordinates into the clock or difference them against the next sample.
		s.time = arrival;
		s.poseIsValid = false;
		trackerSample = s;
		frames.reset();
		InvalidateAlignmentHistory();
		return;
	}
	const auto frameRotation = quaternionNormalize(pose.qWorldFromDriverRotation);
	const auto frameTranslation = vecFromArray(pose.vecWorldFromDriverTranslation);
	if (trackerFrame.primed
		&& (quaternionAngleRad(quaternionNormalize(frameRotation * quaternionConjugate(trackerFrame.rotation))) > 1e-6
			|| vecNorm(vecSub(frameTranslation, trackerFrame.translation)) > 1e-6))
	{
		// Explicit producer coordinates changed. Do not mistake this for local
		// body motion or interpolate/solve across the old Lighthouse universe.
		frames.reset();
		InvalidateAlignmentHistory();
	}
	trackerFrame.primed = true;
	trackerFrame.rotation = frameRotation;
	trackerFrame.translation = frameTranslation;

	// Same math vrserver uses for mDeviceToAbsoluteTracking.
	s.rotation = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

	vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
	double driverLocal[3] = {
		pose.vecPosition[0] + headLocal.v[0],
		pose.vecPosition[1] + headLocal.v[1],
		pose.vecPosition[2] + headLocal.v[2]
	};
	vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
	s.position[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
	s.position[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
	s.position[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];

	// Velocities come in driver space, rotate them into world space.
	vr::HmdVector3d_t vel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecVelocity);
	vr::HmdVector3d_t angVel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecAngularVelocity);
	vr::HmdVector3d_t accel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecAcceleration);
	vr::HmdQuaternion_t deviceToWorld = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation);
	vr::HmdVector3d_t velDevice = quaternionRotateVector(deviceToWorld, pose.vecVelocity);
	vr::HmdVector3d_t angVelDevice = quaternionRotateVector(deviceToWorld, pose.vecAngularVelocity);
	vr::HmdVector3d_t accelDevice = quaternionRotateVector(deviceToWorld, pose.vecAcceleration);
	s.poseTimeOffset = pose.poseTimeOffset;
	s.time = arrival;
	double nominal = QpcSeconds(s.time) + s.poseTimeOffset;
	if (frames.primed && (nominal <= frames.time || nominal - frames.time > 0.1))
	{
		frames.reset();
		InvalidateAlignmentHistory();
	}

	if (frames.primed)
	{
		double span = nominal - frames.time;
		if (span > 0.001 && span < 0.05)
		{
			vr::HmdVector3d_t omegaFd = vecScale(quaternionToRotationVector(s.rotation * quaternionConjugate(frames.rotation)), 1.0 / span);
			vr::HmdVector3d_t velFd = vecScale(vecSub(vecFromArray(s.position), frames.position), 1.0 / span);
			double on = vecDot(omegaFd, omegaFd);
			if (on > 0.25)
			{
				frames.angWorld += vecDot(omegaFd, angVel);
				frames.angDevice += vecDot(omegaFd, angVelDevice);
				frames.angNorm += on;
			}
			double vn = vecDot(velFd, velFd);
			if (vn > 0.09)
			{
				frames.velWorld += vecDot(velFd, vel);
				frames.velDevice += vecDot(velFd, velDevice);
				frames.velNorm += vn;
			}
			if (frames.angNorm > 500.0)
			{
				double w = frames.angWorld / frames.angNorm, d = frames.angDevice / frames.angNorm;
				bool device = d > w;
				if ((!frames.decidedAngular || device != frames.angularDevice) && std::fabs(d - w) > 0.2)
				{
					frames.angularDevice = device;
					frames.decidedAngular = true;
					LOG("Tracker angular velocity frame: %s (agreement world %.2f, device %.2f)", device ? "device" : "world", w, d);
				}
				frames.angWorld *= 0.5; frames.angDevice *= 0.5; frames.angNorm *= 0.5;
			}
			if (frames.velNorm > 100.0)
			{
				double w = frames.velWorld / frames.velNorm, d = frames.velDevice / frames.velNorm;
				bool device = d > w;
				if ((!frames.decidedVelocity || device != frames.velocityDevice) && std::fabs(d - w) > 0.2)
				{
					frames.velocityDevice = device;
					frames.decidedVelocity = true;
					LOG("Tracker velocity frame: %s (agreement world %.2f, device %.2f)", device ? "device" : "world", w, d);
				}
				frames.velWorld *= 0.5; frames.velDevice *= 0.5; frames.velNorm *= 0.5;
			}
		}
	}
	frames.primed = true;
	frames.time = nominal;
	frames.rotation = s.rotation;
	frames.position = vecFromArray(s.position);

	if (frames.angularDevice) angVel = angVelDevice;
	if (frames.velocityDevice) { vel = velDevice; accel = accelDevice; }

	// Gain is the regression slope of a velocity difference onto the reported acceleration, so it is
	// the MSE-optimal shrinkage: 0 on drivers whose acceleration is noise, 1 where it is clean.
	auto& f = frames;
	vr::HmdVector3d_t accelUsed = { 0, 0, 0 };
	if (f.velCount >= FrameConvention::VelRing)
	{
		int oldest = f.velNext;
		double span = nominal - f.velHistoryTime[oldest];
		if (span > 0.01 && span < 0.1)
		{
			vr::HmdVector3d_t accelFd = vecScale(vecSub(vel, f.velHistory[oldest]), 1.0 / span);
			f.accNum += vecDot(accelFd, accel);
			f.accDen += vecDot(accel, accel);
			if (f.accDen > 500.0)
			{
				double gain = f.accNum / f.accDen;
				if (gain < 0.0) gain = 0.0;
				if (gain > 1.0) gain = 1.0;
				f.accGain = gain;
				f.accDecided = true;
				f.accNum *= 0.5;
				f.accDen *= 0.5;
			}
		}
	}
	if (f.accDecided)
		accelUsed = vecScale(accel, f.accGain);

	f.velHistory[f.velNext] = vel;
	f.velHistoryTime[f.velNext] = nominal;
	f.velNext = (f.velNext + 1) % FrameConvention::VelRing;
	if (f.velCount < FrameConvention::VelRing) f.velCount++;

	for (int i = 0; i < 3; i++)
	{
		s.velocity[i] = vel.v[i];
		s.angularVelocity[i] = angVel.v[i];
		s.acceleration[i] = accelUsed.v[i];
	}
	s.linSpeed = vecNorm(vel);
	s.angSpeed = vecNorm(angVel);

	// Calibration offsets are expressed in calibrated metres, whereas the clock
	// series is still in the tracker's raw coordinate system.
	const auto rawMountOffset = vecScale(hmdTracker.offsetTranslation, 1.0 / hmdTracker.calibrationScale);
	vr::HmdVector3d_t headPoint = vecAdd(vecFromArray(s.position), quaternionRotateVector(s.rotation, rawMountOffset));

	trackerSample = s;
	const int priorClockSamples = clock.tracker.count;
	clock.addTrackerPose(nominal, s.rotation, headPoint);
	if (priorClockSamples > 0 && clock.tracker.count == 0)
		InvalidateAlignmentHistory(); // The clock rejected a kinematic discontinuity.
	const auto generation = pairHistory.generation();
	pairHistory.addTracker({nominal, s.rotation, vecFromArray(s.position),
		vecFromArray(s.velocity), vecFromArray(s.angularVelocity)});
	if (pairHistory.generation() != generation)
		InvalidateAlignmentHistory(false); // Keep the newly accepted epoch's first pose.
}

ServerTrackedDeviceProvider::TrackerSample ServerTrackedDeviceProvider::LoadTrackerSample()
{
	return trackerSample;
}

void ServerTrackedDeviceProvider::NoteTrackerState(bool trackerOK, bool usingSlam, const vr::TrackedDevicePose_t& tp,
	double trackerYawDeg, bool relativeYawValid, double relativeYawDeg)
{
	auto& st = trackerState;

	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	const char* source = hmdTracker.followSlam ? "SLAM (follow mode, lighthouse devices follow the HMD)"
		: (usingSlam ? "SLAM+drift" : (trackerOK ? "tracker" : (tp.bPoseIsValid ? "tracker (dead-reckoned, drift frozen)" : "none")));

	if (!st.primed)
	{
		st.primed = true;
		st.wasOK = trackerOK;
		st.wasFallback = usingSlam;
		st.lastResult = tp.eTrackingResult;
		st.lostAt = now;
		st.trackerYawAtLoss = trackerYawDeg;
		st.relativeYawAtLoss = relativeYawDeg;
		st.relativeYawAtLossValid = relativeYawValid;
		LOG("Head tracker: initial state result=%d poseValid=%d connected=%d source=%s", (int)tp.eTrackingResult, (int)tp.bPoseIsValid, (int)tp.bDeviceIsConnected, source);
		return;
	}

	if (trackerOK && !st.wasOK)
	{
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		double lostFor = (now.QuadPart - st.lostAt.QuadPart) / (double)freq.QuadPart;
		double trackerDelta = wrapDeg(trackerYawDeg - st.trackerYawAtLoss);

		if (relativeYawValid && st.relativeYawAtLossValid)
		{
			// If the SLAM->tracker yaw delta stays non-zero, the lighthouse frame moved relative to the room.
			LOG("Head tracker: OK again after %.2f s (last result=%d), tracker yaw %.2f -> %.2f deg (delta %.2f), SLAM->tracker yaw %.2f -> %.2f deg (delta %.2f)",
				lostFor, (int)st.lastResult, st.trackerYawAtLoss, trackerYawDeg, trackerDelta,
				st.relativeYawAtLoss, relativeYawDeg, wrapDeg(relativeYawDeg - st.relativeYawAtLoss));
		}
		else
		{
			LOG("Head tracker: OK again after %.2f s (last result=%d), tracker yaw %.2f -> %.2f deg (delta %.2f)",
				lostFor, (int)st.lastResult, st.trackerYawAtLoss, trackerYawDeg, trackerDelta);
		}
	}
	else if (!trackerOK && st.wasOK)
	{
		st.lostAt = now;
		st.trackerYawAtLoss = trackerYawDeg;
		st.relativeYawAtLoss = relativeYawDeg;
		st.relativeYawAtLossValid = relativeYawValid;
		LOG("Head tracker: lost, result=%d poseValid=%d connected=%d, tracker yaw %.2f deg, SLAM->tracker yaw %.2f deg, head pose source now: %s",
			(int)tp.eTrackingResult, (int)tp.bPoseIsValid, (int)tp.bDeviceIsConnected, trackerYawDeg, relativeYawDeg, source);
	}
	else if (!trackerOK && (tp.eTrackingResult != st.lastResult || usingSlam != st.wasFallback))
	{
		LOG("Head tracker: still lost, result %d -> %d, poseValid=%d, head pose source: %s",
			(int)st.lastResult, (int)tp.eTrackingResult, (int)tp.bPoseIsValid, source);
	}

	st.wasOK = trackerOK;
	st.wasFallback = usingSlam;
	st.lastResult = tp.eTrackingResult;
}

ServerTrackedDeviceProvider::PoseUpdateLease::PoseUpdateLease(ServerTrackedDeviceProvider& owner) : owner(owner)
{
	std::lock_guard<std::mutex> lock(owner.callbackMutex);
	admitted = owner.acceptingPoseUpdates;
	if (admitted) ++owner.activePoseUpdates;
}

ServerTrackedDeviceProvider::PoseUpdateLease::~PoseUpdateLease()
{
	if (!admitted) return;
	std::lock_guard<std::mutex> lock(owner.callbackMutex);
	if (--owner.activePoseUpdates == 0) owner.callbacksDrained.notify_all();
}

void ServerTrackedDeviceProvider::ShutdownPoseUpdates()
{
	std::unique_lock<std::mutex> lock(callbackMutex);
	acceptingPoseUpdates = false;
	callbacksDrained.wait(lock, [this] { return activePoseUpdates == 0; });
	lock.unlock();
	if (alignmentWorker) alignmentWorker->Stop();
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	LARGE_INTEGER arrival;
	QueryPerformanceCounter(&arrival); // Arrival precedes admission and state waits.
	PoseUpdateLease lease(*this);
	if (!lease) return false;
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	OutputPoseTime outputTime{pose, QpcSeconds(arrival) + pose.poseTimeOffset};
	std::unique_lock<std::mutex> lock(stateMutex);
	TimedTrackerPose rawTracker;
	if (hmdTracker.enabled && !hmdTracker.followSlam && openVRID == hmdTracker.hmdID)
	{
		const auto generation = stateGeneration;
		const auto hmdID = hmdTracker.hmdID, trackerID = hmdTracker.trackerID;
		const auto predictionFrames = hmdTracker.predictionTime;
		lock.unlock();
		static SteamVrPoseSource runtime;
		rawTracker = (poseSource ? poseSource : &runtime)->ReadTrackerPose(hmdID, trackerID, predictionFrames);
		lock.lock();
		if (generation != stateGeneration)
			return false; // Do not publish an old-config pose after reconfiguration.
	}
	const bool overrideEnabled = hmdTracker.enabled;
	const bool followSlam = overrideEnabled && hmdTracker.followSlam;
	if (overrideEnabled && openVRID == hmdTracker.hmdID)
		poseCapture.TryRecord(openVRID, spacesync::PoseCapture::Role::Hmd, alignmentEpoch, QpcSeconds(arrival), pose);
	else if (overrideEnabled && openVRID == hmdTracker.trackerID)
		poseCapture.TryRecord(openVRID, spacesync::PoseCapture::Role::Tracker, alignmentEpoch, QpcSeconds(arrival), pose);

	// Follow mode: stash the raw head tracker pose before any transform touches it.
	if (followSlam && openVRID == hmdTracker.trackerID)
		StoreTrackerSample(pose, arrival);

	auto& tf = transforms[openVRID];
	if (tf.enabled && !(followSlam && openVRID == hmdTracker.hmdID))
	{
		pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;
		for (double& velocity : pose.vecVelocity) velocity *= tf.scale;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];

		// Follow mode: move the calibrated lighthouse device into the headset's SLAM space.
		if (followSlam && drift.valid)
			ApplyInverseDrift(pose);
	}

	// Alignment is unaffected, it reads the raw sample stashed above.
	if (followSlam && hmdTracker.hideHeadTracker && openVRID == hmdTracker.trackerID)
	{
		const vr::HmdVector3d_t parked = { 0.0, 9001.0, 0.0 };
		vr::HmdVector3d_t local = quaternionRotateVector(
			quaternionConjugate(quaternionNormalize(pose.qWorldFromDriverRotation)),
			vecSub(parked, vecFromArray(pose.vecWorldFromDriverTranslation)));
		local = vecSub(local, quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation));

		for (int i = 0; i < 3; i++)
		{
			pose.vecPosition[i] = local.v[i];
			pose.vecVelocity[i] = 0.0;
			pose.vecAcceleration[i] = 0.0;
			pose.vecAngularVelocity[i] = 0.0;
			pose.vecAngularAcceleration[i] = 0.0;
		}
	}

	if (overrideEnabled)
	{
		if (openVRID == hmdTracker.hmdID)
		{
			bool rawValid = AlignmentPoseValid(pose);
			vr::HmdQuaternion_t rawRotation = { 1, 0, 0, 0 };
			double rawPosition[3] = { 0, 0, 0 };
			if (rawValid)
			{
				rawRotation = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

				vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
				double driverLocal[3] = {
					pose.vecPosition[0] + headLocal.v[0],
					pose.vecPosition[1] + headLocal.v[1],
					pose.vecPosition[2] + headLocal.v[2]
				};
				vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
				rawPosition[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
				rawPosition[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
				rawPosition[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];
			}

			if (followSlam)
			{
				// Follow mode: headset pose stays untouched, we only measure the drift here.
				TrackerSample ts = LoadTrackerSample();

				const double nowSeconds = QpcSeconds(arrival);
				double age = ts.valid ? nowSeconds - QpcSeconds(ts.time) : 1e9;
				double hmdTime = nowSeconds + pose.poseTimeOffset;
				bool freshPose = !rawValid || lastHmdTime < 0.0 || hmdTime - lastHmdTime > 0.001;
				if (rawValid)
				{
					if (lastHmdTime >= 0.0 && (hmdTime <= lastHmdTime || hmdTime - lastHmdTime > 0.1))
						InvalidateAlignmentHistory();
					// Before the pose enters the clock series, a step there reads as a velocity spike.
					double jumpYaw = 0.0;
					vr::HmdVector3d_t jumpTranslation = { 0, 0, 0 };
					bool frameChanged = false;
					const bool canRebase = DetectHmdFrameJump(pose, jumpYaw, jumpTranslation, frameChanged);
					if (frameChanged) InvalidateAlignmentHistory();
					if (canRebase)
					{

						if (drift.valid)
						{
							drift.estimator.rebase(jumpYaw, jumpTranslation, SlamToCorrectedScale());
							drift.rotation = drift.estimator.rotation();
							drift.translation = drift.estimator.translation;

							// Its sums are built from raw poses, which are now in a different frame.
							refine.clearSums();

							driftLog.yawDeg = quaternionYawDeg(drift.rotation);
							driftLog.translation = drift.translation;
						}
					}

					const int priorClockSamples = clock.hmd.count;
					clock.addHmdPose(hmdTime, rawRotation, vecFromArray(rawPosition));
					if (priorClockSamples > 0 && clock.hmd.count == 0)
						InvalidateAlignmentHistory();
					const auto generation = pairHistory.generation();
					pairHistory.addHmd({hmdTime, rawRotation, vecFromArray(rawPosition), {}, {}});
					if (pairHistory.generation() != generation)
						InvalidateAlignmentHistory(false);
					lastHmdTime = hmdTime;
				}
				else
				{
					InvalidateAlignmentHistory();
					lastHmdTime = -1.0;
					hmdFrame.reset();
				}

				const bool trackerHasPose = ts.valid && age < 0.25 && ts.deviceIsConnected && ts.poseIsValid;
				const bool trackerOK = trackerHasPose && ts.result == vr::TrackingResult_Running_OK;

				vr::TrackedDevicePose_t tpLog = {};
				tpLog.bPoseIsValid = trackerHasPose;
				tpLog.bDeviceIsConnected = ts.valid && ts.deviceIsConnected;
				tpLog.eTrackingResult = ts.valid ? ts.result : vr::TrackingResult_Uninitialized;

				double relativeYaw = drift.valid ? quaternionYawDeg(drift.rotation) : 0.0;
				bool relativeYawValid = drift.valid;

				if (trackerOK && rawValid && freshPose)
				{
					// The continuity pivot is at the current HMD's time. Recover its
					// base orientation from the published mapping and mount correction,
					// not a newest tracker rotation sampled at a different instant.
					const auto currentHeadBase = quaternionNormalize(drift.rotation * rawRotation
						* quaternionConjugate(effective.rotation) * hmdTracker.offsetRotation);
					PublishAlignment(nowSeconds, rawRotation, vecFromArray(rawPosition), currentHeadBase);
					double tauRotNow = clock.tauRot() + tauRotTrim;
					double tauPosNow = clock.pos.primed ? clock.tauPos() : tauRotNow;
					align::RigidPairMatch pair;
					// Estimate from observed, bracketed poses only. This historical
					// measurement does not delay or modify the native Follow HMD.
					if (!pairHistory.matchLatest(hmdTime, tauRotNow, tauPosNow, .30, pair)
						|| pair.hmdTime <= lastPairTime)
					{
						ScheduleAlignment(nowSeconds);
						NoteTrackerState(trackerOK, true, tpLog, quaternionYawDeg(ts.rotation), relativeYawValid, relativeYaw);
						return true;
					}
					lastPairTime = pair.hmdTime;
					const auto pairRawRotation = pair.hmd.pose.rotation;
					const auto pairRawPosition = pair.hmd.pose.position;
					const auto sampleRotation = pair.trackerRotation.pose.rotation;
					const auto samplePosition = pair.trackerPosition.pose.position;
					const double pairLinSpeed = vecNorm(pair.trackerPosition.pose.velocity);
					const double pairAngSpeed = vecNorm(pair.trackerRotation.pose.angularVelocity);

					vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * sampleRotation);
					vr::HmdVector3d_t trackerRefPosition = vecScale(quaternionRotateVector(hmdTracker.calibrationRotation, samplePosition), hmdTracker.calibrationScale);
					trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
					trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
					trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

					vr::HmdQuaternion_t headRotationBase = quaternionNormalize(trackerRefRotation * hmdTracker.offsetRotation);
					vr::HmdVector3d_t headPositionBase = vecAdd(trackerRefPosition, quaternionRotateVector(trackerRefRotation, hmdTracker.offsetTranslation));
					vr::HmdQuaternion_t headRotation = quaternionNormalize(trackerRefRotation * effective.rotation);
					vr::HmdVector3d_t headPositionVec = vecAdd(trackerRefPosition, quaternionRotateVector(trackerRefRotation, effective.translation));
					double headPosition[3] = { headPositionVec.v[0], headPositionVec.v[1], headPositionVec.v[2] };

					vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(headRotation * quaternionConjugate(pairRawRotation)));
					relativeYaw = quaternionYawDeg(instRot);
					relativeYawValid = true;


					double confidence = drift.valid ? DriftSampleConfidence(pairLinSpeed, pairAngSpeed) : 1.0;
					const double maxLinSpeed = 2.75;
					const double maxAngSpeed = 3.5;
					const bool freshPair = age <= 0.05 && QpcNowSeconds() - nowSeconds <= .1
						&& (!drift.valid || (pairLinSpeed < maxLinSpeed && pairAngSpeed < maxAngSpeed));
					if (freshPair)
					{
						if (recoveryStarted < 0.0) recoveryStarted = pair.hmdTime;
						if (recoveryPairs < 3) ++recoveryPairs;
					}
					const bool usable = freshPair && recoveryPairs >= 3 && pair.hmdTime - recoveryStarted >= .025;
					if (usable)
						UpdateDrift(headRotation, headPosition, pairRawRotation, pairRawPosition.v, confidence, pair.hmdTime);

					if (usable && drift.valid && drift.estimator.weightSum >= 1.0)
					{
						vr::HmdVector3d_t angVelCal = quaternionRotateVector(hmdTracker.calibrationRotation, pair.trackerRotation.pose.angularVelocity);
						double omegaYaw = angVelCal.v[1];
						if (std::fabs(omegaYaw) > 1.0)
						{
							double r = wrapRad(quaternionYawRad(instRot) - drift.estimator.yaw);
							if (std::fabs(r) < 0.35)
							{
								residualDiag.yawNum += omegaYaw * r;
								residualDiag.yawDen += omegaYaw * omegaYaw;
								residualDiag.yawFrames++;
							}
						}
						vr::HmdVector3d_t headVel = quaternionRotateVector(hmdTracker.calibrationRotation, pair.trackerPosition.pose.velocity);
						double speed = vecNorm(headVel);
						if (speed > 0.5)
						{
							vr::HmdVector3d_t predicted = vecAdd(quaternionRotateVector(drift.rotation, vecScale(pairRawPosition, SlamToCorrectedScale())), drift.translation);
							vr::HmdVector3d_t e = vecSub(headPositionVec, predicted);
							if (vecNorm(e) < 0.3)
							{
								residualDiag.posNum += vecDot(e, headVel);
								residualDiag.posDen += speed * speed;
								residualDiag.posFrames++;
							}
						}
						if (nowSeconds - residualDiag.lastLog > 10.0 && (residualDiag.yawFrames > 30 || residualDiag.posFrames > 30))
						{
							double yawSlope = residualDiag.yawDen > 0.0 ? residualDiag.yawNum / residualDiag.yawDen : 0.0;
							if (residualDiag.yawFrames >= 50)
							{
								tauRotTrim -= 0.3 * yawSlope;
								if (tauRotTrim > 0.025) tauRotTrim = 0.025;
								if (tauRotTrim < -0.025) tauRotTrim = -0.025;
							}
							LOG("Alignment residual: yaw slope %+.1f ms (%d frames), position slope %+.1f ms (%d frames), tau rot %.1f / pos %.1f ms, trim %+.1f ms",
								yawSlope * 1000.0, residualDiag.yawFrames,
								residualDiag.posDen > 0.0 ? residualDiag.posNum / residualDiag.posDen * 1000.0 : 0.0, residualDiag.posFrames,
								clock.tauRot() * 1000.0, clock.tauPos() * 1000.0, tauRotTrim * 1000.0);
							residualDiag.reset();
							residualDiag.lastLog = nowSeconds;
						}
					}

					double dtRefine = FilterStep(refineLast, refinePrimed, pair.hmdTime);
					refinePrimed = true;
					if (refine.weight() > 0.0 && (std::fabs(tauRotNow - refineTauRot) > 0.015 || std::fabs(tauPosNow - refineTauPos) > 0.015))
					{
						LOG("Mount refinement restarted, time alignment moved (rot %.1f -> %.1f ms, pos %.1f -> %.1f ms)",
							refineTauRot * 1000.0, tauRotNow * 1000.0, refineTauPos * 1000.0, tauPosNow * 1000.0);
						refine.clearSums();
						++alignmentEpoch;
					}
					if (refine.weight() <= 0.0)
					{
						refineTauRot = tauRotNow;
						refineTauPos = tauPosNow;
					}
					if (usable && !refineOutstanding && drift.valid && confidence > 0.5 && clock.rot.primed && clock.pos.primed)
					{
						const auto oldReference = refine.yawReference;
						refine.add(headRotationBase, headPositionBase, pairRawRotation, pairRawPosition, drift.estimator.yaw, SlamToCorrectedScaleBase(), confidence * dtRefine);
						if (refine.yawReference != oldReference) ++alignmentEpoch;
					}
					ScheduleAlignment(nowSeconds);
				}

				NoteTrackerState(trackerOK, true, tpLog, quaternionYawDeg(ts.rotation), relativeYawValid, relativeYaw);
				return true;
			}

			const vr::TrackedDevicePose_t& tp = rawTracker.pose;

			// Lighthouse keeps bPoseIsValid true while dead-reckoning on the IMU (OutOfRange),
			// only Running_OK is a real optical pose.
			const double runtimeAge = QpcNowSeconds() - rawTracker.targetTime;
			const bool trackerHasPose = tp.bDeviceIsConnected && tp.bPoseIsValid
				&& RuntimePoseFinite(rawTracker) && runtimeAge <= .1 && runtimeAge >= -.5;
			const bool trackerOK = trackerHasPose && tp.eTrackingResult == vr::TrackingResult_Running_OK;
			const bool slamAvailable = hmdTracker.slamFallback && drift.valid && rawValid;
			// Dead-reckoned poses only when there's no SLAM fallback.
			const bool useTracker = trackerOK || (trackerHasPose && !slamAvailable);

			vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);

			if (useTracker)
			{
				vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);

				vr::HmdVector3d_t filteredTrackerPos = trackerFilter.translation.update({
					tp.mDeviceToAbsoluteTracking.m[0][3],
					tp.mDeviceToAbsoluteTracking.m[1][3],
					tp.mDeviceToAbsoluteTracking.m[2][3]
				});
				double trackerPos[3] = {
					filteredTrackerPos.v[0],
					filteredTrackerPos.v[1],
					filteredTrackerPos.v[2]
				};

				vr::HmdVector3d_t trackerRefPosition = vecScale(quaternionRotateVector(hmdTracker.calibrationRotation, trackerPos), hmdTracker.calibrationScale);

				trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
				trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
				trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

				vr::HmdQuaternion_t hmdRotation = quaternionNormalize(trackerRefRotation * hmdTracker.offsetRotation);
				vr::HmdVector3d_t offset = quaternionRotateVector(trackerRefRotation, hmdTracker.offsetTranslation.v);

				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;

				pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
				pose.vecDriverFromHeadTranslation[0] = 0;
				pose.vecDriverFromHeadTranslation[1] = 0;
				pose.vecDriverFromHeadTranslation[2] = 0;

				pose.qRotation = hmdRotation;
				pose.vecPosition[0] = trackerRefPosition.v[0] + offset.v[0];
				pose.vecPosition[1] = trackerRefPosition.v[1] + offset.v[1];
				pose.vecPosition[2] = trackerRefPosition.v[2] + offset.v[2];

				if (headFilter.enabled)
				{
					double dt = FilterStep(headFilter.lastUpdate, headFilter.valid, rawTracker.targetTime);
					headFilter.valid = true;

					pose.qRotation = headFilter.rotationFilter.filter(pose.qRotation, dt);

					vr::HmdVector3d_t headPos = headFilter.translationFilter.filter(
						{ pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] }, dt);
					pose.vecPosition[0] = headPos.v[0];
					pose.vecPosition[1] = headPos.v[1];
					pose.vecPosition[2] = headPos.v[2];
				}

				double trackerVel[3] = {
					tp.vVelocity.v[0],
					tp.vVelocity.v[1],
					tp.vVelocity.v[2]
				};

				double trackerAngVel[3] = {
					tp.vAngularVelocity.v[0],
					tp.vAngularVelocity.v[1],
					tp.vAngularVelocity.v[2]
				};

				vr::HmdVector3d_t vel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerVel);
				vel.v[0] *= hmdTracker.calibrationScale;
				vel.v[1] *= hmdTracker.calibrationScale;
				vel.v[2] *= hmdTracker.calibrationScale;

				double dtAng = FilterStep(headVel.lastUpdate, headVel.valid, rawTracker.targetTime);
				vr::HmdVector3d_t headAngVel = { 0, 0, 0 };
				if (headVel.valid)
					headAngVel = headVel.filter.filter(quaternionAngularVelocity(pose.qRotation, headVel.prevRotation, dtAng), dtAng);
				headVel.prevRotation = pose.qRotation;
				headVel.valid = true;

				vr::HmdVector3d_t tangential = {
					headAngVel.v[1] * offset.v[2] - headAngVel.v[2] * offset.v[1],
					headAngVel.v[2] * offset.v[0] - headAngVel.v[0] * offset.v[2],
					headAngVel.v[0] * offset.v[1] - headAngVel.v[1] * offset.v[0]
				};

				for (int i = 0; i < 3; i++)
				{
					pose.vecVelocity[i] = vel.v[i] + tangential.v[i];
					pose.vecAngularVelocity[i] = hmdTracker.enableAngularVelocity ? headAngVel.v[i] : 0.0;
				}

				pose.poseIsValid = true;
				pose.deviceIsConnected = true;
				// Report the real tracking state.
				pose.result = trackerOK ? vr::TrackingResult_Running_OK : tp.eTrackingResult;
				pose.shouldApplyHeadModel = false;
				outputTime.target = rawTracker.targetTime;

				// Log state transitions with the current SLAM->tracker yaw.
				double relativeYaw = drift.valid ? quaternionYawDeg(drift.rotation) : 0.0;
				bool relativeYawValid = drift.valid;
				if (trackerOK && rawValid)
				{
					relativeYaw = quaternionYawDeg(quaternionProjectYaw(quaternionNormalize(pose.qRotation * quaternionConjugate(rawRotation))));
					relativeYawValid = true;
				}
				NoteTrackerState(trackerOK, false, tp, quaternionYawDeg(trackerQuat), relativeYawValid, relativeYaw);

				// Drift only learns from Running_OK poses, so a bad frame can never become the new reference.
				if (trackerOK && rawValid)
				{
					double linSpeed = sqrt(
						trackerVel[0] * trackerVel[0] +
						trackerVel[1] * trackerVel[1] +
						trackerVel[2] * trackerVel[2]);

					double angSpeed = sqrt(
						trackerAngVel[0] * trackerAngVel[0] +
						trackerAngVel[1] * trackerAngVel[1] +
						trackerAngVel[2] * trackerAngVel[2]);

					const double maxLinSpeed = 2.75;
					const double maxAngSpeed = 3.5;

					if (!drift.valid || (linSpeed < maxLinSpeed && angSpeed < maxAngSpeed))
						UpdateDrift(pose.qRotation, pose.vecPosition, rawRotation, rawPosition,
							drift.valid ? DriftSampleConfidence(linSpeed, angSpeed) : 1.0, QpcSeconds(arrival) + pose.poseTimeOffset);
				}
			}
			else {
				headVel.reset();
				trackerFilter.reset();

				NoteTrackerState(false, hmdTracker.slamFallback && drift.valid, tp, quaternionYawDeg(trackerQuat), drift.valid, drift.valid ? quaternionYawDeg(drift.rotation) : 0.0);

				if (!hmdTracker.slamFallback) {
					pose.qWorldFromDriverRotation = hmdTracker.calibrationRotation;
					pose.vecWorldFromDriverTranslation[0] = hmdTracker.calibrationTranslation.v[0];
					pose.vecWorldFromDriverTranslation[1] = hmdTracker.calibrationTranslation.v[1];
					pose.vecWorldFromDriverTranslation[2] = hmdTracker.calibrationTranslation.v[2];

					pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
					pose.vecDriverFromHeadTranslation[0] = 0;
					pose.vecDriverFromHeadTranslation[1] = 0;
					pose.vecDriverFromHeadTranslation[2] = 0;

					pose.qRotation = { 1, 0, 0, 0 };
					pose.vecPosition[0] = 0;
					pose.vecPosition[1] = 0;
					pose.vecPosition[2] = 0;

					for (int i = 0; i < 3; i++)
					{
						pose.vecVelocity[i] = 0;
						pose.vecAngularVelocity[i] = 0;
					}

					pose.poseIsValid = false;
					pose.deviceIsConnected = true;
					pose.result = vr::TrackingResult_Running_OutOfRange;
					pose.shouldApplyHeadModel = false;
					pose.poseTimeOffset = 0;
				}
				else if (drift.valid) {
					// SLAM fallback with the last good drift.
					ApplyDrift(pose);
				}
			}
		}
		else if (slamSync[openVRID] && drift.valid && !followSlam)
		{
			ApplyDrift(pose);
		}
	}

	return true;
}
