// SPDX-License-Identifier: AGPL-3.0-only

#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include "Version.h"

#include <cmath>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

inline vr::HmdQuaternion_t quaternionNormalize(vr::HmdQuaternion_t q) {
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 0.0) {
		q.w /= n; q.x /= n; q.y /= n; q.z /= n;
	}
	return q;
}

inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& q) {
	return { q.w, -q.x, -q.y, -q.z };
}

inline vr::HmdQuaternion_t quaternionProjectYaw(const vr::HmdQuaternion_t& q) {
	double n = std::sqrt(q.w * q.w + q.y * q.y);
	if (n < 1e-9)
		return { 1, 0, 0, 0 };
	return { q.w / n, 0.0, q.y / n, 0.0 };
}

inline vr::HmdVector3d_t quaternionAngularVelocity(const vr::HmdQuaternion_t& cur, const vr::HmdQuaternion_t& prev, double dt) {
	if (dt <= 0.0)
		return { 0, 0, 0 };

	vr::HmdQuaternion_t d = quaternionNormalize(cur * quaternionConjugate(prev));
	if (d.w < 0.0) { d.w = -d.w; d.x = -d.x; d.y = -d.y; d.z = -d.z; }

	double s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
	if (s < 1e-9)
		return { 0, 0, 0 };

	double scale = (2.0 * std::atan2(s, d.w)) / (s * dt);
	return { d.x * scale, d.y * scale, d.z * scale };
}

// Yaw of the forward (-Z) axis in degrees, just for logging.
inline double quaternionYawDeg(const vr::HmdQuaternion_t& q) {
	double fwd[3] = { 0.0, 0.0, -1.0 };
	vr::HmdVector3d_t f = quaternionRotateVector(q, fwd);
	return std::atan2(-f.v[0], -f.v[2]) * 180.0 / 3.14159265358979323846;
}

inline double wrapDeg(double d) {
	while (d > 180.0) d -= 360.0;
	while (d < -180.0) d += 360.0;
	return d;
}

// Rotation from a constant angular velocity over dt.
inline vr::HmdQuaternion_t quaternionFromAngularVelocity(const double(&omega)[3], double dt) {
	double wx = omega[0] * dt, wy = omega[1] * dt, wz = omega[2] * dt;
	double angle = std::sqrt(wx * wx + wy * wy + wz * wz);
	if (angle < 1e-12)
		return { 1, 0, 0, 0 };
	double s = std::sin(angle * 0.5) / angle;
	return { std::cos(angle * 0.5), wx * s, wy * s, wz * s };
}

template < class T >
inline vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
	vr::HmdQuaternion_t q{};

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

static double FilterStep(LARGE_INTEGER& lastUpdate, bool primed)
{
	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	double dt = primed ? (now.QuadPart - lastUpdate.QuadPart) / (double)freq.QuadPart : 0.0;
	lastUpdate = now;
	if (dt <= 0.0 || isnan(dt)) dt = 1.0 / 90.0;
	if (dt > 0.1) dt = 0.1;
	return dt;
}

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	OpenLogFile();
	LOG("SpaceSync driver " SPACECAL_VERSION_STRING " loaded");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);

	drift.rotationFilter.params = { 3.0, 1.3, 0.6 };
	drift.translationFilter.params = { 3.0, 1.3, 0.6 };
	headFilter.rotationFilter.params = { 5.0, 0.8, 1.0 };
	headFilter.translationFilter.params = { 5.0, 0.8, 1.0 };
	headVel.filter.params = { 8.0, 1.0, 1.0 };

	trackerFilter.translation.SetQ(2.5e-7);
  	trackerFilter.translation.SetR(1.0e-5);
	trackerFilter.translation.SetAdaptiveGain(4.0);

	InjectHooks(pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	LOG("SpaceSync driver unloaded");
	CloseLogFile();

	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;

	auto& tf = transforms[newTransform.openVRID];

	// Values first, enabled last, so the pose thread never sees enabled with a half-written transform.
	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;

	tf.enabled = newTransform.enabled;
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& cmd)
{
	// Disable first, enable last, so the pose thread never runs with a half-written config.
	if (!cmd.enabled)
		hmdTracker.enabled.store(false, std::memory_order_release);

	hmdTracker.followSlam = cmd.followSlamHmd;
	// Native mode only makes sense while the tracker drives the headset.
	hmdTracker.native = cmd.native && !cmd.followSlamHmd;
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
	hmdTracker.hmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.rotationFilter.reset();
		drift.translationFilter.reset();
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		trackerState.reset();
		driftLog.reset();
		latency.reset();
		{
			std::lock_guard<std::mutex> lock(trackerSampleMutex);
			trackerSample.valid = false;
		}
		memset(slamSync, 0, sizeof slamSync);
	}
	else
	{
		hmdTracker.enabled.store(true, std::memory_order_release);
	}
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
		slamSync[cmd.openVRID] = cmd.enabled;
}

void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = p.minCutoff < 0.01 ? 0.01 : p.minCutoff;
		out.beta = p.beta < 0.0 ? 0.0 : p.beta;
		out.dCutoff = p.dCutoff < 0.01 ? 0.01 : p.dCutoff;
		return out;
	};

	headFilter.rotationFilter.params = toParams(cmd.head);
	headFilter.translationFilter.params = toParams(cmd.head);
	drift.rotationFilter.params = toParams(cmd.drift);
	drift.translationFilter.params = toParams(cmd.drift);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3], double confidence)
{
	vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(correctedRotation * quaternionConjugate(rawRotation)));

	double dt = FilterStep(drift.lastUpdate, drift.valid);

	// Filter the rotation first and derive the translation from the filtered rotation,
	// otherwise rotation and translation don't fit together while the filter settles.
	drift.rotation = drift.rotationFilter.filter(instRot, dt, confidence);

	vr::HmdVector3d_t rotatedRaw = quaternionRotateVector(drift.rotation, rawPosition);

	double slamScale = SlamToCorrectedScale();
	vr::HmdVector3d_t instTrans = {
		correctedPosition[0] - rotatedRaw.v[0] * slamScale,
		correctedPosition[1] - rotatedRaw.v[1] * slamScale,
		correctedPosition[2] - rotatedRaw.v[2] * slamScale
	};

	drift.translation = drift.translationFilter.filter(instTrans, dt, confidence);
	drift.valid = true;

	// Log when the drift jumps (>1 deg yaw or >5 cm), max once a second. Makes silent SLAM re-localisations visible.
	double yawDeg = quaternionYawDeg(drift.rotation);
	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);
	double tauMs = latency.tau * 1000.0;
	if (!driftLog.primed)
	{
		driftLog.primed = true;
		driftLog.yawDeg = yawDeg;
		driftLog.translation = drift.translation;
		driftLog.tauMs = tauMs;
		driftLog.lastLog = now;
		LOG("Drift (SLAM->tracker) initialised: yaw %.2f deg, translation (%.3f, %.3f, %.3f) m, latency offset %.1f ms",
			yawDeg, drift.translation.v[0], drift.translation.v[1], drift.translation.v[2], tauMs);
	}
	else
	{
		double dYaw = wrapDeg(yawDeg - driftLog.yawDeg);
		double dx = drift.translation.v[0] - driftLog.translation.v[0];
		double dy = drift.translation.v[1] - driftLog.translation.v[1];
		double dz = drift.translation.v[2] - driftLog.translation.v[2];
		double dTrans = std::sqrt(dx * dx + dy * dy + dz * dz);
		double dTau = tauMs - driftLog.tauMs;
		double sinceLog = (now.QuadPart - driftLog.lastLog.QuadPart) / (double)freq.QuadPart;

		if ((std::fabs(dYaw) > 1.0 || dTrans > 0.05 || std::fabs(dTau) > 5.0) && sinceLog > 1.0)
		{
			LOG("Drift (SLAM->tracker) changed: yaw %.2f -> %.2f deg (delta %.2f), translation (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f) m (delta %.1f cm), latency offset %.1f -> %.1f ms",
				driftLog.yawDeg, yawDeg, dYaw,
				driftLog.translation.v[0], driftLog.translation.v[1], driftLog.translation.v[2],
				drift.translation.v[0], drift.translation.v[1], drift.translation.v[2], dTrans * 100.0,
				driftLog.tauMs, tauMs);
			driftLog.yawDeg = yawDeg;
			driftLog.translation = drift.translation;
			driftLog.tauMs = tauMs;
			driftLog.lastLog = now;
		}
	}
}

void ServerTrackedDeviceProvider::ApplyDrift(vr::DriverPose_t& pose) const
{
	double slamScale = SlamToCorrectedScale();

	pose.qWorldFromDriverRotation = quaternionNormalize(drift.rotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;

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

void ServerTrackedDeviceProvider::StoreTrackerSample(const vr::DriverPose_t& pose)
{
	TrackerSample s;
	s.valid = true;
	s.poseIsValid = pose.poseIsValid;
	s.deviceIsConnected = pose.deviceIsConnected;
	s.result = pose.result;

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
	for (int i = 0; i < 3; i++)
	{
		s.velocity[i] = vel.v[i];
		s.angularVelocity[i] = angVel.v[i];
	}
	s.linSpeed = std::sqrt(vel.v[0] * vel.v[0] + vel.v[1] * vel.v[1] + vel.v[2] * vel.v[2]);
	s.angSpeed = std::sqrt(angVel.v[0] * angVel.v[0] + angVel.v[1] * angVel.v[1] + angVel.v[2] * angVel.v[2]);
	s.poseTimeOffset = pose.poseTimeOffset;
	QueryPerformanceCounter(&s.time);

	std::lock_guard<std::mutex> lock(trackerSampleMutex);
	trackerSample = s;
}

ServerTrackedDeviceProvider::TrackerSample ServerTrackedDeviceProvider::LoadTrackerSample()
{
	std::lock_guard<std::mutex> lock(trackerSampleMutex);
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

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	const bool overrideEnabled = hmdTracker.enabled.load(std::memory_order_acquire);
	const bool followSlam = overrideEnabled && hmdTracker.followSlam;

	// Follow mode: stash the raw head tracker pose before any transform touches it.
	if (followSlam && openVRID == hmdTracker.trackerID)
		StoreTrackerSample(pose);

	auto& tf = transforms[openVRID];
	if (tf.enabled && !hmdTracker.native)
	{
		pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];

		// Follow mode: move the calibrated lighthouse device into the headset's SLAM space.
		if (followSlam && drift.valid)
			ApplyInverseDrift(pose);
	}

	if (overrideEnabled)
	{
		if (openVRID == hmdTracker.hmdID)
		{
			bool rawValid = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
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

				LARGE_INTEGER now, freq;
				QueryPerformanceCounter(&now);
				QueryPerformanceFrequency(&freq);
				double age = ts.valid ? (now.QuadPart - ts.time.QuadPart) / (double)freq.QuadPart : 1e9;

				const bool trackerHasPose = ts.valid && age < 0.25 && ts.deviceIsConnected && ts.poseIsValid;
				const bool trackerOK = trackerHasPose && ts.result == vr::TrackingResult_Running_OK;

				vr::TrackedDevicePose_t tpLog = {};
				tpLog.bPoseIsValid = trackerHasPose;
				tpLog.bDeviceIsConnected = ts.valid && ts.deviceIsConnected;
				tpLog.eTrackingResult = ts.valid ? ts.result : vr::TrackingResult_Uninitialized;

				double relativeYaw = drift.valid ? quaternionYawDeg(drift.rotation) : 0.0;
				bool relativeYawValid = drift.valid;

				if (trackerOK && rawValid)
				{
					// Predict the tracker sample to the headset pose's time (reported offsets + learned tau),
					// otherwise head motion leaks into the drift as speed x time gap.
					double dtAlign = (pose.poseTimeOffset) - (ts.poseTimeOffset - age) + latency.tau;
					if (dtAlign > 0.30) dtAlign = 0.30;
					if (dtAlign < -0.10) dtAlign = -0.10;

					vr::HmdQuaternion_t sampleRotation = quaternionNormalize(quaternionFromAngularVelocity(ts.angularVelocity, dtAlign) * ts.rotation);
					double samplePosition[3] = {
						ts.position[0] + ts.velocity[0] * dtAlign,
						ts.position[1] + ts.velocity[1] * dtAlign,
						ts.position[2] + ts.velocity[2] * dtAlign
					};

					vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * sampleRotation);
					vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, samplePosition);
					trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
					trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
					trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

					vr::HmdQuaternion_t headRotation = quaternionNormalize(trackerRefRotation * hmdTracker.offsetRotation);
					vr::HmdVector3d_t offset = quaternionRotateVector(trackerRefRotation, hmdTracker.offsetTranslation.v);
					double headPosition[3] = {
						trackerRefPosition.v[0] + offset.v[0],
						trackerRefPosition.v[1] + offset.v[1],
						trackerRefPosition.v[2] + offset.v[2]
					};

					vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(headRotation * quaternionConjugate(rawRotation)));
					relativeYaw = quaternionYawDeg(instRot);
					relativeYawValid = true;

					// Learn the leftover time offset from head yaw motion (see LatencyEstimate).
					vr::HmdVector3d_t angVelCal = quaternionRotateVector(hmdTracker.calibrationRotation, ts.angularVelocity);
					double omegaYaw = angVelCal.v[1];

					double dtLat = FilterStep(latency.lastUpdate, latency.primed);
					latency.primed = true;

					if (drift.valid && std::fabs(omegaYaw) > 0.5)
					{
						// residual between instantaneous and filtered drift yaw
						vr::HmdQuaternion_t rel = quaternionNormalize(instRot * quaternionConjugate(drift.rotation));
						double r = 2.0 * std::atan2(rel.y, rel.w);
						if (r > 3.14159265358979323846) r -= 2.0 * 3.14159265358979323846;
						if (r < -3.14159265358979323846) r += 2.0 * 3.14159265358979323846;

						// big residuals are re-localisations, not latency - skip them
						if (std::fabs(r) < 0.35)
						{
							double step = -(r / omegaYaw);            // how far off tau still is
							if (step > 0.1) step = 0.1;
							if (step < -0.1) step = -0.1;
							latency.tau += step * (dtLat / 3.0);      // settles after ~3 s of head motion
							if (latency.tau > 0.25) latency.tau = 0.25;
							if (latency.tau < -0.05) latency.tau = -0.05;
						}
					}

					const double maxLinSpeed = 2.75;
					const double maxAngSpeed = 3.5;
					if (!drift.valid || (ts.linSpeed < maxLinSpeed && ts.angSpeed < maxAngSpeed))
						UpdateDrift(headRotation, headPosition, rawRotation, rawPosition,
							drift.valid ? DriftSampleConfidence(ts.linSpeed, ts.angSpeed) : 1.0);
				}

				NoteTrackerState(trackerOK, true, tpLog, quaternionYawDeg(ts.rotation), relativeYawValid, relativeYaw);
				return true;
			}

			vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);

			float displayFrequency = vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
			if (!(displayFrequency > 0.0f))
				displayFrequency = 90.0f;

			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses((float)((1.0 / displayFrequency) * hmdTracker.predictionTime), poses, vr::k_unMaxTrackedDeviceCount);

			vr::TrackedDevicePose_t tp = {};
			if (hmdTracker.trackerID < vr::k_unMaxTrackedDeviceCount)
				tp = poses[hmdTracker.trackerID];

			// Lighthouse keeps bPoseIsValid true while dead-reckoning on the IMU (OutOfRange),
			// only Running_OK is a real optical pose.
			const bool trackerHasPose = tp.bDeviceIsConnected && tp.bPoseIsValid;
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

				vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, trackerPos);

				trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
				trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
				trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

				vr::HmdQuaternion_t hmdRotation = quaternionNormalize(hmdTracker.native ? trackerQuat * hmdTracker.offsetRotation : trackerRefRotation * hmdTracker.offsetRotation);
				vr::HmdVector3d_t offset = quaternionRotateVector(hmdTracker.native ? trackerQuat : trackerRefRotation, hmdTracker.offsetTranslation.v);

				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;

				pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
				pose.vecDriverFromHeadTranslation[0] = 0;
				pose.vecDriverFromHeadTranslation[1] = 0;
				pose.vecDriverFromHeadTranslation[2] = 0;

				if (hmdTracker.native) {
					pose.qRotation = hmdRotation;
					pose.vecPosition[0] = trackerPos[0] + offset.v[0];
					pose.vecPosition[1] = trackerPos[1] + offset.v[1];
					pose.vecPosition[2] = trackerPos[2] + offset.v[2];
				}
				else {
					pose.qRotation = hmdRotation;
					pose.vecPosition[0] = trackerRefPosition.v[0] + offset.v[0];
					pose.vecPosition[1] = trackerRefPosition.v[1] + offset.v[1];
					pose.vecPosition[2] = trackerRefPosition.v[2] + offset.v[2];
				}

				if (headFilter.enabled)
				{
					double dt = FilterStep(headFilter.lastUpdate, headFilter.valid);
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

				double dtAng = FilterStep(headVel.lastUpdate, headVel.valid);
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
					double baseVel = hmdTracker.native ? trackerVel[i] : vel.v[i];
					pose.vecVelocity[i] = baseVel + tangential.v[i];
					pose.vecAngularVelocity[i] = hmdTracker.enableAngularVelocity ? headAngVel.v[i] : 0.0;
				}

				pose.poseIsValid = true;
				pose.deviceIsConnected = true;
				// Report the real tracking state.
				pose.result = trackerOK ? vr::TrackingResult_Running_OK : tp.eTrackingResult;
				pose.shouldApplyHeadModel = false;
				pose.poseTimeOffset = 0;

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
							drift.valid ? DriftSampleConfidence(linSpeed, angSpeed) : 1.0);
				}
			}
			else {
				headVel.reset();
				trackerFilter.reset();

				NoteTrackerState(false, hmdTracker.slamFallback && drift.valid, tp, quaternionYawDeg(trackerQuat), drift.valid, drift.valid ? quaternionYawDeg(drift.rotation) : 0.0);

				if (!hmdTracker.slamFallback) {
					if (hmdTracker.native) {
						pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
						pose.vecWorldFromDriverTranslation[0] = 0;
						pose.vecWorldFromDriverTranslation[1] = 0;
						pose.vecWorldFromDriverTranslation[2] = 0;
					}
					else {
						pose.qWorldFromDriverRotation = hmdTracker.calibrationRotation;
						pose.vecWorldFromDriverTranslation[0] = hmdTracker.calibrationTranslation.v[0];
						pose.vecWorldFromDriverTranslation[1] = hmdTracker.calibrationTranslation.v[1];
						pose.vecWorldFromDriverTranslation[2] = hmdTracker.calibrationTranslation.v[2];
					}

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
