// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include "IPCServer.h"
#include "OneEuroFilter.h"
#include "KalmanFilter.h"

#include <openvr_driver.h>

#include <atomic>
#include <mutex>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetHmdTracker(const protocol::SetHmdTracker &cmd);
	void SetSlamSync(const protocol::SetSlamSync &cmd);
	void SetOneEuro(const protocol::SetOneEuro &cmd);
	void GetStatus(protocol::DriverStatus &status);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	// confidence 0..1 = how much this sample may move the drift estimate.
	void UpdateDrift(const vr::HmdQuaternion_t &correctedRotation, const double (&correctedPosition)[3],
		const vr::HmdQuaternion_t &rawRotation, const double (&rawPosition)[3], double confidence);

	// Drift changes slowly, but fast head motion makes a single sample unreliable, so weight by speed.
	static double DriftSampleConfidence(double linSpeed, double angSpeed)
	{
		// Fast head motion: don't touch the drift at all, it gets re-measured once the head is calm.
		const double linFreeze = 1.0;  // m/s
		const double angFreeze = 1.0;  // rad/s
		if (linSpeed > linFreeze || angSpeed > angFreeze)
			return 0.0;

		const double linRef = 0.5;   // m/s   -> weight 0.5
		const double angRef = 0.4;   // rad/s -> weight 0.5
		double l = linSpeed / linRef, a = angSpeed / angRef;
		return 1.0 / (1.0 + l * l + a * a);
	}

	// Leftover time offset between headset pose and tracker pose (streamers pre-predict), learned
	// while the head turns: yaw residual r ~= omega * (tau - tau_true), so nudge tau by -r/omega.
	struct LatencyEstimate
	{
		double tauRot = 0.0;
		double tauPos = 0.0;
		bool primed = false;
		LARGE_INTEGER lastUpdate = {};

		void reset() { tauRot = 0.0; tauPos = 0.0; primed = false; }
	} latency;
	void ApplyDrift(vr::DriverPose_t &pose) const;
	void ApplyInverseDrift(vr::DriverPose_t &pose) const;
	void NoteTrackerState(bool trackerOK, bool usingSlam, const vr::TrackedDevicePose_t &tp,
		double trackerYawDeg, bool relativeYawValid, double relativeYawDeg);

	// Raw head tracker pose grabbed in the hook. Follow mode needs it because the tracker is
	// re-aligned in vrserver and can't be read back raw. Lighthouse thread writes, HMD thread reads.
	struct TrackerSample
	{
		bool valid = false;
		bool poseIsValid = false;
		bool deviceIsConnected = false;
		vr::ETrackingResult result = vr::TrackingResult_Uninitialized;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
		double velocity[3] = { 0, 0, 0 };         // world space, m/s
		double angularVelocity[3] = { 0, 0, 0 };  // world space, rad/s (axis-angle rate)
		double linSpeed = 0.0;
		double angSpeed = 0.0;
		double poseTimeOffset = 0.0;              // seconds, as reported by the driver
		LARGE_INTEGER time = {};                  // when the sample was received
	};
	std::mutex trackerSampleMutex;
	TrackerSample trackerSample;

	void StoreTrackerSample(const vr::DriverPose_t &pose);
	TrackerSample LoadTrackerSample();

	double SlamToCorrectedScale() const
	{
		double k = hmdTracker.hmdScale > 0.0 ? 1.0 / hmdTracker.hmdScale : 1.0;
		return hmdTracker.native ? k : k * hmdTracker.calibrationScale;
	}

	IPCServer server;

	struct DeviceTransform
	{
		bool enabled = false;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];

	struct HmdTracker
	{
		// Written last / read first so the pose thread never sees a half-written config.
		std::atomic<bool> enabled{ false };
		bool native = false;
		// Follow SLAM HMD: headset keeps its SLAM pose, lighthouse devices follow it.
		bool followSlam = false;
		bool slamFallback = true;
		bool enableAngularVelocity = false;
		float predictionTime = 1.0f;
		uint32_t hmdID = vr::k_unTrackedDeviceIndex_Hmd;
		uint32_t trackerID = vr::k_unTrackedDeviceIndexInvalid;
		vr::HmdQuaternion_t offsetRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t offsetTranslation = { 0, 0, 0 };
		vr::HmdQuaternion_t calibrationRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t calibrationTranslation = { 0, 0, 0 };
		double calibrationScale = 1.0;
		double hmdScale = 1.0;
	} hmdTracker;

	bool slamSync[vr::k_unMaxTrackedDeviceCount];

	struct DriftCorrection
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };

		LARGE_INTEGER lastUpdate = {};
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;
	} drift;

	struct HeadFilter
	{
		bool enabled = false;
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;

		void reset() { valid = false; rotationFilter.reset(); translationFilter.reset(); }
	} headFilter;

	struct TrackerFilter
	{
		KalmanFilterXYZ translation;
		void reset() { translation.reset(); }
	} trackerFilter;

	struct HeadVelocity
	{
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		vr::HmdQuaternion_t prevRotation = { 1, 0, 0, 0 };
		oneeuro::Vec3 filter;

		void reset() { valid = false; filter.reset(); }
	} headVel;

	// Only for logging tracker state changes (OK <-> lost) with the yaw numbers around them.
	struct TrackerState
	{
		bool primed = false;
		bool wasOK = false;
		bool wasFallback = false;
		vr::ETrackingResult lastResult = vr::TrackingResult_Uninitialized;
		LARGE_INTEGER lostAt = {};
		double trackerYawAtLoss = 0.0;      // raw tracker yaw (deg)
		double relativeYawAtLoss = 0.0;     // drift yaw (deg)
		bool relativeYawAtLossValid = false;

		void reset() { primed = false; wasOK = false; wasFallback = false; lastResult = vr::TrackingResult_Uninitialized; relativeYawAtLossValid = false; }
	} trackerState;

	struct JumpDetector
	{
		int pending = 0;
		vr::HmdQuaternion_t firstRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t firstTranslation = { 0, 0, 0 };
		uint32_t compensated = 0;

		void reset() { pending = 0; }
	} jump;

	struct MountCheck
	{
		bool primed = false;
		double tiltDeg = 0.0;
		double translationDeviation = 0.0;
		vr::HmdVector3d_t meanTranslation = { 0, 0, 0 };
		bool suspected = false;

		void reset() { primed = false; tiltDeg = 0.0; translationDeviation = 0.0; suspected = false; }
	} mount;

	// Last logged drift, so a jump of the SLAM<->lighthouse relation shows up in the log.
	struct DriftLog
	{
		bool primed = false;
		double yawDeg = 0.0;
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		double tauMs = 0.0;
		LARGE_INTEGER lastLog = {};

		void reset() { primed = false; }
	} driftLog;
};