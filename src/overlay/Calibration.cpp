// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#define WIN32_LEAN_AND_MEAN

#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "Sound.h"

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <array>

#include <Dense>


static IPCClient Driver;
CalibrationContext CalCtx;

void InitCalibrator()
{
	Driver.Connect();
}

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i,j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x,y,z)) { }
};

struct Sample
{
	Pose ref, target;
	bool valid;
	Sample() : valid(false) { }
	Sample(Pose ref, Pose target) : valid(true), ref(ref), target(target) { }
};

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rot)
{
	Eigen::AngleAxisd aa(rot);
	return aa.angle() * aa.axis();
}

double AngleFromRotationMatrix3(const Eigen::Matrix3d& rot)
{
	double c = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0;
	return acos(max(-1.0, min(1.0, c)));
}

struct DetectionState
{
	std::vector<uint32_t> candidates;
	std::vector<std::vector<double>> candidateSpeeds;
	std::vector<double> hmdSpeeds;
	std::vector<Eigen::Matrix3d> prevRot; // [0] = HMD, [i+1] = candidates[i]
	bool havePrev = false;
	double prevTime = 0;

	void Clear()
	{
		candidates.clear();
		candidateSpeeds.clear();
		hmdSpeeds.clear();
		prevRot.clear();
		havePrev = false;
		prevTime = 0;
	}
};

static DetectionState Detection;

static std::string GetDeviceSerial(uint32_t id)
{
	char serial[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, vr::k_unMaxPropertyStringSize);
	return std::string(serial);
}

static std::string GetDeviceTrackingSystem(uint32_t id)
{
	char system[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, system, vr::k_unMaxPropertyStringSize);
	return std::string(system);
}

static std::string GetDeviceModelNumber(uint32_t id)
{
	char model[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, model, vr::k_unMaxPropertyStringSize);
	return std::string(model);
}

struct ModelScaleEntry
{
	const char *pattern;
	double scale;
};

static const ModelScaleEntry ModelScales[] = {
	{ "tundra tracker",           0.9969 	},  	// Tundra Tracker
	{ "vive tracker 3.0 mv",      1.0034 	},  	// HTC Vive Tracker 3.0
	{ "vive tracker mv",          1.00585 	}, 		// HTC Vive Tracker 1.0 / 2018
};

static double GetLighthouseModelScale(uint32_t id)
{
	if (id == vr::k_unTrackedDeviceIndexInvalid)
		return 0.0;

	std::string model = GetDeviceModelNumber(id);
	std::transform(model.begin(), model.end(), model.begin(),
		[](unsigned char c) { return (char)std::tolower(c); });

	for (auto &entry : ModelScales)
	{
		if (model.find(entry.pattern) != std::string::npos)
			return entry.scale;
	}

	return 0.0;
}

static double AngularSpeedBetween(const Eigen::Matrix3d &cur, const Eigen::Matrix3d &prev, double dt)
{
	Eigen::Matrix3d delta = cur * prev.transpose();
	double c = (delta(0,0) + delta(1,1) + delta(2,2) - 1.0) / 2.0;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return acos(c) / dt;
}

static double PearsonCorrelation(const std::vector<double> &a, const std::vector<double> &b)
{
	if (a.size() != b.size() || a.empty())
		return 0.0;

	double meanA = 0, meanB = 0;
	for (size_t i = 0; i < a.size(); i++) { meanA += a[i]; meanB += b[i]; }
	meanA /= a.size();
	meanB /= b.size();

	double cov = 0, varA = 0, varB = 0;
	for (size_t i = 0; i < a.size(); i++)
	{
		double da = a[i] - meanA, db = b[i] - meanB;
		cov += da * db;
		varA += da * da;
		varB += db * db;
	}

	if (varA < 1e-9 || varB < 1e-9)
		return 0.0;

	return cov / std::sqrt(varA * varB);
}

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = RotationVector(dref);
	ds.target = RotationVector(dtarget);

	// Reject samples that were too close to each other.
	auto refA = AngleFromRotationMatrix3(dref);
	auto targetA = AngleFromRotationMatrix3(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	return ds;
}

Eigen::Vector3d CalibrateRotation(const std::vector<Sample>& samples)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	CalCtx.Log(buf);
	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		targetPoints.row(i) = deltas[i].target;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
	{
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	CalCtx.Log(buf);
	return euler;
}

static const double ScaleSpreadThreshold = 0.5;
static const double MinCalibratedScale = 0.97;
static const double MaxCalibratedScale = 1.03;
static constexpr double MaxCalibrationPositionMeters = 100.0;
static constexpr double MaxMountMeters = 2.0;

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation, double scale)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		Sample s_i = samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = scale * (rotation * s_i.target.trans);

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = scale * (rotation * s_j.target.trans);

			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CB, dQB));
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	CalCtx.Log(buf);
	return transcm;
}

static double EstimateHmdSpaceScale(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation, double targetModelScale, bool *measured = nullptr)
{
	if (measured) *measured = false;
	if (samples.empty()) return 1.0;
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for (auto &sample : samples)
		centroid += rotation * sample.target.trans;
	centroid /= (double)samples.size();

	double spread = 0;
	for (auto &sample : samples)
		spread += (rotation * sample.target.trans - centroid).squaredNorm();
	spread = std::sqrt(spread / (double)samples.size());

	char buf[256];
	if (spread < ScaleSpreadThreshold)
	{
		snprintf(buf, sizeof buf, "Headset scale assumed 1.000: movement spread %.2f m is below %.1f m.\n", spread, ScaleSpreadThreshold);
		CalCtx.Log(buf);
		return 1.0;
	}

	Eigen::MatrixXd coefficients(samples.size() * 3, 7);
	Eigen::VectorXd constants(samples.size() * 3);

	for (size_t i = 0; i < samples.size(); i++)
	{
		Eigen::Vector3d rotatedPos = rotation * samples[i].target.trans;
		Eigen::Matrix3d rotatedRot = rotation * samples[i].target.rot;

		coefficients.block<3, 1>(i * 3, 0) = rotatedPos;
		coefficients.block<3, 3>(i * 3, 1) = Eigen::Matrix3d::Identity();
		coefficients.block<3, 3>(i * 3, 4) = rotatedRot;
		constants.segment<3>(i * 3) = samples[i].ref.trans;
	}

	auto decomposition = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	// Float pose noise can make a physically singular scale/mount system appear
	// full-rank at Eigen's machine-epsilon default. Require a usable separation.
	decomposition.setThreshold(1e-4);
	if (decomposition.rank() < 7)
	{
		CalCtx.Log("Headset scale is not independently observable, assuming 1\n");
		return 1.0;
	}
	Eigen::VectorXd result = decomposition.solve(constants);
	double fittedScale = result(0);

	if (!std::isfinite(fittedScale) || fittedScale < MinCalibratedScale || fittedScale > MaxCalibratedScale)
	{
		snprintf(buf, sizeof buf, "Fitted headset scale %.5f is outside %.2f..%.2f, assuming 1\n", fittedScale, MinCalibratedScale, MaxCalibratedScale);
		CalCtx.Log(buf);
		return 1.0;
	}

	snprintf(buf, sizeof buf, "Fitted headset space scale relative to lighthouse: %.5f (%+.2f%%) from %.2f m of movement, implied absolute headset scale: %.5f\n",
		fittedScale, (fittedScale - 1.0) * 100.0, spread, fittedScale / targetModelScale);
	CalCtx.Log(buf);
	if (measured) *measured = true;
	return fittedScale;
}

// Relative rotation vectors represent physical axes. Quaternion covariance also
// measures the curvature of a one-axis arc and rejects ordinary yaw/pitch sweeps.
static bool HasObservableMotion(const std::vector<Sample> &samples)
{
	Eigen::Matrix3d refAxes = Eigen::Matrix3d::Zero(), targetAxes = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d refTranslation = Eigen::Matrix3d::Zero(), targetTranslation = Eigen::Matrix3d::Zero();
	size_t accepted = 0;
	for (size_t i = 0; i < samples.size(); ++i)
		for (size_t j = 0; j < i; ++j)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (!delta.valid) continue;
			refAxes += delta.ref * delta.ref.transpose();
			targetAxes += delta.target * delta.target.transpose();
			Eigen::Matrix3d a = samples[i].ref.rot.transpose() - samples[j].ref.rot.transpose();
			Eigen::Matrix3d b = samples[i].target.rot.transpose() - samples[j].target.rot.transpose();
			refTranslation += a.transpose() * a;
			targetTranslation += b.transpose() * b;
			++accepted;
		}
	if (accepted < 20) return false;
	auto observable = [&](const Eigen::Matrix3d &information, int requiredAxis) {
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(information / static_cast<double>(accepted));
		if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) return false;
		const auto &values = solver.eigenvalues();
		return values(requiredAxis) > 0.001 && values(requiredAxis) > values(2) * 0.01;
	};
	// Two nonparallel physical rotation axes and a full-rank translation solve
	// must be observable in both input spaces, irrespective of their alignment.
	return observable(refAxes, 1) && observable(targetAxes, 1)
		&& observable(refTranslation, 0) && observable(targetTranslation, 0);
}

static std::string MissingMotionGuidance(const std::vector<Sample> &samples)
{
	// Relative to the first headset pose, local X is nodding and Y is turning.
	// These cues supplement the sequence; the full information test decides
	// acceptance and also handles arbitrary tracker mounting orientations.
	Eigen::Vector3d extent = Eigen::Vector3d::Zero();
	if (!samples.empty())
		for (const auto &sample : samples)
			extent = extent.cwiseMax(RotationVector(samples.front().ref.rot.transpose() * sample.ref.rot).cwiseAbs());
	if (extent.y() > 0.4 && extent.x() < 0.4)
		return "Add gentle nods: look up and down.";
	if (extent.x() > 0.4 && extent.y() < 0.4)
		return "Add gentle turns: look left and right.";
	return "Turn left and right, then look up and down.";
}

static Eigen::Vector3d ComputeRefToTargetOffset(const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calScale * (calRot * sample.target.trans) + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	double accum = 0;

	for (auto &sample : samples)
		accum += (calScale * (calRot * sample.target.trans) + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

Sample CollectSample(const CalibrationContext &ctx)
{
	if (ctx.targetID >= vr::k_unMaxTrackedDeviceCount)
		return Sample();
	auto usable = [](const vr::TrackedDevicePose_t &pose) {
		if (!pose.bPoseIsValid || !pose.bDeviceIsConnected || pose.eTrackingResult != vr::TrackingResult_Running_OK)
			return false;
		Pose p(pose.mDeviceToAbsoluteTracking);
		return p.rot.allFinite() && p.trans.allFinite() && p.trans.cwiseAbs().maxCoeff() <= MaxCalibrationPositionMeters
			&& (p.rot.transpose() * p.rot - Eigen::Matrix3d::Identity()).norm() < 0.01
			&& std::abs(p.rot.determinant() - 1.0) < 0.01;
	};
	const auto &reference = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	const auto &target = ctx.devicePoses[ctx.targetID];
	if (!usable(reference) || !usable(target))
		return Sample();
	return Sample(Pose(reference.mDeviceToAbsoluteTracking), Pose(target.mDeviceToAbsoluteTracking));
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat =
		Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	vr::HmdQuaternion_t vrRotQuat;
	vrRotQuat.x = rotQuat.coeffs()[0];
	vrRotQuat.y = rotQuat.coeffs()[1];
	vrRotQuat.z = rotQuat.coeffs()[2];
	vrRotQuat.w = rotQuat.coeffs()[3];
	return vrRotQuat;
}

vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
{
	auto trans = transcm * 0.01;
	vr::HmdVector3d_t vrTrans;
	vrTrans.v[0] = trans[0];
	vrTrans.v[1] = trans[1];
	vrTrans.v[2] = trans[2];
	return vrTrans;
}

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	Driver.SendBlocking(req);
}

void SendOneEuroParams()
{
	protocol::Request req(protocol::RequestSetOneEuro);
	req.setOneEuro.headEnabled = CalCtx.headFilterEnabled;
	req.setOneEuro.head = CalCtx.headFilterParams;
	req.setOneEuro.drift = CalCtx.driftFilterParams;

	try
	{
		Driver.SendBlocking(req);
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to send One Euro params: " << e.what() << std::endl;
	}
}

void SendHmdTrackerCommand(uint32_t hmdID, uint32_t trackerID, bool enabled)
{
	protocol::Request req(protocol::RequestSetHmdTracker);
	req.setHmdTracker.hmdID = hmdID;
	req.setHmdTracker.trackerID = trackerID;
	req.setHmdTracker.enabled = enabled;
	req.setHmdTracker.slamFallback = CalCtx.fallbackToSlam;
	req.setHmdTracker.predictionTime = CalCtx.predictionTime;
	req.setHmdTracker.enableAngularVelocity = CalCtx.enableAngularVelocity;
	req.setHmdTracker.offsetRotation = CalCtx.relativeRotation;
	req.setHmdTracker.offsetTranslation = CalCtx.relativeTranslation;
	req.setHmdTracker.calibrationRotation = VRRotationQuat(CalCtx.calibratedRotation);
	req.setHmdTracker.calibrationTranslation = VRTranslationVec(CalCtx.calibratedTranslation);
	req.setHmdTracker.calibrationScale = CalCtx.calibratedScale;
	req.setHmdTracker.hmdScale = CalCtx.hmdScale;
	req.setHmdTracker.followSlamHmd = CalCtx.followSlamHmd;
	// Calibration reads the tracker back through SteamVR.
	req.setHmdTracker.hideHeadTracker = CalCtx.hideHeadTracker && CalCtx.state == CalibrationState::None;
	Driver.SendBlocking(req);
}

// https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/27410865
void ComputeRelativeOffset(CalibrationContext &ctx, const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	if (samples.empty())
		return;

	Eigen::Matrix4d quatAccum = Eigen::Matrix4d::Zero();
	Eigen::Vector3d transAccum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
		Eigen::Vector3d trackerTrans = calScale * (calRot * sample.target.trans) + calTrans;

		Eigen::Matrix3d offsetRot = trackerRot.transpose() * sample.ref.rot;
		Eigen::Vector3d offsetTrans = trackerRot.transpose() * (sample.ref.trans - trackerTrans);

		Eigen::Quaterniond q(offsetRot);
		Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
		quatAccum += v * v.transpose();
		transAccum += offsetTrans;
	}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quatAccum);
	Eigen::Vector4d avg = solver.eigenvectors().col(3).normalized();

	Eigen::Quaterniond q(avg(0), avg(1), avg(2), avg(3));
	q.normalize();
	if (q.w() < 0)
		q.coeffs() = -q.coeffs();

	transAccum /= (double)samples.size();

	ctx.relativeRotation.w = q.w();
	ctx.relativeRotation.x = q.x();
	ctx.relativeRotation.y = q.y();
	ctx.relativeRotation.z = q.z();
	ctx.relativeTranslation.v[0] = transAccum.x();
	ctx.relativeTranslation.v[1] = transAccum.y();
	ctx.relativeTranslation.v[2] = transAccum.z();
	ctx.validRelativeOffset = true;
}

struct AlignmentCheck
{
	bool passed = false;
	double positionRms = 0.0, positionP95 = 0.0, positionMax = 0.0;
	double angularRms = 0.0, angularP95 = 0.0, angularMax = 0.0; // degrees
};

static bool FiniteCandidate(const CalibrationContext &candidate)
{
	const auto &q = candidate.relativeRotation;
	const Eigen::Quaterniond mount(q.w, q.x, q.y, q.z);
	const auto &p = candidate.relativeTranslation.v;
	const Eigen::Vector3d translation(p[0], p[1], p[2]);
	return candidate.calibratedRotation.allFinite() && candidate.calibratedTranslation.allFinite()
		&& candidate.calibratedRotation.cwiseAbs().maxCoeff() <= 360.0
		&& candidate.calibratedTranslation.cwiseAbs().maxCoeff() <= 200.0 * MaxCalibrationPositionMeters
		&& std::isfinite(candidate.calibratedScale) && candidate.calibratedScale > 0.0 && candidate.calibratedScale <= 2.0
		&& std::isfinite(candidate.hmdScale) && candidate.hmdScale >= MinCalibratedScale && candidate.hmdScale <= MaxCalibratedScale
		&& candidate.validRelativeOffset && mount.coeffs().allFinite() && std::abs(mount.norm() - 1.0) < 1e-6
		&& translation.allFinite() && translation.norm() <= MaxMountMeters;
}

static AlignmentCheck CheckAlignment(const CalibrationContext &candidate, const std::vector<Sample> &samples)
{
	AlignmentCheck result;
	if (!FiniteCandidate(candidate) || samples.empty()) return result;
	const auto q = VRRotationQuat(candidate.calibratedRotation);
	const Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
	const Eigen::Vector3d translation = candidate.calibratedTranslation * 0.01;
	const auto &mq = candidate.relativeRotation;
	const Eigen::Quaterniond mountRotation(mq.w, mq.x, mq.y, mq.z);
	const auto &mp = candidate.relativeTranslation.v;
	const Eigen::Vector3d mountPosition(mp[0], mp[1], mp[2]);
	std::vector<double> positionErrors, angularErrors;
	for (const auto &sample : samples)
	{
		if (!sample.valid || !sample.ref.rot.allFinite() || !sample.target.rot.allFinite()
			|| !sample.ref.trans.allFinite() || !sample.target.trans.allFinite()) return result;
		const Eigen::Matrix3d trackerRotation = rotation * sample.target.rot;
		const Eigen::Vector3d predictedPosition = candidate.calibratedScale * (rotation * sample.target.trans)
			+ translation + trackerRotation * mountPosition;
		const Eigen::Quaterniond predictedRotation = Eigen::Quaterniond(trackerRotation).normalized() * mountRotation;
		const double positionError = (predictedPosition - sample.ref.trans / candidate.hmdScale).norm();
		const double angularError = predictedRotation.angularDistance(Eigen::Quaterniond(sample.ref.rot).normalized()) * 180.0 / EIGEN_PI;
		if (!std::isfinite(positionError) || !std::isfinite(angularError)) return result;
		positionErrors.push_back(positionError);
		angularErrors.push_back(angularError);
		result.positionRms += positionError * positionError;
		result.angularRms += angularError * angularError;
	}
	result.positionRms = std::sqrt(result.positionRms / samples.size());
	result.angularRms = std::sqrt(result.angularRms / samples.size());
	std::sort(positionErrors.begin(), positionErrors.end());
	std::sort(angularErrors.begin(), angularErrors.end());
	const size_t p95 = static_cast<size_t>(std::ceil(0.95 * samples.size())) - 1;
	result.positionP95 = positionErrors[p95];
	result.angularP95 = angularErrors[p95];
	result.positionMax = positionErrors.back();
	result.angularMax = angularErrors.back();
	// Conservative mismatch guards for the sampled alignment, not ground-truth
	// accuracy limits. Enforce them per motion segment as well as across the run
	// so a short bad direction cannot disappear in a long, otherwise good fit.
	result.passed = result.positionRms <= 0.03 && result.positionP95 <= 0.05 && result.positionMax <= 0.10
		&& result.angularRms <= 2.0 && result.angularP95 <= 3.0 && result.angularMax <= 6.0;
	return result;
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	if (ctx.enabled)
	{
		ctx.targetID = vr::k_unTrackedDeviceIndexInvalid;
		if (!ctx.trackerSerial.empty())
		{
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if (vr::VRSystem()->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_Invalid)
					continue;
				if (GetDeviceSerial(id) == ctx.trackerSerial)
				{
					ctx.targetID = id;
					break;
				}
			}
		}
	}

	bool overrideActive = ctx.enabled && ctx.validRelativeOffset && ctx.targetID != vr::k_unTrackedDeviceIndexInvalid && !ctx.noHeadTracker;

	// Follow mode: send the HMD command first so the driver is already in follow mode when the
	// head tracker gets its transform. Otherwise transforms first, HMD command last.
	bool noTrackerFollow = ctx.enabled && ctx.validProfile && ctx.followSlamHmd && ctx.noHeadTracker;
	bool hmdCommandSent = false;
	if (overrideActive && ctx.followSlamHmd)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true);
		hmdCommandSent = true;
	}
	else if (noTrackerFollow)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, true);
		hmdCommandSent = true;
	}

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// One message per device. Disable-then-enable would let a frame through untransformed.
		bool applyCalibration = false;

		if (ctx.enabled && id != vr::k_unTrackedDeviceIndex_Hmd)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

			if (err == vr::TrackedProp_Success && std::string(buffer) == ctx.targetTrackingSystem)
			{
				// Head tracker stays raw while it drives the headset, in follow mode it's aligned like the rest.
				bool isHeadTracker = deviceClass == vr::TrackedDeviceClass_GenericTracker && id == ctx.targetID;
				applyCalibration = !isHeadTracker || ctx.followSlamHmd;
			}
		}

		if (applyCalibration)
		{
			double known = GetLighthouseModelScale(id);
			double deviceScale = ctx.calibratedScale * (known > 0.0 ? known : ctx.targetModelScale) / ctx.targetModelScale;
			protocol::Request req(protocol::RequestSetDeviceTransform);
			req.setDeviceTransform = {
				id,
				true,
				VRTranslationVec(ctx.calibratedTranslation),
				VRRotationQuat(ctx.calibratedRotation),
				deviceScale
			};
			Driver.SendBlocking(req);
		}
		else
		{
			// Everything else stays raw.
			ResetAndDisableOffsets(id);
		}
	}

	for (uint32_t id = 0; overrideActive && id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// Follow mode: the world is SLAM space already, SLAM devices need no sync.
		bool sync = ctx.continuousSync
			&& !ctx.followSlamHmd
			&& id != vr::k_unTrackedDeviceIndex_Hmd
			&& deviceClass != vr::TrackedDeviceClass_TrackingReference;

		if (sync)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
			sync = err == vr::TrackedProp_Success && std::string(buffer) != ctx.targetTrackingSystem;
		}

		protocol::Request req(protocol::RequestSetSlamSync);
		req.setSlamSync = { id, sync };
		Driver.SendBlocking(req);
	}

	if (!hmdCommandSent)
	{
		if (overrideActive)
			SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true);
		else
			SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);
	}

	SendOneEuroParams();

	try
	{
		protocol::Request statusReq(protocol::RequestGetStatus);
		protocol::Response statusResp = Driver.SendBlocking(statusReq);
		if (statusResp.type == protocol::ResponseStatus)
		{
			ctx.driverStatus = statusResp.status;
			const auto &st = ctx.driverStatus;
			if (st.refinementValid && ctx.enabled && ctx.validProfile && st.refinementSolves >= 1)
			{
				auto differs = [](double a, double b) { return std::fabs(a - b) > 1e-12; };
				bool changed = differs(st.offsetRotation.w, ctx.relativeRotation.w) || differs(st.offsetRotation.x, ctx.relativeRotation.x)
					|| differs(st.offsetRotation.y, ctx.relativeRotation.y) || differs(st.offsetRotation.z, ctx.relativeRotation.z)
					|| differs(st.offsetTranslation.v[0], ctx.relativeTranslation.v[0]) || differs(st.offsetTranslation.v[1], ctx.relativeTranslation.v[1])
					|| differs(st.offsetTranslation.v[2], ctx.relativeTranslation.v[2]) || differs(st.hmdScale, ctx.hmdScale);
				if (changed)
				{
					ctx.relativeRotation = st.offsetRotation;
					ctx.relativeTranslation = st.offsetTranslation;
					ctx.hmdScale = st.hmdScale;
					ctx.refinementDirty = true;
				}
				if (ctx.refinementDirty && ctx.timeLastTick - ctx.timeRefinementSaved > 30.0)
				{
					SaveProfile(ctx);
					ctx.refinementDirty = false;
					ctx.timeRefinementSaved = ctx.timeLastTick;
				}
			}
		}
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to read driver status: " << e.what() << std::endl;
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

struct CalibrationIdentity
{
	std::string trackingSystem, hmdSerial, trackerSerial;
};
static CalibrationIdentity pendingIdentity;

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID, double time)
{
	ctx.targetID = targetID;
	// Application shutdown/settings saves can occur while sampling. Leave the
	// persisted identity paired with its existing mount until the solve commits.
	pendingIdentity.trackingSystem = GetDeviceTrackingSystem(targetID);
	pendingIdentity.hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
	pendingIdentity.trackerSerial = GetDeviceSerial(targetID);

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n", pendingIdentity.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.sequenceStart = time;
	ctx.sequenceSteps = ctx.SequenceStepCount();
	ctx.sequenceStep = 0;
	ctx.state = CalibrationState::Sampling;
	ctx.wantedUpdateInterval = 0.0;
	ctx.Log("Starting calibration...\n");
}

static std::vector<Sample> collectedSamples;
static int coplanarRetries = 0;
static std::optional<CalibrationContext> previousCalibration;
static std::optional<CalibrationContext> candidateCalibration;
static bool validationNeedsStart = false;
// Segments use overlay elapsed time only. OpenVR's polled poses do not provide
// native source timestamps here, so this check cannot certify latency alignment.
static std::array<std::vector<Sample>, CalibrationContext::SequenceCycle> validationSegments;
static std::optional<double> invalidSince;
static constexpr double TrackingLossTimeout = 2.0;
static constexpr int MaxSamplingWindows = 3;
static constexpr size_t MaxCalibrationSamples = 720;
static double detectionStart = 0.0;

void StartCalibration()
{
	if (CalCtx.state == CalibrationState::Begin || CalCtx.state == CalibrationState::Detect || CalCtx.state == CalibrationState::Sampling)
		return;
	// Preserve the in-memory profile, including any unsaved edits/refinement.
	// Registry reloads are neither a stable nor complete rollback source.
	previousCalibration = CalCtx;
	CalCtx.lastCalibrationOk = false;
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	CalCtx.validating = false;
	CalCtx.motionGuidance = "Turn left and right, then look up and down.";
	Detection.Clear();
	collectedSamples.clear();
	invalidSince.reset();
	coplanarRetries = 0;
	candidateCalibration.reset();
	validationNeedsStart = false;
	for (auto &segment : validationSegments) segment.clear();
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	const auto rejectedTarget = ctx.targetID;
	const double lastTick = ctx.timeLastTick;
	auto messages = std::move(ctx.messages);
	if (previousCalibration)
		ctx = std::move(*previousCalibration);
	previousCalibration.reset();
	ctx.messages = std::move(messages);
	ctx.timeLastTick = lastTick;
	ctx.timeLastScan = lastTick - 1.0; // reapply the restored profile on the next tick
	ctx.state = CalibrationState::None;
	ctx.lastCalibrationOk = false;
	ctx.wantedUpdateInterval = 1.0;
	collectedSamples.clear();
	Detection.Clear();
	invalidSince.reset();
	coplanarRetries = 0;
	candidateCalibration.reset();
	validationNeedsStart = false;
	for (auto &segment : validationSegments) segment.clear();
	ctx.validating = false;
	ctx.motionGuidance.clear();
	// Restore state even if the external connection failed during the run.
	if (rejectedTarget < vr::k_unMaxTrackedDeviceCount)
		try { ResetAndDisableOffsets(rejectedTarget); }
		catch (const std::runtime_error &e) { ctx.Log(std::string("Could not reset tracker offsets: ") + e.what() + "\n"); }
}

void CancelCalibration()
{
	auto &ctx = CalCtx;
	if (ctx.state != CalibrationState::Begin && ctx.state != CalibrationState::Detect && ctx.state != CalibrationState::Sampling)
		return;

	ctx.Log("Calibration cancelled by user\n");
	Detection.Clear();
	AbortAndRestoreProfile(ctx);
}

static void UpdateSequenceStep(CalibrationContext &ctx, double time)
{
	if (ctx.state != CalibrationState::Sampling)
	{
		ctx.sequenceStep = 0;
		return;
	}

	int steps = ctx.sequenceSteps > 0 ? ctx.sequenceSteps : ctx.SequenceStepCount();
	int step = (int)((time - ctx.sequenceStart) / CalibrationContext::StepSeconds);
	ctx.sequenceStep = step < 0 ? 0 : (step > steps - 1 ? steps - 1 : step);
}

static void UpdateCalibrationSounds(CalibrationContext &ctx)
{
	static const char *directions[] = { "look_left", "look_center", "look_right", "look_center", "look_up", "look_center", "look_down", "look_center" };
	static CalibrationState lastState = CalibrationState::None;
	static int lastStep = -1;

	const int cycle = (int)(sizeof directions / sizeof directions[0]);
	static_assert(cycle == CalibrationContext::SequenceCycle, "voice cues and wizard steps must line up");

	if (ctx.state == CalibrationState::Sampling)
	{
		if (ctx.sequenceStep != lastStep)
		{
			sound::Stop();
			sound::Play(directions[ctx.sequenceStep % cycle]);
			lastStep = ctx.sequenceStep;
		}
	}
	else
	{
		if (lastState == CalibrationState::Sampling)
		{
			sound::Stop();
			if (ctx.lastCalibrationOk)
				sound::Play("next");
		}
		lastStep = -1;
	}
	lastState = ctx.state;
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	ctx.timeLastTick = time;
	if (ctx.validating && validationNeedsStart)
	{
		// Solver time is not part of the user's new validation sequence.
		ctx.sequenceStart = time;
		validationNeedsStart = false;
	}
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);
	UpdateSequenceStep(ctx, time);
	UpdateCalibrationSounds(ctx);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) != vr::TrackedDeviceClass_HMD ||
			!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
		{
			CalCtx.Log("No tracking HMD found, aborting calibration!\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		std::string hmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);

		Detection.Clear();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);

			bool usable = deviceClass == vr::TrackedDeviceClass_GenericTracker
				|| (ctx.noHeadTracker && deviceClass == vr::TrackedDeviceClass_Controller);
			if (!usable)
				continue;
			if (!ctx.devicePoses[id].bPoseIsValid)
				continue;

			if (GetDeviceTrackingSystem(id) == hmdSystem)
				continue;

			Detection.candidates.push_back(id);
		}

		if (Detection.candidates.empty())
		{
			if (ctx.noHeadTracker)
				CalCtx.Log("No tracker or controller from a different tracking system detected, aborting! Turn on the device you want to hold against your head.\n");
			else
				CalCtx.Log("No trackers from a different tracking system detected, aborting!\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		if (Detection.candidates.size() == 1)
		{
			ctx.targetID = Detection.candidates[0];
			BeginSamplingPhase(ctx, Detection.candidates[0], time);
			return;
		}

		Detection.candidateSpeeds.resize(Detection.candidates.size());
		CalCtx.Log("Move your head around to identify the headset tracker...\n");
		ctx.state = CalibrationState::Detect;
		detectionStart = time;
		ctx.wantedUpdateInterval = 0.0;
		return;
	}

	if (ctx.state == CalibrationState::Detect)
	{
		if (time - detectionStart > 10.0)
		{
			ctx.Log("Tracker detection timed out. Previous calibration restored.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}
		if (!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
			return;

		Eigen::Matrix3d hmdRot = Pose(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking).rot;

		std::vector<Eigen::Matrix3d> curRot(Detection.candidates.size());
		for (size_t i = 0; i < Detection.candidates.size(); i++)
			curRot[i] = Pose(ctx.devicePoses[Detection.candidates[i]].mDeviceToAbsoluteTracking).rot;

		double dt = time - Detection.prevTime;
		if (Detection.havePrev && dt > 1e-4)
		{
			Detection.hmdSpeeds.push_back(AngularSpeedBetween(hmdRot, Detection.prevRot[0], dt));
			for (size_t i = 0; i < Detection.candidates.size(); i++)
				Detection.candidateSpeeds[i].push_back(AngularSpeedBetween(curRot[i], Detection.prevRot[i + 1], dt));

			CalCtx.Progress((int) Detection.hmdSpeeds.size(), 40);
		}

		Detection.prevRot.assign(1, hmdRot);
		Detection.prevRot.insert(Detection.prevRot.end(), curRot.begin(), curRot.end());
		Detection.prevTime = time;
		Detection.havePrev = true;

		if ((int) Detection.hmdSpeeds.size() < 40)
			return;

		double hmdPeak = 0;
		for (double s : Detection.hmdSpeeds)
			hmdPeak = max(hmdPeak, s);

		if (hmdPeak < 0.5)
		{
			CalCtx.Log("Didn't detect enough head movement, aborting! Try again and move your head more.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		double bestCorr = -2, secondCorr = -2;
		int bestIdx = -1;
		for (size_t i = 0; i < Detection.candidates.size(); i++)
		{
			double corr = PearsonCorrelation(Detection.hmdSpeeds, Detection.candidateSpeeds[i]);
			if (corr > bestCorr)
			{
				secondCorr = bestCorr;
				bestCorr = corr;
				bestIdx = (int) i;
			}
			else if (corr > secondCorr)
			{
				secondCorr = corr;
			}
		}

		if (bestIdx == -1 || bestCorr < 0.7 || (bestCorr - secondCorr) < 0.1)
		{
			CalCtx.Log("Couldn't clearly identify the headset tracker, aborting! Make sure only the headset tracker moves with your head, then try again.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		uint32_t targetID = Detection.candidates[bestIdx];
		Detection.Clear();
		BeginSamplingPhase(ctx, targetID, time);
		return;
	}

	auto sample = CollectSample(ctx);
	if (!sample.valid)
	{
		if (!invalidSince)
		{
			invalidSince = time;
			ctx.Log("Tracking interrupted; waiting briefly for a valid sample.\n");
		}
		ctx.motionGuidance = "Tracking interrupted. Keep the tracker visible.";
		if (time - *invalidSince >= TrackingLossTimeout)
		{
			ctx.Log("Tracking did not recover. Previous calibration restored.\n");
			AbortAndRestoreProfile(ctx);
		}
		return;
	}
	invalidSince.reset();

	auto &samples = collectedSamples;
	if (samples.size() == MaxCalibrationSamples)
		samples.erase(samples.begin());
	samples.push_back(sample);

	double elapsed = time - ctx.sequenceStart;
	double total = ctx.validating ? CalibrationContext::SequenceCycle * CalibrationContext::StepSeconds : ctx.SequenceSeconds();
	if (ctx.validating)
	{
		const size_t segment = static_cast<size_t>(std::clamp(static_cast<int>(elapsed / CalibrationContext::StepSeconds), 0, CalibrationContext::SequenceCycle - 1));
		validationSegments[segment].push_back(sample);
	}
	ctx.motionGuidance = MissingMotionGuidance(samples);
	CalCtx.Progress((int)(elapsed * 1000.0), (int)(total * 1000.0));

	if (elapsed >= total)
	{
		CalCtx.Log("\n");
		AlignmentCheck check;
		if (ctx.validating)
		{
			if (candidateCalibration) check = CheckAlignment(*candidateCalibration, samples);
			const bool badSegment = candidateCalibration && std::any_of(validationSegments.begin(), validationSegments.end(),
				[&](const auto &segment) { return segment.size() >= 10 && !CheckAlignment(*candidateCalibration, segment).passed; });
			char summary[256];
			snprintf(summary, sizeof summary, "Independent check: %.1f mm / %.2f deg RMS; %.1f mm / %.2f deg p95.\n",
				check.positionRms * 1000.0, check.angularRms, check.positionP95 * 1000.0, check.angularP95);
			ctx.Log(summary);
			// Missing coverage is recoverable; contradictory evidence is not.
			// Do not discard a bad direction and retry until a candidate looks good.
			if (!check.passed || badSegment)
			{
				ctx.Log("Independent motion did not match. Check the rigid mount, then retry with smooth motion. Previous calibration restored.\n");
				AbortAndRestoreProfile(ctx);
				return;
			}
		}

		const bool segmentCoverage = !ctx.validating || std::all_of(validationSegments.begin(), validationSegments.end(),
			[](const auto &segment) { return segment.size() >= 10; });
		if (samples.size() < 40 || !segmentCoverage || !HasObservableMotion(samples))
		{
			if (++coplanarRetries >= MaxSamplingWindows)
			{
				ctx.Log("Not enough independent head motion after three sampling windows. Previous calibration restored.\n");
				AbortAndRestoreProfile(ctx);
				return;
			}
			ctx.Log(segmentCoverage ? ctx.motionGuidance + " Starting another sequence...\n"
				: "Too few valid poses across the check. Repeat the look-around sequence.\n");
			// Start a real window, including fresh voice cues. Retain the useful
			// data (bounded above), rather than deleting a quarter every tick.
			ctx.sequenceStart = time;
			ctx.sequenceStep = 0;
			if (ctx.validating)
			{
				// A validation attempt must cover a complete new sequence. Never
				// supplement it with training data or refit the frozen candidate.
				samples.clear();
				for (auto &segment : validationSegments) segment.clear();
			}
			return;
		}
		coplanarRetries = 0;

		try
		{
			if (ctx.validating)
			{
				auto &candidate = *candidateCalibration;
				candidate.calibrationCheck.passed = true;
				candidate.calibrationCheck.positionRmsMm = check.positionRms * 1000.0;
				candidate.calibrationCheck.angularRmsDegrees = check.angularRms;
				// Persist the fully checked profile before exposing it to other
				// overlay/driver paths. Failure leaves the start-of-run snapshot.
				if (!SaveProfile(candidate))
					throw std::runtime_error("could not save the checked profile");
				ctx.calibratedRotation = candidate.calibratedRotation;
				ctx.calibratedTranslation = candidate.calibratedTranslation;
				ctx.calibratedScale = candidate.calibratedScale;
				ctx.targetModelScale = candidate.targetModelScale;
				ctx.hmdScale = candidate.hmdScale;
				ctx.relativeRotation = candidate.relativeRotation;
				ctx.relativeTranslation = candidate.relativeTranslation;
				ctx.validRelativeOffset = candidate.validRelativeOffset;
				ctx.targetTrackingSystem = candidate.targetTrackingSystem;
				ctx.hmdSerial = candidate.hmdSerial;
				ctx.trackerSerial = candidate.trackerSerial;
				ctx.calibrationCheck = candidate.calibrationCheck;
				ctx.validProfile = ctx.lastCalibrationOk = true;
				ctx.validating = false;
				ctx.motionGuidance.clear();
				ctx.Log("Alignment check passed; profile saved. This checks agreement, not absolute tracking accuracy.\n");
				if (ctx.notificationId != 0)
				{
					vr::VRNotifications()->RemoveNotification(ctx.notificationId);
					ctx.notificationId = 0;
				}
				ctx.state = CalibrationState::None;
				ctx.timeLastScan = time - 1.0;
				previousCalibration.reset();
				candidateCalibration.reset();
				samples.clear();
				for (auto &segment : validationSegments) segment.clear();
				return;
			}

			CalibrationContext candidate = ctx;
			candidate.calibrationCheck = {};
			candidate.calibratedRotation = CalibrateRotation(samples);

			Eigen::Vector3d eulerRad = candidate.calibratedRotation * EIGEN_PI / 180.0;
			Eigen::Matrix3d calRot =
				(Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
				 Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
				 Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX())).toRotationMatrix();

			double calScale = 1.0;
			candidate.calibratedScale = calScale;
			candidate.targetModelScale = GetLighthouseModelScale(ctx.targetID);
			if (candidate.targetModelScale <= 0.0)
				candidate.targetModelScale = 1.0;

			candidate.hmdScale = EstimateHmdSpaceScale(samples, calRot, candidate.targetModelScale, &candidate.calibrationCheck.scaleMeasured);
			candidate.calibrationCheck.scale = candidate.hmdScale;

			// Only the private training buffer is rescaled. Validation is collected
			// afterward from fresh, unmodified OpenVR poses.
			for (auto &sample : samples)
				sample.ref.trans /= candidate.hmdScale;

			candidate.calibratedTranslation = CalibrateTranslation(samples, calRot, calScale);
			Eigen::Vector3d calTransM = candidate.calibratedTranslation * 0.01;

			Eigen::Vector3d hmdToTarget = ComputeRefToTargetOffset(samples, calRot, calTransM, calScale);
			double rmsError = RetargetingErrorRMS(samples, hmdToTarget, calRot, calTransM, calScale);

			char buf2[256];
			snprintf(buf2, sizeof buf2, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
			CalCtx.Log(buf2);

			ComputeRelativeOffset(candidate, samples, calRot, calTransM, calScale);
			if (!std::isfinite(rmsError) || !FiniteCandidate(candidate) || rmsError > 0.1)
			{
				CalCtx.Log("Calibration quality is too low, aborting! Previous calibration restored. Try again with a slower calibration speed, moving smoothly.\n");
				AbortAndRestoreProfile(ctx);
				return;
			}

			candidate.targetTrackingSystem = pendingIdentity.trackingSystem;
			candidate.hmdSerial = pendingIdentity.hmdSerial;
			candidate.trackerSerial = pendingIdentity.trackerSerial;
			candidate.validProfile = true;
			candidateCalibration = std::move(candidate);
			samples.clear();
			ctx.validating = true;
			validationNeedsStart = true;
			ctx.sequenceStart = time;
			ctx.sequenceSteps = CalibrationContext::SequenceCycle;
			ctx.sequenceStep = 0;
			ctx.motionGuidance = "Repeat the look-around sequence to check alignment.";
			ctx.Progress(0, static_cast<int>(CalibrationContext::SequenceCycle * CalibrationContext::StepSeconds * 1000.0));
			ctx.Log("Fit complete. Checking a separate motion sequence before saving...\n");
		}
		catch (const std::runtime_error &e)
		{
			ctx.Log(std::string("Calibration solve failed: ") + e.what() + "\n");
			AbortAndRestoreProfile(ctx);
		}
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(CalCtx.chaperone.geometry.data(), &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(CalCtx.chaperone.geometry.data(), CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
