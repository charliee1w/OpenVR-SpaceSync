// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-24. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include "PoseMath.h"

#include <cstdint>
#include <cmath>

namespace align {

struct YawTranslationEstimator
{
	double memorySeconds = 3.0;
	double primeSeconds = 1.0;
	double sigmaSeconds = 5.0;
	double minYawJump = 0.4 * POSE_PI / 180.0;
	double minTranslationJump = 0.02;
	double kSigma = 1.0;
	double hSigma = 8.0;
	int minFrames = 6;
	int coarseFrames = 8;
	int referenceFrames = 6;
	double cooldownSeconds = 1.0;
	double windowMaxSeconds = 10.0;
	double yawGate = 5.0 * POSE_PI / 180.0;
	double translationGate = 10.0 * POSE_PI / 180.0;
	double coarseYaw = 2.0 * POSE_PI / 180.0;
	double coarseTranslation = 0.05;
	double coarseYawTolerance = 1.0 * POSE_PI / 180.0;
	double coarseTranslationTolerance = 0.02;
	double projectionDistance = 0.3;
	double calmThreshold = 0.5;
	double stillThreshold = 0.8;
	double distanceMemory = 0.5;
	double translationFloorWeight = 0.1;

	bool valid = false;
	double yaw = 0.0;
	vr::HmdVector3d_t translation = { 0, 0, 0 };
	vr::HmdVector3d_t meanRaw = { 0, 0, 0 };
	vr::HmdVector3d_t lastRaw = { 0, 0, 0 };
	double weightSum = 0.0;
	double weightSumTranslation = 0.0;
	double varYaw = 0.0;
	double varTranslation = 0.0;
	double cooldown = 0.0;
	uint32_t jumps = 0;
	uint32_t coarseJumps = 0;
	double lastJumpYaw = 0.0;
	vr::HmdVector3d_t lastJumpTranslation = { 0, 0, 0 };
	int lastJumpFrames = 0;

	struct Cusum
	{
		double pos = 0.0;
		double neg = 0.0;
		int len = 0;
		void reset() { pos = 0.0; neg = 0.0; len = 0; }
	};

	struct Detector
	{
		int refCount = 0;
		double age = 0.0;
		double sumSin = 0.0;
		double sumCos = 0.0;
		double refYaw = 0.0;
		double frozenYaw = 0.0;
		vr::HmdVector3d_t sumTrans = { 0, 0, 0 };
		vr::HmdVector3d_t sumRaw = { 0, 0, 0 };
		vr::HmdVector3d_t refTrans = { 0, 0, 0 };
		vr::HmdVector3d_t refRaw = { 0, 0, 0 };
		vr::HmdQuaternion_t startRotation = { 1, 0, 0, 0 };
		Cusum cusum[3];

		void reset()
		{
			refCount = 0;
			age = 0.0;
			sumSin = 0.0;
			sumCos = 0.0;
			sumTrans = { 0, 0, 0 };
			sumRaw = { 0, 0, 0 };
			for (int i = 0; i < 3; i++) cusum[i].reset();
		}
	};
	Detector yawDetector;
	Detector translationDetector;

	static const int RingSize = 32;
	struct Sample
	{
		double yawInst;
		vr::HmdVector3d_t corrected;
		vr::HmdVector3d_t raw;
		double residualYaw;
		vr::HmdVector3d_t residualTranslation;
	};
	Sample ring[RingSize] = {};
	int ringCount = 0;
	int ringNext = 0;

	void reset()
	{
		valid = false;
		tilt = { 1, 0, 0, 0 };
		weightSum = 0.0;
		weightSumTranslation = 0.0;
		cooldown = 0.0;
		varYaw = 0.0;
		varTranslation = 0.0;
		lastJumpYaw = 0.0;
		lastJumpTranslation = { 0, 0, 0 };
		lastJumpFrames = 0;
		ringCount = 0;
		ringNext = 0;
		yawDetector.reset();
		translationDetector.reset();
	}

	double sigmaYaw() const
	{
		double s = std::sqrt(varYaw);
		double lo = 0.02 * POSE_PI / 180.0, hi = 1.0 * POSE_PI / 180.0;
		return s < lo ? lo : (s > hi ? hi : s);
	}

	double sigmaTranslation() const
	{
		double s = std::sqrt(varTranslation);
		double lo = 0.0005, hi = 0.03;
		return s < lo ? lo : (s > hi ? hi : s);
	}

	vr::HmdQuaternion_t tilt = { 1, 0, 0, 0 };

	vr::HmdQuaternion_t rotationFor(double yawValue) const { return quaternionNormalize(tilt * quaternionFromYaw(yawValue)); }
	vr::HmdQuaternion_t rotation() const { return rotationFor(yaw); }

	// t' = t + (tilt - tilt') * R(yaw) * (scale*raw)
	void setTilt(const vr::HmdQuaternion_t& newTilt, const vr::HmdVector3d_t& raw, double scale)
	{
		if (!valid) { tilt = quaternionNormalize(newTilt); return; }
		vr::HmdVector3d_t before = quaternionRotateVector(rotation(), vecScale(raw, scale));
		tilt = quaternionNormalize(newTilt);
		vr::HmdVector3d_t after = quaternionRotateVector(rotation(), vecScale(raw, scale));
		translation = vecAdd(translation, vecSub(before, after));
	}

	void resetDetectors()
	{
		yawDetector.reset();
		translationDetector.reset();
	}

	vr::HmdVector3d_t translationFor(double yawValue, const vr::HmdVector3d_t& corrected, const vr::HmdVector3d_t& raw, double scale) const
	{
		vr::HmdVector3d_t rotated = quaternionRotateVector(rotationFor(yawValue), raw);
		return vecSub(corrected, vecScale(rotated, scale));
	}

	bool update(double yawInst, const vr::HmdVector3d_t& corrected, const vr::HmdVector3d_t& raw, const vr::HmdQuaternion_t& rawRotation,
		double scale, double confidence, double dt)
	{
		if (dt <= 0.0) dt = 1.0 / 90.0;
		if (confidence < 0.0) confidence = 0.0;
		if (confidence > 1.0) confidence = 1.0;
		const bool calm = confidence > calmThreshold;
		const bool still = confidence > stillThreshold;

		if (!valid)
		{
			yaw = yawInst;
			translation = translationFor(yaw, corrected, raw, scale);
			meanRaw = raw;
			lastRaw = raw;
			weightSum = dt;
			weightSumTranslation = dt;
			varYaw = 0.0;
			varTranslation = 0.0;
			cooldown = 0.0;
			ringCount = 0;
			ringNext = 0;
			yawDetector.reset();
			translationDetector.reset();
			valid = true;
			return false;
		}

		if (cooldown > 0.0) cooldown -= dt;

		double residualYaw = wrapRad(yawInst - yaw);
		vr::HmdVector3d_t residualTranslation = project(vecSub(translationFor(yaw, corrected, raw, scale), translation), raw, meanRaw);

		Sample& s = ring[ringNext];
		s.yawInst = yawInst;
		s.corrected = corrected;
		s.raw = raw;
		s.residualYaw = residualYaw;
		s.residualTranslation = residualTranslation;
		ringNext = (ringNext + 1) % RingSize;
		if (ringCount < RingSize) ringCount++;

		bool primed = weightSum >= primeSeconds;
		bool snapped = false;

		if (still && primed)
		{
			double sy = sigmaYaw(), st = sigmaTranslation();
			double kY = kSigma * sy > minYawJump * 0.5 ? kSigma * sy : minYawJump * 0.5;
			double hY = hSigma * sy > minYawJump ? hSigma * sy : minYawJump;
			double kT = kSigma * st > minTranslationJump * 0.5 ? kSigma * st : minTranslationJump * 0.5;
			double hT = hSigma * st > minTranslationJump ? hSigma * st : minTranslationJump;

			int yawLen = 0, transLen = 0;
			double rY = 0.0;
			vr::HmdVector3d_t rT = { 0, 0, 0 };
			bool haveY = fineYaw(yawInst, rawRotation, dt, kY, hY, yawLen, rY);
			bool haveT = fineTranslation(corrected, raw, rawRotation, scale, dt, kT, hT, transLen, rT);

			if (cooldown <= 0.0)
			{
				if (yawLen >= minFrames && consistentYaw(yawLen, rY, kY))
				{
					double yawNew;
					vr::HmdVector3d_t translationNew;
					candidate(yawLen, scale, true, yawNew, translationNew);
					if (std::fabs(wrapRad(yawNew - yaw)) >= hY)
					{
						snap(yawLen, scale, true);
						snapped = true;
					}
					else
						yawDetector.reset();
				}
				else if (transLen >= minFrames && consistentTranslation(transLen, rT, kT, scale))
				{
					double yawNew;
					vr::HmdVector3d_t translationNew;
					candidate(transLen, scale, false, yawNew, translationNew);
					if (vecNorm(project(vecSub(translationNew, translation), raw, meanRaw)) >= hT)
					{
						snap(transLen, scale, false);
						snapped = true;
					}
					else
						translationDetector.reset();
				}
			}

			if (!snapped)
			{
				double gain = dt / sigmaSeconds;
				if (haveY && std::fabs(rY) < 4.0 * sy + minYawJump * 0.5)
					varYaw += gain * (rY * rY - varYaw);
				double rt2 = vecDot(rT, rT) / 3.0;
				if (haveT && std::sqrt(rt2) < 4.0 * st + minTranslationJump * 0.5)
					varTranslation += gain * (rt2 - varTranslation);
			}
		}
		else
		{
			yawDetector.reset();
			translationDetector.reset();
		}

		if (!snapped && cooldown <= 0.0 && primed && calm)
		{
			int axis = -1;
			if (coarse(axis))
			{
				snap(coarseFrames, scale, axis < 0);
				coarseJumps++;
				snapped = true;
			}
		}

		if (snapped)
			return true;

		double moved = vecNorm(vecSub(raw, lastRaw));
		lastRaw = raw;
		weightSumTranslation *= std::exp(-moved / distanceMemory);

		double w = confidence * confidence * dt;
		if (w > 0.0)
		{
			weightSum += w;
			if (weightSum > memorySeconds) weightSum = memorySeconds;
			double g = w / weightSum;
			yaw = wrapRad(yaw + g * residualYaw);
		}

		double floorWeight = confidence * confidence > translationFloorWeight ? confidence * confidence : translationFloorWeight;
		double wT = floorWeight * dt;
		weightSumTranslation += wT;
		if (weightSumTranslation > memorySeconds) weightSumTranslation = memorySeconds;
		double gT = wT / weightSumTranslation;
		meanRaw = vecAdd(meanRaw, vecScale(vecSub(raw, meanRaw), gT));
		vr::HmdVector3d_t now = translationFor(yaw, corrected, raw, scale);
		translation = vecAdd(translation, vecScale(vecSub(now, translation), gT));
		return false;
	}

	void absorbRefinement(const vr::HmdVector3d_t& dRotation, const vr::HmdVector3d_t& dTranslation, double dScale,
		const vr::HmdQuaternion_t& rawRotation, const vr::HmdVector3d_t& rawPosition, const vr::HmdQuaternion_t& headRotationBase,
		double baseScale, double effectiveScaleBefore)
	{
		if (!valid) return;
		double dYaw = -quaternionRotateVector(rawRotation, dRotation).v[1];
		vr::HmdVector3d_t rotatedRaw = quaternionRotateVector(rotation(), rawPosition);
		vr::HmdVector3d_t rs = vecScale(rotatedRaw, effectiveScaleBefore);
		vr::HmdVector3d_t dT = vecScale(vr::HmdVector3d_t{ rs.v[2], 0.0, -rs.v[0] }, -dYaw);
		dT = vecSub(dT, vecScale(rotatedRaw, baseScale * dScale));
		dT = vecSub(dT, quaternionRotateVector(headRotationBase, dTranslation));
		yaw = wrapRad(yaw + dYaw);
		translation = vecAdd(translation, dT);
		resetDetectors();
	}

	void rebase(double dYaw, const vr::HmdVector3d_t& dTranslation, double scale)
	{
		if (!valid) return;

		double yawNew = wrapRad(yaw - dYaw);
		vr::HmdVector3d_t translationNew = vecSub(translation,
			quaternionRotateVector(rotationFor(yawNew), vecScale(dTranslation, scale)));

		lastJumpYaw = wrapRad(yawNew - yaw);
		lastJumpTranslation = vecSub(translationNew, translation);
		lastJumpFrames = 0;
		yaw = yawNew;
		translation = translationNew;

		vr::HmdQuaternion_t step = quaternionFromYaw(dYaw);
		meanRaw = vecAdd(quaternionRotateVector(step, meanRaw), dTranslation);
		lastRaw = vecAdd(quaternionRotateVector(step, lastRaw), dTranslation);
		for (int i = 0; i < ringCount; i++)
		{
			ring[i].raw = vecAdd(quaternionRotateVector(step, ring[i].raw), dTranslation);
			ring[i].yawInst = wrapRad(ring[i].yawInst - dYaw);
		}

		cooldown = cooldownSeconds;
		jumps++;
		resetDetectors();
	}

private:
	vr::HmdVector3d_t project(const vr::HmdVector3d_t& r, const vr::HmdVector3d_t& raw, const vr::HmdVector3d_t& reference) const
	{
		vr::HmdVector3d_t d = { raw.v[2] - reference.v[2], 0.0, reference.v[0] - raw.v[0] };
		double n = vecNorm(d);
		if (n < projectionDistance)
			return r;
		vr::HmdVector3d_t u = vecScale(d, 1.0 / n);
		return vecSub(r, vecScale(u, vecDot(r, u)));
	}

	static double tiltBetween(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
	{
		vr::HmdQuaternion_t sa = quaternionNormalize(a * quaternionConjugate(quaternionProjectYaw(a)));
		vr::HmdQuaternion_t sb = quaternionNormalize(b * quaternionConjugate(quaternionProjectYaw(b)));
		return quaternionAngleRad(sa * quaternionConjugate(sb));
	}

	static void step(Cusum& c, double r, double k, double h, int& alarmLen)
	{
		c.pos = c.pos + r - k; if (c.pos < 0.0) c.pos = 0.0;
		c.neg = c.neg - r - k; if (c.neg < 0.0) c.neg = 0.0;
		if (c.pos > 0.0 || c.neg > 0.0) c.len++; else c.len = 0;
		if ((c.pos > h || c.neg > h) && c.len > alarmLen)
			alarmLen = c.len;
	}

	bool fineYaw(double yawInst, const vr::HmdQuaternion_t& rawRotation, double dt, double k, double h, int& alarmLen, double& residual)
	{
		Detector& d = yawDetector;
		if (d.refCount > 0 && (tiltBetween(rawRotation, d.startRotation) > yawGate || d.age > windowMaxSeconds))
			d.reset();
		if (d.refCount == 0)
			d.startRotation = rawRotation;
		d.age += dt;
		if (d.refCount < referenceFrames)
		{
			d.sumSin += std::sin(yawInst);
			d.sumCos += std::cos(yawInst);
			d.refCount++;
			if (d.refCount == referenceFrames)
				d.refYaw = std::atan2(d.sumSin, d.sumCos);
			return false;
		}
		residual = wrapRad(yawInst - d.refYaw);
		step(d.cusum[0], residual, k, h, alarmLen);
		return true;
	}

	bool fineTranslation(const vr::HmdVector3d_t& corrected, const vr::HmdVector3d_t& raw, const vr::HmdQuaternion_t& rawRotation, double scale,
		double dt, double k, double h, int& alarmLen, vr::HmdVector3d_t& residual)
	{
		Detector& d = translationDetector;
		if (d.refCount > 0 && (quaternionAngleRad(rawRotation * quaternionConjugate(d.startRotation)) > translationGate || d.age > windowMaxSeconds))
			d.reset();
		if (d.refCount == 0)
		{
			d.startRotation = rawRotation;
			d.frozenYaw = yaw;
		}
		d.age += dt;
		vr::HmdVector3d_t inst = translationFor(d.frozenYaw, corrected, raw, scale);
		if (d.refCount < referenceFrames)
		{
			d.sumTrans = vecAdd(d.sumTrans, inst);
			d.sumRaw = vecAdd(d.sumRaw, raw);
			d.refCount++;
			if (d.refCount == referenceFrames)
			{
				d.refTrans = vecScale(d.sumTrans, 1.0 / referenceFrames);
				d.refRaw = vecScale(d.sumRaw, 1.0 / referenceFrames);
			}
			return false;
		}
		residual = project(vecSub(inst, d.refTrans), raw, d.refRaw);
		for (int i = 0; i < 3; i++)
			step(d.cusum[i], residual.v[i], k, h, alarmLen);
		return true;
	}

	const Sample& fromEnd(int back) const
	{
		int idx = (ringNext - 1 - back + 2 * RingSize) % RingSize;
		return ring[idx];
	}

	bool consistentYaw(int len, double lastResidual, double k) const
	{
		if (ringCount < minFrames) return false;
		int sign = lastResidual > 0.0 ? 1 : -1;
		double base = fromEnd(0).yawInst;
		for (int b = 0; b < minFrames; b++)
		{
			const Sample& s = fromEnd(b);
			double r = wrapRad(s.yawInst - yawDetector.refYaw);
			if (r * sign <= k) return false;
			if (std::fabs(wrapRad(s.yawInst - base)) > coarseYawTolerance) return false;
		}
		return true;
	}

	bool consistentTranslation(int len, const vr::HmdVector3d_t& lastResidual, double k, double scale) const
	{
		if (ringCount < minFrames) return false;
		int axis = 0;
		for (int i = 1; i < 3; i++) if (std::fabs(lastResidual.v[i]) > std::fabs(lastResidual.v[axis])) axis = i;
		int sign = lastResidual.v[axis] > 0.0 ? 1 : -1;
		const Detector& d = translationDetector;
		vr::HmdVector3d_t base = { 0, 0, 0 };
		for (int b = 0; b < minFrames; b++)
		{
			const Sample& s = fromEnd(b);
			vr::HmdVector3d_t r = project(vecSub(translationFor(d.frozenYaw, s.corrected, s.raw, scale), d.refTrans), s.raw, d.refRaw);
			if (b == 0) base = r;
			if (r.v[axis] * sign <= k) return false;
			if (vecNorm(vecSub(r, base)) > coarseTranslationTolerance) return false;
		}
		return true;
	}

	bool coarse(int& axis) const
	{
		if (ringCount < coarseFrames) return false;
		bool yawJump = true, transJump = true;
		double yawBase = fromEnd(0).residualYaw;
		vr::HmdVector3d_t transBase = fromEnd(0).residualTranslation;
		int sign = yawBase > 0.0 ? 1 : -1;
		for (int b = 0; b < coarseFrames; b++)
		{
			const Sample& s = fromEnd(b);
			if (s.residualYaw * sign <= coarseYaw || std::fabs(s.residualYaw - yawBase) > coarseYawTolerance)
				yawJump = false;
			if (vecNorm(s.residualTranslation) <= coarseTranslation || vecNorm(vecSub(s.residualTranslation, transBase)) > coarseTranslationTolerance)
				transJump = false;
		}
		if (yawJump) { axis = -1; return true; }
		if (transJump) { axis = 0; return true; }
		return false;
	}

	int candidate(int len, double scale, bool snapYaw, double& yawNew, vr::HmdVector3d_t& translationNew, vr::HmdVector3d_t* meanRawOut = nullptr) const
	{
		int n = len < ringCount ? len : ringCount;
		if (n > RingSize) n = RingSize;
		if (n < 1) n = 1;
		double sumSin = 0.0, sumCos = 0.0;
		vr::HmdVector3d_t sumCorrected = { 0, 0, 0 }, sumRaw = { 0, 0, 0 };
		for (int b = 0; b < n; b++)
		{
			const Sample& s = fromEnd(b);
			sumSin += std::sin(s.yawInst);
			sumCos += std::cos(s.yawInst);
			sumCorrected = vecAdd(sumCorrected, s.corrected);
			sumRaw = vecAdd(sumRaw, s.raw);
		}
		yawNew = snapYaw ? std::atan2(sumSin, sumCos) : yaw;
		vr::HmdVector3d_t meanCorrected = vecScale(sumCorrected, 1.0 / n);
		vr::HmdVector3d_t meanRawWindow = vecScale(sumRaw, 1.0 / n);
		translationNew = translationFor(yawNew, meanCorrected, meanRawWindow, scale);
		if (meanRawOut) *meanRawOut = meanRawWindow;
		return n;
	}

	void snap(int len, double scale, bool snapYaw)
	{
		double yawNew;
		vr::HmdVector3d_t translationNew, meanRawWindow;
		int n = candidate(len, scale, snapYaw, yawNew, translationNew, &meanRawWindow);

		lastJumpYaw = wrapRad(yawNew - yaw);
		lastJumpTranslation = vecSub(translationNew, translation);
		lastJumpFrames = n;
		yaw = yawNew;
		translation = translationNew;
		meanRaw = meanRawWindow;
		if (weightSum > 1.0) weightSum = 1.0;
		if (weightSumTranslation > 1.0) weightSumTranslation = 1.0;
		cooldown = cooldownSeconds;
		jumps++;
		yawDetector.reset();
		translationDetector.reset();
	}
};


// m(phi) = tau + R_head(phi) * eps: tau is the DC term, eps the first harmonic, the rest non-rigid.
struct OrientationModel
{
	static const int Bins = 16;
	static const int Terms = 5;

	double binForget = 300.0;
	double minBinWeight = 2.0;
	int minBins = 9;
	double maxAliasing = 0.35;
	double solveInterval = 5.0;
	double maxTilt = 5.0 * POSE_PI / 180.0;
	double maxWarp = 1.0 * POSE_PI / 180.0;
	double cvMargin = 0.15;
	double slewRate = 0.05 * POSE_PI / 180.0;

	struct Bin
	{
		double weight = 0.0;
		double sumX = 0.0, sumZ = 0.0;
	};

	Bin bins[Bins] = {};
	double sinceSolve = 0.0;

	bool validated = false;
	double dc[2] = { 0, 0 };
	double h1[2] = { 0, 0 };
	double warpC[2] = { 0, 0 };
	double warpS[2] = { 0, 0 };
	double press = 0.0;
	double nullSse = 0.0;
	double aliasing = 1.0;
	int occupied = 0;
	uint32_t solves = 0, rejects = 0;

	double appliedDc[2] = { 0, 0 };
	double appliedWarpC[2] = { 0, 0 };
	double appliedWarpS[2] = { 0, 0 };

	void reset()
	{
		for (int b = 0; b < Bins; b++) bins[b] = Bin();
		sinceSolve = 0.0;
		validated = false;
		for (int i = 0; i < 2; i++)
		{
			dc[i] = h1[i] = warpC[i] = warpS[i] = 0.0;
			appliedDc[i] = appliedWarpC[i] = appliedWarpS[i] = 0.0;
		}
		press = nullSse = 0.0;
		aliasing = 1.0;
		occupied = 0;
		solves = rejects = 0;
	}

	static double binCenter(int b) { return (b + 0.5) * (2.0 * POSE_PI / Bins) - POSE_PI; }

	static void basis(double phi, double* B)
	{
		B[0] = 1.0;
		B[1] = std::cos(phi);
		B[2] = std::sin(phi);
		B[3] = std::cos(2.0 * phi);
		B[4] = std::sin(2.0 * phi);
	}

	void add(double phi, double mx, double mz, double w, double dt)
	{
		double f = std::exp(-dt / binForget);
		for (int b = 0; b < Bins; b++)
		{
			bins[b].weight *= f;
			bins[b].sumX *= f;
			bins[b].sumZ *= f;
		}
		sinceSolve += dt;

		if (w <= 0.0) return;
		int b = (int)std::floor((wrapRad(phi) + POSE_PI) / (2.0 * POSE_PI / Bins));
		if (b < 0) b = 0;
		if (b >= Bins) b = Bins - 1;
		bins[b].weight += w;
		bins[b].sumX += w * mx;
		bins[b].sumZ += w * mz;
	}

	bool due() const { return sinceSolve >= solveInterval; }

	bool solve()
	{
		sinceSolve = 0.0;

		int idx[Bins];
		int n = 0;
		double sumCos = 0.0, sumSin = 0.0;
		for (int b = 0; b < Bins; b++)
		{
			if (bins[b].weight < minBinWeight) continue;
			idx[n++] = b;
			sumCos += std::cos(binCenter(b));
			sumSin += std::sin(binCenter(b));
		}
		occupied = n;
		if (n < minBins) { validated = false; return false; }

		aliasing = std::sqrt(sumCos * sumCos + sumSin * sumSin) / n;
		if (aliasing > maxAliasing) { validated = false; return false; }

		double phi[Bins], mx[Bins], mz[Bins];
		for (int i = 0; i < n; i++)
		{
			const Bin& bin = bins[idx[i]];
			phi[i] = binCenter(idx[i]);
			mx[i] = bin.sumX / bin.weight;
			mz[i] = bin.sumZ / bin.weight;
		}

		double pressSum = 0.0, nullSum = 0.0;
		for (int hold = 0; hold < n; hold++)
		{
			double cx[Terms], cz[Terms];
			if (!fit(n, phi, mx, mz, hold, cx, cz)) { validated = false; return false; }
			double B[Terms];
			basis(phi[hold], B);
			double px = 0.0, pz = 0.0;
			for (int k = 0; k < Terms; k++) { px += cx[k] * B[k]; pz += cz[k] * B[k]; }
			double ex = mx[hold] - px, ez = mz[hold] - pz;
			pressSum += ex * ex + ez * ez;
			nullSum += mx[hold] * mx[hold] + mz[hold] * mz[hold];
		}
		press = pressSum;
		nullSse = nullSum;
		solves++;

		if (pressSum >= (1.0 - cvMargin) * nullSum) { validated = false; rejects++; return false; }

		double cx[Terms], cz[Terms];
		if (!fit(n, phi, mx, mz, -1, cx, cz)) { validated = false; return false; }

		double tiltMag = std::sqrt(cx[0] * cx[0] + cz[0] * cz[0]);
		double warpMag = std::sqrt(cx[3] * cx[3] + cz[3] * cz[3] + cx[4] * cx[4] + cz[4] * cz[4]);
		if (tiltMag > maxTilt || warpMag > maxWarp) { validated = false; rejects++; return false; }

		dc[0] = cx[0]; dc[1] = cz[0];
		h1[0] = std::sqrt(cx[1] * cx[1] + cx[2] * cx[2]);
		h1[1] = std::sqrt(cz[1] * cz[1] + cz[2] * cz[2]);
		warpC[0] = cx[3]; warpC[1] = cz[3];
		warpS[0] = cx[4]; warpS[1] = cz[4];
		validated = true;
		return true;
	}

	void slew(double dt)
	{
		if (!validated) return;
		double step = slewRate * dt;
		for (int i = 0; i < 2; i++)
		{
			appliedDc[i] = toward(appliedDc[i], dc[i], step);
			appliedWarpC[i] = toward(appliedWarpC[i], warpC[i], step);
			appliedWarpS[i] = toward(appliedWarpS[i], warpS[i], step);
		}
	}

	vr::HmdVector3d_t tiltAt(double phi) const
	{
		double c2 = std::cos(2.0 * phi), s2 = std::sin(2.0 * phi);
		return {
			appliedDc[0] + appliedWarpC[0] * c2 + appliedWarpS[0] * s2,
			0.0,
			appliedDc[1] + appliedWarpC[1] * c2 + appliedWarpS[1] * s2
		};
	}

	double appliedTiltDeg() const
	{
		return std::sqrt(appliedDc[0] * appliedDc[0] + appliedDc[1] * appliedDc[1]) * 180.0 / POSE_PI;
	}

	double appliedWarpDeg() const
	{
		return std::sqrt(appliedWarpC[0] * appliedWarpC[0] + appliedWarpC[1] * appliedWarpC[1]
			+ appliedWarpS[0] * appliedWarpS[0] + appliedWarpS[1] * appliedWarpS[1]) * 180.0 / POSE_PI;
	}

private:
	static double toward(double current, double target, double step)
	{
		double d = target - current;
		if (d > step) d = step;
		if (d < -step) d = -step;
		return current + d;
	}

	bool fit(int n, const double* phi, const double* mx, const double* mz, int hold, double* cx, double* cz) const
	{
		double A[Terms * Terms] = {};
		double bx[Terms] = {}, bz[Terms] = {};
		int used = 0;
		for (int i = 0; i < n; i++)
		{
			if (i == hold) continue;
			double B[Terms];
			basis(phi[i], B);
			for (int r = 0; r < Terms; r++)
			{
				for (int c = 0; c < Terms; c++) A[r * Terms + c] += B[r] * B[c];
				bx[r] += B[r] * mx[i];
				bz[r] += B[r] * mz[i];
			}
			used++;
		}
		if (used < Terms + 2) return false;
		double Ax[Terms * Terms];
		for (int i = 0; i < Terms * Terms; i++) Ax[i] = A[i];
		if (!solveLinearSystem(Terms, Ax, bx, cx)) return false;
		return solveLinearSystem(Terms, A, bz, cz);
	}
};


struct ClockAligner
{
	double windowSeconds = 3.0;
	double searchMin = -0.05;
	double searchMax = 0.20;
	double searchStep = 0.005;
	double minStdRot = 0.25;
	double minStdPos = 0.08;
	double minMeanWeight = 0.5;
	double minCorrelation = 0.85;
	double minProminence = 0.05;
	double prominenceDistance = 0.05;
	int minSamples = 60;
	int diffSpan = 5;
	double solveInterval = 1.0;
	double tauMin = -0.05;
	double tauMax = 0.25;
	static const int Votes = 9;
	double smoothing = 0.25;

	struct Series
	{
		static const int N = 1024;
		double time[N] = {};
		double rot[N] = {};
		double pos[N] = {};
		int count = 0;
		int next = 0;

		void clear() { count = 0; next = 0; }

		void add(double t, double r, double p)
		{
			time[next] = t; rot[next] = r; pos[next] = p;
			next = (next + 1) % N;
			if (count < N) count++;
		}

		int physical(int logical) const { return (next - count + logical + 2 * N) % N; }

		double at(int logical, int channel) const { int i = physical(logical); return channel == 0 ? rot[i] : pos[i]; }
		double timeAt(int logical) const { return time[physical(logical)]; }

		bool interpolate(double t, int channel, double& value) const
		{
			if (count < 2) return false;
			if (t < timeAt(0) || t > timeAt(count - 1)) return false;
			int lo = 0, hi = count - 1;
			while (hi - lo > 1)
			{
				int mid = (lo + hi) / 2;
				if (timeAt(mid) <= t) lo = mid; else hi = mid;
			}
			double t0 = timeAt(lo), t1 = timeAt(hi);
			double v0 = at(lo, channel), v1 = at(hi, channel);
			double f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0;
			value = v0 + (v1 - v0) * f;
			return true;
		}
	};

	struct Pose
	{
		double time = 0.0;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t position = { 0, 0, 0 };
	};

	struct Channel
	{
		double tau = 0.0;
		bool primed = false;
		double votes[Votes] = {};
		int voteCount = 0;
		int voteNext = 0;
		double lastFound = 0.0;
		double lastCorrelation = 0.0;
		double lastProminence = 0.0;
		uint32_t solves = 0;

		void reset() { tau = 0.0; primed = false; voteCount = 0; voteNext = 0; lastFound = 0.0; lastCorrelation = 0.0; lastProminence = 0.0; }

		void vote(double found, double lo, double hi, double smoothing)
		{
			votes[voteNext] = found;
			voteNext = (voteNext + 1) % Votes;
			if (voteCount < Votes) voteCount++;
			double sorted[Votes];
			for (int i = 0; i < voteCount; i++) sorted[i] = votes[i];
			for (int i = 1; i < voteCount; i++)
			{
				double v = sorted[i];
				int j = i - 1;
				while (j >= 0 && sorted[j] > v) { sorted[j + 1] = sorted[j]; j--; }
				sorted[j + 1] = v;
			}
			double median = voteCount % 2 == 1 ? sorted[voteCount / 2] : 0.5 * (sorted[voteCount / 2 - 1] + sorted[voteCount / 2]);
			double target = median < lo ? lo : (median > hi ? hi : median);
			if (!primed) tau = target;
			else tau += smoothing * (target - tau);
			primed = true;
			solves++;
		}
	};

	struct PoseRing
	{
		Pose poses[16] = {};
		int count = 0;
		int next = 0;
		void clear() { count = 0; next = 0; }
	};

	Series hmd;
	Series tracker;
	Channel rot;
	Channel pos;
	PoseRing hmdPoses;
	PoseRing trackerPoses;
	double lastSolve = 0.0;

	void reset()
	{
		hmd.clear();
		tracker.clear();
		rot.reset();
		pos.reset();
		hmdPoses.clear();
		trackerPoses.clear();
		lastSolve = 0.0;
	}

	double tauRot() const { return rot.tau; }
	double tauPos() const { return pos.tau; }

	void addHmdPose(double t, const vr::HmdQuaternion_t& rotation, const vr::HmdVector3d_t& position) { push(hmdPoses, hmd, t, rotation, position); }

	void noteHmdDiscontinuity() { hmdPoses.clear(); }

	void addTrackerPose(double t, const vr::HmdQuaternion_t& rotation, const vr::HmdVector3d_t& position) { push(trackerPoses, tracker, t, rotation, position); }

	bool due(double now) const { return now - lastSolve >= solveInterval; }

	bool solve(double now)
	{
		lastSolve = now;
		bool updated = false;
		for (int channel = 0; channel < 2; channel++)
		{
			Channel& c = channel == 0 ? rot : pos;
			double found = 0.0, corr = 0.0, prominence = 0.0;
			if (!search(now, channel, found, corr, prominence))
				continue;
			c.lastFound = found;
			c.lastCorrelation = corr;
			c.lastProminence = prominence;
			c.vote(found, tauMin, tauMax, smoothing);
			updated = true;
		}
		return updated;
	}

private:
	void push(PoseRing& ring, Series& series, double t, const vr::HmdQuaternion_t& rotation, const vr::HmdVector3d_t& position)
	{
		if (ring.count > 0)
		{
			const Pose& last = ring.poses[(ring.next - 1 + 16) % 16];
			double span = t - last.time;
			if (span <= 0.0005) return;
			if (span > 0.1) ring.clear();
		}
		Pose& p = ring.poses[ring.next];
		p.time = t; p.rotation = rotation; p.position = position;
		ring.next = (ring.next + 1) % 16;
		if (ring.count < 16) ring.count++;
		if (ring.count <= diffSpan) return;
		const Pose& old = ring.poses[(ring.next - 1 - diffSpan + 32) % 16];
		double span = t - old.time;
		if (span < 0.005 || span > 0.2) return;
		double ang = quaternionAngleRad(rotation * quaternionConjugate(old.rotation)) / span;
		double lin = vecNorm(vecSub(position, old.position)) / span;
		series.add(0.5 * (t + old.time), ang, lin);
	}

	bool search(double now, int channel, double& found, double& bestCorr, double& prominence) const
	{
		int first = 0;
		while (first < hmd.count && hmd.timeAt(first) < now - windowSeconds) first++;
		int n = hmd.count - first;
		if (n < minSamples) return false;

		double meanH = 0.0;
		for (int i = first; i < hmd.count; i++) meanH += hmd.at(i, channel);
		meanH /= n;
		double varH = 0.0;
		for (int i = first; i < hmd.count; i++) { double d = hmd.at(i, channel) - meanH; varH += d * d; }
		if (std::sqrt(varH / n) < (channel == 0 ? minStdRot : minStdPos)) return false;

		const int steps = (int)std::floor((searchMax - searchMin) / searchStep + 0.5) + 1;
		const double weightGate = (channel == 0 ? minStdRot : minStdPos) * minMeanWeight;
		double corr[128];
		int bestIdx = -1;
		bestCorr = -2.0;
		for (int k = 0; k < steps && k < 128; k++)
		{
			double tau = searchMin + k * searchStep;
			double sumW = 0.0, sumG = 0.0, sumGG = 0.0, sumHG = 0.0, sumH = 0.0, sumHH = 0.0;
			int m = 0;
			for (int i = first; i < hmd.count; i++)
			{
				double g;
				if (!tracker.interpolate(hmd.timeAt(i) + tau, channel, g)) continue;
				double h = hmd.at(i, channel);
				double w = h < g ? h : g;
				if (w <= 0.0) continue;
				sumW += w;
				sumG += w * g; sumGG += w * g * g; sumHG += w * h * g; sumH += w * h; sumHH += w * h * h;
				m++;
			}
			if (m < minSamples || sumW < weightGate * m) { corr[k] = -2.0; continue; }
			double mH = sumH / sumW, mG = sumG / sumW;
			double covHG = sumHG / sumW - mH * mG;
			double vH = sumHH / sumW - mH * mH;
			double vG = sumGG / sumW - mG * mG;
			if (vH <= 1e-12 || vG <= 1e-12) { corr[k] = -2.0; continue; }
			corr[k] = covHG / std::sqrt(vH * vG);
			if (corr[k] > bestCorr) { bestCorr = corr[k]; bestIdx = k; }
		}
		if (bestIdx <= 0 || bestIdx >= steps - 1 || bestCorr < minCorrelation) return false;
		double c0 = corr[bestIdx - 1], c1 = corr[bestIdx], c2 = corr[bestIdx + 1];
		if (c0 < -1.5 || c2 < -1.5) return false;

		int away = (int)std::floor(prominenceDistance / searchStep + 0.5);
		double side = -2.0;
		for (int k = 0; k < steps && k < 128; k++)
		{
			if (k > bestIdx - away && k < bestIdx + away) continue;
			if (corr[k] > side) side = corr[k];
		}
		prominence = side < -1.5 ? 1.0 : bestCorr - side;
		if (prominence < minProminence) return false;

		double denom = c0 - 2.0 * c1 + c2;
		double offset = std::fabs(denom) > 1e-12 ? 0.5 * (c0 - c2) / denom : 0.0;
		if (offset > 1.0) offset = 1.0;
		if (offset < -1.0) offset = -1.0;
		found = searchMin + (bestIdx + offset) * searchStep;
		return true;
	}
};


struct MountRefiner
{
	double forgetSeconds = 90.0;
	double minCalmSeconds = 15.0;
	double fullFractionSeconds = 45.0;
	double solveInterval = 2.0;
	double fraction = 0.3;
	double ridge = 0.05;
	double spreadRidge = 0.09;
	double yawResetAngle = 5.0 * POSE_PI / 180.0;
	double maxRotation = 10.0 * POSE_PI / 180.0;
	double maxTranslation = 0.10;
	double maxScale = 0.02;
	bool scaleEnabled = true;
	double obsMin = 0.05;
	double obsScaleMin = 0.25;

	bool started = false;
	double yawReference = 0.0;
	double weight = 0.0;
	double sinceSolve = 0.0;
	Mat3 sumR = Mat3::zero();
	vr::HmdVector3d_t sumM = { 0, 0, 0 };
	vr::HmdVector3d_t sumRtM = { 0, 0, 0 };
	vr::HmdVector3d_t sumE = { 0, 0, 0 };
	vr::HmdVector3d_t sumRtE = { 0, 0, 0 };
	vr::HmdVector3d_t sumX = { 0, 0, 0 };
	vr::HmdVector3d_t sumRtX = { 0, 0, 0 };
	vr::HmdVector3d_t sumYX = { 0, 0, 0 };
	vr::HmdVector3d_t sumRtYX = { 0, 0, 0 };
	double sumXX = 0.0;
	double sumYXYX = 0.0;
	double sumXE = 0.0;
	double sumYXE = 0.0;

	vr::HmdVector3d_t solvedRotation = { 0, 0, 0 };
	vr::HmdVector3d_t solvedTranslation = { 0, 0, 0 };
	double solvedScale = 0.0;
	double solvedYaw = 0.0;
	vr::HmdVector3d_t rotation = { 0, 0, 0 };
	vr::HmdVector3d_t translation = { 0, 0, 0 };
	double scale = 0.0;
	uint32_t solves = 0;
	bool solvedValid = false;
	double obsRotation[3] = { 0, 0, 0 };
	double obsTranslation[3] = { 0, 0, 0 };
	double obsScale = 0.0;

	struct Delta
	{
		vr::HmdVector3d_t rotation = { 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		double scale = 0.0;
	};

	void reset()
	{
		started = false;
		clearSums();
		solvedRotation = { 0, 0, 0 };
		solvedTranslation = { 0, 0, 0 };
		solvedScale = 0.0;
		solvedYaw = 0.0;
		rotation = { 0, 0, 0 };
		translation = { 0, 0, 0 };
		scale = 0.0;
		solvedValid = false;
	}

	void clearSums()
	{
		weight = 0.0;
		sinceSolve = 0.0;
		sumR = Mat3::zero();
		sumM = sumRtM = sumE = sumRtE = sumX = sumRtX = sumYX = sumRtYX = { 0, 0, 0 };
		sumXX = sumYXYX = sumXE = sumYXE = 0.0;
	}

	double rotationAngle() const { return vecNorm(rotation); }
	double translationDistance() const { return vecNorm(translation); }

	vr::HmdQuaternion_t effectiveRotation(const vr::HmdQuaternion_t& base) const
	{
		return quaternionNormalize(base * quaternionFromRotationVector(vecScale(rotation, -1.0)));
	}

	vr::HmdVector3d_t effectiveTranslation(const vr::HmdQuaternion_t& baseRotation, const vr::HmdVector3d_t& baseTranslation) const
	{
		return vecSub(baseTranslation, quaternionRotateVector(baseRotation, translation));
	}

	double effectiveScale(double baseScale) const { return baseScale * (1.0 + scale); }

	void shift(double yawDelta, const vr::HmdVector3d_t& translationDeltaC)
	{
		if (!started || weight <= 0.0) return;
		vr::HmdVector3d_t dc = quaternionRotateVector(quaternionConjugate(quaternionFromYaw(yawReference)), translationDeltaC);
		vr::HmdVector3d_t up = { 0, 1, 0 };
		sumE = vecAdd(sumE, vecAdd(vecScale(dc, weight), vecScale(sumYX, yawDelta)));
		sumRtE = vecAdd(sumRtE, vecAdd(sumR.mulTransposed(dc), vecScale(sumRtYX, yawDelta)));
		sumXE += vecDot(sumX, dc);
		sumYXE += vecDot(sumYX, dc) + yawDelta * sumYXYX;
		sumM = vecAdd(sumM, vecScale(up, yawDelta * weight));
		sumRtM = vecAdd(sumRtM, vecScale(sumR.mulTransposed(up), yawDelta));
	}

	void add(const vr::HmdQuaternion_t& headRotationBase, const vr::HmdVector3d_t& headPositionBase,
		const vr::HmdQuaternion_t& rawRotation, const vr::HmdVector3d_t& rawPosition,
		double estimatorYaw, double baseScale, double w, double dt)
	{
		if (w <= 0.0) return;
		if (!started || std::fabs(wrapRad(estimatorYaw - yawReference)) > yawResetAngle)
		{
			clearSums();
			yawReference = estimatorYaw;
			started = true;
		}

		vr::HmdQuaternion_t invRef = quaternionConjugate(quaternionFromYaw(yawReference));
		vr::HmdVector3d_t m = quaternionToRotationVector(quaternionNormalize(invRef * headRotationBase * quaternionConjugate(rawRotation)));
		vr::HmdVector3d_t x = vecScale(rawPosition, baseScale);
		vr::HmdVector3d_t e = vecSub(quaternionRotateVector(invRef, headPositionBase), x);
		vr::HmdVector3d_t yx = { x.v[2], 0.0, -x.v[0] };
		Mat3 R = Mat3::fromQuaternion(rawRotation);

		double f = 1.0 - w / forgetSeconds;
		if (f < 0.0) f = 0.0;
		sumR.scale(f);
		sumM = vecScale(sumM, f); sumRtM = vecScale(sumRtM, f);
		sumE = vecScale(sumE, f); sumRtE = vecScale(sumRtE, f);
		sumX = vecScale(sumX, f); sumRtX = vecScale(sumRtX, f);
		sumYX = vecScale(sumYX, f); sumRtYX = vecScale(sumRtYX, f);
		sumXX *= f; sumYXYX *= f; sumXE *= f; sumYXE *= f;
		weight = weight * f + w;

		sumR.addScaled(R, w);
		sumM = vecAdd(sumM, vecScale(m, w));
		sumRtM = vecAdd(sumRtM, vecScale(R.mulTransposed(m), w));
		sumE = vecAdd(sumE, vecScale(e, w));
		sumRtE = vecAdd(sumRtE, vecScale(R.mulTransposed(e), w));
		sumX = vecAdd(sumX, vecScale(x, w));
		sumRtX = vecAdd(sumRtX, vecScale(R.mulTransposed(x), w));
		sumYX = vecAdd(sumYX, vecScale(yx, w));
		sumRtYX = vecAdd(sumRtYX, vecScale(R.mulTransposed(yx), w));
		sumXX += w * vecDot(x, x);
		sumYXYX += w * vecDot(yx, yx);
		sumXE += w * vecDot(x, e);
		sumYXE += w * vecDot(yx, e);
		sinceSolve += dt;
	}

	bool due() const { return weight >= minCalmSeconds && sinceSolve >= solveInterval; }

	bool solve()
	{
		sinceSolve = 0.0;
		if (weight < minCalmSeconds) return false;
		double N = weight;

		vr::HmdVector3d_t a = sumR.mulTransposed({ 0, 1, 0 });
		for (int i = 0; i < 3; i++)
		{
			obsRotation[i] = 1.0 - a.v[i] * a.v[i] / (N * N);
			double col = 0.0;
			for (int k = 0; k < 3; k++) col += sumR.m[k][i] * sumR.m[k][i];
			obsTranslation[i] = 1.0 - col / (N * N);
		}
		obsScale = (sumXX - vecDot(sumX, sumX) / N) / N;
		double A[9], b[3], eps[3];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				A[i * 3 + j] = (i == j ? N + ridge * N : 0.0) - a.v[i] * a.v[j] / N;
			b[i] = sumRtM.v[i] - a.v[i] * sumM.v[1] / N;
		}
		if (!solveLinearSystem(3, A, b, eps)) return false;

		double B[25], c[5], v[5];
		double Cm[15];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++) Cm[i * 5 + j] = sumR.m[i][j];
			Cm[i * 5 + 3] = sumX.v[i];
			Cm[i * 5 + 4] = sumYX.v[i];
		}
		for (int i = 0; i < 25; i++) B[i] = 0.0;
		for (int i = 0; i < 3; i++)
		{
			B[i * 5 + i] = N;
			B[i * 5 + 3] = B[3 * 5 + i] = sumRtX.v[i];
			B[i * 5 + 4] = B[4 * 5 + i] = sumRtYX.v[i];
		}
		B[3 * 5 + 3] = sumXX;
		B[4 * 5 + 4] = sumYXYX;
		for (int i = 0; i < 5; i++)
			for (int j = 0; j < 5; j++)
			{
				double dotc = 0.0;
				for (int k = 0; k < 3; k++) dotc += Cm[k * 5 + i] * Cm[k * 5 + j];
				B[i * 5 + j] -= dotc / N;
			}
		for (int i = 0; i < 3; i++) B[i * 5 + i] += ridge * N;
		B[3 * 5 + 3] += spreadRidge * N + (scaleEnabled ? 0.0 : 1e6 * N);
		B[4 * 5 + 4] += spreadRidge * N;
		double rhs[5] = { sumRtE.v[0], sumRtE.v[1], sumRtE.v[2], sumXE, sumYXE };
		for (int i = 0; i < 5; i++)
		{
			double dotc = 0.0;
			for (int k = 0; k < 3; k++) dotc += Cm[k * 5 + i] * sumE.v[k];
			c[i] = rhs[i] - dotc / N;
		}
		if (!solveLinearSystem(5, B, c, v)) return false;

		vr::HmdVector3d_t epsV = { eps[0], eps[1], eps[2] };
		vr::HmdVector3d_t delV = { v[0], v[1], v[2] };
		double epsN = vecNorm(epsV), delN = vecNorm(delV);
		if (epsN > maxRotation) epsV = vecScale(epsV, maxRotation / epsN);
		if (delN > maxTranslation) delV = vecScale(delV, maxTranslation / delN);
		double kap = v[3];
		if (kap > maxScale) kap = maxScale;
		if (kap < -maxScale) kap = -maxScale;

		solvedRotation = epsV;
		solvedTranslation = delV;
		solvedScale = scaleEnabled ? kap : 0.0;
		solvedYaw = v[4];
		solvedValid = true;
		solves++;
		return true;
	}

	Delta apply()
	{
		vr::HmdVector3d_t oldRotation = rotation, oldTranslation = translation;
		double oldScale = scale;
		double f = fraction * (weight < fullFractionSeconds ? weight / fullFractionSeconds : 1.0);
		for (int i = 0; i < 3; i++)
		{
			if (obsRotation[i] > obsMin)
				rotation.v[i] += f * (solvedRotation.v[i] - rotation.v[i]);
			if (obsTranslation[i] > obsMin)
				translation.v[i] += f * (solvedTranslation.v[i] - translation.v[i]);
		}
		if (scaleEnabled && obsScale > obsScaleMin)
			scale += f * (solvedScale - scale);
		double rn = vecNorm(rotation), tn = vecNorm(translation);
		if (rn > maxRotation) rotation = vecScale(rotation, maxRotation / rn);
		if (tn > maxTranslation) translation = vecScale(translation, maxTranslation / tn);
		if (scale > maxScale) scale = maxScale;
		if (scale < -maxScale) scale = -maxScale;
		Delta d;
		d.rotation = vecSub(rotation, oldRotation);
		d.translation = vecSub(translation, oldTranslation);
		d.scale = scale - oldScale;
		return d;
	}
};

}
