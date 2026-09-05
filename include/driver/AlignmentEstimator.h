// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-24. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include "PoseMath.h"
#include "ObservableRefinement.h"

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

	vr::HmdQuaternion_t rotation() const { return quaternionFromYaw(yaw); }

	void resetDetectors()
	{
		yawDetector.reset();
		translationDetector.reset();
	}

	static vr::HmdVector3d_t translationFor(double yawValue, const vr::HmdVector3d_t& corrected, const vr::HmdVector3d_t& raw, double scale)
	{
		vr::HmdVector3d_t rotated = quaternionRotateVector(quaternionFromYaw(yawValue), raw);
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

		if (still)
		{
			Sample& s = ring[ringNext];
			s.yawInst = yawInst;
			s.corrected = corrected;
			s.raw = raw;
			s.residualYaw = residualYaw;
			s.residualTranslation = residualTranslation;
			ringNext = (ringNext + 1) % RingSize;
			if (ringCount < RingSize) ringCount++;
		}
		else
		{
			ringCount = 0;
			ringNext = 0;
		}

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
			quaternionRotateVector(quaternionFromYaw(yawNew), vecScale(dTranslation, scale)));

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

	// Keep the last reliable clock estimate, but never interpolate or vote
	// with velocity samples from before an invalid-pose interval.
	void noteHmdDiscontinuity() { hmdPoses.clear(); hmd.clear(); }
	void noteTrackerDiscontinuity() { trackerPoses.clear(); tracker.clear(); }

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
			if (span > 0.1) { ring.clear(); series.clear(); }
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
		if (ang > 20.0 || lin > 6.0)
		{
			ring.clear();
			series.clear();
			return;
		}
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
	double blockSeconds = 60.0;
	double obsMin = 0.15;
	double obsScaleMin = 0.5;
	double ridge = 0.05;
	double spreadRidge = 0.25;
	double yawResetAngle = 5.0 * POSE_PI / 180.0;
	double maxRotation = 3.0 * POSE_PI / 180.0;
	double maxTranslation = 0.02;
	double maxScale = 0.01;
	double stepRotation = 0.5 * POSE_PI / 180.0;
	double stepTranslation = 0.01;
	double stepScale = 0.005;
	double minImprovementRatio = 0.8;
	double minImprovementRatioRotation = 0.7;
	double minImprovementTranslation = 0.001;
	double minImprovementRotation = 0.3 * POSE_PI / 180.0;
	double suspectTranslation = 0.03;
	double suspectRotation = 3.0 * POSE_PI / 180.0;
	bool scaleEnabled = true;

	struct Block
	{
		double weight = 0.0;
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
		double sumEE = 0.0;
		double sumMM = 0.0;

		void clear()
		{
			weight = 0.0;
			sumR = Mat3::zero();
			sumM = sumRtM = sumE = sumRtE = sumX = sumRtX = sumYX = sumRtYX = { 0, 0, 0 };
			sumXX = sumYXYX = sumXE = sumYXE = sumEE = sumMM = 0.0;
		}

		void add(const Mat3& R, const vr::HmdVector3d_t& m, const vr::HmdVector3d_t& e, const vr::HmdVector3d_t& x, const vr::HmdVector3d_t& yx, double w)
		{
			weight += w;
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
			sumEE += w * vecDot(e, e);
			sumMM += w * vecDot(m, m);
		}

		void shift(double yawDelta, const vr::HmdVector3d_t& dc)
		{
			if (weight <= 0.0) return;
			vr::HmdVector3d_t up = { 0, 1, 0 };
			sumEE += 2.0 * vecDot(dc, sumE) + 2.0 * yawDelta * sumYXE + weight * vecDot(dc, dc) + 2.0 * yawDelta * vecDot(dc, sumYX) + yawDelta * yawDelta * sumYXYX;
			sumMM += 2.0 * yawDelta * sumM.v[1] + weight * yawDelta * yawDelta;
			sumXE += vecDot(sumX, dc);
			sumYXE += vecDot(sumYX, dc) + yawDelta * sumYXYX;
			sumRtE = vecAdd(sumRtE, vecAdd(sumR.mulTransposed(dc), vecScale(sumRtYX, yawDelta)));
			sumE = vecAdd(sumE, vecAdd(vecScale(dc, weight), vecScale(sumYX, yawDelta)));
			sumRtM = vecAdd(sumRtM, vecScale(sumR.mulTransposed(up), yawDelta));
			sumM = vecAdd(sumM, vecScale(up, yawDelta * weight));
		}

		double translationCost(const vr::HmdVector3d_t& delta, double kappa) const
		{
			double N = weight;
			if (N <= 0.0) return 0.0;
			double rr = sumEE - 2.0 * vecDot(delta, sumRtE) - 2.0 * kappa * sumXE + N * vecDot(delta, delta) + 2.0 * kappa * vecDot(delta, sumRtX) + kappa * kappa * sumXX;
			vr::HmdVector3d_t b1 = vecSub(vecSub(sumE, sumR.mul(delta)), vecScale(sumX, kappa));
			double b2 = sumYXE - vecDot(sumRtYX, delta);
			double M[16] = {
				N, 0, 0, sumYX.v[0],
				0, N, 0, sumYX.v[1],
				0, 0, N, sumYX.v[2],
				sumYX.v[0], sumYX.v[1], sumYX.v[2], sumYXYX + 0.01 * N
			};
			double b[4] = { b1.v[0], b1.v[1], b1.v[2], b2 };
			double v[4];
			if (!solveLinearSystem(4, M, b, v))
				return rr;
			double cost = rr - (b1.v[0] * v[0] + b1.v[1] * v[1] + b1.v[2] * v[2] + b2 * v[3]);
			return cost < 0.0 ? 0.0 : cost;
		}

		double rotationCost(const vr::HmdVector3d_t& eps) const
		{
			double N = weight;
			if (N <= 0.0) return 0.0;
			vr::HmdVector3d_t a = sumR.mulTransposed({ 0, 1, 0 });
			double qq = sumMM - 2.0 * vecDot(eps, sumRtM) + N * vecDot(eps, eps);
			double s = sumM.v[1] - vecDot(a, eps);
			double cost = qq - s * s / N;
			return cost < 0.0 ? 0.0 : cost;
		}

		bool solveRotation(vr::HmdVector3d_t& eps, double obs[3], double ridge, double obsMin) const
		{
			double N = weight;
			if (!(N > 0.0)) return false;
			vr::HmdVector3d_t a = sumR.mulTransposed({ 0, 1, 0 });
			Eigen::Matrix3d A;
			Eigen::Vector3d b, prior(eps.v[0], eps.v[1], eps.v[2]), result;
			for (int i = 0; i < 3; i++)
			{
				obs[i] = 1.0 - a.v[i] * a.v[i] / (N * N);
				for (int j = 0; j < 3; j++)
					A(i, j) = (i == j ? 1.0 : 0.0) - a.v[i] * a.v[j] / (N * N);
				b[i] = (sumRtM.v[i] - a.v[i] * sumM.v[1] / N) / N;
			}
			if (!detail::observableUpdate<3>(A, b, prior, Eigen::Vector3d::Constant(obsMin), Eigen::Vector3d::Constant(ridge), result)) return false;
			eps = { result[0], result[1], result[2] };
			return true;
		}

		bool solveTranslation(vr::HmdVector3d_t& delta, double& kappa, double obs[3], double& obsScale,
			double ridge, double spreadRidge, double obsMin, double obsScaleMin, bool scaleEnabled) const
		{
			double N = weight;
			if (!(N > 0.0)) return false;
			for (int i = 0; i < 3; i++)
			{
				double col = 0.0;
				for (int k = 0; k < 3; k++) col += sumR.m[k][i] * sumR.m[k][i];
				obs[i] = 1.0 - col / (N * N);
			}
			obsScale = (sumXX - vecDot(sumX, sumX) / N) / N;

			double B[25], c[5], Cm[15];
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
			double fullRhs[5] = { sumRtE.v[0], sumRtE.v[1], sumRtE.v[2], sumXE, sumYXE };
			for (int i = 0; i < 5; i++)
			{
				double dotc = 0.0;
				for (int k = 0; k < 3; k++) dotc += Cm[k * 5 + i] * sumE.v[k];
				c[i] = fullRhs[i] - dotc / N;
			}
			// Remove the nuisance yaw before testing mount/scale observability.
			// A yaw ridge would make a confounded mount direction look measurable.
			Eigen::Matrix4d information;
			Eigen::Vector4d rhs;
			const double yawInformation = B[24];
			for (int i = 0; i < 4; ++i)
			{
				rhs[i] = c[i] / N;
				for (int j = 0; j < 4; ++j) information(i, j) = B[i * 5 + j] / N;
				if (yawInformation > 1e-12 * N)
				{
					rhs[i] -= B[i * 5 + 4] * c[4] / (yawInformation * N);
					for (int j = 0; j < 4; ++j)
						information(i, j) -= B[i * 5 + 4] * B[4 * 5 + j] / (yawInformation * N);
				}
			}
			Eigen::Vector4d prior(delta.v[0], delta.v[1], delta.v[2], kappa), result = prior;
			if (scaleEnabled)
			{
				if (!detail::observableUpdate<4>(information, rhs, prior,
					Eigen::Vector4d(obsMin, obsMin, obsMin, obsScaleMin),
					Eigen::Vector4d(ridge, ridge, ridge, spreadRidge), result)) return false;
			}
			else
			{
				// A disabled scale remains fixed, and its known contribution stays
				// in the translation residual rather than becoming a free parameter.
				Eigen::Vector3d translationResult;
				if (!detail::observableUpdate<3>(information.topLeftCorner<3, 3>(),
					rhs.head<3>() - information.topRightCorner<3, 1>() * kappa, prior.head<3>(),
					Eigen::Vector3d::Constant(obsMin), Eigen::Vector3d::Constant(ridge), translationResult)) return false;
				result.head<3>() = translationResult;
			}
			delta = { result[0], result[1], result[2] };
			kappa = result[3];
			return true;
		}
	};

	Block current;
	Block previous;
	bool started = false;
	double yawReference = 0.0;

	vr::HmdVector3d_t rotation = { 0, 0, 0 };
	vr::HmdVector3d_t translation = { 0, 0, 0 };
	double scale = 0.0;
	vr::HmdVector3d_t solvedRotation = { 0, 0, 0 };
	vr::HmdVector3d_t solvedTranslation = { 0, 0, 0 };
	double solvedScale = 0.0;
	double obsRotation[3] = { 0, 0, 0 };
	double obsTranslation[3] = { 0, 0, 0 };
	double obsScale = 0.0;
	double lastRmsBefore = 0.0;
	double lastRmsAfter = 0.0;
	double lastRotBefore = 0.0;
	double lastRotAfter = 0.0;
	bool lastAcceptedTranslation = false;
	bool lastAcceptedRotation = false;
	bool lastHadReference = false;
	uint32_t solves = 0;
	uint32_t applied = 0;
	bool suspected = false;

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
		rotation = { 0, 0, 0 };
		translation = { 0, 0, 0 };
		scale = 0.0;
		solvedRotation = { 0, 0, 0 };
		solvedTranslation = { 0, 0, 0 };
		solvedScale = 0.0;
		suspected = false;
	}

	void clearSums()
	{
		current.clear();
		previous.clear();
	}

	double weight() const { return current.weight; }
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
		if (!started) return;
		vr::HmdVector3d_t dc = quaternionRotateVector(quaternionConjugate(quaternionFromYaw(yawReference)), translationDeltaC);
		current.shift(yawDelta, dc);
		previous.shift(yawDelta, dc);
	}

	void add(const vr::HmdQuaternion_t& headRotationBase, const vr::HmdVector3d_t& headPositionBase,
		const vr::HmdQuaternion_t& rawRotation, const vr::HmdVector3d_t& rawPosition,
		double estimatorYaw, double baseScale, double w)
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
		current.add(R, m, e, x, yx, w);
	}

	static double boundedStepFraction(const vr::HmdVector3d_t& prior, const vr::HmdVector3d_t& step, double limit)
	{
		if (vecNorm(vecAdd(prior, step)) <= limit) return 1.0;
		const double a = vecDot(step, step), b = 2.0 * vecDot(prior, step);
		const double c = vecDot(prior, prior) - limit * limit;
		if (a <= 0.0 || c > 1e-12) return 0.0;
		const double discriminant = b * b - 4.0 * a * c;
		if (discriminant < 0.0) return 0.0;
		const double root = std::sqrt(discriminant);
		const double fraction = b > 0.0 ? -2.0 * c / (b + root) : (-b + root) / (2.0 * a);
		return fraction < 0.0 ? 0.0 : (fraction > 1.0 ? 1.0 : fraction);
	}

	bool evaluate(Delta& out)
	{
		out = Delta();
		if (current.weight < blockSeconds) return false;

		vr::HmdVector3d_t epsSol = rotation, delSol = translation;
		double kapSol = scale;
		bool okR = current.solveRotation(epsSol, obsRotation, ridge, obsMin);
		bool okT = current.solveTranslation(delSol, kapSol, obsTranslation, obsScale, ridge, spreadRidge, obsMin, obsScaleMin, scaleEnabled);
		solves++;

		vr::HmdVector3d_t candR = okR ? epsSol : rotation, candT = okT ? delSol : translation;
		double candS = okT ? kapSol : scale;
		solvedRotation = candR;
		solvedTranslation = candT;
		solvedScale = candS;

		double rn = vecNorm(candR), tn = vecNorm(candT);
		// Bound the proposed step, not the whole state: rescaling the prior
		// would erase its unobservable components. Translation and scale share
		// one step fraction because a measured direction can couple both.
		const auto rotationStep = vecSub(candR, rotation);
		candR = vecAdd(rotation, vecScale(rotationStep, boundedStepFraction(rotation, rotationStep, maxRotation)));
		const auto translationStep = vecSub(candT, translation);
		const double scaleStep = candS - scale;
		double fraction = boundedStepFraction(translation, translationStep, maxTranslation);
		if (candS > maxScale && scaleStep > 0.0)
			fraction = std::fmin(fraction, std::fmax(0.0, (maxScale - scale) / scaleStep));
		if (candS < -maxScale && scaleStep < 0.0)
			fraction = std::fmin(fraction, std::fmax(0.0, (-maxScale - scale) / scaleStep));
		candT = vecAdd(translation, vecScale(translationStep, fraction));
		candS = scale + scaleStep * fraction;

		lastHadReference = previous.weight >= blockSeconds * 0.5;
		lastAcceptedTranslation = false;
		lastAcceptedRotation = false;
		lastRmsBefore = lastRmsAfter = lastRotBefore = lastRotAfter = 0.0;

		if (lastHadReference)
		{
			double N = previous.weight;
			lastRmsBefore = std::sqrt(previous.translationCost(translation, scale) / N);
			lastRmsAfter = std::sqrt(previous.translationCost(candT, candS) / N);
			lastRotBefore = std::sqrt(previous.rotationCost(rotation) / N);
			lastRotAfter = std::sqrt(previous.rotationCost(candR) / N);
			lastAcceptedTranslation = okT && lastRmsAfter <= lastRmsBefore - minImprovementTranslation && lastRmsAfter <= minImprovementRatio * lastRmsBefore;
			lastAcceptedRotation = okR && lastRotAfter <= lastRotBefore - minImprovementRotation && lastRotAfter <= minImprovementRatioRotation * lastRotBefore;

			if (lastAcceptedTranslation)
			{
				vr::HmdVector3d_t step = vecSub(candT, translation);
				double sn = vecNorm(step);
				double ds = candS - scale;
				double stepFraction = sn > stepTranslation ? stepTranslation / sn : 1.0;
				if (std::fabs(ds) > stepScale) stepFraction = std::fmin(stepFraction, stepScale / std::fabs(ds));
				step = vecScale(step, stepFraction);
				ds *= stepFraction;
				translation = vecAdd(translation, step);
				out.translation = step;
				scale += ds;
				out.scale = ds;
				suspected = tn > suspectTranslation;
			}
			if (lastAcceptedRotation)
			{
				vr::HmdVector3d_t step = vecSub(candR, rotation);
				double sn = vecNorm(step);
				if (sn > stepRotation) step = vecScale(step, stepRotation / sn);
				rotation = vecAdd(rotation, step);
				out.rotation = step;
				if (rn > suspectRotation) suspected = true;
			}
			if (lastAcceptedTranslation || lastAcceptedRotation)
				applied++;
		}

		previous = current;
		current.clear();
		return true;
	}
};

}
