// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include "PoseMath.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

namespace align {
struct PoseHistorySample {
    double time = 0;
    vr::HmdQuaternion_t rotation{1, 0, 0, 0};
    vr::HmdVector3d_t position{}, velocity{}, angularVelocity{};
};
struct PoseHistoryQuery {
    PoseHistorySample pose;
    double lowerTime = 0, upperTime = 0;
};
struct RigidPairMatch {
    double hmdTime = 0;
    std::uint64_t generation = 0;
    PoseHistoryQuery hmd, trackerRotation, trackerPosition;
};
class RigidPairHistory {
public:
    // Nominal timestamps must share one monotonic clock (receive QPC plus the
    // driver's poseTimeOffset). Positions and derivatives must use consistent
    // coordinates within an epoch. No allocation or internal synchronization;
    // the caller serializes this object with its configuration/pose state.
    static constexpr std::size_t Capacity = 256;
    static constexpr double RetentionSeconds = 1.0;
    static constexpr double MaxGapSeconds = .10;

    bool addHmd(const PoseHistorySample& pose) { return append(hmd_, pose); }
    bool addTracker(const PoseHistorySample& pose) { return append(tracker_, pose); }

    // Recenter, coordinate changes, device changes and invalid tracking all
    // invalidate BOTH streams. A match identifies its epoch so asynchronous
    // consumers can reject work computed before a reset.
    void reset() {
        hmd_.clear(); tracker_.clear(); ++generation_;
    }
    std::uint64_t generation() const { return generation_; }

    // Positive lag means tracker(t + lag) matches HMD(t), exactly the convention
    // used by ClockAligner. Each channel needs its own valid source bracket.
    // Failure leaves the output untouched. No pose extrapolation is performed.
    bool matchAt(double hmdTime, double tauRot, double tauPos, RigidPairMatch& output) const {
        if (!std::isfinite(hmdTime) || !std::isfinite(tauRot) || !std::isfinite(tauPos)) return false;
        RigidPairMatch match;
        if (!hmd_.sample(hmdTime, match.hmd) ||
            !tracker_.sample(hmdTime + tauRot, match.trackerRotation) ||
            !tracker_.sample(hmdTime + tauPos, match.trackerPosition)) return false;
        match.hmdTime = hmdTime;
        match.generation = generation_;
        output = match;
        return true;
    }

    // Select the newest fully available measurement at or before the caller's
    // time. This delays learning only; live pose output need not be delayed.
    // The caller separately prevents learning twice from the same hmdTime.
    bool matchLatest(double latestHmdTime, double tauRot, double tauPos,
        double maxLookback, RigidPairMatch& output) const {
        if (!std::isfinite(latestHmdTime) || !std::isfinite(tauRot) || !std::isfinite(tauPos) ||
            !std::isfinite(maxLookback) || maxLookback < 0 || hmd_.empty() || tracker_.empty()) return false;
        const double latest = (std::min)({latestHmdTime, hmd_.back().time,
            tracker_.back().time - tauRot, tracker_.back().time - tauPos});
        if (!std::isfinite(latest) || latestHmdTime - latest > maxLookback) return false;
        return matchAt(latest, tauRot, tauPos, output);
    }

private:
    static bool finiteVector(const vr::HmdVector3d_t& v) {
        return std::isfinite(v.v[0]) && std::isfinite(v.v[1]) && std::isfinite(v.v[2]);
    }
    static bool normalize(vr::HmdQuaternion_t& q) {
        if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) return false;
        // Scale before squaring, so finite nonzero quaternions neither overflow
        // nor underflow their norm. A zero quaternion is invalid, not identity.
        const double scale = (std::max)({std::fabs(q.w),std::fabs(q.x),std::fabs(q.y),std::fabs(q.z)});
        if (scale == 0) return false;
        q.w /= scale; q.x /= scale; q.y /= scale; q.z /= scale;
        const double norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
        return true;
    }
    static vr::HmdVector3d_t interpolate(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b, double f) {
        // Convex sum avoids overflow in b-a for opposite finite endpoints.
        return { (1-f)*a.v[0]+f*b.v[0], (1-f)*a.v[1]+f*b.v[1], (1-f)*a.v[2]+f*b.v[2] };
    }
    static vr::HmdQuaternion_t slerp(const vr::HmdQuaternion_t& a, vr::HmdQuaternion_t b, double f) {
        double dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
        if (dot < 0) { b.w=-b.w; b.x=-b.x; b.y=-b.y; b.z=-b.z; dot=-dot; }
        dot = (std::min)(1.0, dot);
        const double angle = std::acos(dot), sine = std::sin(angle);
        double wa = 1-f, wb = f;
        if (sine > 1e-12) { wa=std::sin((1-f)*angle)/sine; wb=std::sin(f*angle)/sine; }
        vr::HmdQuaternion_t result{wa*a.w+wb*b.w,wa*a.x+wb*b.x,wa*a.y+wb*b.y,wa*a.z+wb*b.z};
        normalize(result);
        return result;
    }

    struct Ring {
        std::array<PoseHistorySample, Capacity> samples{};
        std::size_t first = 0, count = 0;
        bool empty() const { return count == 0; }
        void clear() { first = count = 0; }
        const PoseHistorySample& at(std::size_t i) const { return samples[(first+i)%Capacity]; }
        const PoseHistorySample& back() const { return at(count-1); }
        void pop() { first=(first+1)%Capacity; --count; }
        void push(const PoseHistorySample& pose) {
            if(count == Capacity) pop();
            samples[(first+count)%Capacity] = pose; ++count;
            while(count > 1 && at(0).time < pose.time-RetentionSeconds) pop();
        }
        bool sample(double time, PoseHistoryQuery& output) const {
            if(empty() || !std::isfinite(time)) return false;
            // Subtraction then addition of learned lag can round one ulp beyond
            // an endpoint. Clamp only arithmetic roundoff, never extrapolate it.
            const double tolerance = 4*std::numeric_limits<double>::epsilon()*
                (std::max)({1.0,std::fabs(time),std::fabs(at(0).time),std::fabs(back().time)});
            if(time < at(0).time) {
                if(at(0).time-time > tolerance) return false;
                time=at(0).time;
            }
            if(time > back().time) {
                if(time-back().time > tolerance) return false;
                time=back().time;
            }
            std::size_t lo=0, hi=count-1;
            while(lo < hi) {
                const auto middle=lo+(hi-lo)/2;
                if(at(middle).time < time) lo=middle+1; else hi=middle;
            }
            const auto& b=at(lo);
            if(b.time == time) { output={b,b.time,b.time}; return true; }
            if(lo == 0) return false;
            const auto& a=at(lo-1);
            const double f=(time-a.time)/(b.time-a.time);
            PoseHistoryQuery query;
            query.pose.time=time;
            query.pose.rotation=slerp(a.rotation,b.rotation,f);
            query.pose.position=interpolate(a.position,b.position,f);
            query.pose.velocity=interpolate(a.velocity,b.velocity,f);
            query.pose.angularVelocity=interpolate(a.angularVelocity,b.angularVelocity,f);
            query.lowerTime=a.time; query.upperTime=b.time;
            output=query;
            return true;
        }
    };
    bool append(Ring& stream, PoseHistorySample pose) {
        if(!std::isfinite(pose.time) || !finiteVector(pose.position) || !finiteVector(pose.velocity) ||
            !finiteVector(pose.angularVelocity) || !normalize(pose.rotation)) { reset(); return false; }
        if(!stream.empty()) {
            if(pose.time == stream.back().time) return false; // Preserve first accepted observation.
            if(pose.time < stream.back().time) { reset(); return false; }
            if(pose.time-stream.back().time > MaxGapSeconds) reset();
        }
        stream.push(pose);
        return true;
    }
    Ring hmd_, tracker_;
    std::uint64_t generation_ = 1;
};
}
