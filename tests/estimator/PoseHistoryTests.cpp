// SPDX-License-Identifier: AGPL-3.0-only
#include "PoseHistory.h"
#include <algorithm>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
void near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::fabs(actual-expected)>tolerance) {
        std::printf("actual=%.12g expected=%.12g tolerance=%.12g\n",actual,expected,tolerance);
        throw std::runtime_error(message);
    }
}
align::PoseHistorySample linear(double time, double rotLag=0, double posLag=0) {
    align::PoseHistorySample s;
    s.time=time;
    const double angle=2*(time-rotLag);
    s.rotation={std::cos(angle/2),0,std::sin(angle/2),0};
    s.position={3*(time-posLag),-2*(time-posLag),1};
    s.velocity={3,-2,0};
    s.angularVelocity={0,2,0};
    return s;
}
void fill(align::RigidPairHistory& history) {
    for(int i=0;i<=10;++i) {
        require(history.addHmd(linear(i*.01)),"valid HMD history insertion failed");
        require(history.addTracker(linear(i*.01)),"valid tracker history insertion failed");
    }
}
void timing() {
    // Reversing the clock lag sign or sharing one lag across channels breaks this.
    for(const auto lags : {std::pair{.030,.055},std::pair{-.030,-.015},std::pair{-.010,.045}}) {
        align::RigidPairHistory history;
        for(int i=0;i<50;++i) {
            require(history.addHmd(linear(i*.01)),"HMD append failed");
            require(history.addTracker(linear(i*.009+.002,lags.first,lags.second)),"tracker append failed");
        }
        align::RigidPairMatch match;
        require(history.matchAt(.205,lags.first,lags.second,match),"asynchronous samples did not match");
        near(match.trackerRotation.pose.rotation.y,std::sin(.205),1e-12,"rotational lag sign wrong");
        near(match.trackerPosition.pose.position.v[0],.615,1e-12,"positional lag sign wrong");
        near(match.hmd.pose.position.v[0],.615,1e-12,"HMD interpolation wrong");
        near(match.hmd.lowerTime,.20,1e-12,"lower source time lost");
        near(match.hmd.upperTime,.21,1e-12,"upper source time lost");
        near(match.trackerRotation.pose.time,.205+lags.first,1e-12,"rotation query time wrong");
        near(match.trackerPosition.pose.time,.205+lags.second,1e-12,"position query time wrong");
        near(match.trackerPosition.pose.velocity.v[1],-2,1e-12,"historical velocity lost");
        require(match.generation==history.generation(),"pair epoch lost");
    }
    align::RigidPairHistory history; fill(history);
    align::RigidPairMatch match;
    require(history.matchLatest(.11,.02,.03,.05,match),"common bracketed time was not selected");
    near(match.hmdTime,.07,1e-12,"latest time ignored later position target");
    require(!history.matchLatest(.2,0,0,.05,match),"stale common pair accepted");
    require(!history.matchAt(.09,0,.02,match),"tracker extrapolation accepted");
    require(!history.matchAt(-.001,0,0,match),"HMD extrapolation accepted");
    require(!history.matchLatest(.1,0,0,-1,match),"negative lookback accepted");
    require(!history.matchAt(std::numeric_limits<double>::quiet_NaN(),0,0,match),"NaN time accepted");
    require(!history.matchAt(.05,std::numeric_limits<double>::infinity(),0,match),"infinite delay accepted");
    match.hmdTime=123; match.generation=456;
    require(!history.matchAt(3,0,0,match),"unavailable timestamp accepted");
    require(match.hmdTime==123 && match.generation==456,"failed query partially overwrote caller output");
    history.reset();
    constexpr double epochTime=12345678.0;
    for(int i=0;i<=50;++i) {
        auto pose=linear(i*.01); pose.time+=epochTime;
        history.addHmd(pose); history.addTracker(pose);
    }
    require(history.matchLatest(epochTime+.50,-.05,.25,.30,match),"large QPC timestamp lost endpoint to lag roundoff");
    near(match.hmdTime,epochTime+.25,1e-8,"supported extreme lags selected wrong time");
}
void discontinuities() {
    // Any retained pre-gap opposite-stream pose can silently cross coordinate frames.
    align::RigidPairHistory history; fill(history);
    align::RigidPairMatch match;
    const auto initial=history.generation();
    history.reset();
    require(history.generation()!=initial,"explicit reset did not advance epoch");
    require(!history.matchAt(.05,0,0,match),"explicit reset retained old pair");
    require(history.addHmd(linear(.11)),"new HMD epoch rejected");
    require(!history.matchAt(.11,0,0,match),"old tracker reused across reset");
    require(history.addTracker(linear(.11)),"new tracker epoch rejected");
    require(history.matchAt(.11,0,0,match),"fresh exact pair rejected");
    const auto beforeGap=history.generation();
    require(history.addHmd(linear(.30)),"first valid recovery sample rejected");
    require(history.generation()!=beforeGap,"dropout did not advance epoch");
    require(!history.matchAt(.20,0,0,match),"interpolation crossed dropout");
    require(!history.matchAt(.11,0,0,match),"dropout retained old opposite history");
    require(history.addTracker(linear(.30)),"tracker recovery sample rejected");
    require(history.matchAt(.30,0,0,match),"recovery exact pair rejected");
    const auto beforeBackwards=history.generation();
    require(!history.addTracker(linear(.29)),"out-of-order sample accepted");
    require(history.generation()!=beforeBackwards,"backwards time did not invalidate epoch");
    require(!history.matchAt(.30,0,0,match),"backwards time retained usable old history");
    fill(history);
    auto duplicate=linear(.10); duplicate.position.v[0]=9000;
    const auto beforeDuplicate=history.generation();
    require(!history.addTracker(duplicate),"duplicate timestamp replaced accepted pose");
    require(history.generation()==beforeDuplicate,"duplicate timestamp destroyed coherent history");
    require(history.matchAt(.10,0,0,match),"duplicate removed original sample");
    near(match.trackerPosition.pose.position.v[0],.3,1e-12,"duplicate contaminated accepted pose");
    // A copied job can finish after a reset, but its epoch must remain stale.
    const auto snapshot=history;
    history.reset();
    require(snapshot.matchAt(.05,0,0,match),"history copy lost valid snapshot");
    require(match.generation!=history.generation(),"stale worker match appears current after reset");
}
void invalidAndQuaternions() {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for(int invalidField=0;invalidField<6;++invalidField) {
        align::RigidPairHistory history; fill(history);
        auto invalid=linear(.11);
        switch(invalidField) {
        case 0: invalid.time=nan; break;
        case 1: invalid.rotation={0,0,0,0}; break;
        case 2: invalid.rotation.x=nan; break;
        case 3: invalid.position.v[0]=nan; break;
        case 4: invalid.velocity.v[2]=nan; break;
        case 5: invalid.angularVelocity.v[1]=nan; break;
        }
        const auto before=history.generation();
        require(!history.addTracker(invalid),"invalid sample accepted");
        require(history.generation()!=before,"invalid sample did not invalidate epoch");
        align::RigidPairMatch match;
        require(!history.matchAt(.05,0,0,match),"invalid interval retained old match");
    }
    align::RigidPairHistory history;
    auto a=linear(0), b=linear(.01);
    a.rotation={0,2,-2,0};
    b.rotation={0,-2,2,0}; // Same half-turn, opposite quaternion representation.
    require(history.addHmd(a)&&history.addHmd(b),"scaled antipodal quaternions rejected");
    a.rotation={1e300,0,0,0}; b.rotation={1e-300,0,0,0};
    require(history.addTracker(a)&&history.addTracker(b),"finite nonzero quaternion normalization overflowed");
    align::RigidPairMatch match;
    require(history.matchAt(.005,0,0,match),"quaternion interpolation failed");
    near(match.hmd.pose.rotation.x,std::sqrt(.5),1e-12,"antipodal interpolation changed orientation");
    near(match.hmd.pose.rotation.y,-std::sqrt(.5),1e-12,"antipodal mixed axis sign changed");
    near(match.trackerRotation.pose.rotation.w,1,1e-12,"quaternion normalization returned nonunit result");
    // Crossing 180 degrees must take the short arc, rather than pass through identity.
    history.reset(); a=linear(0); b=linear(.01);
    a.rotation={std::cos(179*POSE_PI/360),0,std::sin(179*POSE_PI/360),0};
    b.rotation={std::cos(-179*POSE_PI/360),0,std::sin(-179*POSE_PI/360),0};
    history.addHmd(a); history.addHmd(b); history.addTracker(a); history.addTracker(b);
    require(history.matchAt(.005,0,0,match),"half-turn crossing failed");
    near(std::fabs(match.hmd.pose.rotation.y),1,1e-12,"slerp took long arc around half turn");
}
void bounds() {
    // High-rate history must evict samples even before its time horizon expires.
    align::RigidPairHistory history;
    for(int i=0;i<1024;++i) {
        require(history.addHmd(linear(i*.0001)),"high-rate HMD append failed");
        require(history.addTracker(linear(i*.0001)),"high-rate tracker append failed");
    }
    align::RigidPairMatch match;
    require(!history.matchAt(.01,0,0,match),"capacity failed to evict oldest samples");
    require(history.matchAt(.10225,0,0,match),"wrapped ring cannot interpolate newest pair");
    history.reset();
    for(int i=0;i<201;++i) { history.addHmd(linear(i*.02)); history.addTracker(linear(i*.02)); }
    require(!history.matchAt(2.9,0,0,match),"time retention failed to evict old samples");
    require(history.matchAt(3.99,0,0,match),"retention removed recent samples");
}

// Analytic yaw+translation with acceleration and repeated reversals. The two
// observed channels have different known effective delays; arrival jitter is
// independent of nominal source time. Derivatives below are analytic truth.
align::PoseHistorySample trajectory(double nominal,double rotLag=0,double posLag=0,double rate=1) {
    align::PoseHistorySample s; s.time=nominal;
    double r=rate*(nominal-rotLag), p=rate*(nominal-posLag);
    const double angle=.7*std::sin(7*r)+.18*std::sin(17*r);
    s.rotation={std::cos(angle/2),0,std::sin(angle/2),0};
    s.angularVelocity={0,rate*(4.9*std::cos(7*r)+3.06*std::cos(17*r)),0};
    s.position={.3*std::sin(5*p),1.6+.1*std::cos(9*p),.2*std::sin(11*p)};
    s.velocity={rate*1.5*std::cos(5*p),-rate*.9*std::sin(9*p),rate*2.2*std::cos(11*p)};
    return s;
}
void replayAtRate(double rate) {
    constexpr double tauRot=.027,tauPos=.044;
    align::RigidPairHistory history;
    align::PoseHistorySample latest;
    int trackerIndex=0,count=0;
    double histRot2=0,histPos2=0,latestRot2=0,latestPos2=0;
    double lastPair=-1;
    for(int hmdIndex=0;hmdIndex<1800;++hmdIndex) {
        const double now=hmdIndex/90.0;
        while(true) {
            const double nominal=trackerIndex/120.0+.001;
            const double arrival=nominal+.020+.003*std::sin(trackerIndex*.7);
            if(arrival>now) break;
            latest=trajectory(nominal,tauRot,tauPos,rate);
            require(history.addTracker(latest),"replay tracker append failed");
            ++trackerIndex;
        }
        require(history.addHmd(trajectory(now,0,0,rate)),"replay HMD append failed");
        align::RigidPairMatch pair;
        if(now<.2 || !history.matchLatest(now,tauRot,tauPos,.15,pair)) continue;
        require(pair.hmdTime>lastPair,"replay did not produce advancing historical pairs"); lastPair=pair.hmdTime;
        const double hr=quaternionAngleRad(pair.hmd.pose.rotation*quaternionConjugate(pair.trackerRotation.pose.rotation));
        const double hp=vecNorm(vecSub(pair.hmd.pose.position,pair.trackerPosition.pose.position));
        histRot2+=hr*hr; histPos2+=hp*hp;
        // Baseline is the production latest-sample formula, with perfect supplied
        // angular velocity and acceleration (gain 1), plus its 5cm cap.
        const double dr=now+tauRot-latest.time,dp=now+tauPos-latest.time;
        const auto predictedRotation=quaternionFromAngularVelocity(latest.angularVelocity.v,dr)*latest.rotation;
        const double p=rate*(latest.time-tauPos);
        const vr::HmdVector3d_t acceleration={-rate*rate*7.5*std::sin(5*p),-rate*rate*8.1*std::cos(9*p),-rate*rate*24.2*std::sin(11*p)};
        auto secondOrder=vecScale(acceleration,.5*dp*dp);
        if(vecNorm(secondOrder)>.05) secondOrder=vecScale(secondOrder,.05/vecNorm(secondOrder));
        const auto predictedPosition=vecAdd(vecAdd(latest.position,vecScale(latest.velocity,dp)),secondOrder);
        const auto truth=trajectory(now,0,0,rate);
        const double lr=quaternionAngleRad(truth.rotation*quaternionConjugate(predictedRotation));
        const double lp=vecNorm(vecSub(truth.position,predictedPosition));
        latestRot2+=lr*lr; latestPos2+=lp*lp; ++count;
    }
    require(count>1500,"insufficient replay pairs");
    const double hr=std::sqrt(histRot2/count),hp=std::sqrt(histPos2/count);
    const double lr=std::sqrt(latestRot2/count),lp=std::sqrt(latestPos2/count);
    std::printf("replay rate=%.1f pairs=%d historical_rms_rot_deg=%.9f latest_rms_rot_deg=%.9f historical_rms_pos_mm=%.9f latest_rms_pos_mm=%.9f\n",rate,count,hr*180/POSE_PI,lr*180/POSE_PI,hp*1000,lp*1000);
    require(hr<.002 && hp<.001,"historical interpolation exceeded analytic error bound");
    require(hr<lr*.15 && hp<lp*(rate==1?.15:.5),"historical alignment did not improve accelerated delayed trajectory");
}
void replay() { replayAtRate(1); replayAtRate(.1); }

void controls() {
    // Constant velocity is an exact case for both methods. The benchmark must
    // not manufacture an advantage by giving the baseline incorrect timing.
    align::RigidPairHistory history;
    constexpr double tr=.030,tp=.055;
    for(int i=0;i<50;++i) {
        history.addHmd(linear(i*.01));
        history.addTracker(linear(i*.009+.002,tr,tp));
    }
    const auto latest=linear(.443,tr,tp);
    const auto truth=linear(.49);
    const auto predictedRotation=quaternionFromAngularVelocity(latest.angularVelocity.v,.49+tr-latest.time)*latest.rotation;
    const auto predictedPosition=vecAdd(latest.position,vecScale(latest.velocity,.49+tp-latest.time));
    near(quaternionAngleRad(truth.rotation*quaternionConjugate(predictedRotation)),0,1e-12,"constant-motion latest-sample control failed");
    near(vecNorm(vecSub(truth.position,predictedPosition)),0,1e-12,"constant-motion positional baseline control failed");
    align::RigidPairMatch pair;
    require(history.matchLatest(.49,tr,tp,.2,pair),"constant-motion historical control unavailable");
    near(quaternionAngleRad(pair.hmd.pose.rotation*quaternionConjugate(pair.trackerRotation.pose.rotation)),0,1e-12,"constant-motion historical orientation biased");
    near(vecNorm(vecSub(pair.hmd.pose.position,pair.trackerPosition.pose.position)),0,1e-12,"constant-motion historical translation biased");

    // A fixed mixed-axis extrinsic and lever arm must remain recoverable after
    // asynchronous interpolation; this is a rigid pair, not coincident origins.
    history.reset();
    const vr::HmdQuaternion_t mount{.5,.5,-.5,.5};
    const vr::HmdVector3d_t lever{.03,.12,-.06};
    for(int i=0;i<150;++i) {
        auto tracker=trajectory(i*.007+.001);
        history.addTracker(tracker);
        auto hmd=trajectory(i*.008);
        hmd.position=vecAdd(hmd.position,quaternionRotateVector(hmd.rotation,lever));
        hmd.rotation=hmd.rotation*mount;
        history.addHmd(hmd);
    }
    require(history.matchAt(.801,0,0,pair),"rigid extrinsic control unavailable");
    const auto recoveredRotation=pair.trackerRotation.pose.rotation*mount;
    const auto recoveredPosition=vecAdd(pair.trackerPosition.pose.position,
        quaternionRotateVector(pair.trackerRotation.pose.rotation,lever));
    require(quaternionAngleRad(pair.hmd.pose.rotation*quaternionConjugate(recoveredRotation))<.002,"rigid extrinsic orientation not preserved");
    require(vecNorm(vecSub(pair.hmd.pose.position,recoveredPosition))<.001,"rigid lever arm not preserved");
    std::puts("controls constant_velocity_both_exact=true mixed_axis_mount_error_below_1mm=true");
}
}
int main(int argc,char** argv) {
    try {
        require(argc==2,"expected test case name"); const std::string name=argv[1];
        if(name=="timing") timing(); else if(name=="discontinuities") discontinuities();
        else if(name=="invalid_quaternions") invalidAndQuaternions(); else if(name=="bounds") bounds();
        else if(name=="replay") replay(); else if(name=="controls") controls();
        else throw std::runtime_error("unknown test case");
        std::printf("PASS history_%s\n",argv[1]); return 0;
    } catch(const std::exception& e) { std::fprintf(stderr,"FAIL: %s\n",e.what()); return 1; }
}
