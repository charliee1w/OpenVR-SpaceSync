// SPDX-License-Identifier: AGPL-3.0-only
#include "AlignmentEstimator.h"

#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const char* message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        std::printf("actual=%.12g expected=%.12g tolerance=%.12g\n", actual, expected, tolerance);
        throw std::runtime_error(message);
    }
}

void quaternionTests()
{
    // Independent exact matrices: half-turns about three mixed-sign axes.
    const double matrices[][3][3] = {
        {{0,-1,0},{-1,0,0},{0,0,-1}},
        {{0,0,-1},{0,-1,0},{-1,0,0}},
        {{-1,0,0},{0,0,-1},{0,-1,0}},
        {{1,0,0},{0,1,0},{0,0,1}},
        {{0,0,1},{0,1,0},{-1,0,0}}
    };
    for (const auto& expected : matrices) {
        vr::HmdMatrix34_t input{};
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) input.m[i][j]=static_cast<float>(expected[i][j]);
        const auto q=HmdQuaternion_FromMatrix(input);
        const auto actual=Mat3::fromQuaternion(q);
        near(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z,1,1e-12,"matrix conversion must normalize");
        for (int i=0;i<3;++i) for (int j=0;j<3;++j)
            near(actual.m[i][j],expected[i][j],1e-12,"mixed-axis half-turn orientation changed");
    }
    // Analytic Rodrigues matrices on both sides of pi, rounded as OpenVR floats.
    const double axis[3]={2.0/3,-2.0/3,1.0/3};
    const double angles[]={0.01,1.0,POSE_PI-1e-6,POSE_PI,POSE_PI+1e-6};
    for (double angle:angles) {
        vr::HmdMatrix34_t input{};
        const double c=std::cos(angle), s=std::sin(angle);
        const double cross[3][3]={{0,-axis[2],axis[1]},{axis[2],0,-axis[0]},{-axis[1],axis[0],0}};
        for(int i=0;i<3;++i) for(int j=0;j<3;++j)
            input.m[i][j]=static_cast<float>((i==j?c:0)+(1-c)*axis[i]*axis[j]+s*cross[i][j]);
        const auto actual=Mat3::fromQuaternion(HmdQuaternion_FromMatrix(input));
        for(int i=0;i<3;++i) for(int j=0;j<3;++j)
            near(actual.m[i][j],input.m[i][j],2e-7,"near-half-turn float rotation changed");
    }
    vr::HmdMatrix34_t invalid{};
    invalid.m[1][2]=std::numeric_limits<float>::quiet_NaN();
    const auto fallback=HmdQuaternion_FromMatrix(invalid);
    near(fallback.w,1,0,"invalid matrix did not return finite identity fallback");
    near(fallback.x*fallback.x+fallback.y*fallback.y+fallback.z*fallback.z,0,0,"invalid matrix returned a rotation");
}

void rotationRefinementTests()
{
    align::MountRefiner ref;
    ref.rotation={.01,.01,0};
    const Mat3 R=Mat3::fromQuaternion(quaternionFromRotationVector({0,0,POSE_PI/4}));
    ref.current.add(R,R.mul({.01,.01,.02}),{0,0,0},{0,0,0},{0,0,0},60);
    ref.previous=ref.current;
    align::MountRefiner::Delta out;
    require(ref.evaluate(out),"stationary tilted block should evaluate");
    require(ref.lastAcceptedRotation,"observable rotation improvement should be accepted");
    near(ref.rotation.v[0]+ref.rotation.v[1],.02,1e-12,"fixed tilt erased prior unobservable rotation");
    require(ref.rotation.v[2]>0,"observable rotation component did not improve");
    require(vecNorm(out.rotation)<=ref.stepRotation+1e-12,"rotation step exceeded cap");

    // A large candidate still must not scale the unobservable prior away.
    ref.reset();
    ref.rotation={.015,.015,0};
    ref.current.add(R,R.mul({.015,.015,.09}),{0,0,0},{0,0,0},{0,0,0},60);
    ref.previous=ref.current;
    ref.evaluate(out);
    require(ref.lastAcceptedRotation,"bounded observable correction should improve");
    near(ref.rotation.v[0]+ref.rotation.v[1],.03,1e-12,"rotation bound erased nullspace prior");
    require(vecNorm(ref.rotation)<=ref.maxRotation+1e-12,"rotation total exceeded cap");
}

void translationRefinementTests()
{
    const double a=std::sqrt(.5);
    align::MountRefiner ref;
    ref.translation={.004*a,.004*a,0};
    const vr::HmdVector3d_t truth={.004*a,.004*a,.012};
    // Rotations about n=(1,1,0)/sqrt(2) cannot reveal a lever arm along n.
    for(int i=0;i<120;++i) {
        const double angle=2*POSE_PI*i/120;
        const Mat3 R=Mat3::fromQuaternion(quaternionFromRotationVector({a*angle,a*angle,0}));
        ref.current.add(R,{0,0,0},R.mul(truth),{0,0,0},{0,0,0},.5);
    }
    ref.previous=ref.current;
    align::MountRefiner::Delta out;
    ref.evaluate(out);
    require(ref.lastAcceptedTranslation,"observable lever-arm improvement should be accepted");
    near(a*(ref.translation.v[0]+ref.translation.v[1]),.004,1e-12,"single-axis motion erased prior lever arm");
    require(ref.translation.v[2]>0,"observable lever arm did not improve");

    // x=R*(1,0,0) makes translation_x and scale interchangeable. Exercise
    // both an ordinary correction and one that encounters the total/step caps.
    for (const auto& observable : {vr::HmdVector3d_t{0,0,.012},vr::HmdVector3d_t{.04,0,.09}}) {
        ref.reset();
        ref.translation={-.004,0,0};
        ref.scale=.004;
        for(int i=0;i<240;++i) {
            const double t=2*POSE_PI*i/240;
            const auto q=quaternionFromYaw(t)*quaternionFromRotationVector({1.2*std::sin(3*t),0,.9*std::cos(5*t)});
            const Mat3 R=Mat3::fromQuaternion(q);
            const auto x=R.mul({1,0,0});
            const vr::HmdVector3d_t yx={x.v[2],0,-x.v[0]};
            ref.current.add(R,{0,0,0},R.mul(observable),x,yx,.25);
        }
        ref.previous=ref.current;
        ref.evaluate(out);
        require(ref.lastAcceptedTranslation,"coupled solve should accept observable improvement");
        near(ref.scale-ref.translation.v[0],.008,1e-12,"coupled scale/translation nullspace prior changed");
        require(ref.translation.v[2]>0,"coupled observable lever arm did not improve");
        require(vecNorm(ref.translation)<=ref.maxTranslation+1e-12,"translation total exceeded cap");
        require(std::fabs(ref.scale)<=ref.maxScale+1e-12,"scale total exceeded cap");
        require(vecNorm(out.translation)<=ref.stepTranslation+1e-12,"translation step exceeded cap");
        require(std::fabs(out.scale)<=ref.stepScale+1e-12,"scale step exceeded cap");
    }
}

double speed(double t) { return 1+.5*std::sin(13*t)+.2*std::sin(31*t); }

void seedClock(align::ClockAligner& clock, double lag, double start, double end)
{
    const int n=static_cast<int>(std::round((end-start)*100));
    for(int i=0;i<=n;++i) {
        const double t=start+.01*i;
        clock.hmd.add(t,speed(t),speed(t));
        clock.tracker.add(t,speed(t-lag),speed(t-lag));
    }
}

void clockHistoryTests()
{
    align::ClockAligner clock;
    seedClock(clock,.08,0,4);
    require(clock.solve(4),"healthy timing signal must produce estimate");
    near(clock.tauRot(),.08,.002,"healthy timing estimate is wrong");
    const double learned=clock.tauRot();
    const auto votes=clock.rot.solves;
    clock.noteHmdDiscontinuity();
    require(!clock.solve(4.01),"invalid HMD boundary retained old timing votes");
    near(clock.tauRot(),learned,1e-12,"history boundary erased learned timing");
    require(clock.rot.solves==votes,"history boundary cast another timing vote");

    // Inject unreliable pre-gap timing history, then mark the boundary. The
    // last 50 ms of valid samples cannot legitimize three seconds of old data.
    align::ClockAligner recovered;
    recovered.rot.vote(.012,recovered.tauMin,recovered.tauMax,recovered.smoothing);
    seedClock(recovered,.08,0,4);
    recovered.noteTrackerDiscontinuity();
    seedClock(recovered,0,4.01,4.06);
    require(!recovered.solve(4.06),"short recovery voted using pre-gap tracker history");
    near(recovered.tauRot(),.012,1e-12,"invalid tracker history changed learned timing");
    seedClock(recovered,0,4.07,6.07);
    require(recovered.solve(6.07),"adequate healthy recovery could not learn timing");
    near(recovered.rot.lastFound,0,.002,"recovery still fitted invalid tracker history");

    // A gap at the pose ingress itself must form the same history boundary.
    align::ClockAligner gap;
    for(int i=0;i<=200;++i) {
        const double t=.01*i;
        const double p=t-.5/13*std::cos(13*t)-.2/31*std::cos(31*t);
        gap.addHmdPose(t,quaternionFromYaw(p),{p,0,0});
        gap.addTrackerPose(t,quaternionFromYaw(p),{p,0,0});
    }
    require(gap.solve(2),"valid pose-derived timing signal should solve");
    gap.addHmdPose(2.2,{1,0,0,0},{0,0,0});
    gap.addTrackerPose(2.2,{1,0,0,0},{0,0,0});
    require(!gap.solve(2.2),"pose ingress interpolated across a missing interval");
}

align::MountRefiner::Block richBlock()
{
    align::MountRefiner::Block block;
    // The 24 cube rotations have zero mean, with independent cube positions.
    // This fixture makes all mount and scale directions observable.
    const int permutations[6][3]={{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    const int parity[6]={1,-1,-1,1,1,-1};
    for(int p=0;p<6;++p) for(int signs=0;signs<8;++signs) {
        const int s[3]={(signs&1)?1:-1,(signs&2)?1:-1,(signs&4)?1:-1};
        if(parity[p]*s[0]*s[1]*s[2]!=1) continue;
        Mat3 R=Mat3::zero();
        for(int i=0;i<3;++i) R.m[i][permutations[p][i]]=s[i];
        for(int position=0;position<8;++position) {
            const vr::HmdVector3d_t x={(position&1)?1.0:-1.0,(position&2)?1.0:-1.0,(position&4)?1.0:-1.0};
            const vr::HmdVector3d_t yx={x.v[2],0,-x.v[0]};
            const auto m=vecAdd(R.mul({.012,-.014,.015}),{0,.015,0});
            const auto e=vecAdd(vecAdd(R.mul({.008,-.009,.01}),vecScale(x,.006)),vecAdd({.2,.3,-.1},vecScale(yx,.01)));
            block.add(R,m,e,x,yx,.3125);
        }
    }
    return block;
}

void controls()
{
    align::YawTranslationEstimator e;
    const vr::HmdQuaternion_t identity={1,0,0,0};
    e.update(.3,{2,1,4},{1,1,2},identity,1,1,1.0/90);
    const vr::HmdVector3d_t p={3,2,5}, shift={.5,0,-1};
    const auto before=vecAdd(quaternionRotateVector(e.rotation(),p),e.translation);
    e.rebase(.7,shift,1);
    const auto moved=vecAdd(quaternionRotateVector(quaternionFromYaw(.7),p),shift);
    const auto after=vecAdd(quaternionRotateVector(e.rotation(),moved),e.translation);
    near(vecNorm(vecSub(before,after)),0,1e-12,"rebase failed to preserve mapped point");

    align::MountRefiner ref;
    align::MountRefiner::Delta out;
    require(!ref.evaluate(out),"empty mount block evaluated");
    for(int i=0;i<120;++i)
        ref.current.add(Mat3::identity(),{0,.02,0},{.03,.02,.01},{0,0,0},{0,0,0},.5);
    ref.previous=ref.current;
    ref.evaluate(out);
    require(!ref.lastAcceptedRotation&&!ref.lastAcceptedTranslation,"stationary nuisance offsets changed mount");

    const auto rich=richBlock();
    vr::HmdVector3d_t rotation={0,0,0}, translation={0,0,0};
    double scale=0, obs[3], obsScale=0;
    require(rich.solveRotation(rotation,obs,.05,.15),"rich rotation block rejected");
    require(vecNorm(vecSub(rotation,{.012,-.014,.015}))<.002,"rich rotation correction is inaccurate");
    require(rich.solveTranslation(translation,scale,obs,obsScale,.05,.25,.15,.5,true),"rich translation/scale block rejected");
    require(vecNorm(vecSub(translation,{.008,-.009,.01}))<.002,"rich lever arm correction is inaccurate");
    near(scale,.006,.001,"rich scale correction is inaccurate");
    translation={0,0,0}; scale=.006;
    require(rich.solveTranslation(translation,scale,obs,obsScale,.05,.25,.15,.5,false),"fixed-scale translation solve rejected");
    near(scale,.006,1e-12,"disabled scale changed");
    require(vecNorm(vecSub(translation,{.008,-.009,.01}))<.002,"fixed-scale lever arm correction is inaccurate");

    auto invalid=rich;
    invalid.sumRtM.v[0]=std::numeric_limits<double>::quiet_NaN();
    rotation={.01,.02,.03};
    require(!invalid.solveRotation(rotation,obs,.05,.15),"nonfinite block produced a mount correction");
    near(rotation.v[0],.01,0,"failed solve changed prior rotation");
    near(rotation.v[1],.02,0,"failed solve changed prior rotation");
    near(rotation.v[2],.03,0,"failed solve changed prior rotation");

    align::ClockAligner clock;
    for(int i=0;i<=400;++i) { clock.hmd.add(.01*i,0,0); clock.tracker.add(.01*i,0,0); }
    require(!clock.solve(4),"stationary input produced timing estimate");
}
}

int main(int argc,char** argv)
{
    try {
        require(argc==2,"expected test case name");
        const std::string name=argv[1];
        if(name=="quaternion") quaternionTests();
        else if(name=="rotation_refinement") rotationRefinementTests();
        else if(name=="translation_refinement") translationRefinementTests();
        else if(name=="clock_history") clockHistoryTests();
        else if(name=="controls") controls();
        else throw std::runtime_error("unknown case");
        std::printf("PASS: %s\n",argv[1]);
        return 0;
    } catch(const std::exception& e) { std::printf("FAIL: %s\n",e.what()); return 1; }
}
