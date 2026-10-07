#include "bbot_balance_controller/torso_pitch_control.hpp"
#include "bbot_balance_controller/ground_joint_pd.hpp"
#include <iostream>
#include <stdexcept>

using namespace bbot_jump;
void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }

struct TorsoResult { double peak, final_error, final_rate, torque; };
// Independent moving-hip/rigid-box plant, not a Gazebo wheeled-robot model.
// Include gravity, centripetal/rotational IMU acceleration, actuator delay,
// 100/67/50 Hz sensing, uncertain box inertia and a short landing-force pulse.
TorsoResult run_torso(double mass, int period_ms, double inertia_scale, double initial, double initial_rate) {
    const double cy=.00761282, cz=.12396677, target=.038;
    const double inertia_com=.159013*mass/14.;
    const double inertia_hip=inertia_scale*inertia_com+mass*(cy*cy+cz*cz);
    double pitch=initial, rate=initial_rate, sampled_pitch=pitch;
    double applied=0, pending=0, peak=std::abs(pitch-target), maximum=0;
    TorsoImuObserver imu;
    for (int ms=0;ms<4000;++ms) {
        const double t=.001*ms, stamp=1+t;
        const double ay=5*std::sin(2*M_PI*2*t)*std::exp(-t/.25);
        const double az=18*std::exp(-std::pow((t-.08)/.018,2));
        const double y=cy*std::cos(pitch)+cz*std::sin(pitch);
        const double z=-cy*std::sin(pitch)+cz*std::cos(pitch);
        const double external=mass*((9.81+az)*y-ay*z);
        const double alpha=(applied+external-.05*rate)/inertia_hip;
        if (ms%period_ms==0) {
            // IMU is at COM; compute acceleration from actual plant motion.
            const double acc_y=ay+alpha*z-rate*rate*y;
            const double acc_z=az-alpha*y-rate*rate*z;
            const double fy=acc_y*std::cos(pitch)-(acc_z+9.81)*std::sin(pitch);
            const double fz=acc_y*std::sin(pitch)+(acc_z+9.81)*std::cos(pitch);
            imu.update(stamp,rate,fy,fz);
            sampled_pitch=pitch;
            applied=pending; // one actuator period of delay
        }
        if (ms%5==0) {
            const double horizon=std::clamp(.002*period_ms+.001*(ms%period_ms)+.010,.030,.080);
            const auto cmd=torso_pitch_torque(sampled_pitch,target,imu.rate(),imu.fy(),imu.fz(),mass,55,15,horizon,20);
            pending=2*cmd.command;
            maximum=std::max(maximum,std::abs(pending));
        }
        rate+=.001*(applied+external-.05*rate)/inertia_hip;
        pitch+=.001*rate;
        require(std::isfinite(pitch) && std::isfinite(rate),"nonfinite box dynamics");
        peak=std::max(peak,std::abs(pitch-target));
    }
    return {peak,std::abs(pitch-target),std::abs(rate),maximum};
}

struct TouchdownHandoffResult { double peak_post_capture_rate, final_error, final_rate; };
TouchdownHandoffResult run_touchdown_handoff(bool converge) {
    constexpr double mass=9.5, target=.03, dt=.001;
    constexpr int sensor_period_ms=15, measurement_delay_ms=40;
    const double inertia_com=.159013*mass/14.;
    const double inertia=inertia_com+mass*(.00761282*.00761282+.12396677*.12396677);
    double pitch=.09, rate=-.8, sampled_pitch=pitch, sampled_rate=rate;
    double applied=0.0, pending=0.0;
    std::vector<std::pair<double,std::pair<double,double>>> history;
    TorsoImuObserver delayed_imu;
    double peak_rate=0.0;
    for (int ms=0; ms<3000; ++ms) {
        const double t=dt*ms;
        history.push_back({t,{pitch,rate}});
        if (ms%sensor_period_ms==0) {
            const double wanted=std::max(0.0,t-.001*measurement_delay_ms);
            auto sample=history.front();
            for (const auto & item:history) {
                if (item.first>wanted) break;
                sample=item;
            }
            sampled_pitch=sample.second.first;
            sampled_rate=sample.second.second;
            delayed_imu.update(1.0+sample.first,sampled_rate,0.0,0.0);
            applied=pending; // one sensor-period actuator delay
            while (history.size()>2 && history[1].first<wanted) history.erase(history.begin());
        }
        if (ms%5==0) {
            const double measured_rate=delayed_imu.rate();
            const auto impact=touchdown_torso_pitch_torque(sampled_pitch-target,measured_rate,55,15,20);
            const auto balance=torso_pitch_torque(sampled_pitch,target,measured_rate,
                0.0,0.0,mass,55,15,.060,20);
            const double blend=converge ? touchdown_torso_convergence_blend(t-.50,.20) : 0.0;
            pending=2.0*blend_torso_pitch_torque(
                TorsoPitchTorque{0.0,impact,impact},balance,blend).command;
        }
        const double landing_impulse=3.0*std::sin(2.0*M_PI*2.0*t)*std::exp(-t/.25);
        rate+=dt*(applied+landing_impulse-.05*rate)/inertia;
        pitch+=dt*rate;
        if (t>=.50) peak_rate=std::max(peak_rate,std::abs(rate));
        require(std::isfinite(pitch)&&std::isfinite(rate),"delayed touchdown torso model diverged numerically");
    }
    return {peak_rate,std::abs(pitch-target),std::abs(rate)};
}

int main() {
    require(!touchdown_effort_support_active(false,false) &&
            !touchdown_effort_support_active(true,true),
            "Position support and a pending leg-mode switch must not enable convergence");
    require(touchdown_effort_support_active(true,false),
            "a Position first hop may converge after touchdown Effort support is active");
    require(!touchdown_torso_convergence_ready(
                touchdown_effort_support_active(false,false),.8,true,0.0,0.0,0.0),
            "pre-switch Position support must retain its existing touchdown controller");
    require(!touchdown_torso_convergence_ready(
                touchdown_effort_support_active(true,true),.8,true,0.0,0.0,0.0),
            "pending Effort support switch must retain the impact correction");
    require(touchdown_torso_convergence_ready(
                touchdown_effort_support_active(true,false),.8,true,0.0,0.0,0.0),
            "a captured first-hop landing may converge after Effort support is active");
    require(!touchdown_torso_convergence_ready(true,.499,true,0.0,0.0,0.0),
            "Effort convergence must wait at least 0.50 s after touchdown");
    require(!touchdown_torso_convergence_ready(true,.5,false,0.0,0.0,0.0),
            "stale world COM must not enable touchdown convergence");
    require(!touchdown_torso_convergence_ready(true,.5,true,.1001,0.0,0.0),
            "COM lean outside capture range must not enable convergence");
    require(!touchdown_torso_convergence_ready(true,.5,true,0.0,.3501,0.0),
            "COM velocity outside capture range must not enable convergence");
    require(!touchdown_torso_convergence_ready(true,.5,true,0.0,0.0,.1501),
            "large body angle must retain the asymmetric touchdown correction");
    require(touchdown_torso_convergence_ready(true,.5,true,.10,-.35,-.15),
            "inclusive convergence capture boundaries must be accepted");
    require(touchdown_torso_convergence_blend(0.0)==0.0 &&
            touchdown_torso_convergence_blend(.20)==1.0,
            "convergence blend must begin/end at exact controller endpoints");
    double prior_blend=0.0;
    for (int i=1;i<=200;++i) {
        const double blend=touchdown_torso_convergence_blend(.001*i);
        require(blend>=prior_blend && blend-prior_blend<.008,
                "20 ms blend profile must remain monotonic and continuous");
        prior_blend=blend;
    }
    const TorsoPitchTorque impact_endpoint{0.0,7.0,7.0};
    const TorsoPitchTorque saturated_balance{35.0,12.0,20.0};
    for (int i=0;i<=100;++i) {
        const auto mixed=blend_torso_pitch_torque(impact_endpoint,saturated_balance,.01*i);
        require(std::abs(mixed.command)<=20.0,
                "torso blend must preserve the saturated endpoint torque cap");
    }
    require(blend_torso_pitch_torque(impact_endpoint,saturated_balance,0.0).command==7.0 &&
            blend_torso_pitch_torque(impact_endpoint,saturated_balance,1.0).command==20.0,
            "torso blend must preserve both bounded endpoint commands");
    const auto delayed_legacy=run_touchdown_handoff(false);
    const auto delayed_converged=run_touchdown_handoff(true);
    require(delayed_converged.peak_post_capture_rate<delayed_legacy.peak_post_capture_rate,
            "fresh-COM touchdown convergence must reduce post-capture rate in an independent delayed-sensor model");
    require(delayed_converged.final_error<.01 && delayed_converged.final_rate<.05,
            "smooth torso-law transition must settle the independent delayed-sensor model");

    // The 6.136 s failure row still delivered +0.434 Nm at pitch +1.102 rad,
    // rate +5.361 rad/s. Support cannot cancel the independent torso command.
    const double pitch=1.10237;
    const auto correction=torso_pitch_torque(pitch,.038,5.36116,
        -9.81*std::sin(pitch),9.81*std::cos(pitch),9.5,55,15,.030,20);
    require(correction.command<-10,"large forward divergence has insufficient or wrong-sign correction");
    for (double common:{-200.,0.434327,11.26709,200.}) for (double diff:{-100.,0.,100.}) {
        const JointVector leg(common+diff,-20.8271,common-diff,-19);
        const auto tau=allocate_torso_hips(leg,correction.command,75);
        require(std::abs(.5*(tau[0]+tau[2])-correction.command)<1e-12,"support/shape canceled box correction");
        require(std::abs(tau[0])<=75 && std::abs(tau[2])<=75,"hip difference violates motor limit");
        require(tau[1]==leg[1] && tau[3]==leg[3],"torso allocation changed knee support");
    }
    // Removing mean hip reference before solving must also remove its leakage
    // through the inertia into knee commands; differential/knee damping remains.
    const JointVector q(.2,-.4,.2,-.4), zero=JointVector::Zero();
    const auto m=flight_joint_inertia(q,9.5);
    const JointVector kp(25,45,25,45),kd(3.5,6,3.5,6),mean(1,0,1,0);
    const auto base=discrete_ground_leg_feedback(m,q,zero,q,zero,kp,kd,.025);
    const auto shifted=discrete_ground_leg_feedback(m,q,zero,q+mean,mean,kp,kd,.025);
    require((shifted-base).norm()<1e-12,"common hip IK pulls box or knees");
    const JointVector velocity(.5,1.,-.5,-.8);
    const auto dissipative=discrete_ground_leg_feedback(m,q,velocity,q,zero,kp,kd,.025);
    require(std::abs(dissipative[0]+dissipative[2])<1e-12,"leg servo creates common hip moment");
    require(dissipative.dot(velocity)<0,"differential hip/knee damping removed");

    // Reduced leg dynamics with independent inertia/actuator uncertainty.
    Eigen::Matrix<double,4,3> basis=Eigen::Matrix<double,4,3>::Zero();
    basis(0,0)=std::sqrt(.5); basis(2,0)=-std::sqrt(.5);basis(1,1)=1;basis(3,2)=1;
    for (double scale:{.7,1.,1.3}) for (int period:{10,15,20}) {
        Eigen::Vector3d error(.03,-.02,.025),v(.1,.5,-.3);
        Eigen::Vector3d applied=Eigen::Vector3d::Zero(),pending=applied;
        const Eigen::Matrix3d actual=scale*basis.transpose()*m*basis;
        const auto dynamics=actual.ldlt();
        JointVector qs=q+basis*error,vs=basis*v;
        double peak=0;
        for (int ms=0;ms<3000;++ms) {
            if (ms%period==0) { applied=pending; qs=q+basis*error;vs=basis*v; }
            if (ms%5==0) pending=basis.transpose()*discrete_ground_leg_feedback(
                m,qs,vs,q,zero,kp,kd,std::clamp(.002*period+.001*(ms%period),.025,.060));
            v+=.001*dynamics.solve(applied-.5*v);error+=.001*v;peak=std::max(peak,error.norm());
        }
        require(peak<.1 && error.norm()<.01 && v.norm()<.02,"reduced leg damping diverged");
    }
    TorsoImuObserver imu;
    imu.update(1,0,0,9.81);imu.update(1.01,4,2,9);
    const double filtered=imu.rate();
    require(filtered>2.5 && filtered<2.6,"torso gyro retains long old filter delay");
    imu.update(1.01,-10,-5,4);
    require(imu.rate()==filtered,"duplicate sensor frame filtered twice");
    imu.update(1.02,NAN,0,9.81);
    require(imu.stamp()==1.01,"invalid IMU became fresh");
    require(!imu.fresh(1.1),"stale impact retained as current force");
    imu.update(.5,-1,0,9.81);
    require(imu.fresh(.5) && imu.rate()==-1,"simulation clock reset ignored");
    require(torso_pitch_torque(NAN,0,0,0,0,9.5,55,15,.03,20).command==0,"invalid sensor creates torque");

    int cases=0;double worst_error=0,worst_rate=0;
    for (double mass:{9.5,14.}) for (int period:{10,15,20}) for (double scale:{.7,1.,1.3}) {
        for (const auto & initial: {std::pair<double,double>{-.10,3.5},{.4,2},{.6,5},{-.4,-2},{.04,0}}) {
            const auto result=run_torso(mass,period,scale,initial.first,initial.second);
            require(result.final_error<.01 && result.final_rate<.05 && result.peak<1.3,
                    "sampled torso loop did not recover in independent hinged-box model");
            require(result.torque<=40,"torso correction exceeds existing per-hip 20 Nm cap");
            if (initial.first==-.10) require(result.peak<.25,"near-level touchdown falls forward before correction");
            worst_error=std::max(worst_error,result.final_error);
            worst_rate=std::max(worst_rate,result.final_rate);++cases;
        }
    }
    // Test touchdown_torso_pitch_torque asymmetric damping:
    // 1. Logged failure case: pitch_err = -0.292 rad, pitch_rate = +1.13 rad/s
    // Without asymmetric damping, D term (+16.95 Nm) cancels P term (-16.06 Nm), giving negative torque.
    // With asymmetric damping, D is limited to 45% of P (7.227 Nm), preserving forward restoring torque.
    const double tau_logged = touchdown_torso_pitch_torque(-0.292, 1.13, 55.0, 15.0, 20.0);
    require(tau_logged > 4.40 && tau_logged < 4.43,
            "logged recovering point must deliver positive restoring torque");

    // 2. Near-neutral zone (|pitch_err| <= 0.10): full damping preserved to prevent forward overshoot
    const double tau_neutral = touchdown_torso_pitch_torque(-0.05, 1.0, 55.0, 15.0, 20.0);
    require(std::abs(tau_neutral - (-6.125)) < 1e-12,
            "near-neutral zone must retain full damping to brake forward overshoot");

    // 3. Diverging motion: pitch_err = -0.25 rad, pitch_rate = -1.0 rad/s
    const double tau_diverging = touchdown_torso_pitch_torque(-0.25, -1.0, 55.0, 15.0, 20.0);
    require(std::abs(tau_diverging - 14.375) < 1e-12,
            "diverging motion must receive full reinforcing torque");

    // 4. Clamping at limits
    require(touchdown_torso_pitch_torque(-1.0, -5.0, 55.0, 15.0, 20.0) == 20.0,
            "must clamp to positive limit");
    require(touchdown_torso_pitch_torque(1.0, 5.0, 55.0, 15.0, 20.0) == -20.0,
            "must clamp to negative limit");

    // 5. Continuous smooth transition across 0.10 rad boundary in [0.06, 0.25]
    double prev_tau = touchdown_torso_pitch_torque(-0.04, 3.0, 55.0, 15.0, 20.0);
    for (double err = 0.041; err <= 0.28; err += 0.001) {
        const double curr_tau = touchdown_torso_pitch_torque(-err, 3.0, 55.0, 15.0, 20.0);
        const double step = std::abs(curr_tau - prev_tau);
        require(step < 0.25, "pitch torque must be continuous and smooth across transition zone");
        prev_tau = curr_tau;
    }
    const double tau_0099 = touchdown_torso_pitch_torque(-0.0999, 4.0, 55.0, 15.0, 20.0);
    const double tau_0101 = touchdown_torso_pitch_torque(-0.1001, 4.0, 55.0, 15.0, 20.0);
    require(std::abs(tau_0101 - tau_0099) < 0.02,
            "0.10 rad boundary must not have any discrete jump");

    // 6. Test touchdown_hip_soft_limit_guard
    // Test Case A: q=1.57, q_dot=0, req=+10.0 (pinned against mechanical stop at zero velocity)
    const double tau_pinned = touchdown_hip_soft_limit_guard(1.57, 0.0, 10.0, 20.0);
    require(tau_pinned <= -19.0,
            "q=1.57, q_dot=0, req=+10.0 must output full negative restoring spring torque");

    // Test Case B: q=1.45, q_dot < 0, req=-10.0 (clearly receding from limit with negative request)
    const double tau_receding_neg = touchdown_hip_soft_limit_guard(1.45, -0.5, -10.0, 20.0);
    require(tau_receding_neg == -10.0,
            "q=1.45, q_dot<0, req=-10.0 must be passed through unchanged");

    // Test Case C: q=1.25, q_dot=0, req=+5.0 (inside warning zone [1.20, 1.45], positive torque prohibited)
    const double tau_warning_zero = touchdown_hip_soft_limit_guard(1.25, 0.0, 5.0, 20.0);
    require(tau_warning_zero <= -3.5 && tau_warning_zero < 0.0,
            "q=1.25, q_dot=0 must prohibit positive torque and apply active restoring spring");

    // Test Case D: q=1.10, q_dot=0, req=+15.0 (in safe zone below 1.20 rad)
    const double tau_safe = touchdown_hip_soft_limit_guard(1.10, 0.0, 15.0, 20.0);
    require(tau_safe == 15.0,
            "safe configuration below 1.20 rad must pass through requested torque");

    // Test Case E: High-speed approaching sample q=1.39, q_dot=19.0
    const double tau_brake = touchdown_hip_soft_limit_guard(1.39, 19.0, 5.0, 20.0);
    require(tau_brake <= -18.0,
            "sample q=1.39, q_dot=19 must trigger strong negative soft limit braking");

    // Test Case F: Asymmetric left/right hip states protected independently
    const double tau_left = touchdown_hip_soft_limit_guard(1.05, 0.0, 8.0, 20.0);
    const double tau_right = touchdown_hip_soft_limit_guard(1.57, 0.0, 8.0, 20.0);
    require(tau_left == 8.0,
            "left hip in safe zone must remain at requested +8.0 Nm");
    require(tau_right <= -19.0,
            "right hip at limit must be forced to negative restoring torque");
    require(tau_left != tau_right,
            "left and right hips must be protected independently without shared common clamp");

    std::cout<<"PASS: uncancelled torso torque, retained leg damping; "<<cases
             <<" moving-hip/IMU/delay cases, final error/rate="<<worst_error<<"/"<<worst_rate
             <<", continuous asymmetric damping & independent hip soft limit guard verified\n";
}
