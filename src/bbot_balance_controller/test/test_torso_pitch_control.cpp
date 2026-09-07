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

int main() {
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
    std::cout<<"PASS: uncancelled torso torque, retained leg damping; "<<cases
             <<" moving-hip/IMU/delay cases, final error/rate="<<worst_error<<"/"<<worst_rate
             <<" (not Gazebo validation)\n";
}
