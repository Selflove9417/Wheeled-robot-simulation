#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>
#include <Eigen/Eigenvalues>
#include "bbot_balance_controller/flight_joint_pd.hpp"

using namespace bbot_jump;

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}

struct Ref { JointVector q, v, a; };

Ref quintic(const JointVector & q0, const JointVector & v0,
            const JointVector & qf, const JointVector & vf, double duration, double t)
{
    const double T = duration;
    const double u = std::clamp(t, 0.0, T);
    const JointVector delta = qf - q0;
    const JointVector a3 = (10.0 * delta - (6.0 * v0 + 4.0 * vf) * T) / std::pow(T, 3);
    const JointVector a4 = (-15.0 * delta + (8.0 * v0 + 7.0 * vf) * T) / std::pow(T, 4);
    const JointVector a5 = (6.0 * delta - 3.0 * (v0 + vf) * T) / std::pow(T, 5);
    return {q0 + v0*u + a3*std::pow(u,3) + a4*std::pow(u,4) + a5*std::pow(u,5),
            v0 + 3.0*a3*std::pow(u,2) + 4.0*a4*std::pow(u,3) + 5.0*a5*std::pow(u,4),
            6.0*a3*u + 12.0*a4*std::pow(u,2) + 20.0*a5*std::pow(u,3)};
}

struct Replay {
    JointVector q, v;
    double theta_rate, peak_tau, peak_position, peak_speed, max_legacy_rate_error, gate_time;
    double raw_pitch_rate, legacy_pitch_rate;
    JointVector peak_joint_tau;
    bool gate;
};

PitchJointVector independent_full_bias(const JointVector & q,
                                        const PitchJointVector & velocity,
                                        double body_mass)
{
    PitchJointMatrix derivative[5];
    derivative[0].setZero();
    constexpr double eps=1e-5;
    for (int k=0;k<4;++k) {
        JointVector qp=q,qm=q; qp[k]+=eps; qm[k]-=eps;
        derivative[k+1]=(flight_pitch_joint_mass_matrix(qp,body_mass)-
                         flight_pitch_joint_mass_matrix(qm,body_mass))/(2.0*eps);
    }
    PitchJointVector c=PitchJointVector::Zero();
    for (int i=0;i<5;++i) for (int j=0;j<5;++j) for (int k=0;k<5;++k)
        c[i] += 0.5*(derivative[k](i,j)+derivative[j](i,k)-derivative[i](j,k))*
            velocity[j]*velocity[k];
    return c;
}

Replay replay_arrest(const JointVector & q0, const JointVector & v0,
                     const JointVector & qf, const JointVector & vf,
                     double initial_pitch, double initial_raw_pitch_rate,
                     double initial_legacy_pitch_rate, bool feedforward)
{
    constexpr double duration = 0.060, body_mass = 9.5, dt = 0.001;
    const JointVector limits(75, 60, 75, 60);
    JointVector q=q0, v=v0, qs=q0, vs=v0, tau=JointVector::Zero();
    double theta=-initial_pitch, theta_rate=-initial_raw_pitch_rate;
    double legacy_rate=initial_legacy_pitch_rate, peak_tau=0.0;
    double peak_position=q.cwiseAbs().maxCoeff(), peak_speed=v.cwiseAbs().maxCoeff();
    double max_legacy_error=0.0, gate_time=-1.0;
    JointVector peak_joint_tau=JointVector::Zero();
    bool gate=false;
    constexpr double landing_pitch_ref=0.030+0.055;
    for (int ms=0; ms<=80; ++ms)
    {
        const double t=ms*dt;
        if (ms%5==0)
        {
            if (ms%10==0) { qs=q; vs=v; }
            const Ref ref=quintic(q0,v0,qf,vf,duration,t);
            const double horizon=0.025;
            const JointMatrix M=flight_joint_inertia(qs,body_mass);
            const double u=std::clamp(t/0.28,0.0,1.0);
            const double smooth=u*u*(3.0-2.0*u);
            const double smooth_dot=(u>0.0&&u<1.0)?6.0*u*(1.0-u)/0.28:0.0;
            const double pitch_ref=initial_pitch+(landing_pitch_ref-initial_pitch)*smooth;
            const double pitch_ref_rate=std::clamp((landing_pitch_ref-initial_pitch)*smooth_dot,-0.30,0.30);
            const double pitch=-theta;
            const double rate_error=legacy_rate-pitch_ref_rate;
            max_legacy_error=std::max(max_legacy_error,std::abs(rate_error));
            const double tau_hip=std::clamp(-5.0*rate_error-28.0*(pitch-pitch_ref),-3.0,3.0);
            const JointVector existing_ff(tau_hip,0.0,tau_hip,0.0);
            const double knee_speed=std::max(std::abs(vs[1]),std::abs(vs[3]));
            const double fast_blend=std::clamp((knee_speed-4.0)/4.0,0.0,1.0);
            const JointVector kp(14.0-6.0*fast_blend,18.0-10.0*fast_blend,
                                 14.0-6.0*fast_blend,18.0-10.0*fast_blend);
            const JointVector kd(2.8-1.6*fast_blend,3.2-2.2*fast_blend,
                                 2.8-1.6*fast_blend,3.2-2.2*fast_blend);
            tau=discrete_flight_pd(M,qs,vs,ref.q,ref.v,kp,kd,existing_ff,horizon);
            if (feedforward)
            {
                const JointVector ff=flight_joint_dynamics_feedforward(
                    qs,vs,ref.a,theta_rate,body_mass);
                tau += bound_arrest_dynamics_feedforward(
                    ff,limits,arrest_dynamics_feedforward_blend(t));
            }
            tau=tau.cwiseMax(-limits).cwiseMin(limits);
            peak_tau=std::max(peak_tau,tau.cwiseAbs().maxCoeff());
            peak_joint_tau=peak_joint_tau.cwiseMax(tau.cwiseAbs());
            const bool joint_safe=std::max(std::abs(v[0]),std::abs(v[2]))<=5.0 &&
                std::max(std::abs(v[1]),std::abs(v[3]))<=6.5;
            const bool attitude_safe=std::abs(pitch-pitch_ref)<=0.24 &&
                std::abs(rate_error)<=1.10;
            if (!gate && t>=0.015 && joint_safe && attitude_safe) {
                gate=true; gate_time=t;
            }
            if (gate) break;
        }
        PitchJointVector full_v; full_v << theta_rate,v;
        const PitchJointMassMatrix S=flight_pitch_joint_mass_matrix(q,body_mass);
        const PitchJointVector c=independent_full_bias(q,full_v,body_mass);
        PitchJointVector force; force << 0.0,tau;
        force.tail<4>() -= 0.5*v;  // URDF joint viscous damping.
        const PitchJointVector acceleration=S.ldlt().solve(force-c);
        theta_rate += dt*acceleration[0];
        v += dt*acceleration.tail<4>();
        theta += dt*theta_rate;
        q += dt*v;
        peak_position=std::max(peak_position,q.cwiseAbs().maxCoeff());
        peak_speed=std::max(peak_speed,v.cwiseAbs().maxCoeff());
        if (ms%5==0)
            legacy_rate += 0.15*((-theta_rate)-legacy_rate);
        require(q.allFinite() && v.allFinite() && std::isfinite(theta_rate),"non-finite replay");
    }
    return {q,v,theta_rate,peak_tau,peak_position,peak_speed,max_legacy_error,gate_time,
            -theta_rate,legacy_rate,peak_joint_tau,gate};
}

void mass_and_energy_identities()
{
    const JointVector q(.722195,-.955879,.722194,-.955878);
    const JointVector v(7.0795,-11.7939,7.07944,-11.7939);
    const double body_mass=9.5, theta_rate=-0.00126893;
    const auto S=flight_pitch_joint_mass_matrix(q,body_mass);
    const auto M=flight_joint_inertia(q,body_mass);
    require((S-S.transpose()).norm()<1e-11,"pitch-retained mass is not symmetric");
    require(Eigen::SelfAdjointEigenSolver<PitchJointMassMatrix>(S).eigenvalues().minCoeff()>0.0,
            "pitch-retained mass is not positive definite");
    require((M-(S.bottomRightCorner<4,4>()-
        S.bottomLeftCorner<4,1>()*(S.topLeftCorner<1,1>().ldlt().solve(S.topRightCorner<1,4>())))).norm()<2e-8,
        "nested Schur complement differs from reduced flight inertia");

    JointVector C;
    require(flight_joint_coriolis_bias(q,v,theta_rate,body_mass,C),"valid C_eff rejected");
    PitchJointVector full_v; full_v << theta_rate,v;
    PitchJointMatrix dS[5]; dS[0].setZero();
    constexpr double eps=1e-5;
    for (int k=0;k<4;++k) {
        JointVector qp=q,qm=q; qp[k]+=eps; qm[k]-=eps;
        dS[k+1]=(flight_pitch_joint_mass_matrix(qp,body_mass)-
                 flight_pitch_joint_mass_matrix(qm,body_mass))/(2*eps);
    }
    PitchJointMatrix Sdot=PitchJointMatrix::Zero();
    for (int k=0;k<5;++k) Sdot += dS[k]*full_v[k];
    PitchJointVector c=PitchJointVector::Zero();
    for (int i=0;i<5;++i) for (int j=0;j<5;++j) for (int k=0;k<5;++k)
        c[i]+=0.5*(dS[k](i,j)+dS[j](i,k)-dS[i](j,k))*full_v[j]*full_v[k];
    require(std::abs(full_v.dot(c)-0.5*full_v.dot(Sdot*full_v))<2e-5,
            "Christoffel bias violates kinetic-energy identity");

    // The retained-pitch bias is independently anchored to the real J2 ARREST
    // snapshots; this catches a 4D-only Christoffel model (which assumes zero
    // base angular momentum) as well as a pitch-rate sign inversion.
    const double knee_bias= C[1];
    const JointVector expected_c_q = c.tail<4>() -
        S.block<4,1>(1,0)*(c[0]/S(0,0));
    require((C-expected_c_q).norm()<1e-9,
            "C_eff does not preserve the theta-row angular-momentum coupling");
    require(knee_bias>4.5 && knee_bias<6.2,
            "J2 initial C_eff knee bias outside CAD replay range");
    struct Snapshot { JointVector q,v; double theta_rate, expected_knee; };
    const Snapshot snapshots[] = {
        {q,v,theta_rate,5.394},
        {JointVector(.890850,-1.26482,.890844,-1.26482),
         JointVector(4.37223,-9.65411,4.37191,-9.65389),1.052,3.172},
        {JointVector(.989534,-1.51822,.989517,-1.51821),
         JointVector(2.23757,-6.98049,2.23723,-6.98037),1.59579,1.402},
    };
    for (const auto & s : snapshots) {
        JointVector snapshot_bias;
        require(flight_joint_coriolis_bias(s.q,s.v,s.theta_rate,body_mass,snapshot_bias),
                "J2 snapshot C_eff rejected");
        require(std::abs(snapshot_bias[1]-s.expected_knee)<0.30,
                "5D C_eff disagrees with independent full-theta J2 CAD replay");
    }
    JointVector zero_qdd=JointVector::Zero();
    JointVector inertia, bias;
    const auto no_accel=flight_joint_dynamics_feedforward(q,v,zero_qdd,theta_rate,body_mass,&inertia,&bias);
    require(inertia.norm()<1e-12 && (no_accel-bias).norm()<1e-12,
            "zero acceleration must leave only the velocity bias");
    require(flight_joint_dynamics_feedforward(q,JointVector::Zero(),zero_qdd,0.0,body_mass).norm()<1e-10,
            "zero velocity and acceleration must produce zero feedforward");
    inertia.setConstant(123.0); bias.setConstant(-123.0);
    const auto invalid=flight_joint_dynamics_feedforward(
        q,v,zero_qdd,NAN,body_mass,&inertia,&bias);
    require(invalid.norm()==0.0 && inertia.norm()==0.0 && bias.norm()==0.0,
            "invalid inputs must zero every output diagnostic");
}

void guard_blend_and_bounds()
{
    const JointVector q=JointVector::Zero(), v=JointVector::Zero(), a=JointVector::Ones();
    auto g=arrest_dynamics_guard(false,true,true,false,1,.99,.99,q,v,a,0);
    require(g==ArrestDynamicsGuard::DisabledOrWrongPhase,"disabled flag guard failed");
    g=arrest_dynamics_guard(true,false,true,false,1,.99,.99,q,v,a,0);
    require(g==ArrestDynamicsGuard::DisabledOrWrongPhase,"non-ARREST guard failed");
    g=arrest_dynamics_guard(true,true,false,false,1,.99,.99,q,v,a,0);
    require(g==ArrestDynamicsGuard::EffortUnavailable,"non-Effort guard failed");
    g=arrest_dynamics_guard(true,true,true,true,1,.99,.99,q,v,a,0);
    require(g==ArrestDynamicsGuard::EffortUnavailable,"pending switch guard failed");
    g=arrest_dynamics_guard(true,true,true,false,1,.90,.99,q,v,a,0);
    require(g==ArrestDynamicsGuard::JointSampleStale,"stale joint guard failed");
    g=arrest_dynamics_guard(true,true,true,false,1,.99,.90,q,v,a,0);
    require(g==ArrestDynamicsGuard::ImuSampleStale,"stale IMU guard failed");
    g=arrest_dynamics_guard(true,true,true,false,1,.99,.99,q,v,a,0);
    require(g==ArrestDynamicsGuard::Active,"valid guard rejected");
    JointVector invalid=q; invalid[2]=NAN;
    g=arrest_dynamics_guard(true,true,true,false,1,.99,.99,invalid,v,a,0);
    require(g==ArrestDynamicsGuard::InvalidInput,"non-finite joint guard failed");

    require(arrest_dynamics_feedforward_blend(0.0)==0.0 &&
            arrest_dynamics_feedforward_blend(.010)>.49 &&
            arrest_dynamics_feedforward_blend(.010)<.51 &&
            arrest_dynamics_feedforward_blend(.020)==1.0,
            "ARREST 20 ms entry blend is discontinuous or malformed");
    const JointVector limits(75,60,75,60), raw(100,-100,20,-20);
    const auto bounded=bound_arrest_dynamics_feedforward(raw,limits,.5);
    require((bounded-JointVector(50,-50,10,-10)).norm()<1e-12,
            "bounded feedforward blend/caps are incorrect");
    require(bound_arrest_dynamics_feedforward(raw,limits,0).norm()==0.0,
            "zero blend must produce zero feedforward");
}

void unoptimized_feedforward_cost()
{
    const JointVector q(.722195,-.955879,.722194,-.955878);
    const JointVector v(7.0795,-11.7939,7.07944,-11.7939);
    const JointVector a(120.0,-180.0,120.0,-180.0);
    constexpr double theta_rate=0.00126893, body_mass=9.5;
    volatile double sink=0.0;
    for (int i=0;i<8;++i)
        sink += flight_joint_dynamics_feedforward(q,v,a,theta_rate,body_mass)[i%4];
    std::vector<double> us;
    us.reserve(101);
    for (int i=0;i<101;++i) {
        const auto begin=std::chrono::steady_clock::now();
        const JointVector ff=flight_joint_dynamics_feedforward(q,v,a,theta_rate,body_mass);
        const auto end=std::chrono::steady_clock::now();
        sink += ff[i%4];
        us.push_back(std::chrono::duration<double,std::micro>(end-begin).count());
    }
    std::sort(us.begin(),us.end());
    const double mean=std::accumulate(us.begin(),us.end(),0.0)/us.size();
    std::cout << "unoptimized dynamics FF cost us: mean=" << mean
              << ", p99=" << us[99] << ", max=" << us.back()
              << " (sink=" << sink << ")\n";
    require(us[99]<5000.0,"one dynamics FF evaluation exceeded 5 ms controller period");
}

void measured_j1_j2_forward_model()
{
    struct Case { const char *name; JointVector q0,v0,qf,vf; double pitch,raw_rate,legacy_rate; };
    const Case cases[] = {
        {"J1", JointVector(.609618,-.729113,.609625,-.729114),
         JointVector(7.71009,-9.88278,7.71032,-9.88288),
         JointVector(.900921,-1.14560,.900928,-1.14560),
         JointVector(2,-4,2,-4), .0568635,.327756,-.108642},
        {"J2", JointVector(.722195,-.955879,.722194,-.955878),
         JointVector(7.0795,-11.7939,7.07944,-11.7939),
         JointVector(.99458,-1.42970,.99458,-1.42970),
         JointVector(2,-4,2,-4), .0704341,.00126893,-.0821226},
    };
    for (const auto & c : cases)
    {
        const auto candidate=replay_arrest(c.q0,c.v0,c.qf,c.vf,c.pitch,c.raw_rate,
                                           c.legacy_rate,true);
        require(candidate.gate,
                "ARREST candidate did not reach existing joint and filtered-bodyrate handoff gates by 80 ms");
        require(candidate.peak_position<1.57,
                "forward replay reached the URDF hard position limit");
        require(candidate.peak_speed<30.0,
                "forward replay exceeded the URDF joint speed limit");
        require(candidate.peak_tau<=75.0+1e-9,
                "forward replay exceeded the existing hip effort limit");
        require((candidate.peak_joint_tau.array()<=JointVector(75,60,75,60).array()+1e-9).all(),
                "forward replay exceeded a per-joint effort limit");
        require(std::abs(candidate.v[1])<=6.5 && std::abs(candidate.v[3])<=6.5 &&
                std::abs(candidate.v[0])<=5.0 && std::abs(candidate.v[2])<=5.0,
                "candidate handoff did not satisfy existing hip/knee speed gates");
        std::cout << c.name << " ideal CAD replay (wheel reaction omitted): gate=" << candidate.gate_time
                  << " s, qmax=" << candidate.peak_position
                  << ", vmax=" << candidate.peak_speed
                  << ", filtered rate error max=" << candidate.max_legacy_rate_error
                  << ", raw/legacy pitch rate=" << candidate.raw_pitch_rate << "/"
                  << candidate.legacy_pitch_rate
                  << ", torque peak=" << candidate.peak_tau << '\n';
    }
}
} // namespace

int main()
{
    mass_and_energy_identities();
    guard_blend_and_bounds();
    unoptimized_feedforward_cost();
    measured_j1_j2_forward_model();
    std::cout << "PASS: theta-retained CAD dynamics, energy identity, guards, caps and J2 ARREST replay\n";
}
