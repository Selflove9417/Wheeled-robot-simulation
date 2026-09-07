#include "bbot_balance_controller/effort_allocation.hpp"
#include "bbot_balance_controller/centroidal_state.hpp"
#include <iostream>
#include <stdexcept>

void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
int main() {
    using namespace bbot_jump;
    require(std::abs(signed_force_limit(-.25,22,57)-316)<1e-12,"opposing PD was deducted twice");
    require(std::abs(signed_force_limit(-.25,-22,57)-140)<1e-12,"same-direction PD exceeds remaining budget");
    for (double limit:{57.,71.25,142.5}) {
        for (double j:{-.3,-.1,.1,.3}) for (double other:{-50.,-22.,0.,22.,50.}) {
            const double f=signed_force_limit(j,other,limit);
            for (double fraction:{0.,.25,.5,.75,1.})
                require(std::abs(j*f*fraction+other)<=limit+1e-10,"allocated force violates final motor limit");
            require(std::abs(std::abs(j*f+other)-limit)<1e-10,"usable effort still discarded");
        }
    }
    require(std::isinf(signed_force_limit(0,20,57)),"inactive force axis restricts propulsion");
    require(signed_force_limit(.25,58,57)==0,"infeasible non-propulsive torque accepted");
    require(signed_force_limit(NAN,0,57)==0,"invalid Jacobian creates unbounded force");
    for (bool thrust:{false,true}) for (bool sim:{false,true}) for (bool relax:{false,true}) {
        const auto limits=jump_effort_limits(thrust,sim,relax);
        require(limits.hip==((thrust&&sim&&relax)?150:75),"hip relaxation escaped simulation thrust");
        require(limits.knee==((thrust&&sim&&relax)?150:60),"knee relaxation escaped simulation thrust");
    }

    // Static moment balance, independent of PD tuning: if the total COM is
    // over the axle, inverse-dynamics hip torque must balance the body's own
    // weight about the hip. Pure J^T F omits leg/wheel weight and fails this.
    double largest_missing_hip=0;
    for (double body_mass:{9.5,14.0}) for (double h:{-.15,.2,.7,1.2})
        for (double k:{-.8,-.3,.15}) {
            const std::array<double,4> q{h,k,h,k};
            const auto geometry=centroidal_geometry(q,body_mass);
            const Eigen::Vector3d relative=geometry.com-geometry.axle;
            const double balanced_pitch=-std::atan2(relative.y(),relative.z());
            const Eigen::Vector3d up(0,-std::sin(balanced_pitch),std::cos(balanced_pitch));
            const auto gravity=leg_gravity_torques(q,balanced_pitch,body_mass);
            // axle_jacobian averages two wheels, hence factor 2 for one leg.
            const auto support=(-(body_mass+8)*9.81*geometry.axle_jacobian.transpose()*up).eval();
            const Eigen::Vector3d body_com_from_hip=rotate_about_hip(balanced_pitch*-1,
                {0,.13261282-.125,.05396677+.07});
            const double body_torque_required=-body_mass*9.81*body_com_from_hip.y();
            require(std::abs(support[0]+gravity[0]+support[2]+gravity[2]-body_torque_required)<1e-10,
                    "ground support cannot balance box weight with COM over wheels");
            largest_missing_hip=std::max(largest_missing_hip,std::abs(gravity[0]));
        }
    require(largest_missing_hip>1.0,"static fixture did not expose missing hip moment");

    // Check the gravity sign via the CAD potential-energy derivative with
    // base orientation fixed, including asymmetric legs and mass changes.
    for (int i=0;i<100;++i) {
        std::array<double,4> q{.4*std::sin(i),-.5+.3*std::cos(i),.3*std::sin(2*i),-.4};
        const double pitch=.25*std::cos(.5*i), body_mass=i%2?9.5:14.;
        const auto gravity=leg_gravity_torques(q,pitch,body_mass);
        for (int j=0;j<4;++j) {
            auto plus=q,minus=q; plus[j]+=1e-6; minus[j]-=1e-6;
            const double up=(body_mass+8)*9.81*rotate_about_hip(-pitch,centroidal_geometry(plus,body_mass).com).z();
            const double down=(body_mass+8)*9.81*rotate_about_hip(-pitch,centroidal_geometry(minus,body_mass).com).z();
            require(std::abs((up-down)/2e-6-gravity[j])<1e-7,"leg gravity sign or lever arm incorrect");
        }
    }
    // 4.620 s log row: body leans +0.659 rad while total COM lean is just
    // +0.036 rad. Missing hip bias was -2.78 Nm per leg, not motor saturation.
    const auto recorded=leg_gravity_torques({.827027,-.170859,.827027,-.170859},.659081,9.5);
    require(recorded[0]<-2.7 && recorded[0]>-2.9,"recorded missing body-axis compensation changed sign");
    require(recorded[1]>6 && recorded[1]<7,"knee self-weight not included");
    std::cout<<"PASS: signed force budgets, simulation-only limits, static body moment balance, gravity derivatives\n";
}
