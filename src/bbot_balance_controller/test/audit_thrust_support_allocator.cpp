// Offline native-state replay bridge. No ROS interfaces and no command output.
#include <Eigen/QR>
#include <iomanip>
#include <iostream>
#include "bbot_balance_controller/thrust_support_allocator.hpp"
using namespace bbot_jump;
template<class Derived> void array(const char* name,const Eigen::MatrixBase<Derived>& value) {
    std::cout<<",\""<<name<<"\":[";
    for(int r=0;r<value.rows();++r) for(int c=0;c<value.cols();++c) {
        if(r || c) std::cout<<',';
        std::cout<<value(r,c);
    }
    std::cout<<']';
}
int main() {
    double t;
    while(std::cin>>t) {
        ThrustSupportAllocationInput in;
        SupportQ9 observed_acc;
        SupportActuatorVector observed_axis;
        Eigen::Vector4d issued_leg;
        double hip_cap,knee_cap,cmd;
        for(int i=0;i<9;++i)std::cin>>in.dynamics.q[i];
        for(int i=0;i<9;++i)std::cin>>in.dynamics.v[i];
        for(int i=0;i<9;++i)std::cin>>observed_acc[i];
        for(int i=0;i<6;++i)std::cin>>observed_axis[i];
        std::cin>>hip_cap>>knee_cap>>cmd;
        for(int i=0;i<4;++i)std::cin>>issued_leg[i];
        if(!std::cin)return 2;
        auto& d=in.dynamics;
        d.actuator_limits={hip_cap,knee_cap,hip_cap,knee_cap,50.,50.};
        d.actuator_speed_limits={11.,15.,11.,15.,30.,30.};
        d.joint_position_min={-1.52,-1.56,-1.52,-1.56};
        d.joint_position_max={1.52,1.56,1.52,1.56};
        d.prediction_dt=.010; // controller_manager update_rate=100Hz
        d.wheel_relative_acceleration_reference={observed_acc[7],observed_acc[8]};
        ThrustSupportDynamicsModel native_model;
        if(!thrust_support_dynamics_model(d,native_model))return 3;
        // Preserve observed COM vertical acceleration while requesting zero
        // H-dot and COM-to-mean-axle restoring acceleration. These tasks are
        // local diagnostics, not a 20cm or landing trajectory certificate.
        in.tasks.desired_com_vertical_acceleration=
            native_model.com_jacobian.row(1).dot(observed_acc)+native_model.com_bias[1];
        in.tasks.desired_centroidal_hdot=0.;
        const double relative_v=native_model.com_axle_relative_forward_jacobian.dot(d.v);
        in.tasks.desired_com_axle_relative_forward_acceleration=
            std::clamp(100.*(.010-native_model.com_axle_relative_forward)-10.*relative_v,-20.,20.);
        Eigen::Matrix<double,19,19> legacy_eq;
        Eigen::Matrix<double,19,1> legacy_rhs;
        legacy_eq.topRows<15>()=native_model.equality;
        legacy_rhs.head<15>()=native_model.equality_rhs;
        legacy_eq.bottomRows<4>().setZero();
        for(int j=0;j<4;++j)legacy_eq(15+j,9+j)=1.;
        legacy_rhs.tail<4>()=issued_leg;
        const SupportDecisionVector legacy=legacy_eq.completeOrthogonalDecomposition().solve(legacy_rhs);
        const double legacy_res=(legacy_eq*legacy-legacy_rhs).cwiseAbs().maxCoeff();
        const SupportQ9 legacy_acc=legacy.head<9>();
        const SupportActuatorVector legacy_net=legacy.segment<6>(9)-native_model.joint_damping.segment<6>(3);
        for(int mode=0;mode<3;++mode) {
            in.wheel_mode=mode==0?SupportWheelMode::ObservedAccelerationReplay:
                mode==1?SupportWheelMode::FixedMotorEffort:SupportWheelMode::BoundedWheelServo;
            d.actuator_limits[4]=d.actuator_limits[5]=mode==0?50.:10.;
            if(!support_wheel_servo_effort(cmd,0.,{d.v[7],d.v[8]},1.,true,true,false,in.fixed_wheel_motor_effort))return 4;
            auto out=allocate_thrust_support(in);
            std::cout<<std::setprecision(17)<<"{\"time\":"<<t<<",\"mode\":"<<mode
                <<",\"valid\":"<<(out.valid?"true":"false")<<",\"reason\":\""<<out.reason<<"\""
                <<",\"solve_time_us\":"<<out.solve_time_us
                <<",\"phase_one_slack\":"<<out.phase_one_slack
                <<",\"maximum_equality_residual\":"<<out.maximum_equality_residual
                <<",\"maximum_inequality_violation\":"<<out.residuals.max_inequality_violation
                <<",\"legacy_equality_residual\":"<<legacy_res;
            std::cout<<",\"allocated_linear_command\":"<<out.allocated_linear_command;
            array("q",d.q);array("v",d.v);array("native_acceleration",observed_acc);
            array("observed_axis_load",observed_axis);array("issued_leg_command",issued_leg);
            array("legacy_acceleration",legacy_acc);array("legacy_net_axis_load",legacy_net);
            array("equality",out.model.equality);array("equality_rhs",out.model.equality_rhs);
            array("inequality",out.model.inequality);array("inequality_rhs",out.model.inequality_rhs);
            array("decision",out.decision);
            Eigen::Vector3d attained,desired;
            for(int j=0;j<3;++j){attained[j]=out.achieved_task_lhs[j];desired[j]=out.task_rhs[j];}
            array("achieved_task_lhs",attained);array("task_rhs",desired);
            Eigen::Vector3d projection;
            for(int j=0;j<3;++j)projection[j]=out.task_projection_norm[j];
            array("task_projection_norm",projection);
            Eigen::Vector4i stages,iterations;
            for(int j=0;j<4;++j){stages[j]=static_cast<int>(out.qp_diagnostics[j].status);iterations[j]=out.qp_diagnostics[j].iterations;}
            array("stage_status",stages);array("stage_iterations",iterations);
            double rhs;
            array("vertical_row",support_vertical_acceleration_row(out.model,in.tasks.desired_com_vertical_acceleration,rhs));
            array("hdot_row",support_centroidal_hdot_row(out.model,in.tasks.desired_centroidal_hdot,rhs));
            array("relative_row",support_com_axle_relative_forward_acceleration_row(out.model,in.tasks.desired_com_axle_relative_forward_acceleration,rhs));
            array("com_jacobian",out.model.com_jacobian);array("com_bias",out.model.com_bias);
            std::cout<<"}\n";
        }
    }
}
