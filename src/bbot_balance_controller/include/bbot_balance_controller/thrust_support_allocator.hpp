#pragma once

// Offline support-task allocation. Observed wheel acceleration is a replay
// condition, never a promise about the existing velocity actuator.
#include <Eigen/SVD>
#include <chrono>
#include "bbot_balance_controller/dense_active_set_qp.hpp"
#include "bbot_balance_controller/thrust_support_dynamics.hpp"

namespace bbot_jump {
enum class SupportWheelMode { Unspecified, ObservedAccelerationReplay, FixedMotorEffort, BoundedWheelServo };
struct ThrustSupportAllocationInput {
    ThrustSupportDynamicsInput dynamics;
    ThrustSupportTasks tasks;
    SupportWheelMode wheel_mode = SupportWheelMode::Unspecified;
    std::array<double,2> fixed_wheel_motor_effort{};
    double wheel_servo_gain = 1.;
    // Private capture path only: eliminate antisymmetric leg acceleration
    // before projecting a nearly redundant mean COM/axle task. This is a
    // generalized-acceleration equality inside allocation, never added PD torque.
    bool enforce_hip_difference_acceleration = false;
    bool enforce_equal_normal_loads = false;
    double desired_hip_difference_acceleration = 0.;
    int maximum_qp_iterations = 200;
};
struct ThrustSupportAllocation {
    bool valid = false;
    const char *reason = "invalid_input";
    SupportDecisionVector decision = SupportDecisionVector::Zero();
    ThrustSupportDynamicsModel model;
    ThrustSupportTaskResiduals residuals;
    std::array<double,4> achieved_task_lhs{};
    std::array<double,4> task_rhs{};
    std::array<bbot_balance_controller::QpDiagnostics,5> qp_diagnostics{};
    double phase_one_slack = 0.;
    double maximum_equality_residual = 0.;
    double solve_time_us = 0.;
    std::array<double,2> allocated_wheel_rate_target{};
    double allocated_linear_command = 0.;
    std::array<double,4> task_projection_norm{};
};

// Convert the controller's already protected per-wheel vertical force and
// synchronized momentum/relative-position observations into support tasks.
// H and Hdot use the model's theta-axis sign and SI units (kg m^2/s,
// kg m^2/s^2). The caller owns sensor freshness and contact guards.
inline bool make_thrust_support_task_reference(
    double protected_vertical_force_per_wheel, double total_mass,
    double centroidal_h, double h_target, double remaining_window,
    double com_axle_forward, double com_axle_forward_velocity,
    ThrustSupportTasks &tasks)
{
    const std::array<double, 7> values{protected_vertical_force_per_wheel,
        total_mass, centroidal_h, h_target, remaining_window,
        com_axle_forward, com_axle_forward_velocity};
    for (double value : values) if (!std::isfinite(value)) return false;
    if (total_mass <= 0.0 || remaining_window <= 0.0) return false;
    tasks.desired_com_vertical_acceleration =
        2.0 * protected_vertical_force_per_wheel / total_mass - 9.81;
    tasks.desired_centroidal_hdot = (h_target - centroidal_h) / remaining_window;
    tasks.desired_com_axle_relative_forward_acceleration = std::clamp(
        100.0 * (0.010 - com_axle_forward) - 10.0 * com_axle_forward_velocity,
        -20.0, 20.0);
    tasks.desired_com_forward_acceleration = 0.0;
    return std::isfinite(tasks.desired_com_vertical_acceleration) &&
        std::isfinite(tasks.desired_centroidal_hdot) &&
        std::isfinite(tasks.desired_com_axle_relative_forward_acceleration);
}

// Discrete body-pitch acceleration feedback inside the allocation task, not
// an extra actuator PD. Evaluate the reference at the end of the 10 ms input
// hold to avoid continuous PD pretending there is no actuator/sensor delay.
inline bool make_thrust_body_pitch_reference(double pitch,double pitch_rate,
    double target_pitch,double input_hold,ThrustSupportTasks &tasks) {
    if(!std::isfinite(pitch)||!std::isfinite(pitch_rate)||
       !std::isfinite(target_pitch)||std::abs(target_pitch)>.15||
       !std::isfinite(input_hold)||input_hold<=0.||input_hold>.030)return false;
    constexpr double kp=144.,kd=24.;
    const double acceleration=(kp*(target_pitch-pitch-pitch_rate*input_hold)-kd*pitch_rate)/
        (1.+kd*input_hold+.5*kp*input_hold*input_hold);
    tasks.desired_body_pitch_acceleration=std::clamp(acceleration,-35.,35.);
    tasks.body_pitch_enabled=true;
    return std::isfinite(tasks.desired_body_pitch_acceleration);
}

// Match the existing optional bounded wheel-effort velocity servo. The caller
// supplies stamp freshness/zero-effort state; this function reads no sensors.
inline bool support_wheel_servo_effort(double linear, double angular,
    const std::array<double,2>& measured, double gain, bool command_fresh,
    bool joints_fresh, bool zero_effort, std::array<double,2>& effort) {
    effort = {};
    if (!std::isfinite(linear) || !std::isfinite(angular) ||
        !std::isfinite(measured[0]) || !std::isfinite(measured[1]) ||
        (gain != .5 && gain != 1. && gain != 2.)) return false;
    if (!command_fresh || !joints_fresh || zero_effort) return true;
    for (int side=0;side<2;++side) {
        const double target=std::clamp((linear+(side==0 ? -1. : 1.)*.182*angular)/.07,-30.,30.);
        effort[side]=std::clamp(gain*(target-measured[side]),-10.,10.);
    }
    return true;
}

namespace support_allocator_detail {
struct ReducedSet {
    Eigen::VectorXd origin;
    Eigen::MatrixXd nullspace, inequality;
    Eigen::VectorXd rhs;
};
inline void update_inequalities(const ThrustSupportDynamicsModel& model, ReducedSet& out) {
    const int rows=model.inequality_count;
    out.inequality=model.inequality.topRows(rows)*out.nullspace;
    out.rhs=model.inequality_rhs.head(rows)-model.inequality.topRows(rows)*out.origin;
    for(int i=0;i<rows;++i) {
        const double norm=out.inequality.row(i).norm();
        // A constant row can have a roundoff residual after a boundary solve.
        // Keep genuinely inconsistent constants; final physical-unit residuals
        // independently gate every returned solution.
        if(norm<1e-12 && out.rhs[i]>=-1e-8) {
            out.inequality.row(i).setZero();out.rhs[i]=std::max(0.,out.rhs[i]);
        }
        const double scale=std::max(1e-6,norm);
        out.inequality.row(i)/=scale;out.rhs[i]/=scale;
    }
}
inline bool reduce(const Eigen::MatrixXd& eq, const Eigen::VectorXd& rhs,
    const ThrustSupportDynamicsModel& model, ReducedSet& out) {
    Eigen::MatrixXd scaled=eq;
    Eigen::VectorXd scaled_rhs=rhs;
    for (int i=0;i<eq.rows();++i) {
        const double s=std::max(1e-12,eq.row(i).norm());
        scaled.row(i)/=s; scaled_rhs[i]/=s;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled,Eigen::ComputeFullU|Eigen::ComputeFullV);
    svd.setThreshold(1e-10);
    out.origin=svd.solve(scaled_rhs);
    if (!out.origin.allFinite() || (eq*out.origin-rhs).cwiseAbs().maxCoeff()>1e-7) return false;
    out.nullspace=svd.matrixV().rightCols(19-svd.rank());
    update_inequalities(model,out);
    return out.inequality.allFinite() && out.rhs.allFinite();
}
inline bool feasible_start(const ReducedSet& set, int iterations,
    Eigen::VectorXd& z, double& slack) {
    using namespace bbot_balance_controller;
    const int n=set.nullspace.cols(),m=set.inequality.rows();
    z=Eigen::VectorXd::Zero(n);
    slack=0.;
    if (set.rhs.minCoeff()>=-1e-9) return true;
    if (n==0) { slack=-set.rhs.minCoeff(); return false; }
    // Phase I: one nonnegative common violation variable. A linear penalty
    // dominates the tiny strictly-convex tie-breaker. Hard constraints are
    // never relaxed in a returned allocation.
    Eigen::MatrixXd a=Eigen::MatrixXd::Zero(m+1,n+1);
    a.topLeftCorner(m,n)=set.inequality;
    a.topRightCorner(m,1).setConstant(-1.);
    a(m,n)=-1.;
    Eigen::VectorXd b=Eigen::VectorXd::Zero(m+1); b.head(m)=set.rhs;
    Eigen::MatrixXd h=1e-8*Eigen::MatrixXd::Identity(n+1,n+1);
    Eigen::VectorXd g=Eigen::VectorXd::Zero(n+1); g[n]=1.;
    Eigen::VectorXd start=Eigen::VectorXd::Zero(n+1);
    start[n]=std::max(0.,-set.rhs.minCoeff())+1.;
    DenseActiveSetQp qp(n+1,m+1);
    QpTolerances tolerance; tolerance.max_iterations=iterations;
    tolerance.step_relative=1e-9; qp.set_tolerances(tolerance);
    if (qp.solve(h,g,a,b,nullptr,&start)!=QpStatus::kConverged) return false;
    z=qp.solution().head(n); slack=qp.solution()[n];
    return slack<=1e-7 && (set.inequality*z-set.rhs).maxCoeff()<=1e-7;
}
} // namespace support_allocator_detail

inline ThrustSupportAllocation allocate_thrust_support(const ThrustSupportAllocationInput& in) {
    using namespace bbot_balance_controller;
    using namespace support_allocator_detail;
    const auto start=std::chrono::steady_clock::now();
    ThrustSupportAllocation out;
    const auto finish=[&](const char* reason) {
        out.reason=reason;
        out.solve_time_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
        return out;
    };
    if (in.wheel_mode==SupportWheelMode::Unspecified || in.maximum_qp_iterations<=0 ||
        in.maximum_qp_iterations>1000 || !thrust_support_dynamics_model(in.dynamics,out.model))
        return finish("invalid_input");
    if (in.wheel_mode==SupportWheelMode::FixedMotorEffort) {
        for (int side=0;side<2;++side) {
            const double effort=in.fixed_wheel_motor_effort[side];
            if (!std::isfinite(effort) || std::abs(effort)>std::min(10.,in.dynamics.actuator_limits[4+side]))
                return finish("invalid_wheel_effort");
            out.model.equality.row(13+side).setZero();
            out.model.equality(13+side,13+side)=1.;
            out.model.equality_rhs[13+side]=effort;
        }
    } else if (in.wheel_mode==SupportWheelMode::BoundedWheelServo) {
        if (in.wheel_servo_gain!=.5 && in.wheel_servo_gain!=1. && in.wheel_servo_gain!=2.)
            return finish("invalid_wheel_servo_gain");
        // Invert the existing P servo exactly: target = measured + tau/gain.
        // Both wheel targets must be equal (straight sagittal jump), and the
        // common linear command stays inside +/-2m/s. A speed reference is
        // not an acceleration equality. The wheel acceleration remains free.
        out.model.equality.bottomRows<2>().setZero();
        out.model.equality_rhs.tail<2>().setZero();
        out.model.equality(13,13)=1.; out.model.equality(13,14)=-1.;
        out.model.equality_rhs[13]=in.wheel_servo_gain*(in.dynamics.v[8]-in.dynamics.v[7]);
        for(int side=0;side<2;++side) {
            const double cap=std::min(10.,in.dynamics.actuator_limits[4+side]);
            constexpr double max_reference_rate=2./.07;
            out.model.inequality_rhs[8+2*side]=std::min(cap,
                in.wheel_servo_gain*(max_reference_rate-in.dynamics.v[7+side]));
            out.model.inequality_rhs[9+2*side]=std::min(cap,
                in.wheel_servo_gain*(max_reference_rate+in.dynamics.v[7+side]));
        }
    } else if (in.wheel_mode!=SupportWheelMode::ObservedAccelerationReplay) {
        return finish("invalid_wheel_mode");
    }
    const int task_count=in.tasks.body_pitch_enabled?4:3;
    std::array<SupportTaskRow,4> task;
    task[0]=support_vertical_acceleration_row(out.model,in.tasks.desired_com_vertical_acceleration,out.task_rhs[0]);
    int h_stage=1,rel_stage=2;
    if(in.tasks.body_pitch_enabled) {
        task[1]=support_body_pitch_acceleration_row(out.model,in.tasks.desired_body_pitch_acceleration,out.task_rhs[1]);
        h_stage=2;rel_stage=3;
    }
    task[h_stage]=support_centroidal_hdot_row(out.model,in.tasks.desired_centroidal_hdot,out.task_rhs[h_stage]);
    task[rel_stage]=support_com_axle_relative_forward_acceleration_row(out.model,
        in.tasks.desired_com_axle_relative_forward_acceleration,out.task_rhs[rel_stage]);
    for(int j=0;j<task_count;++j) if(!std::isfinite(out.task_rhs[j]))return finish("invalid_task");
    if(!std::isfinite(in.tasks.desired_com_forward_acceleration))return finish("invalid_task");
    Eigen::MatrixXd eq=out.model.equality;
    Eigen::VectorXd eq_rhs=out.model.equality_rhs;
    if(in.enforce_hip_difference_acceleration) {
        if(!std::isfinite(in.desired_hip_difference_acceleration))return finish("invalid_hip_difference_task");
        const int row=eq.rows();eq.conservativeResize(row+1,19);eq.row(row).setZero();
        eq(row,3)=1.;eq(row,5)=-1.;eq_rhs.conservativeResize(row+1);
        eq_rhs[row]=in.desired_hip_difference_acceleration;
    }
    if(in.enforce_equal_normal_loads) {
        // The planar model has no lateral body rotation. Its free left/right
        // normal-load split must not be used to satisfy a mean sagittal task.
        const int row=eq.rows();eq.conservativeResize(row+1,19);eq.row(row).setZero();
        eq(row,16)=1.;eq(row,18)=-1.;eq_rhs.conservativeResize(row+1);eq_rhs[row]=0.;
    }

    ReducedSet reduced;
    if (!reduce(eq,eq_rhs,out.model,reduced)) return finish("inconsistent_equalities");
    for (int stage=0;stage<=task_count;++stage) {
        const int n=reduced.nullspace.cols();
        Eigen::VectorXd feasible;
        double slack=0.;
        if (!feasible_start(reduced,in.maximum_qp_iterations,feasible,slack)) {
            out.phase_one_slack=slack; return finish("no_feasible_start");
        }
        out.phase_one_slack=std::max(out.phase_one_slack,slack);
        if (n==0) out.decision=reduced.origin;
        else {
            Eigen::MatrixXd h;
            Eigen::VectorXd g;
            if (stage<task_count) {
                Eigen::RowVectorXd a=task[stage]*reduced.nullspace;
                double b=out.task_rhs[stage]-task[stage].dot(reduced.origin);
                // Normalize in free coordinates: a fixed absolute ridge can
                // dominate a small COM row after higher tasks are frozen.
                // Treat projected rows below 1e-8 as numerically dependent.
                // Nearly symmetric legs otherwise turn a constant task into
                // an enormous artificial gradient. The final physical-unit
                // frozen-task residual check still applies to these rows.
                // The ridge adds a small bias; independent LP replay bounds
                // that bias. Higher tasks are frozen before lower tasks.
                const double norm=a.norm();
                out.task_projection_norm[stage]=norm;
                if(norm>1e-8) { a/=norm; b/=norm; }
                else { a.setZero(); b=0.; }
                h=a.transpose()*a+1e-6*Eigen::MatrixXd::Identity(n,n);
                g=-b*a.transpose();
            } else {
                // Minimum scaled effort/acceleration/contact magnitude after
                // all achieved task values have been frozen.
                Eigen::VectorXd scale(19);
                scale.head(9).setConstant(100.);
                for (int j=0;j<6;++j) scale[9+j]=in.dynamics.actuator_limits[j];
                scale.tail(4).setConstant(200.);
                const Eigen::MatrixXd w=scale.cwiseInverse().asDiagonal();
                const Eigen::MatrixXd a=w*reduced.nullspace;
                h=a.transpose()*a+1e-8*Eigen::MatrixXd::Identity(n,n);
                g=a.transpose()*w*reduced.origin;
            }
            DenseActiveSetQp qp(n,reduced.inequality.rows());
            QpTolerances tolerance; tolerance.max_iterations=in.maximum_qp_iterations;
            tolerance.step_relative=1e-9; qp.set_tolerances(tolerance);
            const auto status=qp.solve(h,g,reduced.inequality,reduced.rhs,nullptr,&feasible);
            out.qp_diagnostics[stage]=qp.diagnostics();
            if (status!=QpStatus::kConverged) return finish(qp_status_name(status));
            out.decision=reduced.origin+reduced.nullspace*qp.solution();
        }
        if (stage<task_count) {
            out.achieved_task_lhs[stage]=task[stage].dot(out.decision);
            const int rows=eq.rows();
            eq.conservativeResize(rows+1,19); eq_rhs.conservativeResize(rows+1);
            eq.row(rows)=task[stage]; eq_rhs[rows]=out.achieved_task_lhs[stage];
            // Freeze the task in the current free coordinates, retaining the
            // achieved feasible point. Re-solving all augmented equalities
            // from scratch loses precision for almost symmetric native legs.
            const Eigen::RowVectorXd projected=task[stage]*reduced.nullspace;
            if(n>0 && projected.norm()>1e-8) {
                Eigen::MatrixXd normalized(1,n);normalized.row(0)=projected/projected.norm();
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized,Eigen::ComputeFullV);
                reduced.nullspace=(reduced.nullspace*svd.matrixV().rightCols(n-1)).eval();
            }
            reduced.origin=out.decision;
            update_inequalities(out.model,reduced);
        }
    }
    out.residuals=evaluate_thrust_support_solution(out.model,out.decision,in.tasks);
    out.maximum_equality_residual=(eq*out.decision-eq_rhs).cwiseAbs().maxCoeff();
    if (!out.residuals.valid || out.maximum_equality_residual>1e-6 ||
        out.residuals.max_inequality_violation>1e-6) return finish("final_residual_rejected");
    if(in.wheel_mode==SupportWheelMode::BoundedWheelServo) {
        for(int side=0;side<2;++side)
            out.allocated_wheel_rate_target[side]=in.dynamics.v[7+side]+out.decision[13+side]/in.wheel_servo_gain;
        out.allocated_linear_command=.035*(out.allocated_wheel_rate_target[0]+out.allocated_wheel_rate_target[1]);
        std::array<double,2> effort;
        if(std::abs(out.allocated_linear_command)>2.+1e-7 ||
            !support_wheel_servo_effort(out.allocated_linear_command,0.,
                {in.dynamics.v[7],in.dynamics.v[8]},in.wheel_servo_gain,true,true,false,effort) ||
            std::abs(effort[0]-out.decision[13])>1e-6 ||
            std::abs(effort[1]-out.decision[14])>1e-6) return finish("wheel_servo_mapping_rejected");
    }
    out.valid=true;
    return finish("hard_feasible");
}
} // namespace bbot_jump
