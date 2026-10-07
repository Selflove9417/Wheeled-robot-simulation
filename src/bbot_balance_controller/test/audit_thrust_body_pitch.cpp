// Algebra replay of one bounded P wheel mode; no ROS command output.
#include <iostream>
#include <iomanip>
#include "bbot_balance_controller/thrust_support_allocator.hpp"
#include "bbot_balance_controller/landing_repair_momentum.hpp"
using namespace bbot_jump;
template<class D>void array(const char*n,const Eigen::MatrixBase<D>&v){std::cout<<",\""<<n<<"\":[";for(int r=0;r<v.rows();++r)for(int c=0;c<v.cols();++c){if(r||c)std::cout<<',';std::cout<<v(r,c);}std::cout<<']';}
int main(){double t,F,remaining;while(std::cin>>t){ThrustSupportAllocationInput in;auto &d=in.dynamics;for(int j=0;j<9;++j)std::cin>>d.q[j];for(int j=0;j<9;++j)std::cin>>d.v[j];std::cin>>F>>remaining;if(!std::cin)return 2;
 d.actuator_limits={150.,150.,150.,150.,10.,10.};d.actuator_speed_limits={11.,15.,11.,15.,30.,30.};d.joint_position_min={-1.52,-1.56,-1.52,-1.56};d.joint_position_max={1.52,1.56,1.52,1.56};d.prediction_dt=.010;in.wheel_mode=SupportWheelMode::BoundedWheelServo;
 ThrustSupportDynamicsModel m;if(!thrust_support_dynamics_model(d,m))return 3;const auto H=landing_momentum(d.q.segment<4>(3),d.v.segment<4>(3),{d.v[7],d.v[8]},-d.v[2],d.body_mass);
 if(!make_thrust_support_task_reference(F,17.5,H.total,-.08,remaining,m.com_axle_relative_forward,m.com_axle_relative_forward_jacobian.dot(d.v),in.tasks)||!make_thrust_body_pitch_reference(-d.q[2],-d.v[2],.088,.010,in.tasks))return 4;
 auto o=allocate_thrust_support(in);std::cout<<std::setprecision(17)<<"{\"time\":"<<t<<",\"valid\":"<<(o.valid?"true":"false")<<",\"reason\":\""<<o.reason<<"\",\"solve_us\":"<<o.solve_time_us;
 array("equality",o.model.equality);array("equality_rhs",o.model.equality_rhs);array("inequality",o.model.inequality);array("inequality_rhs",o.model.inequality_rhs);array("decision",o.decision);
 Eigen::Vector4d achieved,rhs,projection;for(int j=0;j<4;++j){achieved[j]=o.achieved_task_lhs[j];rhs[j]=o.task_rhs[j];projection[j]=o.task_projection_norm[j];}array("achieved_task_lhs",achieved);array("task_rhs",rhs);array("projection",projection);
 double b;array("vertical_row",support_vertical_acceleration_row(o.model,in.tasks.desired_com_vertical_acceleration,b));array("pitch_row",support_body_pitch_acceleration_row(o.model,in.tasks.desired_body_pitch_acceleration,b));array("hdot_row",support_centroidal_hdot_row(o.model,in.tasks.desired_centroidal_hdot,b));array("relative_row",support_com_axle_relative_forward_acceleration_row(o.model,in.tasks.desired_com_axle_relative_forward_acceleration,b));std::cout<<"}\n";
}}
