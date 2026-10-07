// Offline actual simulation-input response audit. No controller or publishers.
#include <Eigen/QR>
#include <iomanip>
#include <iostream>
#include "bbot_balance_controller/thrust_support_dynamics.hpp"
int main() {
  using namespace bbot_jump;
  double t; int bilateral;
  while (std::cin >> t >> bilateral) {
    ThrustSupportDynamicsInput in;
    SupportActuatorVector cmd, net;
    SupportQ9 observed;
    for(int i=0;i<9;++i) std::cin >> in.q[i];
    for(int i=0;i<9;++i) std::cin >> in.v[i];
    for(int i=0;i<9;++i) std::cin >> observed[i];
    for(int i=0;i<6;++i) std::cin >> cmd[i];
    for(int i=0;i<6;++i) std::cin >> net[i];
    if(!std::cin) return 2;
    ThrustSupportDynamicsModel m;
    if(!thrust_support_dynamics_model(in,m)) return 3;
    SupportQ9 rhs=-(m.velocity_bias+m.gravity+m.joint_damping);
    rhs.tail<6>()+=cmd;
    SupportQ9 prediction; SupportContactVector force=SupportContactVector::Zero();
    double residual;
    if(bilateral) {
      Eigen::Matrix<double,13,13> a=Eigen::Matrix<double,13,13>::Zero();
      a.topLeftCorner<9,9>()=m.mass;
      a.topRightCorner<9,4>()=-m.contact_jacobian.transpose();
      a.bottomLeftCorner<4,9>()=m.contact_jacobian;
      Eigen::Matrix<double,13,1> b; b.head<9>()=rhs;b.tail<4>()=-m.contact_bias;
      const Eigen::Matrix<double,13,1> x=a.completeOrthogonalDecomposition().solve(b);
      prediction=x.head<9>();force=x.tail<4>();residual=(a*x-b).cwiseAbs().maxCoeff();
    } else {
      prediction=m.mass.ldlt().solve(rhs);residual=(m.mass*prediction-rhs).cwiseAbs().maxCoeff();
    }
    std::cout << std::setprecision(17) << t << ' ' << bilateral << ' ' << residual;
    for(int i=0;i<9;++i) std::cout << ' ' << prediction[i];
    for(int i=0;i<9;++i) std::cout << ' ' << observed[i];
    for(int i=0;i<4;++i) std::cout << ' ' << force[i];
    const auto cr=m.contact_jacobian*observed+m.contact_bias;
    for(int i=0;i<4;++i) std::cout << ' ' << cr[i];
    // Aggregate joint load compared separately, never substituted for input.
    for(int i=0;i<6;++i) std::cout << ' ' << cmd[i]-m.joint_damping[3+i]-net[i];
    std::cout << '\n';
  }
}
