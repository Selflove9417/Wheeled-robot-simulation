// Same 41-input/35-output protocol as audit_native_command_response.cpp.
// This bridge is standalone and offline; it has no ROS/controller dependencies.
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include "bbot_balance_controller/ground_friction_response.hpp"

#ifndef BBOT_GROUND_RESPONSE_PROFILE
#define BBOT_GROUND_RESPONSE_PROFILE 0
#endif

int main()
{
    using namespace bbot_jump;
    GroundFrictionResponseOptions options;
    options.response_profile = static_cast<GroundUrdfResponseProfile>(BBOT_GROUND_RESPONSE_PROFILE);
    std::cout << std::setprecision(17);
    double time = 0.0;
    int bilateral = 0;
    std::size_t row = 0;
    while (std::cin >> time >> bilateral)
    {
        ++row;
        GroundFrictionResponseInput in;
        in.bilateral_contact = bilateral == 1;
        for (int i=0;i<9;++i) std::cin >> in.q[i];
        for (int i=0;i<9;++i) std::cin >> in.v[i];
        SupportQ9 observed;
        SupportActuatorVector command, net;
        for (int i=0;i<9;++i) std::cin >> observed[i];
        for (int i=0;i<6;++i) std::cin >> command[i];
        for (int i=0;i<6;++i) std::cin >> net[i];
        if (!std::cin)
        {
            std::cerr << "REJECT row=" << row << " reason=malformed_41_value_input\n";
            return 2;
        }
        in.command = command;
        if (!std::isfinite(time) || !observed.allFinite() || !net.allFinite())
        {
            std::cerr << "REJECT row=" << row << " reason=nonfinite_bridge_input\n";
            return 3;
        }

        const auto result = ground_friction_response(in, options);
        if (!result.valid)
        {
            std::cerr << "REJECT row=" << row << " time=" << time << " reason=" << result.reason << '\n';
            const double nan = std::numeric_limits<double>::quiet_NaN();
            std::cout << time << ' ' << bilateral << " -1";
            for (int i=0;i<9;++i) std::cout << ' ' << nan; // unavailable prediction
            for (int i=0;i<9;++i) std::cout << ' ' << observed[i];
            for (int i=0;i<8;++i) std::cout << ' ' << nan; // no force/residual witness
            for (int i=0;i<6;++i) std::cout << ' ' << nan; // aggregate friction diagnostic unavailable
            std::cout << '\n';
            continue; // preserve every frame; never substitute the legacy model.
        }

        ThrustSupportDynamicsInput dynamics_input;
        dynamics_input.q = in.q;
        dynamics_input.v = in.v;
        dynamics_input.prediction_dt = in.dt;
        dynamics_input.friction_coefficient = in.wheel_ground_friction;
        GroundUrdfResponseModel model;
        if (!build_ground_urdf_response_model(dynamics_input, options.response_profile, model))
        {
            std::cerr << "REJECT row=" << row << " time=" << time << " reason=fixed_cad_model_invalid_for_diagnostics\n";
            const double nan = std::numeric_limits<double>::quiet_NaN();
            std::cout << time << ' ' << bilateral << " -1";
            for (int i=0;i<9;++i) std::cout << ' ' << nan;
            for (int i=0;i<9;++i) std::cout << ' ' << observed[i];
            for (int i=0;i<14;++i) std::cout << ' ' << nan;
            std::cout << '\n';
            continue;
        }
        const auto contact_error = model.contactJacobian * observed + model.contactBias;
        const SupportActuatorVector aggregate = ground_friction_aggregate_difference(
            command, result.viscous_torque, result.leg_friction_torque, net);

        std::cout << time << ' ' << bilateral << ' ' << result.max_equation_residual;
        for (int i=0;i<9;++i) std::cout << ' ' << result.acceleration[i];
        for (int i=0;i<9;++i) std::cout << ' ' << observed[i];
        for (int i=0;i<4;++i) std::cout << ' ' << result.contact_force[i];
        for (int i=0;i<4;++i) std::cout << ' ' << contact_error[i];
        for (int i=0;i<6;++i) std::cout << ' ' << aggregate[i];
        std::cout << '\n';
    }
    return 0;
}
