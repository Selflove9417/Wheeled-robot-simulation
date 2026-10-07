#include <cstdlib>
#include <iostream>

#include "bbot_balance_controller/thrust_output_ownership.hpp"

int main() {
    using bbot_jump::ThrustOutputAction;
    using bbot_jump::ground_input_effort_hold_valid;
    using bbot_jump::ground_input_jump_allowed;
    using bbot_jump::thrust_output_action;
    auto require = [](bool ok, const char *message) {
        if (!ok) { std::cerr << message << '\n'; std::exit(1); }
    };
    require(thrust_output_action(false, false, false) == ThrustOutputAction::AwaitInitialEffort,
            "initial switch must retain the pre-handoff path");
    require(thrust_output_action(false, false, true) == ThrustOutputAction::AwaitInitialEffort,
            "initial switch pending must not count as a lost handoff");
    require(thrust_output_action(false, true, false) == ThrustOutputAction::RunOwnedThrust,
            "confirmed effort mode must own THRUST");
    require(thrust_output_action(true, false, false) == ThrustOutputAction::LockAfterHandoff,
            "effort loss after acquisition must lock outputs");
    require(thrust_output_action(true, true, true) == ThrustOutputAction::LockAfterHandoff,
            "switch after acquisition must lock outputs");
    require(thrust_output_action(true, true, false) == ThrustOutputAction::RunOwnedThrust,
            "valid owned mode must continue THRUST");
    require(!ground_input_jump_allowed(true) && ground_input_jump_allowed(false),
            "ground-input mode must reject jump commands");
    require(ground_input_effort_hold_valid(true, false, true, true),
            "fresh bilateral effort hold should remain enabled");
    require(!ground_input_effort_hold_valid(false, false, true, true) &&
            !ground_input_effort_hold_valid(true, true, true, true) &&
            !ground_input_effort_hold_valid(true, false, false, true) &&
            !ground_input_effort_hold_valid(true, false, true, false),
            "lost mode, pending switch, stale data, or lost contact must fail closed");
    return 0;
}
