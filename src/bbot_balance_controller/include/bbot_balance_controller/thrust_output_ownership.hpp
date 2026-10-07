#pragma once

namespace bbot_jump {

inline bool ground_input_jump_allowed(bool experiment_enabled) {
    return !experiment_enabled;
}

inline bool ground_input_effort_hold_valid(bool effort_active, bool switch_pending,
                                           bool state_fresh, bool bilateral_contact) {
    return effort_active && !switch_pending && state_fresh && bilateral_contact;
}

inline bool controller_has_protected_output_owner(bool thrust_effort_owned,
                                                   bool ground_input_effort_hold) {
    return thrust_effort_owned || ground_input_effort_hold;
}

enum class ThrustOutputAction { AwaitInitialEffort, RunOwnedThrust, LockAfterHandoff };

// Once THRUST has owned the effort controller, loss of mode or an in-flight
// controller switch must never re-enter the legacy POSITION/wheel path.
inline ThrustOutputAction thrust_output_action(bool ever_owned,
                                               bool effort_active,
                                               bool switch_pending) {
    if (ever_owned && (!effort_active || switch_pending))
        return ThrustOutputAction::LockAfterHandoff;
    if (!effort_active)
        return ThrustOutputAction::AwaitInitialEffort;
    if (switch_pending)
        return ThrustOutputAction::LockAfterHandoff;
    return ThrustOutputAction::RunOwnedThrust;
}

}  // namespace bbot_jump
