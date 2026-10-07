#include <cstdlib>
#include <iostream>

#include "bbot_balance_controller/latest_sensor_qos.hpp"

int main() {
    const auto qos = bbot_jump::latest_sensor_qos().get_rmw_qos_profile();
    if (qos.history != RMW_QOS_POLICY_HISTORY_KEEP_LAST || qos.depth != 1 ||
        qos.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ||
        qos.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE) {
        std::cerr << "latest sensor QoS must be best-effort, volatile, keep-last 1\n";
        return EXIT_FAILURE;
    }
    std::cout << "latest sensor QoS tests passed\n";
    return EXIT_SUCCESS;
}
