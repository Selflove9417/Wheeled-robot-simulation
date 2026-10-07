#pragma once

#include <rclcpp/rclcpp.hpp>

namespace bbot_jump {

// Control uses the newest available sensor sample, never a backlog of old
// IMU/odometry/joint messages. Raw recorders keep their own loss-detecting QoS.
inline rclcpp::QoS latest_sensor_qos() {
    return rclcpp::SensorDataQoS().keep_last(1);
}

}  // namespace bbot_jump
