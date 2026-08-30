#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include "bbot_motor_driver/can_interface.hpp"

class MotorDriverNode : public rclcpp::Node
{
public:

    MotorDriverNode()
    : Node("motor_driver")
    {
        can_.open("can0");

        cmd_sub_ =
            create_subscription<
            std_msgs::msg::Float32MultiArray>(
                "/motor_cmd",
                10,
                std::bind(
                    &MotorDriverNode::cmdCallback,
                    this,
                    std::placeholders::_1));

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(5),
                std::bind(
                    &MotorDriverNode::readFeedback,
                    this));
    }

private:

    static uint32_t float_to_uint(
        float x,
        float xmin,
        float xmax,
        int bits)
    {
        float span = xmax - xmin;

        float offset = x - xmin;

        return (uint32_t)
            ((offset * ((1<<bits)-1))
            / span);
    }

    void sendMitCommand(
        uint16_t motor_id,
        float kp,
        float kd,
        float pos,
        float spd,
        float tor)
    {
        int kp_int =
            float_to_uint(
                kp,
                0.0f,
                500.0f,
                12);

        int kd_int =
            float_to_uint(
                kd,
                0.0f,
                5.0f,
                9);

        int pos_int =
            float_to_uint(
                pos,
                -12.5f,
                12.5f,
                16);

        int spd_int =
            float_to_uint(
                spd,
                -18.0f,
                18.0f,
                12);

        int tor_int =
            float_to_uint(
                tor,
                -30.0f,
                30.0f,
                12);

        uint8_t data[8];

        data[0] =
            kp_int >> 7;

        data[1] =
            ((kp_int & 0x7F) << 1)
            |
            ((kd_int & 0x100)>>8);

        data[2] =
            kd_int & 0xFF;

        data[3] =
            pos_int >> 8;

        data[4] =
            pos_int & 0xFF;

        data[5] =
            spd_int >> 4;

        data[6] =
            ((spd_int & 0x0F)<<4)
            |
            (tor_int>>8);

        data[7] =
            tor_int & 0xFF;

        can_.sendFrame(
            motor_id,
            data,
            8);
    }

    void cmdCallback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() < 2)
            return;

        float left_torque =
            msg->data[0];

        float right_torque =
            msg->data[1];

        sendMitCommand(
            1,
            0.0,
            0.0,
            0.0,
            0.0,
            left_torque);

        sendMitCommand(
            2,
            0.0,
            0.0,
            0.0,
            0.0,
            right_torque);
    }

    void readFeedback()
    {
        struct can_frame frame;

        if(can_.receiveFrame(frame))
        {
            RCLCPP_INFO(
                get_logger(),
                "ID=0x%03X",
                frame.can_id);
        }
    }

    CanInterface can_;

    rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MotorDriverNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}