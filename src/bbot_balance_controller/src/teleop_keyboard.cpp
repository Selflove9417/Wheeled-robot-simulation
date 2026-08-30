#include <iostream>
#include <string>
#include <chrono>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

class TeleopKeyboardNode : public rclcpp::Node
{
public:
    TeleopKeyboardNode()
        : Node("teleop_keyboard")
    {
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_height_ = this->create_publisher<std_msgs::msg::Float64>("/target_height", 10);
        pub_mode_ = this->create_publisher<std_msgs::msg::String>("/robot_mode", 10);

        speed_ = 0.35;
        turn_ = 0.60;
        height_ = 0.440;
        min_height_ = 0.30;
        max_height_ = 0.50;

        print_banner();
        setup_terminal();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TeleopKeyboardNode::spin_keyboard, this));
    }

    ~TeleopKeyboardNode()
    {
        restore_terminal();
    }

private:
    void print_banner()
    {
        std::cout << "\n"
                  << "╔══════════════════════════════════════════════════════════╗\n"
                  << "║             BBOT 键盘遥控终端 (C++ Native)                ║\n"
                  << "╠══════════════════════════════════════════════════════════╣\n"
                  << "║  [移动控制]                                              ║\n"
                  << "║      W / ↑   : 前进 (+0.35 m/s)                          ║\n"
                  << "║      S / ↓   : 后退 (-0.35 m/s)                          ║\n"
                  << "║      A / ←   : 左转 (+0.60 rad/s)                        ║\n"
                  << "║      D / →   : 右转 (-0.60 rad/s)                        ║\n"
                  << "║      Space   : 停止移动 (保持原地平衡)                     ║\n"
                  << "║                                                          ║\n"
                  << "║  [高度控制]                                              ║\n"
                  << "║      Q       : 升高机身 (+1cm, 范围 0.30 ~ 0.445m)       ║\n"
                  << "║      E       : 降低机身 (-1cm, 范围 0.30 ~ 0.445m)       ║\n"
                  << "║                                                          ║\n"
                  << "║  [状态模式]                                              ║\n"
                  << "║      R       : 触发倒地自恢复起立                         ║\n"
                  << "║      X       : 紧急停机                                  ║\n"
                  << "║      Ctrl+C  : 退出控制终端                              ║\n"
                  << "╚══════════════════════════════════════════════════════════╝\n"
                  << std::endl;
    }

    void setup_terminal()
    {
        if (tcgetattr(STDIN_FILENO, &orig_termios_) == 0)
        {
            struct termios raw = orig_termios_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            terminal_setup_ = true;
        }
    }

    void restore_terminal()
    {
        if (terminal_setup_)
        {
            publish_twist(0.0, 0.0);
            tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
            terminal_setup_ = false;
        }
    }

    void publish_twist(double linear_x, double angular_z)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_x;
        twist.angular.z = angular_z;
        pub_cmd_vel_->publish(twist);
    }

    void publish_height(double height)
    {
        std_msgs::msg::Float64 msg;
        msg.data = height;
        pub_height_->publish(msg);
    }

    void publish_mode(const std::string & mode)
    {
        std_msgs::msg::String msg;
        msg.data = mode;
        pub_mode_->publish(msg);
    }

    void spin_keyboard()
    {
        char c = 0;
        int n = read(STDIN_FILENO, &c, 1);
        if (n <= 0)
            return;

        if (c == 27) // ESC sequence
        {
            char seq[2] = {0, 0};
            if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0)
            {
                if (seq[0] == '[')
                {
                    switch (seq[1])
                    {
                    case 'A': // UP
                        c = 'w';
                        break;
                    case 'B': // DOWN
                        c = 's';
                        break;
                    case 'D': // LEFT
                        c = 'a';
                        break;
                    case 'C': // RIGHT
                        c = 'd';
                        break;
                    }
                }
            }
        }

        if (c == 'w' || c == 'W')
        {
            publish_twist(speed_, 0.0);
            printf("\r[指令] 前进 → 速度: +%.2f m/s, 转向: +0.00 rad/s                 \n", speed_);
            fflush(stdout);
        }
        else if (c == 's' || c == 'S')
        {
            publish_twist(-speed_, 0.0);
            printf("\r[指令] 后退 → 速度: -%.2f m/s, 转向: +0.00 rad/s                 \n", speed_);
            fflush(stdout);
        }
        else if (c == 'a' || c == 'A')
        {
            publish_twist(0.0, turn_);
            printf("\r[指令] 左转 → 速度: +0.00 m/s, 转向: +%.2f rad/s                 \n", turn_);
            fflush(stdout);
        }
        else if (c == 'd' || c == 'D')
        {
            publish_twist(0.0, -turn_);
            printf("\r[指令] 右转 → 速度: +0.00 m/s, 转向: -%.2f rad/s                 \n", turn_);
            fflush(stdout);
        }
        else if (c == ' ')
        {
            publish_twist(0.0, 0.0);
            printf("\r[指令] 停止移动 (保持原地自平衡)                                 \n");
            fflush(stdout);
        }
        else if (c == 'q' || c == 'Q')
        {
            height_ = std::min(max_height_, height_ + 0.01);
            publish_height(height_);
            printf("\r[指令] 升高机身 → 目标高度: %.3f m                               \n", height_);
            fflush(stdout);
        }
        else if (c == 'e' || c == 'E')
        {
            height_ = std::max(min_height_, height_ - 0.01);
            publish_height(height_);
            printf("\r[指令] 降低机身 → 目标高度: %.3f m                               \n", height_);
            fflush(stdout);
        }
        else if (c == 'r' || c == 'R')
        {
            publish_mode("standup");
            printf("\r[指令] 触发自恢复起立模式！                                      \n");
            fflush(stdout);
        }
        else if (c == 'x' || c == 'X')
        {
            publish_mode("emergency");
            printf("\r[指令] 紧急停机！                                                \n");
            fflush(stdout);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_height_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
    rclcpp::TimerBase::SharedPtr timer_;

    struct termios orig_termios_;
    bool terminal_setup_ = false;

    double speed_;
    double turn_;
    double height_;
    double min_height_;
    double max_height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
