#include "bbot_torque_control/torque_monitor.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace bbot_torque_control
{

TorqueMonitor::TorqueMonitor(const rclcpp::NodeOptions & options)
: Node("torque_monitor", options)
{
  // 声明参数
  this->declare_parameter("pub_rate_hz", 100);
  this->declare_parameter("imu_topic", "/imu");
  this->declare_parameter("joint_state_topic", "/joint_states");
  this->declare_parameter("log_enabled", false);  // 默认关，键盘控制器统一记录到 data_logs
  this->declare_parameter("log_interval", 0.05);

  pub_rate_hz_ = this->get_parameter("pub_rate_hz").as_int();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  log_enabled_ = this->get_parameter("log_enabled").as_bool();
  log_interval_ = this->get_parameter("log_interval").as_double();

  // 创建订阅者
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10,
    std::bind(&TorqueMonitor::imu_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 10,
    std::bind(&TorqueMonitor::joint_state_callback, this, std::placeholders::_1));

  // 创建发布者
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/bbot/torque_monitor/gravity_torques", 10);

  com_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/bbot/torque_monitor/com_state", 10);

  // 创建定时器
  auto period = std::chrono::milliseconds(1000 / pub_rate_hz_);
  timer_ = this->create_wall_timer(period, std::bind(&TorqueMonitor::control_loop, this));

  // 打开日志文件
  if (log_enabled_) {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "torque_log_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") << ".csv";
    log_path_ = ss.str();
    log_file_.open(log_path_);
    if (log_file_.is_open()) {
      write_log_header();
      RCLCPP_INFO(this->get_logger(), "日志文件已创建: %s", log_path_.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "力矩监控节点已启动，发布频率: %d Hz", pub_rate_hz_);
}

TorqueMonitor::~TorqueMonitor()
{
  if (log_file_.is_open()) {
    log_file_.close();
    RCLCPP_INFO(this->get_logger(), "日志已保存到: %s", log_path_.c_str());
  }
}

void TorqueMonitor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // 从四元数提取俯仰角 (绕 y 轴旋转)
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;

  // pitch = asin(2*(qw*qy - qx*qz))
  double sin_pitch = 2.0 * (qw * qy - qx * qz);
  sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
  body_pitch_ = std::asin(sin_pitch);

  // 俯仰角速度 (陀螺仪 y 轴)
  body_pitch_rate_ = msg->angular_velocity.y;

  imu_received_ = true;
}

void TorqueMonitor::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 提取左右腿髋和膝关节角度
  hip_angle_ = get_joint_position(*msg, "link_002_joint", 0.0);
  knee_angle_ = get_joint_position(*msg, "link_003_joint", 0.0);

  joint_state_received_ = true;
}

double TorqueMonitor::get_joint_position(
  const sensor_msgs::msg::JointState & js,
  const std::string & name, double default_val) const
{
  for (size_t i = 0; i < js.name.size(); i++) {
    if (js.name[i] == name) {
      return js.position[i];
    }
  }
  return default_val;
}

void TorqueMonitor::control_loop()
{
  if (!imu_received_ || !joint_state_received_) {
    return;
  }

  double now = this->now().seconds();

  // 1. 正运动学: 计算当前质心高度
  double com_height = kinematics_.calculate_com_height(
    body_pitch_, hip_angle_, knee_angle_);

  // 2. 计算重力补偿力矩
  bbot_kinematics::JointTorques torques = kinematics_.compute_gravity_torques(
    body_pitch_, hip_angle_, knee_angle_);

  // 3. 估算轮毂平衡力矩
  double wheel_balance = kinematics_.estimate_balance_torque(com_height, body_pitch_);

  // 4. 发布重力矩
  auto torque_msg = std_msgs::msg::Float64MultiArray();
  torque_msg.data.resize(3);
  torque_msg.data[0] = torques.hip_torque;     // 髋关节重力矩
  torque_msg.data[1] = torques.knee_torque;    // 膝关节重力矩
  torque_msg.data[2] = wheel_balance;           // 轮毂平衡力矩
  gravity_torque_pub_->publish(torque_msg);

  // 5. 发布质心状态
  auto com_msg = std_msgs::msg::Float64MultiArray();
  com_msg.data.resize(4);
  com_msg.data[0] = com_height;                // 质心高度
  com_msg.data[1] = body_pitch_;               // 机身俯仰角
  com_msg.data[2] = hip_angle_;                // 髋关节角
  com_msg.data[3] = knee_angle_;               // 膝关节角
  com_state_pub_->publish(com_msg);

  // 6. 数据记录
  if (log_file_.is_open() && (last_log_time_ < 0 || (now - last_log_time_) >= log_interval_)) {
    write_log_row(now, com_height,
      torques.hip_torque, torques.knee_torque, wheel_balance,
      hip_angle_, knee_angle_, body_pitch_);
    last_log_time_ = now;
  }
}

void TorqueMonitor::write_log_header()
{
  log_file_ << "timestamp,com_height,hip_torque,knee_torque,wheel_torque,"
            << "hip_angle,knee_angle,body_pitch\n";
}

void TorqueMonitor::write_log_row(double t, double com_h,
  double hip_t, double knee_t, double wheel_t,
  double hip_angle, double knee_angle, double pitch)
{
  log_file_ << t << "," << com_h << ","
            << hip_t << "," << knee_t << "," << wheel_t << ","
            << hip_angle << "," << knee_angle << "," << pitch << "\n";
}

}  // namespace bbot_torque_control

// 独立可执行文件入口
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bbot_torque_control::TorqueMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
