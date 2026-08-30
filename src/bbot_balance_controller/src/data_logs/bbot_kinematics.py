#!/usr/bin/env python3
"""
bbot_kinematics.py
==================
独立的运动学和力矩计算模块 (纯 Python，不依赖 ROS)。

提供与 C++ bbot_kinematics 库完全一致的计算功能：
  - 正运动学 (FK): 关节角 -> 质心高度
  - 逆运动学 (IK): 目标高度 -> 关节角
  - 雅可比矩阵: 关节速度 -> 质心速度
  - 重力补偿力矩: 各高度下所需的关节力矩

用法:
  from bbot_kinematics import Kinematics, RobotParams

  params = RobotParams()
  kin = Kinematics(params)

  # 正运动学
  com_h = kin.calculate_com_height(body_pitch=0.0, hip_angle=0.4, knee_angle=-0.8)

  # 逆运动学
  hip, knee, ankle = kin.inverse_kinematics(target_z=0.55, body_pitch=0.0)

  # 重力矩
  hip_t, knee_t, wheel_t = kin.compute_gravity_torques(0.0, hip, knee)

  # 遍历整个高度范围
  results = kin.compute_torque_vs_height(body_pitch=0.0, num_points=100)
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Dict, Optional


@dataclass
class RobotParams:
    """机器人物理参数"""
    # 几何
    l1: float = 0.30       # 小腿长度 [m]
    l2: float = 0.30       # 大腿长度 [m]
    l3: float = 0.10       # 髋到机身质心 [m]
    x1c: float = 0.15      # 小腿质心 (半长)
    x2c: float = 0.15      # 大腿质心 (半长)
    wheel_radius: float = 0.075
    wheel_separation: float = 0.35

    # 质量 (双腿总质量，平面模型等效，与 URDF 一致)
    m0: float = 2.13       # 双轮 [kg] (2 × 1.065)
    m1: float = 0.70       # 双腿小腿 [kg] (2 × 0.35)
    m2: float = 1.20       # 双腿大腿 [kg] (2 × 0.60)
    m3: float = 15.6       # 机身 [kg]

    # 范围 (与 C++ 控制器一致)
    L_MIN: float = 0.4419  # 蹲下质心高度 [m]
    L_MAX: float = 0.5949  # 站立质心高度 [m]

    # 力矩限制
    tau_max_hip: float = 120.0
    tau_max_knee: float = 120.0
    tau_max_wheel: float = 5.0

    # 重力
    g: float = 9.81

    @property
    def M_total(self) -> float:
        return self.m0 + self.m1 + self.m2 + self.m3

    @property
    def a1(self) -> float:
        """小腿等效质量-长度"""
        return self.m1 * self.x1c + (self.m2 + self.m3) * self.l1

    @property
    def b1(self) -> float:
        """大腿等效质量-长度"""
        return self.m2 * self.x2c + self.m3 * self.l2

    @property
    def c1_m(self) -> float:
        """机身等效质量-长度"""
        return self.m3 * self.l3


@dataclass
class IKSolution:
    """逆运动学求解结果

    运动学链: 轮轴 ─[l1:小腿]─→ 膝 ─[l2:大腿]─→ 髋 ─[l3]─→ 机身质心
    绝对角度 (从竖直向上=0):
      theta_shank = body_pitch - theta_hip - theta_knee  (小腿绝对角)
    闭合约束: body_pitch = theta_shank + theta_knee + theta_hip

    URDF 约定:
      hip_urdf  = -theta_hip   (大腿相对于机身)
      knee_urdf = -theta_knee  (小腿相对于大腿)
    """
    theta_shank: float  # θ₁: 小腿绝对角
    theta_knee: float   # θ₂: 膝关节角
    theta_hip: float    # θ₃: 髋关节角 (由闭合约束反推)


@dataclass
class JointTorques:
    """关节力矩"""
    hip_torque: float
    knee_torque: float
    wheel_torque: float


class Kinematics:
    """BBot 运动学求解器"""

    def __init__(self, params: Optional[RobotParams] = None):
        self.params = params if params is not None else RobotParams()

    # ======================== 正运动学 ========================

    def calculate_com_height(
        self, body_pitch: float, hip_angle: float, knee_angle: float
    ) -> float:
        """
        计算质心高度
        Args:
            body_pitch: 机身俯仰角 [rad]
            hip_angle: 髋关节角 [rad]
            knee_angle: 膝关节角 [rad]
        Returns:
            质心高度 [m] (相对于轮轴)
        """
        p = self.params
        theta_body = body_pitch
        theta_thigh = theta_body - hip_angle
        theta_shank = theta_thigh - knee_angle

        z_knee = p.l1 * np.cos(theta_shank)
        z_hip = z_knee + p.l2 * np.cos(theta_thigh)
        z_body = z_hip + p.l3 * np.cos(theta_body)

        z_c1 = p.x1c * np.cos(theta_shank)
        z_c2 = z_knee + p.x2c * np.cos(theta_thigh)

        return (p.m0 * 0 + p.m1 * z_c1 + p.m2 * z_c2 + p.m3 * z_body) / p.M_total

    def forward_kinematics(
        self, body_pitch: float, hip_angle: float, knee_angle: float
    ) -> Dict[str, float]:
        """
        完整正运动学
        Returns:
            dict with keys: z_knee, z_hip, z_body, z_c1, z_c2, x_com, z_com,
                            theta_thigh, theta_shank
        """
        p = self.params
        theta_body = body_pitch
        theta_thigh = theta_body - hip_angle
        theta_shank = theta_thigh - knee_angle

        x_knee = p.l1 * np.sin(theta_shank)
        x_hip = x_knee + p.l2 * np.sin(theta_thigh)
        x_body = x_hip + p.l3 * np.sin(theta_body)

        z_knee = p.l1 * np.cos(theta_shank)
        z_hip = z_knee + p.l2 * np.cos(theta_thigh)
        z_body = z_hip + p.l3 * np.cos(theta_body)

        x_c1 = p.x1c * np.sin(theta_shank)
        x_c2 = x_knee + p.x2c * np.sin(theta_thigh)
        z_c1 = p.x1c * np.cos(theta_shank)
        z_c2 = z_knee + p.x2c * np.cos(theta_thigh)

        x_com = (p.m0 * 0 + p.m1 * x_c1 + p.m2 * x_c2 + p.m3 * x_body) / p.M_total
        z_com = (p.m0 * 0 + p.m1 * z_c1 + p.m2 * z_c2 + p.m3 * z_body) / p.M_total

        return {
            'z_knee': z_knee, 'z_hip': z_hip, 'z_body': z_body,
            'z_c1': z_c1, 'z_c2': z_c2,
            'x_com': x_com, 'z_com': z_com,
            'theta_thigh': theta_thigh, 'theta_shank': theta_shank,
        }

    # ======================== 逆运动学 ========================

    def inverse_kinematics(
        self, target_z: float, body_pitch: float = 0.0
    ) -> IKSolution:
        """
        逆运动学求解
        Args:
            target_z: 目标质心高度 [m]
            body_pitch: 机身俯仰角 [rad]
        Returns:
            IKSolution (theta_hip, theta_knee, theta_ankle)
        """
        p = self.params
        xc, zc = 0.0, target_z
        xw, zw = 0.0, 0.0
        theta0 = np.pi / 2.0

        c0123 = np.cos(body_pitch)
        s0123 = np.sin(body_pitch)

        c1 = p.M_total * (xc - xw) - p.m3 * p.l3 * s0123
        d1 = p.M_total * (zc - zw) - p.m3 * p.l3 * c0123

        dist_sq = c1**2 + d1**2
        dist = np.sqrt(dist_sq)

        # 膝关节角 (余弦定律)
        cos_theta2 = np.clip(
            (dist_sq - p.a1**2 - p.b1**2) / (2.0 * p.a1 * p.b1), -1, 1)
        theta2 = np.arccos(cos_theta2)

        # 髋关节角
        alpha = np.arctan2(d1, c1)
        cos_beta = np.clip(
            (p.a1**2 + dist_sq - p.b1**2) / (2.0 * p.a1 * dist), -1, 1)
        beta = np.arccos(cos_beta)

        theta01 = alpha - beta
        theta1 = theta01 - theta0               # θ₁: 小腿绝对角
        theta3 = body_pitch - (theta1 + theta2)  # θ₃: 髋关节角 (闭合约束)

        return IKSolution(theta_shank=theta1, theta_knee=theta2, theta_hip=theta3)

    # ======================== 雅可比矩阵 ========================

    def compute_jacobian(
        self, body_pitch: float, hip_angle: float, knee_angle: float
    ) -> Dict[str, float]:
        """
        计算质心雅可比矩阵
        J = [∂x/∂θ_hip, ∂x/∂θ_knee, ∂x/∂θ_body]
            [∂z/∂θ_hip, ∂z/∂θ_knee, ∂z/∂θ_body]
        """
        p = self.params
        theta_body = body_pitch
        theta_thigh = theta_body - hip_angle
        theta_shank = theta_thigh - knee_angle

        sin_s, cos_s = np.sin(theta_shank), np.cos(theta_shank)
        sin_t, cos_t = np.sin(theta_thigh), np.cos(theta_thigh)
        sin_b, cos_b = np.sin(theta_body), np.cos(theta_body)

        inv_M = 1.0 / p.M_total

        return {
            'Jx_hip': (-p.a1 * cos_s - p.b1 * cos_t) * inv_M,
            'Jx_knee': (-p.a1 * cos_s) * inv_M,
            'Jx_body': (p.a1 * cos_s + p.b1 * cos_t + p.c1_m * cos_b) * inv_M,
            'Jz_hip': (p.a1 * sin_s + p.b1 * sin_t) * inv_M,
            'Jz_knee': (p.a1 * sin_s) * inv_M,
            'Jz_body': (-p.a1 * sin_s - p.b1 * sin_t - p.c1_m * sin_b) * inv_M,
        }

    # ======================== 重力矩 ========================

    def compute_gravity_torques(
        self, body_pitch: float, hip_angle: float, knee_angle: float
    ) -> JointTorques:
        """
        计算重力补偿力矩
        τ = J^T * [0, -M*g]  (虚功原理)
        """
        J = self.compute_jacobian(body_pitch, hip_angle, knee_angle)
        Mg = self.params.M_total * self.params.g

        return JointTorques(
            hip_torque=-Mg * J['Jz_hip'],
            knee_torque=-Mg * J['Jz_knee'],
            wheel_torque=-Mg * J['Jz_body'],
        )

    def compute_gravity_torques_at_height(
        self, target_height: float, body_pitch: float = 0.0
    ) -> JointTorques:
        """计算指定高度下的重力补偿力矩"""
        ik = self.inverse_kinematics(target_height, body_pitch)
        return self.compute_gravity_torques(body_pitch, ik.theta_hip, ik.theta_knee)

    def compute_torque_vs_height(
        self, body_pitch: float = 0.0, num_points: int = 200
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        遍历高度范围计算力矩
        Returns:
            heights, hip_angles, knee_angles, hip_torques, knee_torques
        """
        heights = np.linspace(self.params.L_MIN, self.params.L_MAX, num_points)
        hip_a = np.zeros(num_points)
        knee_a = np.zeros(num_points)
        hip_t = np.zeros(num_points)
        knee_t = np.zeros(num_points)
        wheel_t = np.zeros(num_points)

        for i, h in enumerate(heights):
            ik = self.inverse_kinematics(h, body_pitch)
            hip_a[i] = ik.theta_hip
            knee_a[i] = ik.theta_knee
            torques = self.compute_gravity_torques(body_pitch, ik.theta_hip, ik.theta_knee)
            hip_t[i] = torques.hip_torque
            knee_t[i] = torques.knee_torque
            wheel_t[i] = torques.wheel_torque

        return heights, hip_a, knee_a, hip_t, knee_t, wheel_t

    def estimate_balance_torque(
        self, com_height: float, body_pitch: float
    ) -> float:
        """倒立摆模型估算轮毂平衡力矩"""
        return self.params.M_total * self.params.g * com_height * np.sin(body_pitch)


# ======================== 便捷函数 ========================

def print_torque_table():
    """打印关键高度的力矩表"""
    kin = Kinematics()
    print("\n" + "=" * 70)
    print("  BBot 重力补偿力矩表 (body_pitch=0)")
    print("=" * 70)
    print(f"  {'高度 [m]':>10}  {'髋角 [°]':>10}  {'膝角 [°]':>10}  "
          f"{'髋力矩 [Nm]':>12}  {'膝力矩 [Nm]':>12}  {'轮力矩 [Nm]':>12}")
    print("-" * 70)

    for h in np.arange(0.30, 0.62, 0.05):
        try:
            ik = kin.inverse_kinematics(h, 0.0)
            t = kin.compute_gravity_torques(0.0, ik.theta_hip, ik.theta_knee)
            print(f"  {h:10.2f}  {np.degrees(ik.theta_hip):10.1f}  "
                  f"{np.degrees(ik.theta_knee):10.1f}  "
                  f"{t.hip_torque:12.2f}  {t.knee_torque:12.2f}  {t.wheel_torque:12.2f}")
        except (ValueError, RuntimeWarning):
            print(f"  {h:10.2f}  {'N/A':>10}  {'N/A':>10}  {'N/A':>12}  {'N/A':>12}")

    print("=" * 70)
    print()


if __name__ == '__main__':
    print_torque_table()
