#!/usr/bin/env python3
"""
plot_jump_log.py
================
读取仿真生成的 jump_control_log.csv，绘制跳跃全流程动力学与状态响应曲线。
包含：
  1. 质心高度与轮子离地高度
  2. 垂直运动速度与离地目标速度
  3. 关节电机实际力矩与指令力矩响应
  4. 机身俯仰角 (Pitch) 动态平衡响应
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 配置中文字体与负号显示
plt.rcParams['font.sans-serif'] = [
    'Noto Sans CJK JP', 'Noto Sans CJK SC', 'Droid Sans Fallback',
    'WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans'
]
plt.rcParams['axes.unicode_minus'] = False


def load_csv(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        sys.exit(1)
    data = np.genfromtxt(filepath, delimiter=',', names=True, skip_header=0)
    print(f"[INFO] 已加载 {len(data)} 行数据, 时长 {data['timestamp'][-1] - data['timestamp'][0]:.2f}s")
    return data


def field_or_default(data, field, default):
    """Read an optional telemetry field without breaking legacy logs."""
    names = data.dtype.names or ()
    return data[field] if field in names else default


def compute_jump_metrics(data):
    t = data['timestamp'] - data['timestamp'][0]
    states = data['state'].astype(int)
    z_leg = data['z']
    z_dot = data['z_dot']

    # 新日志直接使用 Gazebo 世界坐标。旧日志才回退到腿长弹道估计；
    # 腿长变化不能当作质心高度变化，否则原地伸腿也会被画成“跳起”。
    if 'gazebo_world_z' in (data.dtype.names or ()):
        z_com = data['gazebo_world_z']
        wheel_radius = 0.07
        wheel_clearance = np.maximum(0.0, z_com - z_leg - wheel_radius)
        return t, z_com, wheel_clearance

    # 计算质心高度与轮子离地高度
    z_com = np.copy(z_leg)
    wheel_clearance = np.zeros_like(z_leg)

    flight_indices = np.where(states == 3)[0]  # 3: FLIGHT 腾空相
    if len(flight_indices) > 0:
        takeoff_idx = flight_indices[0]
        t_takeoff = t[takeoff_idx]
        z0 = z_leg[takeoff_idx]
        v0 = z_dot[takeoff_idx]

        for idx in flight_indices:
            t_air = t[idx] - t_takeoff
            # 腾空弹道自由落体方程: z(t) = z0 + v0*t - 0.5*g*t^2
            z_ballistic = z0 + v0 * t_air - 0.5 * 9.81 * (t_air ** 2)
            z_com[idx] = max(z_ballistic, z_leg[idx])
            wheel_clearance[idx] = max(0.0, z_com[idx] - z_leg[idx])

    return t, z_com, wheel_clearance


def add_phase_spans(axes, t, states, flight_subphase, thrust_blocked):
    """Add background spans across all axes for jump phases and subphases."""
    if len(t) < 2:
        return

    added_labels = set()

    def get_phase_info(state, subphase, blocked):
        if state == 1:
            return ('#FFF3E0', 0.45, '下蹲蓄力 (SQUAT)')
        elif state == 2:
            if blocked > 0.5:
                return ('#FFCDD2', 0.55, '推地姿态阻塞 (THRUST BLOCKED)')
            else:
                return ('#FFF9C4', 0.50, '爆发推地 (THRUST)')
        elif state == 3:
            if subphase == 0:
                return ('#E1BEE7', 0.55, '腾空-姿态刹车 (ARREST)')
            elif subphase == 1:
                return ('#BBDEFB', 0.55, '腾空-平滑收腿 (TUCK)')
            elif subphase == 2:
                return ('#B2EBF2', 0.55, '腾空-顶点展腿 (EXTEND)')
            elif subphase == 3:
                return ('#FFE0B2', 0.65, '腾空-保护展腿 (PROTECTIVE)')
            else:
                return ('#E1BEE7', 0.40, '腾空相 (FLIGHT)')
        elif state == 4:
            return ('#ECEFF1', 0.55, '触地缓冲 (BUFFER)')
        elif state == 5:
            return ('#E8F5E9', 0.55, '自平衡恢复 (RECOVERY)')
        return (None, 0.0, None)

    n = len(t)
    i = 0
    while i < n:
        color, alpha, label = get_phase_info(states[i], flight_subphase[i], thrust_blocked[i])
        if color is None:
            i += 1
            continue
        start_idx = i
        while (i + 1 < n and
               get_phase_info(states[i+1], flight_subphase[i+1], thrust_blocked[i+1]) == (color, alpha, label)):
            i += 1
        t_start = t[start_idx]
        t_end = t[i]
        lbl = label if label not in added_labels else None
        if lbl:
            added_labels.add(lbl)
        for ax in axes:
            ax.axvspan(t_start, t_end, color=color, alpha=alpha, label=lbl, zorder=0)
        i += 1


def plot_jump_performance(data, output_path):
    t, z_com, wheel_clearance = compute_jump_metrics(data)
    states = data['state'].astype(int)
    pitch = data['pitch']
    pitch_rate = field_or_default(data, 'pitch_rate', np.zeros_like(t))
    flight_subphase = field_or_default(data, 'flight_subphase', np.full_like(t, -1))
    thrust_blocked = field_or_default(data, 'thrust_attitude_blocked', np.zeros_like(t))
    left_wvel = field_or_default(data, 'left_wheel_vel', np.zeros_like(t))
    right_wvel = field_or_default(data, 'right_wheel_vel', np.zeros_like(t))
    cmd_x = field_or_default(data, 'cmd_x', np.zeros_like(t))

    fig, axes = plt.subplots(5, 1, figsize=(14, 15.0), sharex=True)

    # ── 1. 质心高度与轮子离地高度 ──
    ax1 = axes[0]
    ax1.plot(t, z_com, color='#1565C0', linewidth=2.0, label='质心高度 [m]')
    ax1.plot(t, wheel_clearance, color='#FF6F00', linewidth=2.2, label='轮子离地高度 [m]')
    ax1.set_ylabel('高度 [m]', fontsize=11, fontweight='bold')
    ax1.set_title('BBot 双轮腿机器人跳跃控制动力学响应', fontsize=14, fontweight='bold', pad=12)
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.legend(loc='upper right', fontsize=9.0, framealpha=0.92)

    # ── 2. 垂直运动速度 ──
    ax2 = axes[1]
    world_z_dot = field_or_default(data, 'gazebo_world_z_dot', data['z_dot'])
    velocity_label = ('世界竖直速度 [m/s]' if 'gazebo_world_z_dot' in (data.dtype.names or ())
                      else '腿长变化率 z_dot [m/s]')
    ax2.plot(t, world_z_dot, color='#8E24AA', linewidth=1.6, label=velocity_label)
    target_velocity = float(np.nanmax(field_or_default(data, 'target_takeoff_velocity', 1.98091)))
    ax2.axhline(y=target_velocity, color='#2E7D32', linestyle=':', linewidth=1.6,
                alpha=0.85, label='目标起跳离地速度 (%.2f m/s)' % target_velocity)
    ax2.axhline(y=0.0, color='gray', linestyle='--', alpha=0.4)
    airborne_confidence = field_or_default(data, 'airborne_confidence', np.zeros_like(t))
    airborne_flag = field_or_default(data, 'wheels_airborne', np.zeros_like(t))
    ax2b = ax2.twinx()
    ax2b.step(t, airborne_confidence, where='post', color='#00897B', alpha=0.55,
              linewidth=1.2, label='离地连续确认计数')
    ax2b.step(t, airborne_flag * 3.0, where='post', color='#F57C00', alpha=0.75,
              linewidth=1.0, linestyle='--', label='离地标志')
    ax2b.set_ylim(-0.2, 20.5)
    ax2b.set_ylabel('离地确认计数', fontsize=9, color='#00897B')
    ax2b.tick_params(axis='y', labelcolor='#00897B')
    ax2.set_ylabel('垂直速度 [m/s]', fontsize=11, fontweight='bold')
    ax2.grid(True, alpha=0.3, linestyle='--')
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2,
               loc='upper right', fontsize=8.5, framealpha=0.92)

    # ── 3. 关节电机力矩 ──
    ax3 = axes[2]
    ax3.plot(t, data['knee_cmd_left'], color='#D32F2F', linewidth=1.5, label='左膝关节力矩 [N·m]')
    ax3.plot(t, data['hip_cmd_left'], color='#1976D2', linewidth=1.5, label='左髋关节力矩 [N·m]')
    ax3.axhline(y=0.0, color='gray', linestyle='--', alpha=0.4)
    ax3.set_ylabel('关节力矩 [N·m]', fontsize=11, fontweight='bold')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.legend(loc='upper right', fontsize=9.0, framealpha=0.92)

    # ── 4. 机身俯仰角与俯仰角速度动态 ──
    ax4 = axes[3]
    line_pitch = ax4.plot(t, np.degrees(pitch), color='#212121', linewidth=1.6, label='机身俯仰角 Pitch [deg]')
    ax4.axhline(y=0.0, color='gray', linestyle='--', alpha=0.5, label='水平基准 (0 deg)')
    ax4.set_ylabel('俯仰角 [deg]', fontsize=11, fontweight='bold')
    ax4.grid(True, alpha=0.3, linestyle='--')

    ax4b = ax4.twinx()
    line_gyro = ax4b.plot(t, np.degrees(pitch_rate), color='#8E24AA', linewidth=1.2, linestyle='-', alpha=0.85,
                          label='俯仰角速度 Pitch Rate [deg/s]')
    ax4b.set_ylabel('角速度 [deg/s]', fontsize=9.5, color='#8E24AA')
    ax4b.tick_params(axis='y', labelcolor='#8E24AA')

    # 标记收腿开始时刻
    tuck_indices = np.where((states == 3) & (flight_subphase == 1))[0]
    line_tuck = []
    if len(tuck_indices) > 0:
        t_tuck_start = t[tuck_indices[0]]
        vl = ax4.axvline(x=t_tuck_start, color='#E91E63', linestyle='-.', linewidth=1.8,
                         label=f'收腿开始时刻 ({t_tuck_start:.2f}s)')
        line_tuck.append(vl)

    lines4_a, labels4_a = ax4.get_legend_handles_labels()
    lines4_b, labels4_b = ax4b.get_legend_handles_labels()
    ax4.legend(lines4_a + lines4_b, labels4_a + labels4_b,
               loc='upper right', fontsize=8.5, framealpha=0.92)

    # ── 5. 轮速指令与实际左右轮响应 ──
    ax5 = axes[4]
    ax5.plot(t, cmd_x, color='#1565C0', linewidth=1.8, label='车轮线速度指令 cmd_x [m/s]')
    wheel_radius = 0.07
    ax5.plot(t, -wheel_radius * left_wvel, color='#D32F2F', linewidth=1.3, linestyle='--',
             label='左轮实际线速度 [m/s]')
    ax5.plot(t, -wheel_radius * right_wvel, color='#388E3C', linewidth=1.3, linestyle=':',
             label='右轮实际线速度 [m/s]')
    air_cmd = field_or_default(data, 'air_wheel_cmd_raw', None)
    if air_cmd is not None and np.any(np.abs(air_cmd) > 1e-4):
        ax5.plot(t, air_cmd, color='#FF6F00', linewidth=1.1, alpha=0.75,
                 label='空中轮速修正原始指令 [m/s]')
    ax5.axhline(y=0.0, color='gray', linestyle='--', alpha=0.4)
    ax5.set_xlabel('时间 [s]', fontsize=12, fontweight='bold')
    ax5.set_ylabel('轮速 [m/s]', fontsize=11, fontweight='bold')
    ax5.grid(True, alpha=0.3, linestyle='--')
    ax5.legend(loc='upper right', fontsize=8.5, framealpha=0.92)

    # 叠加推地与腾空子阶段背景色块标注
    add_phase_spans(axes, t, states, flight_subphase, thrust_blocked)

    plt.tight_layout()
    plt.savefig(output_path, dpi=180, bbox_inches='tight')
    print(f"[INFO] 分析图像已成功生成并保存至: {output_path}")


def main():
    data_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='BBot 跳跃全流程响应分析')
    velocity_log = os.path.join(data_dir, 'jump_velocity_control_log.csv')
    legacy_log = os.path.join(data_dir, 'jump_control_log.csv')
    default_log = velocity_log if os.path.exists(velocity_log) else legacy_log
    parser.add_argument('csvfile', nargs='?',
                        default=default_log,
                        help='跳跃 CSV 日志文件')
    parser.add_argument('--output', '-o', type=str,
                        default=os.path.join(data_dir, 'jump_performance_analysis.png'),
                        help='输出图表路径')
    args = parser.parse_args()

    data = load_csv(args.csvfile)
    plot_jump_performance(data, args.output)


if __name__ == '__main__':
    main()
