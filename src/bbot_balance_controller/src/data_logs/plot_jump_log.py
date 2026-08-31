#!/usr/bin/env python3
"""
plot_jump_log.py
================
读取仿真生成的 jump_control_log.csv，绘制跳跃全流程 5 阶段状态机、高度、速度、力矩及冲击力响应曲线。

用法:
  python3 plot_jump_log.py                          # 默认读取 jump_control_log.csv
  python3 plot_jump_log.py my_jump_log.csv          # 指定文件
  python3 plot_jump_log.py --output jump_plot.png   # 指定输出
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse
import os
import sys


def load_csv(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        sys.exit(1)
    data = np.genfromtxt(filepath, delimiter=',', names=True, skip_header=0)
    print(f"[INFO] 已加载 {len(data)} 行数据, 时长 {data['timestamp'][-1] - data['timestamp'][0]:.2f}s")
    return data


def plot_jump_performance(data, output_path):
    t = data['timestamp'] - data['timestamp'][0]

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    # 1. 腿长高度与状态机流转
    ax1 = axes[0]
    ax1.plot(t, data['z'], 'b-', linewidth=2.0, label='CoM Height z [m]')
    ax1.axhline(y=0.25, color='r', linestyle='--', alpha=0.6, label='Hard Min (0.25m)')
    ax1.axhline(y=0.39, color='g', linestyle='--', alpha=0.6, label='Takeoff Target (0.39m)')
    ax1.axhline(y=0.45, color='r', linestyle='--', alpha=0.6, label='Hard Max (0.45m)')
    ax1.set_ylabel('Height [m]', fontsize=11)
    ax1.set_title('BBOT Jump Control: 5-Stage FSM Telemetry & Dynamics Response', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)

    # 标注状态流转背景色
    states = data['state']
    state_changes = np.where(np.diff(states) != 0)[0]
    state_names = ["BALANCE", "SQUAT", "THRUST", "FLIGHT", "BUFFER", "RECOVERY", "STANDUP", "EMERGENCY"]
    colors = ['#E8F5E9', '#FFF3E0', '#FFEBEE', '#E3F2FD', '#EDE7F6', '#F1F8E9', '#ECEFF1', '#FFCDD2']

    start_idx = 0
    for change_idx in list(state_changes) + [len(data) - 1]:
        s = int(states[start_idx])
        if s < len(colors):
            ax1.axvspan(t[start_idx], t[change_idx], color=colors[s], alpha=0.4)
            mid_t = 0.5 * (t[start_idx] + t[change_idx])
            s_name = state_names[s] if s < len(state_names) else f"S{s}"
            ax1.text(mid_t, ax1.get_ylim()[0] + 0.02, s_name,
                     ha='center', va='bottom', fontsize=9, fontweight='bold', alpha=0.7)
        start_idx = change_idx + 1

    # 2. 竖直速度与加速度
    ax2 = axes[1]
    ax2.plot(t, data['z_dot'], 'm-', linewidth=1.5, label='Vertical Velocity z_dot [m/s]')
    ax2.axhline(y=1.80, color='g', linestyle=':', alpha=0.7, label='Target v_takeoff (1.80 m/s)')
    ax2.axhline(y=0.0, color='gray', linestyle='--', alpha=0.4)
    ax2.set_ylabel('Velocity [m/s]', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=9)

    # 3. 关节力矩与物理限幅
    ax3 = axes[2]
    ax3.plot(t, data['tau_ff_knee'], 'r-', linewidth=1.8, label='Left Knee Torque (cmd/ff) [Nm]')
    ax3.plot(t, data['tau_ff_hip'], 'b-', linewidth=1.2, label='Left Hip Torque (cmd/ff) [Nm]')
    ax3.axhline(y=60.0, color='r', linestyle='--', alpha=0.5, label='Knee Max Limit (60 Nm)')
    ax3.axhline(y=-60.0, color='r', linestyle='--', alpha=0.5)
    ax3.axhline(y=75.0, color='b', linestyle='--', alpha=0.5, label='Hip Max Limit (75 Nm)')
    ax3.axhline(y=-75.0, color='b', linestyle='--', alpha=0.5)
    ax3.axhline(y=0.0, color='gray', linestyle=':', alpha=0.3)
    ax3.set_ylabel('Torque [Nm]', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', fontsize=9)

    # 4. 机身俯仰角与垂直作用力
    ax4 = axes[3]
    ax4.plot(t, np.degrees(data['pitch']), 'k-', linewidth=1.5, label='Pitch Angle [deg]')
    ax4.axhline(y=0.0, color='gray', linestyle='--', alpha=0.5)
    ax4.set_xlabel('Time [s]', fontsize=12)
    ax4.set_ylabel('Pitch [deg]', fontsize=11)
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='upper right', fontsize=9)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"[INFO] 分析图像已成功保存至: {output_path}")


def main():
    data_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='BBot 跳跃全流程响应分析')
    parser.add_argument('csvfile', nargs='?',
                        default=os.path.join(data_dir, 'jump_control_log.csv'),
                        help='跳跃 CSV 日志文件')
    parser.add_argument('--output', '-o', type=str,
                        default=os.path.join(data_dir, 'jump_performance_analysis.png'),
                        help='输出图表路径')
    args = parser.parse_args()

    data = load_csv(args.csvfile)
    plot_jump_performance(data, args.output)


if __name__ == '__main__':
    main()
