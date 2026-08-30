#!/usr/bin/env python3
"""
plot_height_torque.py
=====================
读取仿真生成的 height_torque_log.csv，绘制高度变化时关节力矩随时间变化曲线。

用法:
  python3 plot_height_torque.py                          # 默认读取 height_torque_log.csv
  python3 plot_height_torque.py my_log.csv               # 指定文件
  python3 plot_height_torque.py --output my_plot.png     # 指定输出
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse
import os
import sys


def load_csv(filepath):
    """加载仿真 CSV 日志"""
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        sys.exit(1)
    data = np.genfromtxt(filepath, delimiter=',', names=True, skip_header=0)
    print(f"[INFO] 已加载 {len(data)} 行, "
          f"时长 {data['timestamp'][-1] - data['timestamp'][0]:.1f}s")
    print(f"  高度: {data['com_height'].min():.3f} ~ {data['com_height'].max():.3f} m")
    print(f"  髋角: {np.degrees(data['hip_angle_rad']).min():.1f} ~ "
          f"{np.degrees(data['hip_angle_rad']).max():.1f} deg")
    print(f"  膝角: {np.degrees(data['knee_angle_rad']).min():.1f} ~ "
          f"{np.degrees(data['knee_angle_rad']).max():.1f} deg")
    has_effort = 'hip_effort' in data.dtype.names
    if has_effort:
        print(f"  髋力矩(calc): {data['hip_torque'].min():.1f}~{data['hip_torque'].max():.1f} Nm  "
              f"(sim): {data['hip_effort'].min():.1f}~{data['hip_effort'].max():.1f} Nm")
        print(f"  膝力矩(calc): {data['knee_torque'].min():.1f}~{data['knee_torque'].max():.1f} Nm  "
              f"(sim): {data['knee_effort'].min():.1f}~{data['knee_effort'].max():.1f} Nm")
    return data


def plot_height_torque(data, output_path):
    """绘制时间序列: 高度 + 髋力矩 + 膝力矩"""
    has_effort = 'hip_effort' in data.dtype.names
    t = data['timestamp'] - data['timestamp'][0]  # 相对时间

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # ======== 子图 1: 质心高度 vs 时间 ========
    ax1.plot(t, data['com_height'], 'k-', linewidth=1.2)
    ax1.set_ylabel('CoM Height [m]', fontsize=12)
    ax1.set_title('Joint Torque Response during Height Change', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.legend(['CoM Height'], fontsize=9, loc='upper right')

    # ======== 子图 2: 髋关节力矩 vs 时间 ========
    ax2.plot(t, data['hip_torque'], 'b-', linewidth=1.0, alpha=0.6, label='calc (J^T·g)')
    if has_effort:
        ax2.plot(t, data['hip_effort'], 'b-', linewidth=1.2, label='sim actual')
    ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.4)
    ax2.set_ylabel('Hip Torque [Nm]', fontsize=12)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # ======== 子图 3: 膝关节力矩 vs 时间 ========
    ax3.plot(t, data['knee_torque'], 'r-', linewidth=1.0, alpha=0.6, label='calc (J^T·g)')
    if has_effort:
        ax3.plot(t, data['knee_effort'], 'r-', linewidth=1.2, label='sim actual')
    ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.4)
    ax3.set_xlabel('Time [s]', fontsize=12)
    ax3.set_ylabel('Knee Torque [Nm]', fontsize=12)
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"[INFO] 已保存: {output_path}")


def main():
    data_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='BBot 关节力矩时序分析')
    parser.add_argument('csvfile', nargs='?',
                        default=os.path.join(data_dir, 'height_torque_log.csv'),
                        help='仿真 CSV 日志文件路径')
    parser.add_argument('--output', '-o', type=str,
                        default=os.path.join(data_dir, 'height_torque_analysis.png'),
                        help='输出图片路径')
    args = parser.parse_args()

    data = load_csv(args.csvfile)
    plot_height_torque(data, args.output)
    print("[INFO] 完成!")

    # 自动打开生成的图片
    import subprocess
    subprocess.run(['xdg-open', args.output])


if __name__ == '__main__':
    main()
