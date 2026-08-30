#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

data_dir = os.path.dirname(os.path.abspath(__file__))
base_path = data_dir + "/"

angle_files = {
    "data": base_path + "angle_data.txt",
    "target": base_path + "target_angle_data.txt",
    "timestamp": base_path + "timestamp_angle.txt",
    "target_timestamp": base_path + "timestamp_target_angle.txt",
}

speed_files = {
    "data": base_path + "speed_data.txt",
    "target": base_path + "target_speed_data.txt",
    "timestamp": base_path + "timestamp_speed.txt",
    "target_timestamp": base_path + "timestamp_target_speed.txt",
}

gyro_files = {
    "data": base_path + "gyro_data.txt",
    "target": base_path + "target_gyro_data.txt",
    "timestamp": base_path + "timestamp_gyro.txt",
    "target_timestamp": base_path + "timestamp_target_gyro.txt",
}


def read_data(file_path, data_name):
    if not os.path.exists(file_path):
        print(f"{data_name} 文件不存在: {file_path}")
        return []
    try:
        with open(file_path, "r") as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f"{data_name}: {len(data)} 个数据点")
        return data
    except Exception as e:
        print(f"读取 {data_name} 失败: {e}")
        return []


def process_dataset(files_dict, label):
    data = read_data(files_dict["data"], f"{label}实际值")
    target_data = read_data(files_dict["target"], f"{label}目标值")
    timestamps = read_data(files_dict["timestamp"], f"{label}时间戳")
    target_timestamps = read_data(files_dict["target_timestamp"], f"{label}目标时间戳")

    if len(data) == 0:
        print(f"警告: {label} 实际数据为空！")
        return None

    # 如果时间戳为空，默认按 200Hz 生成时间戳
    if len(timestamps) == 0:
        timestamps = [i * 0.005 for i in range(len(data))]

    min_len = min(len(data), len(timestamps))
    data = np.array(data[:min_len])
    timestamps = np.array(timestamps[:min_len])

    # 如果目标数据为空，默认全 0
    if len(target_data) == 0:
        print(f"提示: {label} 目标值为空，自动填充为 0.0")
        target_data = np.zeros(min_len)
        target_timestamps = timestamps
    else:
        target_min_len = min(len(target_data), min_len)
        data = data[:target_min_len]
        timestamps = timestamps[:target_min_len]
        target_data = np.array(target_data[:target_min_len])
        if len(target_timestamps) >= target_min_len:
            target_timestamps = np.array(target_timestamps[:target_min_len])
        else:
            target_timestamps = timestamps

    relative_time = timestamps - timestamps[0] if len(timestamps) > 0 else np.zeros_like(data)
    target_relative_time = target_timestamps - target_timestamps[0] if len(target_timestamps) > 0 else relative_time

    return {
        "data": data,
        "target_data": target_data,
        "relative_time": relative_time,
        "target_relative_time": target_relative_time,
        "label": label,
    }


def load_from_csv(csv_path):
    if not os.path.exists(csv_path):
        return None
    try:
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            csv_data = np.genfromtxt(csv_path, delimiter=",", names=True, invalid_raise=False)
        if csv_data is None or len(csv_data) == 0 or csv_data.ndim == 0 or csv_data.dtype.names is None:
            return None

        cols = csv_data.dtype.names
        print(f"成功从 CSV 加载数据: {csv_path} (共 {len(csv_data)} 行)")

        t = csv_data["timestamp"]
        rel_t = t - t[0]

        # Pitch
        pitch_actual = csv_data["pitch"]
        pitch_target = csv_data["eff_target_pitch"] if "eff_target_pitch" in cols else np.zeros_like(pitch_actual)
        angle_data = {
            "data": pitch_actual,
            "target_data": pitch_target,
            "relative_time": rel_t,
            "target_relative_time": rel_t,
            "label": "angle",
        }

        # Speed
        speed_actual = csv_data["x_dot"] if "x_dot" in cols else np.zeros_like(pitch_actual)
        speed_target = csv_data["target_speed"] if "target_speed" in cols else np.zeros_like(speed_actual)
        speed_data = {
            "data": speed_actual,
            "target_data": speed_target,
            "relative_time": rel_t,
            "target_relative_time": rel_t,
            "label": "speed",
        }

        # Gyro
        gyro_actual = csv_data["pitch_rate"] if "pitch_rate" in cols else np.zeros_like(pitch_actual)
        gyro_target = csv_data["eff_target_pitch_rate"] if "eff_target_pitch_rate" in cols else np.zeros_like(gyro_actual)
        gyro_data = {
            "data": gyro_actual,
            "target_data": gyro_target,
            "relative_time": rel_t,
            "target_relative_time": rel_t,
            "label": "gyro",
        }

        return angle_data, speed_data, gyro_data
    except Exception as e:
        print(f"读取 CSV 失败: {e}")
        return None


def add_stats_box(ax, data, target_data, unit):
    error = data - target_data
    rmse = np.sqrt(np.mean(error**2))
    mae = np.mean(np.abs(error))
    max_error = np.max(np.abs(error))

    text = (
        f"RMSE: {rmse:.4f} {unit}\n"
        f"MAE: {mae:.4f} {unit}\n"
        f"MaxE: {max_error:.4f} {unit}\n"
        f"N: {len(data)}"
    )

    ax.text(
        0.98,
        0.98,
        text,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    return rmse, mae, max_error


def main():
    print("=" * 60)
    print("Balance Controller Data Plot")
    print("=" * 60)

    parser = argparse.ArgumentParser(description="BBot 平衡控制器数据绘图工具")
    parser.add_argument("--csv", "-c", type=str, default=None, help="指定 CSV 日志文件路径 (例如 lqr_control_log.csv)")
    parser.add_argument("--use-txt", action="store_true", help="强制从 txt 文件读取")
    parser.add_argument("--no-show", action="store_true", help="不弹出窗口，仅保存图片")
    args = parser.parse_args()

    angle_data = None
    speed_data = None
    gyro_data = None

    default_csv = os.path.join(data_dir, "lqr_control_log.csv")
    csv_file = args.csv if args.csv else default_csv

    # 优先尝试从 CSV 读取（如果有且未指定 --use-txt）
    if not args.use_txt and os.path.exists(csv_file) and os.path.getsize(csv_file) > 0:
        csv_res = load_from_csv(csv_file)
        if csv_res is not None:
            angle_data, speed_data, gyro_data = csv_res

    # 如果未能从 CSV 读取，则从 txt 读取
    if angle_data is None:
        print("从 TXT 文件读取日志数据...")
        angle_data = process_dataset(angle_files, "angle")
        speed_data = process_dataset(speed_files, "speed")
        gyro_data = process_dataset(gyro_files, "gyro")

    if angle_data is None and speed_data is None and gyro_data is None:
        print("错误：无法读取任何有效数据，无法绘图。")
        return

    # 创建子图
    plots = [p for p in [angle_data, speed_data, gyro_data] if p is not None]
    num_plots = len(plots)

    fig, axes = plt.subplots(num_plots, 1, figsize=(16, 4 * num_plots), sharex=True)
    if num_plots == 1:
        axes = [axes]

    idx = 0
    angle_stats, speed_stats, gyro_stats = None, None, None

    if angle_data is not None:
        ax = axes[idx]
        ax.plot(angle_data["relative_time"], angle_data["data"], linewidth=2, label="current_pitch")
        ax.plot(angle_data["target_relative_time"], angle_data["target_data"], linewidth=2, label="target_pitch", linestyle="--")
        ax.set_ylabel("rad")
        ax.set_title("Pitch Tracking")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.legend(loc="upper right")
        angle_stats = add_stats_box(ax, angle_data["data"], angle_data["target_data"], "rad")
        idx += 1

    if speed_data is not None:
        ax = axes[idx]
        ax.plot(speed_data["relative_time"], speed_data["data"], linewidth=2, label="current_speed")
        ax.plot(speed_data["target_relative_time"], speed_data["target_data"], linewidth=2, label="target_speed", linestyle="--")
        ax.set_ylabel("m/s")
        ax.set_title("Speed Tracking")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.legend(loc="upper right")
        speed_stats = add_stats_box(ax, speed_data["data"], speed_data["target_data"], "m/s")
        idx += 1

    if gyro_data is not None:
        ax = axes[idx]
        ax.plot(gyro_data["relative_time"], gyro_data["data"], linewidth=2, label="current_pitch_rate")
        ax.plot(gyro_data["target_relative_time"], gyro_data["target_data"], linewidth=2, label="target_pitch_rate", linestyle="--")
        ax.set_xlabel("time / s")
        ax.set_ylabel("rad/s")
        ax.set_title("Pitch Rate Tracking")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.legend(loc="upper right")
        gyro_stats = add_stats_box(ax, gyro_data["data"], gyro_data["target_data"], "rad/s")
        idx += 1

    plt.tight_layout()

    output_path = os.path.join(data_dir, "all_metrics_comparison.png")
    plt.savefig(output_path, dpi=300, bbox_inches="tight")

    print("\n图像已保存：")
    print(output_path)

    print("\n统计结果：")
    if angle_stats:
        print(f"Pitch      RMSE={angle_stats[0]:.4f}, MAE={angle_stats[1]:.4f}, MaxE={angle_stats[2]:.4f}")
    if speed_stats:
        print(f"Speed      RMSE={speed_stats[0]:.4f}, MAE={speed_stats[1]:.4f}, MaxE={speed_stats[2]:.4f}")
    if gyro_stats:
        print(f"PitchRate  RMSE={gyro_stats[0]:.4f}, MAE={gyro_stats[1]:.4f}, MaxE={gyro_stats[2]:.4f}")

    if not args.no_show:
        try:
            plt.show()
        except Exception:
            pass


if __name__ == "__main__":
    main()
