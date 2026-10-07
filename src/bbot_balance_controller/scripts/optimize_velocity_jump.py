#!/usr/bin/env python3
"""Batch Bayesian optimization for the BBot velocity-targeted jump.

Each evaluation launches the existing Gazebo launch file with one candidate
trajectory, triggers /jump_cmd, and scores jump_velocity_summary.csv.  The
controller itself remains deterministic and real-time safe; this script is
only an offline experiment driver.
"""
import argparse
import csv
import math
import os
import random
import signal
import subprocess
import time

import numpy as np


BOUNDS = np.array([
    [0.10, 0.28],   # thrust_duration
    [1.20, 3.50],   # thrust_peak_ratio
    [0.20, 1.40],   # thrust_shape_early
    [0.20, 1.40],   # thrust_shape_late
])


def candidate_from_unit(u):
    return BOUNDS[:, 0] + np.asarray(u) * (BOUNDS[:, 1] - BOUNDS[:, 0])


def score(row, target_height):
    failed = float(row.get("failed", 1))
    recovery = float(row.get("recovery_completed", 0))
    height_delta = float(row.get("apex_world_z_delta", row.get("apex_height_delta", 0.0)))
    height_err = abs(height_delta - target_height)
    speed_err = abs(float(row.get("takeoff_velocity", 0.0)) -
                   math.sqrt(2.0 * 9.81 * target_height))
    pitch = abs(float(row.get("max_abs_pitch", 99.0)))
    takeoff_pitch_rate = abs(float(row.get("takeoff_pitch_rate", 0.0)))
    landing_pitch_err = abs(float(row.get("landing_pitch_err", 0.0)))
    touchdown_drift_x = abs(float(row.get("touchdown_drift_x", 0.0)))
    hip = abs(float(row.get("max_abs_hip_torque", 99.0)))
    knee = abs(float(row.get("max_abs_knee_torque", 99.0)))
    work = abs(float(row.get("mechanical_work", 999.0)))
    # Lower is better. Safety and recovery dominate all efficiency terms.
    return (10000.0 * failed + 4000.0 * (1.0 - recovery) +
            2200.0 * height_err + 700.0 * speed_err +
            150.0 * pitch + 80.0 * takeoff_pitch_rate +
            100.0 * landing_pitch_err + 120.0 * touchdown_drift_x +
            0.8 * max(0.0, hip - 55.0) +
            0.8 * max(0.0, knee - 45.0) + 0.03 * work)


def read_summary(path):
    if not os.path.exists(path):
        return None
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    return rows[-1] if rows else None


def evaluate(x, args):
    summary = os.path.expanduser(args.summary)
    # 每次试验必须产生自己的汇总行；禁止启动失败时误读上一次结果。
    try:
        os.unlink(summary)
    except FileNotFoundError:
        pass
    trial_started = time.time()
    launch = [
        "ros2", "launch", "bbot_bringup", "bbot_gazebo.launch.py",
        "controller_type:=jump_velocity", "world:=empty.sdf",
        "jump_height:=%.6f" % args.jump_height,
        "thrust_duration:=%.6f" % x[0],
        "thrust_peak_ratio:=%.6f" % x[1],
        "thrust_shape_early:=%.6f" % x[2],
        "thrust_shape_late:=%.6f" % x[3],
    ]
    proc = subprocess.Popen(launch, start_new_session=True)
    row = None
    try:
        startup_deadline = time.monotonic() + args.startup_seconds
        while time.monotonic() < startup_deadline:
            if proc.poll() is not None:
                return 1.0e6, {"failed": "1", "recovery_completed": "0",
                               "reason": "launch exited before jump command"}
            time.sleep(0.1)

        pub = subprocess.run([
            "ros2", "topic", "pub", "--once", "/jump_cmd",
            "std_msgs/msg/String", "{data: jump}",
        ], check=False, timeout=10.0)
        if pub.returncode != 0:
            return 1.0e6, {"failed": "1", "recovery_completed": "0",
                           "reason": "failed to publish jump command"}

        trial_deadline = time.monotonic() + args.trial_seconds
        while time.monotonic() < trial_deadline:
            if os.path.exists(summary) and os.path.getmtime(summary) >= trial_started:
                row = read_summary(summary)
                if row is not None:
                    break
            if proc.poll() is not None:
                break
            time.sleep(0.1)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=3.0)
    if row is None:
        return 1.0e6, {"failed": "1", "recovery_completed": "0",
                       "reason": "no fresh terminal summary before timeout"}
    return score(row, args.jump_height), row


def gp_predict(X, y, grid):
    if len(X) < 2:
        return np.full(len(grid), np.mean(y)), np.full(len(grid), 1.0)
    length = 0.25
    d = X[:, None, :] - X[None, :, :]
    K = np.exp(-0.5 * np.sum(d * d, axis=2) / (length * length))
    K += 1.0e-6 * np.eye(len(X))
    Ks = np.exp(-0.5 * np.sum((grid[:, None, :] - X[None, :, :]) ** 2, axis=2) /
                (length * length))
    alpha = np.linalg.solve(K, y)
    mean = Ks @ alpha
    solved_cross_cov = np.linalg.solve(K, Ks.T).T
    variance = 1.0 - np.sum(Ks * solved_cross_cov, axis=1)
    return mean, np.sqrt(np.maximum(variance, 1.0e-9))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--jump-height", type=float, default=0.20)
    parser.add_argument("--startup-seconds", type=float, default=5.0)
    parser.add_argument("--trial-seconds", type=float, default=12.0)
    parser.add_argument("--summary", default="~/bbot_ws_new/src/bbot_balance_controller/src/data_logs/jump_velocity_summary.csv")
    parser.add_argument("--output", default="velocity_jump_best_params.yaml")
    args = parser.parse_args()
    rng = random.Random(20260902)
    X, y, records = [], [], []
    for trial in range(args.trials):
        if trial < 5 or len(X) < 2:
            unit = [rng.random() for _ in range(4)]
        else:
            pool = np.random.default_rng(1000 + trial).random((512, 4))
            mean, std = gp_predict(np.asarray(X), np.asarray(y), pool)
            best = min(y)
            z = (best - mean) / std
            cdf = 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))
            pdf = np.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)
            ei = (best - mean) * cdf + std * pdf
            unit = pool[int(np.argmax(ei))]
        x = candidate_from_unit(unit)
        value, row = evaluate(x, args)
        X.append(np.asarray(unit))
        y.append(value)
        records.append((value, x, row))
        print("trial %02d score %.3f params %s" % (trial + 1, value,
              " ".join("%.4f" % v for v in x)), flush=True)
    best = min(records, key=lambda item: item[0])
    with open(args.output, "w") as stream:
        stream.write("jump_height: %.6f\n" % args.jump_height)
        stream.write("thrust_duration: %.6f\n" % best[1][0])
        stream.write("thrust_peak_ratio: %.6f\n" % best[1][1])
        stream.write("thrust_shape_early: %.6f\n" % best[1][2])
        stream.write("thrust_shape_late: %.6f\n" % best[1][3])
        stream.write("score: %.6f\n" % best[0])
    print("best score %.3f written to %s" % (best[0], args.output))


if __name__ == "__main__":
    main()
