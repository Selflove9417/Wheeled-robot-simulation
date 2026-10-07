#!/usr/bin/env python3
"""Compare matched CLI and persistent /jump_cmd delivery trials."""
import argparse
import csv
from pathlib import Path

from analyze_flat_jump_repeatability import number, read_events, spread, validity


def metric_range(values):
    finite = [value for value in values if value is not None]
    return max(finite) - min(finite) if len(finite) >= 2 else None


def fmt(value, digits=1):
    return "—" if value is None else f"{value:.{digits}f}"


def accepted_timing(events):
    pre = events.get("PRE_COMMAND", {})
    jump = events.get("PRE_JUMP", {})
    dispatch = number(pre.get("dispatch_sim_stamp"))
    receive = number(jump.get("jump_cmd_rx_stamp"))
    accept = number(jump.get("jump_cmd_accept_stamp"))
    first = number(jump.get("timestamp"))
    if None in (dispatch, receive, accept, first):
        return None, "missing command timestamp"
    if min(dispatch, receive, accept, first) <= 0:
        return None, "non-positive command timestamp"
    if receive < dispatch - 0.060 or accept < receive or first < accept:
        return None, "inconsistent sim-time order"
    return {
        "dispatch_to_receive_ms": (receive - dispatch) * 1000.0,
        "receive_to_accept_ms": (accept - receive) * 1000.0,
        "accept_to_first_ms": (first - accept) * 1000.0,
        "pre_to_first_ms": (first - number(pre.get("timestamp"))) * 1000.0,
    }, ""


def precommand_matched(left, right):
    checks = {
        "pitch": 0.010, "pitch_rate": 0.050,
        "capture_com_velocity": 0.030,
        "left_wheel_vel": 0.200, "right_wheel_vel": 0.200,
        "hip_pos_left": 0.020, "hip_pos_right": 0.020,
        "knee_pos_left": 0.020, "knee_pos_right": 0.020,
    }
    a = left.get("PRE_COMMAND", {})
    b = right.get("PRE_COMMAND", {})
    for key, limit in checks.items():
        av, bv = number(a.get(key)), number(b.get(key))
        if av is None or bv is None or abs(av - bv) > limit:
            return False
    for row in (a, b):
        for sensor in ("com", "odom", "imu", "joint"):
            age = number(row.get(sensor + "_age"))
            if age is None or age > 0.080:
                return False
    return True


def analyze(root):
    root = Path(root)
    trials = []
    for event_file in sorted(root.rglob("trial_*_events.csv")):
        events = read_events(event_file)
        mode = event_file.parent.name
        pair = event_file.parent.parent.name
        valid, reason = validity(events)
        timing, timing_reason = accepted_timing(events)
        declared = events.get("PRE_COMMAND", {}).get("command_transport", "")
        if mode not in ("cli", "persistent") or declared != mode:
            valid, reason = False, "transport label mismatch"
        if timing is None:
            valid, reason = False, timing_reason
        summary_file = event_file.with_name(event_file.name.replace("_events.csv", "_summary.csv"))
        summary = {}
        if summary_file.is_file():
            with summary_file.open(newline="", encoding="utf-8") as source:
                summary = next(iter(csv.DictReader(source)), {})
        trials.append({
            "pair": pair, "mode": mode, "events": events,
            "valid": valid, "reason": reason, "timing": timing,
            "exit": summary.get("exit_code", "—"),
        })
    lines = [
        "# 跳跃命令传输方式同期对照", "",
        "命令开始时刻由同一试验节点的仿真时钟记录；CLI 数值包含其进程启动，"
        "常驻方式数值从 publish 调用起算。两者均到控制器回调时间，"
        "不是单独的网络传输延迟。", "",
        "| 配对 | 方式 | 有效 | 开始→收到 (ms) | 收到→接受 (ms) | 接受→首行 (ms) | 命令前快照→PRE_JUMP (ms) | PRE_JUMP 左轮速 | FLIGHT 左轮/左髋/左膝速度 | 退出 |",
        "|---|---|---|---:|---:|---:|---:|---:|---|---|",
    ]
    for trial in trials:
        e = trial["events"]
        t = trial["timing"] or {}
        flight = e.get("FLIGHT", {})
        v = "是" if trial["valid"] else "否：" + trial["reason"]
        flight_state = "/".join(
            fmt(number(flight.get(key)), 2)
            for key in ("left_wheel_vel", "hip_vel_left", "knee_vel_left")
        )
        lines.append(
            f"| {trial['pair']} | {trial['mode']} | {v} | "
            f"{fmt(t.get('dispatch_to_receive_ms'))} | "
            f"{fmt(t.get('receive_to_accept_ms'))} | "
            f"{fmt(t.get('accept_to_first_ms'))} | "
            f"{fmt(t.get('pre_to_first_ms'))} | "
            f"{fmt(number(e.get('PRE_JUMP', {}).get('left_wheel_vel')), 2)} | "
            f"{flight_state} | {trial['exit']} |"
        )
    paired = {}
    for trial in trials:
        paired.setdefault(trial["pair"], {})[trial["mode"]] = trial
    matching_pairs = [
        pair for pair in paired.values()
        if all(mode in pair and pair[mode]["valid"] for mode in ("cli", "persistent"))
        and precommand_matched(pair["cli"]["events"], pair["persistent"]["events"])
    ]
    lines.append(f"\n- 初态及采样龄合格的同期配对：{len(matching_pairs)}/{len(paired)}。")
    if len(matching_pairs) < 3:
        lines.append("- 结论：不足三组可比配对；不判定哪一种发令方式更可重复。")
        return "\n".join(lines) + "\n"
    grouped = {
        mode: [pair[mode] for pair in matching_pairs]
        for mode in ("cli", "persistent")
    }
    ranges = {}
    lines.extend(["", "| 方式 | 开始→收到极差 (ms) | 命令前→PRE_JUMP 极差 (ms) | PRE_JUMP 左轮速极差 (rad/s) | THRUST 左轮速极差 (rad/s) | FLIGHT 左轮/左髋/左膝速度极差 (rad/s) |",
                  "|---|---:|---:|---:|---:|---|"])
    for mode, runs in grouped.items():
        timing_range = metric_range([
            run["timing"]["dispatch_to_receive_ms"] for run in runs
        ])
        pre_range = metric_range([run["timing"]["pre_to_first_ms"] for run in runs])
        snapshots = [run["events"] for run in runs]
        pre_wheel = spread(snapshots, "PRE_JUMP", "left_wheel_vel")
        thrust_wheel = spread(snapshots, "THRUST", "left_wheel_vel")
        flight_spreads = "/".join(
            fmt(spread(snapshots, "FLIGHT", key), 2)
            for key in ("left_wheel_vel", "hip_vel_left", "knee_vel_left")
        )
        ranges[mode] = (timing_range, pre_range, pre_wheel)
        lines.append(
            f"| {mode} | {fmt(timing_range)} | {fmt(pre_range)} | "
            f"{fmt(pre_wheel, 3)} | {fmt(thrust_wheel, 3)} | {flight_spreads} |"
        )
    cli_t, _, cli_w = ranges["cli"]
    direct_t, _, direct_w = ranges["persistent"]
    time_better = (
        None not in (cli_t, direct_t) and direct_t <= 100.0
        and direct_t <= 0.5 * cli_t
    )
    wheel_better = (
        None not in (cli_w, direct_w) and direct_w <= 0.050
        and direct_w <= 0.5 * cli_w
    )
    if time_better and wheel_better:
        conclusion = (
            "常驻发送同时缩小命令时序和早期轮速离散；"
            "只在可重复性观测模式将其设为默认，普通试验入口保持 CLI。"
        )
    elif time_better:
        conclusion = (
            "常驻发送缩小命令时序，但未同步缩小早期轮速离散；"
            "下一轮转查 SQUAT/THRUST 采样与轮腿命令。"
        )
    else:
        conclusion = (
            "未证实命令发送是早期离散的主要来源；"
            "保留现有默认方式，下一轮转查内部阶段时序。"
        )
    lines.append(f"- 判定：{conclusion} 本试验不改变跳跃控制或落地参数。")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="Root with pair_N/cli and pair_N/persistent trial folders")
    args = parser.parse_args()
    print(analyze(args.root), end="")


if __name__ == "__main__":
    main()
