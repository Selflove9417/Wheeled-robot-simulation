#!/usr/bin/env python3
"""Read-only phase-aligned diagnostics for baseline flat-ground jumps.

The runner writes one event sidecar per attempt. This module never sends robot
commands; it only copies values already present in the controller CSV.
"""
import argparse
import csv
import math
# No external dependencies: analysis works on archived CSV files.
from pathlib import Path

EVENTS = ("PRE_COMMAND", "PRE_JUMP", "SQUAT", "THRUST", "FLIGHT", "TOUCHDOWN_BUFFER")
REQUIRED_EVENTS = ("PRE_COMMAND", "PRE_JUMP", "THRUST", "FLIGHT", "TOUCHDOWN_BUFFER")
OBSERVABLES = (
    "timestamp", "state_name", "pitch", "pitch_rate", "capture_com_velocity",
    "com_world_z", "com_world_vz", "com_forward_from_axle", "com_lean",
    "left_wheel_vel", "right_wheel_vel", "hip_pos_left", "hip_pos_right",
    "knee_pos_left", "knee_pos_right", "hip_vel_left", "hip_vel_right",
    "knee_vel_left", "knee_vel_right", "wheel_clearance", "cmd_x",
    "com_sample_stamp", "odom_sample_stamp", "imu_sample_stamp",
    "joint_sample_stamp", "com_age", "odom_age", "imu_age", "joint_age",
    "controller_mode", "flight_subphase", "command_transport",
    "dispatch_sim_stamp", "jump_cmd_rx_stamp", "jump_cmd_accept_stamp",
)
FIELDS = ("event",) + OBSERVABLES
FRESH_AGE_LIMIT = 0.080
# Operational comparison bands, not estimated physical noise bounds.
BANDS = {
    "pitch": 0.060,
    "pitch_rate": 0.350,
    "capture_com_velocity": 0.100,
    "com_world_vz": 0.150,
}


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def snapshot(event, row):
    snap = {name: row.get(name, "") for name in FIELDS}
    snap["event"] = event
    now = number(row.get("timestamp"))
    for sensor in ("com", "odom", "imu", "joint"):
        stamp = number(row.get(sensor + "_sample_stamp"))
        age = None if now is None or stamp is None or stamp <= 0 else now - stamp
        snap[sensor + "_age"] = (
            f"{age:.6f}" if age is not None and -0.001 <= age <= 2.0 else ""
        )
    return snap


def write_event_csv(log_path, event_path, pre_command_row=None):
    """Save first row of each phase; a command snapshot is the last read before publish."""
    observed = {}
    if pre_command_row:
        observed["PRE_COMMAND"] = snapshot("PRE_COMMAND", pre_command_row)
    if Path(log_path).is_file():
        with open(log_path, newline="", encoding="utf-8", errors="replace") as source:
            for row in csv.DictReader(source):
                name = row.get("state_name", "")
                if name in EVENTS[1:] and name not in observed:
                    observed[name] = snapshot(name, row)
                if len(observed) == len(EVENTS):
                    break
    with open(event_path, "w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=FIELDS)
        writer.writeheader()
        for name in EVENTS:
            if name in observed:
                writer.writerow(observed[name])
    return observed


def validity(events):
    missing = [name for name in REQUIRED_EVENTS if name not in events]
    if missing:
        return False, "missing " + ",".join(missing)
    flight = events["FLIGHT"]
    stale = []
    for sensor in ("com", "odom", "imu", "joint"):
        age = number(flight.get(sensor + "_age"))
        if age is None or age > FRESH_AGE_LIMIT:
            stale.append(sensor)
    if stale:
        return False, "missing/stale FLIGHT " + ",".join(stale)
    return True, ""


def read_events(path):
    with open(path, newline="", encoding="utf-8") as source:
        return {row["event"]: row for row in csv.DictReader(source)}


def spread(rows, event, field):
    values = [number(row[event].get(field)) for row in rows if event in row]
    values = [value for value in values if value is not None]
    return max(values) - min(values) if len(values) >= 2 else None


def first_divergent_stage(rows):
    """Find the first event whose cross-trial spread breaches a declared band."""
    for event in EVENTS:
        over = [
            field for field, band in BANDS.items()
            if (width := spread(rows, event, field)) is not None and width > band
        ]
        if over:
            return event, over
    return None, []


def matched_flight_pairs(rows):
    matched = []
    for index, left in enumerate(rows):
        for right in rows[index + 1:]:
            a, b = left["FLIGHT"], right["FLIGHT"]
            pa, pb = number(a.get("pitch")), number(b.get("pitch"))
            ra, rb = number(a.get("pitch_rate")), number(b.get("pitch_rate"))
            if None not in (pa, pb, ra, rb) and abs(pa - pb) <= 0.060 and abs(ra - rb) <= 0.350:
                matched.append((left, right))
    return matched


def fmt(value, digits=3):
    return "—" if value is None else f"{value:.{digits}f}"


def analyze_campaign(folder, minimum=6):
    folder = Path(folder)
    attempts = []
    for path in sorted(folder.glob("trial_*_events.csv")):
        events = read_events(path)
        valid, reason = validity(events)
        summary_path = path.with_name(path.name.replace("_events.csv", "_summary.csv"))
        summary = {}
        if summary_path.is_file():
            with open(summary_path, newline="", encoding="utf-8") as source:
                summary = next(iter(csv.DictReader(source)), {})
        attempts.append((path, events, valid, reason, summary))
    valid_rows = [events for _, events, valid, _, _ in attempts if valid]
    lines = [
        "# 平地跳跃初态与时序对比",
        "",
        f"- 启动 {len(attempts)} 次，有效完整阶段 {len(valid_rows)} 次；"
        f"初始/记录无效 {len(attempts) - len(valid_rows)} 次。"
        "有效指原五个核心节点齐全，且 FLIGHT 首行四类采样龄均不超过 80 ms；"
        "落地验收失败仍算有效观测。",
        "- 下表使用控制器同一日志行的数值；传感器不同步由采样龄单独标出。"
        "阶段极差和阈值只用于发现何时开始不可比，不证明动力学因果。",
        "",
        "| 试验 | 有效性 | 离地俯仰/角速度 | 触地俯仰/角速度 | 推地→离地→触地 (ms) | 退出码 |",
        "|---|---|---|---|---|---|",
    ]
    for path, events, valid, reason, summary in attempts:
        th = events.get("THRUST", {})
        fl = events.get("FLIGHT", {})
        td = events.get("TOUCHDOWN_BUFFER", {})
        tth, tfl, ttd = (number(row.get("timestamp")) for row in (th, fl, td))
        intervals = (
            f"{fmt(1000 * (tfl - tth), 0)} / {fmt(1000 * (ttd - tfl), 0)}"
            if None not in (tth, tfl, ttd) else "—"
        )
        state = "有效" if valid else "无效：" + reason
        flight_state = f"{fmt(number(fl.get('pitch')))} / {fmt(number(fl.get('pitch_rate')))}"
        touchdown_state = f"{fmt(number(td.get('pitch')))} / {fmt(number(td.get('pitch_rate')))}"
        lines.append(
            f"| {path.stem.replace('_events', '')} | {state} | {flight_state} | "
            f"{touchdown_state} | {intervals} | {summary.get('exit_code', '—')} |"
        )
    lines.extend(["", "## 阶段离散与判定", "",
                  "| 阶段 | 俯仰极差 (rad) | 角速度极差 (rad/s) | COM 前速极差 (m/s) | COM 竖速极差 (m/s) |",
                  "|---|---:|---:|---:|---:|"])
    for event in EVENTS:
        values = [spread(valid_rows, event, key) for key in BANDS]
        lines.append("| " + event + " | " + " | ".join(fmt(x) for x in values) + " |")
    lines.extend([
        "",
        "| 阶段 | 左轮速极差 (rad/s) | 左髋速度极差 (rad/s) | 左膝速度极差 (rad/s) | 左髋角极差 (rad) | 左膝角极差 (rad) |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    joint_fields = ("left_wheel_vel", "hip_vel_left", "knee_vel_left",
                    "hip_pos_left", "knee_pos_left")
    for event in EVENTS:
        lines.append("| " + event + " | " +
                     " | ".join(fmt(spread(valid_rows, event, key)) for key in joint_fields) + " |")
    lines.extend(["", "| 时段 | 耗时范围 (ms) |", "|---|---:|"])
    for start, end in zip(EVENTS, EVENTS[1:]):
        durations = []
        for row in valid_rows:
            left = number(row.get(start, {}).get("timestamp"))
            right = number(row.get(end, {}).get("timestamp"))
            if left is not None and right is not None:
                durations.append(1000.0 * (right - left))
        duration_range = (f"{fmt(min(durations), 0)}–{fmt(max(durations), 0)}"
                          if durations else "—")
        lines.append(f"| {start} → {end} | {duration_range} |")
    pairs = matched_flight_pairs(valid_rows)
    total_pairs = len(valid_rows) * (len(valid_rows) - 1) // 2
    lines.append(
        f"\n- FLIGHT 入段仅按俯仰 ≤0.06 rad、角速度 ≤0.35 rad/s 配对："
        f"{len(pairs)}/{total_pairs} 对；这不代表轮、腿或质心速度也匹配。"
    )
    if valid_rows:
        launch_delay = [
            1000.0 * (number(row["PRE_JUMP"]["timestamp"]) -
                      number(row["PRE_COMMAND"]["timestamp"]))
            for row in valid_rows
        ]
        if max(launch_delay) - min(launch_delay) > 300.0:
            lines.append(
                "- PRE_COMMAND 快照到 PRE_JUMP 的间隔波动超过 300 ms；"
                "这段可能包含发布进程启动、控制器接收和日志采样，"
                "须结合命令时间戳分解，不能直接等同消息传输延迟。"
            )
    if valid_rows:
        ages = [
            number(row["FLIGHT"].get(sensor + "_age"))
            for row in valid_rows for sensor in ("com", "odom", "imu", "joint")
        ]
        ages = [x for x in ages if x is not None]
        lines.append(f"- 有效样本 FLIGHT 最大采样龄：{fmt(max(ages) if ages else None, 4)} s。")
    if len(valid_rows) < minimum:
        lines.append(
            f"- 结论：未达到至少 {minimum} 次有效样本；先解决启动/记录复现率，停止调参。"
        )
    else:
        event, fields = first_divergent_stage(valid_rows)
        if event is None:
            branch = "所列阶段未超过比较带；下一轮优先建模空中到触地的耦合。"
        elif event in ("PRE_COMMAND", "PRE_JUMP", "SQUAT"):
            branch = "跳跃命令前后已不可比；下一轮先收紧脚本初态触发条件。"
        elif event == "THRUST":
            branch = "推地入段已不可比；下一轮核查启动与推地前的状态/采样时序。"
        else:
            branch = (
                "推地至离地阶段出现不可比；下一轮核查该段的采样、"
                "状态切换和轮—腿命令时序。"
                if event == "FLIGHT" else
                "离地初态相对可比、触地后分散；下一轮建立空中耦合与触地目标模型。"
            )
        where = event or "无"
        lines.append(
            f"- 首个超出比较带的阶段：{where}"
            f"（{', '.join(fields) if fields else '无超限项'}）。{branch}"
            "该分类只是下一轮排查入口，不是因果结论。"
        )
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, help="Directory with trial_*_events.csv")
    parser.add_argument("--minimum", type=int, default=6)
    args = parser.parse_args()
    print(analyze_campaign(args.folder, args.minimum), end="")


if __name__ == "__main__":
    main()
