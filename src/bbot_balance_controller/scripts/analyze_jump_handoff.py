#!/usr/bin/env python3
"""Classify the earliest observed failure in flat-ground jump attempts."""

import argparse
import csv
from pathlib import Path


def read_rows(path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8", errors="replace") as stream:
        return list(csv.DictReader(stream))


def number(value, fallback=-1.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def classify_attempt(rows, outcome):
    """Return one earliest-stage category; never infer switch state from controller_mode."""
    reason = outcome.get("reason", "")
    states = {row.get("state_name", "") for row in rows}
    if "PRE_JUMP" not in states:
        if "command not received" in reason.lower():
            return "COMMAND_NOT_RECEIVED"
        return "STARTUP_OR_BALANCE"
    if "THRUST" not in states:
        return "PRE_THRUST_FAILURE"
    if "FLIGHT" in states:
        return "FLIGHT_REACHED"

    thrust = [row for row in rows if row.get("state_name") == "THRUST"]
    request = next((number(row.get("effort_switch_request_stamp")) for row in thrust
                    if number(row.get("effort_switch_request_stamp")) >= 0), -1.0)
    if request < 0:
        return "SWITCH_NOT_REQUESTED"
    switch_rows = [row for row in rows if number(row.get("effort_switch_request_stamp")) == request]
    final = switch_rows[-1] if switch_rows else thrust[-1]
    result = int(number(final.get("effort_switch_result")))
    ack = number(final.get("effort_switch_ack_stamp"))
    if result in (2, 3):
        return "SWITCH_FAILED"
    if result != 1 or ack < 0:
        return "SWITCH_UNCONFIRMED"
    if ack - request > 0.20:
        return "SWITCH_DELAYED"
    if max(number(row.get("thrust_gate_open_stamp")) for row in thrust) < 0:
        return "THRUST_GATE_TIMEOUT"
    return "POST_GATE_FAILURE"


def analyze(folder):
    folder = Path(folder)
    lines = ["# 平地跳跃最早故障阶段与飞行到达情况", "",
             "| 试验 | 分类 | 切换请求→确认 (ms) | 初始门槛最大连续达标 (ms) | 原始原因 |",
             "|---|---|---:|---:|---|"]
    counts = {}
    for outcome_path in sorted(folder.glob("trial_*_outcome.csv")):
        outcome_rows = read_rows(outcome_path)
        outcome = outcome_rows[-1] if outcome_rows else {}
        log_path = outcome_path.with_name(outcome_path.name.replace("_outcome.csv", "_log.csv"))
        rows = read_rows(log_path)
        category = classify_attempt(rows, outcome)
        counts[category] = counts.get(category, 0) + 1
        thrust = [row for row in rows if row.get("state_name") == "THRUST"]
        requested = [number(row.get("effort_switch_request_stamp")) for row in thrust
                     if number(row.get("effort_switch_request_stamp")) >= 0]
        acked = [number(row.get("effort_switch_ack_stamp")) for row in thrust
                 if number(row.get("effort_switch_ack_stamp")) >= 0]
        delay = f"{1000 * (acked[0] - requested[0]):.0f}" if requested and acked else "—"
        initial_gate = []
        for row in thrust:
            initial_gate.append(row)
            if number(row.get("thrust_gate_open_stamp")) >= 0:
                break
        stable = max((number(row.get("thrust_gate_stable_elapsed"), 0.0)
                      for row in initial_gate), default=0.0)
        safe_reason = outcome.get("reason", "").replace("|", "/").replace("\n", " ")
        lines.append(f"| {outcome_path.stem.replace('_outcome', '')} | {category} | "
                     f"{delay} | {stable * 1000:.0f} | {safe_reason} |")
    lines.extend(["", "分类计数：" + ("、".join(f"{key}={value}" for key, value in sorted(counts.items()))
                                  if counts else "无本轮结果")])
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    print(analyze(args.folder), end="")


if __name__ == "__main__":
    main()
