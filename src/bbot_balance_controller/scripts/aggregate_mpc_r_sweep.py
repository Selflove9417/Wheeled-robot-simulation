#!/usr/bin/env python3
"""第一阶段 MPC R 扫描的按窗口聚合,带三分类重复类别。

只读:所有数字都从 run_mpc_r_sweep.py 留下的原始逐重复 CSV 重新推导,因此可以在
扫描仍在进行时运行,且绝不改变已测得的结果。它刻意不信任驱动自身的接受/拒绝判断
——驱动会重试任何缺少完整指标字典的运行,把"控制器从未稳定"和"实验根本没有运行"
混为一谈。二者不同,必须区分:

    protocol_invalid     没有控制行,或接管晚于 VALID_TAKEOVER_S(/imu 迟到、
                         发布丢失、数据缺失)。根本不算控制器结果。
    engaged_failed_gate  回路已闭合并运行,但 2 s 平衡门限(theta <= 0.57 度、
                         |pitch_rate| <= 0.02、|x_dot| <= 0.01)从未满足。这是
                         控制器失败,应计入 R 存活能力统计。
    fell                 触发 0.50 rad 安全门限(可能发生在门限已满足之后)。
                         控制器失败。
    gate_met             已接管、保持平衡门限、未触发。只有这些重复参与性能平均。

另两条规则:
 - 指标在每试验的分析窗口内计算,绝不在整个日志上计算,这样启动/腿部伸展瞬态
   就无法左右 R 的排名;
 - 求解器延迟给出 mean / p50 / p95 / p99 / max 以及 QP 工作集大小,因为在 5 ms
   周期下一次调度尖峰就会让 max 产生误导,而激活集个数才能解释成本趋势。

队列设计,因为驱动会重试任何缺指标的运行,不稳定的 R 值因此比稳定值获得更多尝试:

    primary cohort       最先出现的 --repeats 次真实控制器结果。protocol_invalid
                         被跳过且不占用名额;fell / engaged_failed_gate / gate_met
                         各占一个。这是每单元固定 n=3 的设计,对所有 R 一致。
    retry_diagnostic     队列填满之后的一切。保留在 JSON 和 aggregate.md 的
                         "all attempts" 中,绝不用于改动主结果。

性能平均只取主队列中 gate_met 的成员,这样单元可以诚实地报告"3 个控制器结果,
0 个 gate_met -> 无可评估性能",而不是被反复重试到好看为止。

门限常数与门限函数从 run_mpc_balance_trials.py 导入,因此本文件不会偏离实际运行的内容。

用法:
    python3 aggregate_mpc_r_sweep.py --base /tmp/mpc_r_sweep_full
"""

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from run_mpc_balance_trials import (  # noqa: E402
    STABLE_HOLD_S,
    engaged_window,
    stability_gate,
)

VALID_TAKEOVER_S = 0.20
RAMP_SETTLE_MARGIN_S = 0.5
RESPONSE_WINDOW_S = 3.0
SETTLE_DELAY_S = 3.0
PUSH_RECOVERY_DELAY_S = 5.0
PERIOD_US = 5000.0
# 每个试验在汇总表中取一个主窗口;下方的逐窗口明细表保留全部窗口。
HEADLINE_WINDOW = {"static": "steady", "position": "settled", "speed": "cruise",
                   "push": "response"}
FLOAT_COLUMNS = ("time", "height", "x_ref", "x_error", "x_dot", "v_ref", "pitch",
                 "pitch_rate", "theta_error", "u_mpc", "tau_each",
                 "total_torque_saturated", "wheel_torque_saturated",
                 "theta_band_binding", "solver_time_us", "solver_iterations",
                 "working_set_size", "control_enabled")


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        rows = [row for row in reader
                if row.get("stage") not in (None, "")
                and all(row.get(key) not in (None, "") for key in columns)]
    for row in rows:
        for key in FLOAT_COLUMNS:
            if key in row:
                try:
                    row[key] = float(row[key])
                except ValueError:
                    row[key] = float("nan")
    return rows


def col(rows, key):
    return np.array([row[key] for row in rows], dtype=float)


def ramp_end_time(rows):
    heights = col(rows, "height")
    if heights.size == 0:
        return 0.0
    below = np.flatnonzero(heights < heights.max() - 1.0e-6)
    if below.size == 0:
        return 0.0
    return float(col(rows, "time")[below][-1]) + RAMP_SETTLE_MARGIN_S


def step_time(rows):
    reference = None
    for row in rows:
        value = row["x_ref"]
        if reference is None:
            reference = value
        elif abs(value - reference) > 1.0e-3:
            return float(row["time"])
    return None


def cruise_window_rows(rows):
    v_ref = col(rows, "v_ref")
    commanded = float(np.max(np.abs(v_ref))) if v_ref.size else 0.0
    if commanded <= 1.0e-3:
        return None
    active = np.abs(v_ref) >= 0.9 * commanded
    if not active.any():
        return None
    times = col(rows, "time")
    return float(times[active.argmax()]), float(times[np.flatnonzero(active)[-1]])


def classify(rows, label):
    """一次重复 -> CATEGORIES 之一,并附带证据。"""
    engaged = engaged_window(rows)
    controlled = [row for row in engaged if row.get("control_enabled") == 1.0]
    if not controlled:
        return {"label": label, "category": "protocol_invalid", "rows": [],
                "reason": "no row with control_enabled=1", "control_rows": 0}
    takeover = float(controlled[0]["time"])
    gate_time = stability_gate(engaged, STABLE_HOLD_S)
    trip = next((row for row in engaged if row["stage"] == "safety_disabled"), None)
    base = {"label": label, "control_rows": len(controlled), "takeover_s": round(takeover, 4),
            "gate_time_s": round(float(gate_time), 3) if gate_time is not None else None,
            "trip_time_s": round(float(trip["time"]), 3) if trip is not None else None}
    if takeover > VALID_TAKEOVER_S:
        return {**base, "category": "protocol_invalid", "rows": [],
                "reason": f"takeover at {takeover:.3f} s > {VALID_TAKEOVER_S} s"}
    if trip is not None:
        return {**base, "category": "fell", "rows": engaged,
                "reason": "safety gate tripped"}
    if gate_time is None:
        return {**base, "category": "engaged_failed_gate", "rows": engaged,
                "reason": f"{STABLE_HOLD_S:.1f} s stability gate never satisfied"}
    return {**base, "category": "gate_met", "rows": engaged, "reason": ""}


def window_metrics(rows, start, end):
    inside = [row for row in rows if start <= row["time"] <= end]
    if len(inside) < 20:
        return None
    theta = col(inside, "theta_error")
    x_error = col(inside, "x_error")
    u = col(inside, "u_mpc")
    speed_error = col(inside, "x_dot") - col(inside, "v_ref")
    return {
        "samples": len(inside),
        "window_s": [round(start, 3), round(end, 3)],
        "theta_rms_deg": round(float(np.degrees(np.sqrt(np.mean(theta ** 2)))), 4),
        "theta_max_deg": round(float(np.degrees(np.max(np.abs(theta)))), 4),
        "x_rms_m": round(float(np.sqrt(np.mean(x_error ** 2))), 5),
        "x_mean_m": round(float(np.mean(x_error)), 5),
        "x_absmax_m": round(float(np.max(np.abs(x_error))), 5),
        "speed_error_rms_mps": round(float(np.sqrt(np.mean(speed_error ** 2))), 5),
        "u_rms_Nm": round(float(np.sqrt(np.mean(u ** 2))), 4),
        "u_absmax_Nm": round(float(np.max(np.abs(u))), 4),
        "sat_frac": round(float(np.mean(col(inside, "total_torque_saturated"))), 4),
        "wheel_sat_frac": round(float(np.mean(col(inside, "wheel_torque_saturated"))), 4),
        "band_bind_frac": round(float(np.mean(col(inside, "theta_band_binding"))), 4),
        "fallback_rows": int(sum(1 for row in inside if row["stage"] != "solved")),
    }


def trial_windows(trial, entries):
    """entries: gate_met 的重复。返回 {窗口名: [逐次重复指标]}。"""
    windows = {}
    for entry in entries:
        rows, end = entry["rows"], entry["rows"][-1]["time"]
        ramp_end = ramp_end_time(rows)
        if trial == "static":
            windows.setdefault("steady", []).append(
                window_metrics(rows, max(ramp_end, 0.0), end))
        elif trial == "position":
            step = step_time(rows)
            if step is None:
                continue
            windows.setdefault("baseline", []).append(window_metrics(rows, ramp_end, step))
            windows.setdefault("response", []).append(
                window_metrics(rows, step, step + RESPONSE_WINDOW_S))
            windows.setdefault("settled", []).append(
                window_metrics(rows, step + SETTLE_DELAY_S, end))
        elif trial == "speed":
            cruise = cruise_window_rows(rows)
            if cruise is None:
                continue
            windows.setdefault("cruise", []).append(window_metrics(rows, cruise[0], cruise[1]))
        elif trial == "push":
            # 只在腿部伸展完成后搜索扰动峰值:启动瞬态往往比推动本身更大,
            # 若让它占优,"恢复窗口"会落在斜坡上而不是脉冲上。
            theta = col(rows, "theta_error")
            times = col(rows, "time")
            late = np.flatnonzero(times >= ramp_end)
            if theta.size == 0 or late.size == 0:
                continue
            peak_index = int(np.argmax(np.abs(theta[late])))
            pulse = float(times[late[peak_index]])
            windows.setdefault("response", []).append(
                window_metrics(rows, pulse, pulse + RESPONSE_WINDOW_S))
            windows.setdefault("settled", []).append(
                window_metrics(rows, pulse + PUSH_RECOVERY_DELAY_S, end))
    return windows


def summarise_windows(windows):
    merged = {}
    for name, per_rep in windows.items():
        usable = [item for item in per_rep if item]
        if not usable:
            continue
        summary = {"reps": len(usable),
                   "samples_mean": round(statistics.fmean(item["samples"] for item in usable), 1)}
        for key, value in usable[0].items():
            if key in ("samples", "window_s"):
                continue
            numbers = [item[key] for item in usable if isinstance(item[key], (int, float))]
            if numbers:
                summary[key] = round(float(statistics.fmean(numbers)), 5)
                summary[key + "_reps"] = [round(float(number), 5) for number in numbers]
        merged[name] = summary
    return merged


def latency_stats(entries):
    rows = [row for entry in entries for row in entry["rows"]]
    if not rows:
        return None
    times = col(rows, "solver_time_us")
    times = times[np.isfinite(times)]
    working = col(rows, "working_set_size")
    iterations = col(rows, "solver_iterations")
    return {
        "samples": int(times.size),
        "mean_us": round(float(np.mean(times)), 1),
        "p50_us": round(float(np.percentile(times, 50)), 1),
        "p95_us": round(float(np.percentile(times, 95)), 1),
        "p99_us": round(float(np.percentile(times, 99)), 1),
        "max_us": round(float(np.max(times)), 1),
        "over_period_rows": int(np.sum(times > PERIOD_US)),
        "iterations_mean": round(float(np.mean(iterations)), 2),
        "iterations_max": int(np.max(iterations)),
        "working_set_mean": round(float(np.mean(working)), 2),
        "working_set_p95": round(float(np.percentile(working, 95)), 2),
        "working_set_max": int(np.max(working)),
    }


def startup_peaks(reps):
    """每个 gate_met 重复在腿部伸展结束前的 |theta_error| 峰值。"""
    peaks = []
    for entry in reps:
        early = [row for row in entry["rows"] if row["time"] <= ramp_end_time(entry["rows"])]
        if early:
            peaks.append(round(float(np.degrees(np.max(np.abs(col(early, "theta_error"))))), 3))
    return peaks


def cohort(entries, repeats):
    """把已分类的尝试切分为主队列和诊断队列。

    protocol_invalid 不占用名额(实验没有运行,后续重试可以取代它)。任何控制器
    结果都占用名额,无论是摔倒、从未稳定还是 gate_met。目标是
    n_controller_results = repeats,而不是 n_gate_met = repeats——一直重试直到凑满
    三个站立样本,会把几乎不可用的 R 变成漂亮的成绩表。
    """
    invalid = []
    results = []
    for entry in entries:
        if entry["category"] == "protocol_invalid":
            entry["cohort"] = "protocol_invalid"
            invalid.append(entry)
            continue
        if len(results) < repeats:
            entry["cohort"] = "primary"
            results.append(entry)
        else:
            entry["cohort"] = "retry_diagnostic"
    return results, invalid


def tally(entries):
    return {
        "n_gate_met": sum(1 for entry in entries if entry["category"] == "gate_met"),
        "n_fell": sum(1 for entry in entries if entry["category"] == "fell"),
        "n_failed_gate": sum(1 for entry in entries
                             if entry["category"] == "engaged_failed_gate"),
    }


def block(trial, entries):
    sample = [entry for entry in entries if entry["category"] == "gate_met"]
    return {
        "n_controller_results": len(entries),
        **tally(entries),
        "performance_reps": len(sample),
        "startup_theta_peak_deg": startup_peaks(sample),
        "windows": summarise_windows(trial_windows(trial, sample)),
        "solver_latency": latency_stats(sample),
        "labels": [entry["label"] for entry in entries],
    }


def analyse_trial(trial, classified, repeats):
    primary, invalid = cohort(classified, repeats)
    retries = [entry for entry in classified if entry["cohort"] == "retry_diagnostic"]
    main = block(trial, primary)
    ledger_keys = ("label", "category", "reason", "cohort", "takeover_s", "gate_time_s",
                   "trip_time_s", "control_rows")
    return {
        "n_attempts": len(classified),
        "n_protocol_invalid": len(invalid),
        "n_retries_beyond_primary": len(retries),
        **main,
        "success_fraction": f"{main['n_gate_met']}/{main['n_controller_results']}",
        "evaluable_performance": main["performance_reps"] > 0,
        "all_attempts": block(trial, primary + retries),
        "ledger": [{key: entry.get(key) for key in ledger_keys} for entry in classified],
    }


def collect(base, repeats, trials):
    results = {}
    for path in sorted(base.iterdir(), key=lambda item: item.name):
        if not path.is_dir() or not path.name.startswith("R"):
            continue
        try:
            r_value = path.name[1:]
            float(r_value)
        except ValueError:
            continue
        per_trial = {}
        for trial in trials:
            sources = [path] + sorted(item for item in path.glob("retry*") if item.is_dir())
            classified = []
            for source in sources:
                for csv_path in sorted(source.glob(f"{trial}_rep*.csv")):
                    label = f"{source.relative_to(base)}/{csv_path.name}"
                    classified.append(classify(load_rows(csv_path), label))
            if classified:
                per_trial[trial] = analyse_trial(trial, classified, repeats)
        results[r_value] = per_trial
    return results


def write_markdown(path, results, repeats):
    trials = [name for name in ("static", "position", "speed", "push", "saturation")
              if any(name in cell for cell in results.values())]
    lines = ["# R sweep - primary cohort (n=3 controller results) with diagnostics", "",
             "## Cohort rule", "",
             "Attempts are read in chronological order (main cell, then retry1, retry2).", "",
             "```",
             "protocol_invalid      -> dropped, does not consume a slot (the experiment",
             "                        never ran: no takeover, or takeover > "
             f"{VALID_TAKEOVER_S} s)",
             "engaged_failed_gate   -> consumes a slot, counted as a controller failure",
             "fell                  -> consumes a slot, counted as a controller failure",
             "gate_met              -> consumes a slot, the only class that feeds the",
             "                        performance averages",
             "take the first " + str(repeats) + " slot-consuming attempts = primary cohort; anything",
             "after that is kept as retry_diagnostic and never moves the primary result",
             "```", "",
             "Target is n_controller_results = " + str(repeats) + ", NOT n_gate_met = " + str(repeats)
             + ": retrying until",
             "three standing samples are found would let a nearly unusable R show a clean",
             "performance table. A cell can therefore legitimately report",
             "`performance_reps = 0` -> no evaluable performance.", "",
             "Counts use unambiguous names: `n_attempts`, `n_protocol_invalid`,",
             "`n_controller_results`, `n_gate_met`, `n_fell`, `n_failed_gate`.",
             "`success` below means n_gate_met (the loop engaged in time, held the "
             f"{STABLE_HOLD_S} s",
             "stability gate and never tripped the 0.50 rad gate). No metric is called "
             "'survived'.", ""]

    for label, key in (("primary cohort", None), ("all attempts (diagnostic)", "all_attempts")):
        lines += [f"## {label} - headline table", "",
                  "| R | trial | success | fell | failed gate | n_ctrl | perf reps | "
                  "window | th_rms(deg) | u_rms(Nm) | sat |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r_value, cell in results.items():
            for trial in trials:
                entry = cell.get(trial)
                if entry is None:
                    continue
                source = entry if key is None else entry[key]
                name = HEADLINE_WINDOW.get(trial)
                window = (source.get("windows") or {}).get(name) if name else None
                dash = "no evaluable performance" if source["performance_reps"] == 0 else "-"
                lines.append(
                    f"| {r_value} | {trial} | {source['n_gate_met']}/"
                    f"{source['n_controller_results']} | {source['n_fell']} | "
                    f"{source['n_failed_gate']} | {source['n_controller_results']} | "
                    f"{source['performance_reps']} | {name or '-'} | "
                    f"{window['theta_rms_deg'] if window else dash} | "
                    f"{window['u_rms_Nm'] if window else '-'} | "
                    f"{window['sat_frac'] if window else '-'} |")
        lines.append("")

    lines += ["## Attempts and retries", "",
              "| R | trial | n_attempts | n_protocol_invalid | n_retries_beyond_primary |",
              "|---|---|---|---|---|"]
    for r_value, cell in results.items():
        for trial, entry in sorted(cell.items()):
            lines.append(f"| {r_value} | {trial} | {entry['n_attempts']} | "
                         f"{entry['n_protocol_invalid']} | "
                         f"{entry['n_retries_beyond_primary']} |")
    lines.append("")

    for trial in trials:
        lines += [f"## {trial} - every window (primary cohort)", "",
                  "| R | window | perf reps | th_rms(deg) | th_max(deg) | x_rms(m) | "
                  "x_mean(m) | x_max(m) | v_err(rms m/s) | u_rms(Nm) | u_max(Nm) | sat | "
                  "wheel_sat | band | fallback |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r_value, cell in results.items():
            entry = cell.get(trial)
            if entry is None:
                continue
            for name, window in entry["windows"].items():
                lines.append(
                    f"| {r_value} | {name} | {window['reps']} | {window['theta_rms_deg']} | "
                    f"{window['theta_max_deg']} | {window['x_rms_m']} | {window['x_mean_m']} | "
                    f"{window['x_absmax_m']} | {window['speed_error_rms_mps']} | "
                    f"{window['u_rms_Nm']} | {window['u_absmax_Nm']} | {window['sat_frac']} | "
                    f"{window['wheel_sat_frac']} | {window['band_bind_frac']} | "
                    f"{window['fallback_rows']} |")
            for name, window in sorted((entry["all_attempts"]["windows"]).items()):
                if name in entry["windows"]:
                    continue
                lines.append(
                    f"| {r_value} | {name} (retry only) | {window['reps']} | "
                    f"{window['theta_rms_deg']} | {window['theta_max_deg']} | "
                    f"{window['x_rms_m']} | {window['x_mean_m']} | {window['x_absmax_m']} | "
                    f"{window['speed_error_rms_mps']} | {window['u_rms_Nm']} | "
                    f"{window['u_absmax_Nm']} | {window['sat_frac']} | "
                    f"{window['wheel_sat_frac']} | {window['band_bind_frac']} | "
                    f"{window['fallback_rows']} |")
        lines.append("")

    lines += ["## Solver latency and active set (primary cohort, gate_met reps only)", "",
              "| R | trial | samples | mean(us) | p50 | p95 | p99 | max | >5ms rows | "
              "it mean/max | ws mean/p95/max |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r_value, cell in results.items():
        for trial, entry in sorted(cell.items()):
            for name, latency in (("primary", entry.get("solver_latency")),
                                  ("all attempts", entry["all_attempts"].get("solver_latency"))):
                if not latency or name == "all attempts" and latency == entry.get("solver_latency"):
                    continue
                lines.append(f"| {r_value} | {trial} ({name}) | {latency['samples']} | "
                             f"{latency['mean_us']} | {latency['p50_us']} | "
                             f"{latency['p95_us']} | {latency['p99_us']} | {latency['max_us']} | "
                             f"{latency['over_period_rows']} | "
                             f"{latency['iterations_mean']}/{latency['iterations_max']} | "
                             f"{latency['working_set_mean']}/{latency['working_set_p95']}/"
                             f"{latency['working_set_max']} |")
    lines.append("")
    lines += ["## Per-attempt ledger (nothing hidden)", "",
              "| R | trial | cohort | category | reason | takeover(s) | gate(s) | trip(s) | "
              "control rows | file |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r_value, cell in results.items():
        for trial, entry in sorted(cell.items()):
            for item in entry["ledger"]:
                lines.append(f"| {r_value} | {trial} | {item['cohort']} | "
                             f"{item['category']} | {item.get('reason') or ''} | "
                             f"{item.get('takeover_s')} | {item.get('gate_time_s')} | "
                             f"{item.get('trip_time_s')} | {item.get('control_rows')} | "
                             f"{item['label']} |")
    lines.append("")
    path.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="/tmp/mpc_r_sweep_full")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--trials", default="static,position,speed,push,saturation")
    args = parser.parse_args()

    base = Path(args.base)
    trials = tuple(item for item in args.trials.split(",") if item)
    results = collect(base, args.repeats, trials)
    (base / "aggregate.json").write_text(json.dumps(results, ensure_ascii=False, indent=2))
    write_markdown(base / "aggregate.md", results, args.repeats)
    print(f"{base / 'aggregate.md'}\n{base / 'aggregate.json'}")
    for r_value, cell in results.items():
        for trial, entry in sorted(cell.items()):
            print(f"R={r_value:<5} {trial:<11} attempts={entry['n_attempts']} "
                  f"protocol_invalid={entry['n_protocol_invalid']} retries_beyond="
                  f"{entry['n_retries_beyond_primary']} | primary n_ctrl="
                  f"{entry['n_controller_results']} gate_met={entry['n_gate_met']} "
                  f"fell={entry['n_fell']} failed_gate={entry['n_failed_gate']} "
                  f"perf_reps={entry['performance_reps']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
