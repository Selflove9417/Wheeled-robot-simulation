#!/usr/bin/env python3
"""固定配置下第一阶段 Linear MPC 的单变量 R 扫描。

只改 mpc.r。其余全部固定为仓库中的 config/mpc_balance_params.yaml,加上两个明确的
对齐选择(theta_eq_source=table、torque_pid_initial_roll=0.0),因此扫描结果可与
adaptive_lqr 对齐的参考运行直接比较。

只有协议有效的重复才计入:试验必须在仿真时钟启动后 VALID_TAKEOVER_S 内开始计算。
2026-09-28 实测:由于 launch 固定 2.8 s 自动解除暂停,/imu 桥接有时在 2.6-10 s 才
就绪,这些运行在第一个控制周期之前就已摔倒,但仍生成了完整 CSV。接管缺失/迟到属于
协议错误,因此重试;控制器摔倒则保留为该次重复的结果。

用法(需在已 source 工作空间的 shell 中运行,虽然每次调用都会重新 source):
    python3 run_mpc_r_sweep.py --out-dir /tmp/mpc_r_sweep
    python3 run_mpc_r_sweep.py --r-values 2,4,6 --repeats 1 --trials static
"""

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
TRIAL_SCRIPT = SCRIPT_DIR / "run_mpc_balance_trials.py"
VALID_TAKEOVER_S = 0.20

# 对一个 (R, trial) 单元的有效重复求均值的指标。
METRICS = (
    "theta_error_rms_deg", "theta_error_max_deg",
    "x_error_rms_m", "x_error_absmax_m", "x_error_final_mean_m",
    "velocity_error_rms_mps", "u_rms_Nm", "u_absmax_Nm",
    "saturated_fraction", "fallback_events",
    "solver_mean_us", "solver_max_us", "solver_iterations_max",
)


def sourced(command, ws_root):
    return (f"source /opt/ros/iron/setup.bash && source {ws_root}/install/setup.bash "
            f"&& {' '.join(command)}")


def run_cell(args, r_value, out_dir, trials, repeats, label):
    command = [
        sys.executable, str(TRIAL_SCRIPT),
        "--ws-root", args.ws_root,
        "--r", f"{r_value:g}",
        "--trials", trials,
        "--repeats", str(repeats),
        "--out-dir", str(out_dir),
    ]
    if args.duration is not None:
        command += ["--duration", f"{args.duration:g}"]
    started = time.monotonic()
    result = subprocess.run(["bash", "-lc", sourced(command, args.ws_root)],
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            timeout=args.cell_timeout)
    summary_path = out_dir / "summary.json"
    if not summary_path.exists():
        (out_dir / "driver_raw.log").write_text(result.stdout or "")
        # 该单元承诺的结果全部下落不明,每个试验都缺一个结果;由调用方决定是否重试。
        return [], [], (f"{label}: no summary.json (rc={result.returncode}, "
                        f"{time.monotonic() - started:.0f}s)")
    runs = json.loads(summary_path.read_text())["results"]
    for run in runs:
        run["r"] = r_value
        run["source"] = label
    controller_results, protocol_invalid, schema_note = partition_runs(runs)
    return controller_results, protocol_invalid, (
        f"{label}: {len(controller_results)} controller results / "
        f"{len(runs)} launched in {time.monotonic() - started:.0f}s; protocol_invalid="
        + json.dumps([{"trial": run.get("trial_name", "(missing)"), "fail": run.get("fail"),
                       "first_ctrl": (run.get("notes") or {}).get("first_controlled_time_s")}
                      for run in protocol_invalid], ensure_ascii=False)
        + (f"; {schema_note}" if schema_note else ""))


def trial_name_of(run):
    """run_mpc_balance_trials.py 写入的权威基础试验名。

    刻意不从 run["trial"] 推导:repeats > 1 时它是显示标签 "<name>_rep<k>",
    repeats == 1 时是裸名称——正是解析它导致每次重试都像空单元,把四个试验全部重跑。
    缺失该字段说明 schema 出错,因此直接抛异常而不是猜测。
    """
    name = run.get("trial_name")
    if not isinstance(name, str) or not name:
        raise KeyError(f"summary entry has no usable 'trial_name': "
                       f"{json.dumps({k: run.get(k) for k in ('trial', 'repetition')})}")
    return name


def is_controller_result(run):
    """试验确实产生了控制器结果时为 True。

    涵盖 gate_met、摔倒以及"已接管但从未稳定":达到平衡门限本身就是被测对象,
    没达到也是结果,不是实验损坏。只有接管缺失或迟到(无反馈、桥接失效、发布
    丢失、框架异常)才算协议失败。
    """
    if run.get("harness_exception"):
        return False
    takeover = (run.get("notes") or {}).get("first_controlled_time_s")
    if takeover is None:
        return False
    try:
        return float(takeover) <= VALID_TAKEOVER_S
    except (TypeError, ValueError):
        return False


def classify_runs(runs):
    controller_results, protocol_invalid = [], []
    for run in runs:
        trial_name_of(run)
        (controller_results if is_controller_result(run) else protocol_invalid).append(run)
    return controller_results, protocol_invalid


def partition_runs(runs):
    """不会让整个批次中断的 classify_runs()。

    缺少 trial_name 的 summary 说明两个脚本之间的 schema 不匹配。驱动会报告该错误
    并把整个单元视为 protocol_invalid,使扫描其余部分继续运行,而不是在第一条坏
    记录上崩溃。
    """
    try:
        controller_results, protocol_invalid = classify_runs(runs)
        return controller_results, protocol_invalid, ""
    except KeyError as error:
        return [], list(runs), f"summary schema error: {error}"


def slot_counts(controller_results, trials):
    counts = {trial: 0 for trial in trials}
    for run in controller_results:
        name = trial_name_of(run)
        if name in counts:
            counts[name] += 1
    return counts


def missing_slots(counts, trials, repeats):
    return {trial: repeats - counts.get(trial, 0) for trial in trials
            if counts.get(trial, 0) < repeats}


def aggregate(cells):
    """仅驱动侧的执行记录,以权威 trial_name 为键。

    冻结的分析在 aggregate_mpc_r_sweep.py 中,它从原始 CSV 重建主队列;此表只是让
    批次无需重新解析 600 MB 日志即可读取。
    """
    by_trial = {}
    for run in cells:
        by_trial.setdefault(trial_name_of(run), []).append(run)
    table = {}
    for trial, runs in by_trial.items():
        row = {"n_controller_results": len(runs),
               "falls": sum(1 for run in runs if run.get("fell")),
               "failed_gate": sum(1 for run in runs if run.get("fail")),
               "sources": sorted({run["source"] for run in runs})}
        for metric in METRICS:
            values = [run[metric] for run in runs if metric in run]
            if not values:
                continue
            row[metric] = round(statistics.fmean(values), 4)
            row[metric + "_min"] = round(min(values), 4)
            row[metric + "_max"] = round(max(values), 4)
        table[trial] = row
    return table


def write_markdown(path, sweep):
    lines = ["# Linear MPC phase-1 R sweep (all other parameters pinned)", "",
             f"Generated {time.strftime('%Y-%m-%d %H:%M:%S')} · "
             f"H_hip={sweep['config']['height_m']} m · theta_eq_source="
             f"{sweep['config']['theta_eq_source']} · initial_roll="
             f"{sweep['config']['initial_roll_rad']} rad · Q={sweep['config']['q']} · "
             f"N={sweep['config']['horizon']} · repeats target="
             f"{sweep['config']['repeats']}", "",
             "Driver execution record only. Each row counts controller results "
             "(gate_met, fell, or engaged-but-never-stabilised); protocol failures are "
             "retried and not counted. Final analysis comes from "
             "aggregate_mpc_r_sweep.py over the raw CSVs.",
             ""]
    for trial in sweep["trials"]:
        lines += [f"## {trial}", "",
                  "| R | ctrl results | falls | failed gate | th_rms(deg) | th_max(deg) | "
                  "x_rms(m) | x_final(m) | v_err(rms m/s) | u_rms(Nm) | u_max(Nm) | sat | "
                  "fallback | solve mean/max(us) |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r_value in sweep["r_values"]:
            row = sweep["results"].get(str(r_value), {}).get(trial)
            if row is None:
                continue
            lines.append(
                f"| {r_value:g} | {row['n_controller_results']} | {row['falls']} | "
                f"{row['failed_gate']} | "
                f"{row.get('theta_error_rms_deg')} | {row.get('theta_error_max_deg')} | "
                f"{row.get('x_error_rms_m')} | {row.get('x_error_final_mean_m')} | "
                f"{row.get('velocity_error_rms_mps')} | {row.get('u_rms_Nm')} | "
                f"{row.get('u_absmax_Nm')} | {row.get('saturated_fraction')} | "
                f"{row.get('fallback_events')} | "
                f"{row.get('solver_mean_us')}/{row.get('solver_max_us')} |")
        lines.append("")
    path.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ws-root", default="/home/admin/bbot_ws_new")
    parser.add_argument("--r-values", default="0.5,1,2,4,6,8,12")
    parser.add_argument("--trials", default="static,position,speed,push")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--max-retries", type=int, default=2)
    parser.add_argument("--duration", type=float, default=None,
                        help="override the trial duration (only for smoke tests)")
    parser.add_argument("--cell-timeout", type=float, default=7200.0,
                        help="wall seconds allowed for one R value's full cell set")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    r_values = [float(item) for item in args.r_values.split(",") if item]
    trials = [item for item in args.trials.split(",") if item]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = Path(args.out_dir or f"/tmp/mpc_r_sweep/{stamp}")
    base.mkdir(parents=True, exist_ok=True)
    log = (base / "driver.log").open("w", buffering=1)

    def note(message):
        print(message, flush=True)
        log.write(message + "\n")

    sweep = {"config": {"height_m": 0.40, "theta_eq_source": "table",
                        "initial_roll_rad": 0.0, "horizon": 20,
                        "q": [100.0, 5000.0, 3000.0, 1200.0],
                        "total_torque_max_Nm": 20.0, "wheel_torque_max_Nm": 10.0,
                        "startup_hip_axle_m": 0.36, "startup_hold_time_s": 2.0,
                        "repeats": args.repeats,
                        "valid_takeover_s": VALID_TAKEOVER_S,
                        "ws_root": args.ws_root, "stamp": stamp},
             "r_values": r_values, "trials": trials, "results": {}, "runs": []}
    (base / "sweep_summary.json").write_text(json.dumps(sweep, ensure_ascii=False, indent=2))

    for r_value in r_values:
        r_dir = base / f"R{r_value:g}"
        note(f"\n=== R={r_value:g} -> {r_dir}")
        collected, _, message = run_cell(args, r_value, r_dir, args.trials, args.repeats, "main")
        note(message)
        counts = slot_counts(collected, trials)
        retry = 0
        while retry < args.max_retries:
            deficit = missing_slots(counts, trials, args.repeats)
            if not deficit:
                break
            retry += 1
            # 每个缺额试验单独调用一次,且只补它自己的缺额,避免只差一个结果的
            # 试验被多跑几次。
            for trial in sorted(deficit):
                shortfall = deficit[trial]
                if counts.get(trial, 0) >= args.repeats:
                    continue
                retry_dir = r_dir / f"retry{retry}_{trial}"
                results, invalid, message = run_cell(
                    args, r_value, retry_dir, trial, shortfall, f"retry{retry}:{trial}")
                note(message)
                for run in results:
                    if counts.get(trial, 0) >= args.repeats:
                        break
                    collected.append(run)
                    counts[trial] = counts.get(trial, 0) + 1
        still_missing = missing_slots(counts, trials, args.repeats)
        if still_missing:
            note(f"R={r_value:g} STILL SHORT of {args.repeats} controller results: "
                 f"{json.dumps(still_missing)}")
        sweep["runs"].extend(collected)
        sweep["results"][str(r_value)] = aggregate(collected)
        (base / "sweep_summary.json").write_text(json.dumps(sweep, ensure_ascii=False, indent=2))
        write_markdown(base / "sweep_table.md", sweep)
        for trial in trials:
            row = sweep["results"][str(r_value)].get(trial)
            if row:
                note(f"  R={r_value:g} {trial:<9} valid={row['reps_valid']} "
                     f"falls={row['falls']} th_rms={row.get('theta_error_rms_deg')} "
                     f"u_rms={row.get('u_rms_Nm')} sat={row.get('saturated_fraction')} "
                     f"x_final={row.get('x_error_final_mean_m')}")

    note(f"\naggregate: {base / 'sweep_table.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
