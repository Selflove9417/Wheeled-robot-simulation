#!/usr/bin/env python3
"""冻结一次 MPC 实验批次的审计快照。

写 <base>/config_snapshot.md（及 .json），记录复现或事后审计扫描所需的全部信息：
当时生效的 R 值与批次常量、节点实际使用的参数（YAML 默认值加上试验脚本
命令行传入的覆盖项）、分析窗口常量、所有相关源文件的 sha256、git 状态与
构建类型，以及运行期间刻意未冻结/未修复的已知测试框架缺陷。

只读：从不改动控制器、launch 文件、原始 CSV 或聚合规则。可在批次进行中运行，
并在每次修复后再运行一次，两份快照即可精确显示改了什么、何时改的。

用法：
    python3 snapshot_mpc_experiment_config.py --base /tmp/mpc_r_sweep_full
"""

import argparse
import hashlib
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parents[2]

# 定义实验内容的文件。只记录哈希而不复制，快照保持很小，
# 且之后任何改动都能被检测到。
SOURCES = (
    "src/bbot_balance_controller/src/linear_mpc_balance_controller.cpp",
    "src/bbot_balance_controller/include/bbot_balance_controller/linear_mpc.hpp",
    "src/bbot_balance_controller/include/bbot_balance_controller/dense_active_set_qp.hpp",
    "src/bbot_balance_controller/include/bbot_balance_controller/lqr_plant_model.hpp",
    "src/bbot_balance_controller/config/mpc_balance_params.yaml",
    "src/bbot_balance_controller/scripts/run_mpc_balance_trials.py",
    "src/bbot_balance_controller/scripts/run_mpc_r_sweep.py",
    "src/bbot_balance_controller/scripts/aggregate_mpc_r_sweep.py",
    "src/bbot_bringup/launch/bbot_gazebo.launch.py",
    "src/bbot_bringup/launch/gs_lqr_ready_unpause.py",
    "src/bbot_kinematics/src/kinematics.cpp",
    "src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp",
)

# 试验脚本在命令行上固定的值，即覆盖或补充 YAML 的项。
# 保留为字面列表，避免快照与 launch_command() 静默不一致 ——
# 下面会与脚本文本交叉校验。
PINNED_BY_RUNNER = (
    "gazebo_start_paused:=true", "auto_unpause:=true", "headless:=true", "gui:=false",
    "controller_type:=mpc", "torque_pid_initial_roll:=0.0",
    "mpc_theta_eq_source:=<args.theta_eq_source, default table>",
    "mpc_target_height:=<args.height, default 0.40>",
    "mpc_spawn_z:=<startup_hip_axle + 0.14>",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_state(ws_root):
    def run(*arguments):
        result = subprocess.run(["git", *arguments], cwd=ws_root, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        return result.stdout.strip()
    return {
        "head": run("rev-parse", "HEAD"),
        "branch": run("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": run("status", "--short", "--", "src/bbot_balance_controller",
                     "src/bbot_bringup", "src/bbot_kinematics").splitlines(),
    }


def build_type(ws_root):
    cache = Path(ws_root) / "build" / "bbot_balance_controller" / "CMakeCache.txt"
    if not cache.exists():
        return {"cmake_build_type": "(no CMakeCache.txt)", "cache_path": str(cache)}
    match = re.search(r"^CMAKE_BUILD_TYPE:STRING=(.*)$", cache.read_text(), re.MULTILINE)
    value = match.group(1).strip() if match else "(unset)"
    return {
        "cmake_build_type": value or "(empty -> unoptimised, no -O flags)",
        "cache_path": str(cache),
        "note": ("empty CMAKE_BUILD_TYPE is the repository default: solver times in this "
                 "batch are unoptimised-build numbers"),
    }


def analysis_constants():
    """直接导入实际使用的常量，保证快照不会与之不一致。"""
    sys.path.insert(0, str(SCRIPT_DIR))
    import aggregate_mpc_r_sweep as agg  # noqa: E402
    import run_mpc_balance_trials as trials  # noqa: E402
    return {
        "VALID_TAKEOVER_S": agg.VALID_TAKEOVER_S,
        "RAMP_SETTLE_MARGIN_S": agg.RAMP_SETTLE_MARGIN_S,
        "RESPONSE_WINDOW_S": agg.RESPONSE_WINDOW_S,
        "SETTLE_DELAY_S": agg.SETTLE_DELAY_S,
        "PUSH_RECOVERY_DELAY_S": agg.PUSH_RECOVERY_DELAY_S,
        "PERIOD_US": agg.PERIOD_US,
        "HEADLINE_WINDOW": agg.HEADLINE_WINDOW,
        "stability_gate": {
            "STABLE_ANGLE_RAD": trials.STABLE_ANGLE_RAD,
            "STABLE_RATE_RAD_S": trials.STABLE_RATE_RAD_S,
            "STABLE_VELOCITY_M_S": trials.STABLE_VELOCITY_M_S,
            "STABLE_HOLD_S": trials.STABLE_HOLD_S,
        },
        "cohort_rule": ("first --repeats attempts that are controller results "
                        "(fell / engaged_failed_gate / gate_met); protocol_invalid is "
                        "skipped and does not consume a slot; later attempts are "
                        "retry_diagnostic only"),
    }


def node_parameters(ws_root):
    path = Path(ws_root) / "src/bbot_balance_controller/config/mpc_balance_params.yaml"
    document = yaml.safe_load(path.read_text())
    return document["linear_mpc_balance_controller"]["ros__parameters"]


def runner_defaults():
    text = (SCRIPT_DIR / "run_mpc_balance_trials.py").read_text()
    found = {}
    for match in re.finditer(r'add_argument\("--([a-z0-9-]+)"(?:,\s*type=(\w+))?,\s*'
                             r'default=([^,\)]+)', text):
        found[match.group(1)] = match.group(3).strip()
    return found


def sweep_state(base):
    state = {"cells": {}, "harness_failures": [], "in_progress": []}
    for path in sorted(Path(base).iterdir()):
        if not path.is_dir() or not path.name.startswith("R"):
            continue
        for cell in [path] + sorted(item for item in path.glob("retry*") if item.is_dir()):
            relative = str(cell.relative_to(base))
            csvs = sorted(item.name for item in cell.glob("*_rep*.csv"))
            # 驱动脚本只在某次调用未生成 summary.json 就退出时写 driver_raw.log，
            # 所以用该文件（而非 summary 的缺失）来区分已死单元和仍在运行的单元。
            if (cell / "summary.json").exists():
                status = "complete"
            elif (cell / "driver_raw.log").exists():
                status = "harness_failure"
            else:
                status = "in_progress"
            state["cells"][relative] = {"csv_count": len(csvs), "status": status}
            if status == "harness_failure":
                state["harness_failures"].append(
                    {"cell": relative, "csv_count": len(csvs),
                     "issue": "invocation died before writing summary.json; see "
                              "driver_raw.log in that cell"})
            elif status == "in_progress":
                state["in_progress"].append(relative)
    return state


def write_markdown(path, snapshot):
    config = snapshot["node_parameters"]
    lines = ["# Experiment configuration snapshot", "",
             f"Generated {snapshot['generated']} · batch dir `{snapshot['base']}`", "",
             "## Sweep design", "",
             f"- R values: {snapshot['sweep']['r_values']}",
             f"- trials: {snapshot['sweep']['trials']}",
             f"- repeats target: {snapshot['sweep']['repeats']} controller results per cell",
             f"- cohort rule: {snapshot['analysis']['cohort_rule']}", "",
             "## Pinned node parameters (config/mpc_balance_params.yaml)", "",
             "```yaml", yaml.safe_dump(config, sort_keys=False).strip(), "```", "",
             "## Pinned on the command line by the trial runner", "",
             "```", *[f"{item}" for item in PINNED_BY_RUNNER], "```", "",
             f"runner argument defaults: `{json.dumps(snapshot['runner_defaults'], ensure_ascii=False)}`",
             "", "## Analysis constants (frozen aggregator)", "",
             "```json", json.dumps(snapshot["analysis"], indent=2, ensure_ascii=False), "```", "",
             "## Build", "",
             f"- CMAKE_BUILD_TYPE: `{snapshot['build']['cmake_build_type']}`",
             f"- {snapshot['build']['note']}", "",
             "## Source integrity (sha256)", "",
             "| file | sha256 |", "|---|---|"]
    for item in snapshot["sources"]:
        lines.append(f"| {item['path']} | `{item['sha256'][:16]}…` |")
    lines += ["", "## Git state", "",
              f"- HEAD: `{snapshot['git']['head']}` on `{snapshot['git']['branch']}`",
              f"- modified/new under the three packages: {len(snapshot['git']['dirty'])} entries",
              "", "```", *snapshot["git"]["dirty"], "```", "",
              "## Batch state at snapshot time", "",
              "| cell | CSVs | status |", "|---|---|---|"]
    for cell, info in snapshot["sweep_state"]["cells"].items():
        lines.append(f"| {cell} | {info['csv_count']} | {info['status']} |")
    lines += ["",
              f"- in progress: {snapshot['sweep_state']['in_progress'] or 'none'}",
              "- harness failures: "
              f"{[item['cell'] for item in snapshot['sweep_state']['harness_failures']] or 'none'}",
              "",
              "## Known harness defects (left unfixed during the batch on purpose)", ""]
    for defect in snapshot["known_defects"]:
        lines.append(f"- **{defect['id']}** {defect['text']}")
    lines.append("")
    path.write_text("\n".join(lines))


KNOWN_DEFECTS = (
    {"id": "push-empty-response-window",
     "text": "run_mpc_balance_trials.py:343-350 takes np.max() over the post-pulse response "
             "window without guarding the empty case. When the stability gate is slow to be "
             "met (low R) the pulse lands with less than RESPONSE_WINDOW_S of log left, the "
             "window is empty and the whole invocation dies with rc=1. Observed: R1/retry2 "
             "died at push_rep2. Effect on the primary cohort: none, because the cohort is "
             "rebuilt from the raw CSVs and that cell was a retry. Not patched during the "
             "batch to avoid mixing script versions."},
    {"id": "driver-retry-label-mismatch",
     "text": "run_mpc_r_sweep.py counts completed trials by run['trial'], but "
             "run_mpc_balance_trials.py writes that key as '<trial>_rep<k>' when repeats>1, "
             "so the driver never sees a filled cell and re-runs all four trials three times "
             "per R value (9 attempts per cell instead of ~3). Effect: wall time roughly 3x. "
             "Effect on the primary cohort: none, because the cohort takes the first three "
             "controller results in chronological order and the extra attempts are "
             "retry_diagnostic. Left as is during the batch."},
)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="/tmp/mpc_r_sweep_full")
    parser.add_argument("--r-values", default="0.5,1,2,4,6,8,12")
    parser.add_argument("--trials", default="static,position,speed,push")
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()

    base = Path(args.base)
    snapshot = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "base": str(base),
        "sweep": {"r_values": [float(item) for item in args.r_values.split(",")],
                  "trials": args.trials.split(","), "repeats": args.repeats},
        "node_parameters": node_parameters(WS_ROOT),
        "runner_defaults": runner_defaults(),
        "analysis": analysis_constants(),
        "build": build_type(WS_ROOT),
        "sources": [{"path": item, "sha256": sha256(WS_ROOT / item)} for item in SOURCES
                    if (WS_ROOT / item).exists()],
        "git": git_state(WS_ROOT),
        "sweep_state": sweep_state(base),
        "known_defects": list(KNOWN_DEFECTS),
    }
    (base / "config_snapshot.json").write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))
    write_markdown(base / "config_snapshot.md", snapshot)
    print(f"{base / 'config_snapshot.md'}\n{base / 'config_snapshot.json'}")
    print(f"sources hashed: {len(snapshot['sources'])}, git HEAD "
          f"{snapshot['git']['head'][:10]}, build type "
          f"{snapshot['build']['cmake_build_type']}")
    print(f"cells: {len(snapshot['sweep_state']['cells'])} "
          f"(complete={sum(1 for item in snapshot['sweep_state']['cells'].values() if item['status'] == 'complete')}, "
          f"in_progress={len(snapshot['sweep_state']['in_progress'])}, "
          f"harness_failure={len(snapshot['sweep_state']['harness_failures'])})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
