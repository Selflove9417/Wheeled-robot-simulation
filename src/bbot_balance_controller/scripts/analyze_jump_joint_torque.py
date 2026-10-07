#!/usr/bin/env python3
"""Export command-torque plots without treating controller commands as sensors."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_flight_momentum import momentum
from audit_ground_contact_frames import parse_frame, read_frame_rows
from audit_landing_geometry import parse_geometry_rows

COMMANDS = ("hip_cmd_left", "knee_cmd_left", "hip_cmd_right", "knee_cmd_right")
VELOCITIES = ("hip_vel_left", "knee_vel_left", "hip_vel_right", "knee_vel_right")
JOINTS = ("左髋", "左膝", "右髋", "右膝")


def number(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"nonfinite {key}")
    return value


def first(rows, predicate):
    return next((number(r, "timestamp") for r in rows if predicate(r)), None)


def series(rows, stamp_key, keys):
    """Keep one sample per actual source stamp; no invented interpolation."""
    seen = {}
    for row in rows:
        stamp = number(row, stamp_key)
        if stamp >= 0:
            seen.setdefault(stamp, [number(row, k) for k in keys])
    stamps = sorted(seen)
    return np.array(stamps), np.array([seen[t] for t in stamps])


def save_figure(fig, stem):
    for suffix in ("png", "pdf", "svg"):
        fig.savefig(stem.with_suffix("." + suffix), dpi=180, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--jump-id", default="1")
    parser.add_argument("--feedback-csv", type=Path,
                        help="Optional record_joint_torque.py joint-feedback CSV")
    args = parser.parse_args()
    output = args.output_dir or args.log.parent / "joint_torque_analysis"
    output.mkdir(parents=True, exist_ok=True)
    with args.log.open(newline="") as stream:
        all_rows = list(csv.DictReader(stream))
    rows = [r for r in all_rows if r.get("jump_id") == args.jump_id]
    if not rows:
        raise ValueError("no rows for requested jump")
    if any(number(b, "timestamp") <= number(a, "timestamp")
           for a, b in zip(rows, rows[1:])):
        raise ValueError("controller timestamps must increase")
    thrust = first(rows, lambda r: r["state_name"] == "THRUST")
    flight = first(rows, lambda r: r["state_name"] == "FLIGHT")
    if thrust is None or flight is None:
        raise ValueError("THRUST and FLIGHT required for takeoff analysis")
    effort_rows = [r for r in rows if r.get("effort_mode_active") == "1"]
    if not effort_rows:
        raise ValueError("no confirmed active effort mode")
    # controller_mode is an older controller-selection label, not the live
    # actuator interface. Command columns can be stale in position mode.
    t = np.array([number(r, "timestamp") for r in effort_rows])
    torque = np.array([[number(r, k) for k in COMMANDS] for r in effort_rows])
    tq, qdot = series(effort_rows, "joint_sample_stamp", VELOCITIES)
    prefix = args.log.name.removesuffix("_log.csv")
    geometry_path = args.log.with_name(prefix + "_ground_geometry.csv")
    contacts_path = args.log.with_name(prefix + "_ground_frames.csv")
    native_t = native_pitch = native_rate = None
    contact_events = {}
    source_notes = []
    if geometry_path.exists() and contacts_path.exists():
        geometry = parse_geometry_rows(geometry_path)
        geometry_by_stamp = {r["stamp"]: r for r in geometry}
        frames = []
        for raw in read_frame_rows(contacts_path):
            try:
                frames.append(parse_frame(raw))
            except ValueError as exc:
                stamp = int(raw["sim_time_ns"]) / 1e9
                if stamp >= flight:
                    if "first_contact_audit_failure" not in contact_events:
                        contact_events["first_contact_audit_failure"] = stamp
                        source_notes.append(f"contact audit failure at {stamp:.6f}s: {exc}")
                    if "non-wheel or unknown ground collision" in str(exc):
                        contact_events.setdefault("first_nonwheel", stamp)
        entry_ns = round(flight * 1e9)
        prior = next(f for f in reversed(frames) if f[0] < entry_ns and f[4])
        touch = next(f for f in frames if f[0] >= entry_ns and f[4])
        for frame in (prior, touch):
            geom = geometry_by_stamp[frame[0]]
            if not geom["valid"] or (geom["seq"], geom["iteration"]) != (frame[1], frame[2]):
                raise ValueError("native contact and geometry sequence mismatch")
        contact_events["native_airborne"] = (prior[0] + 1_000_000) / 1e9
        contact_events["native_touchdown"] = touch[0] / 1e9
        valid = [g for g in geometry if g["valid"] and thrust - .1 <= g["stamp"]/1e9 <= thrust + 1.1]
        if len(valid) < 2 or any(b["stamp"] - a["stamp"] != 1_000_000
                                 for a, b in zip(valid, valid[1:])):
            raise ValueError("native geometry interval incomplete")
        native_t = np.array([g["stamp"]/1e9 for g in valid])
        native_pitch = np.array([g["pitch"] for g in valid])
        delta = np.diff(native_pitch)
        native_rate = np.r_[np.nan, np.arctan2(np.sin(delta), np.cos(delta))/.001]
    else:
        source_notes.append("native sidecars absent; contact events unavailable")

    events = {"卸力开始": first(rows, lambda r: float(r.get("thrust_ground_exit_brake_blend", "0")) > 0),
              "真实离地": contact_events.get("native_airborne"),
              "控制器腾空": flight,
              "保护部署": first(rows, lambda r: r.get("flight_subphase") == "3"),
              "首次轮接触": contact_events.get("native_touchdown"),
              "非轮碰撞": contact_events.get("first_nonwheel"),
              "失稳保护": first(rows, lambda r: r["state_name"] == "EMERGENCY")}
    aligned = [r for r in effort_rows if
               abs(number(r, "imu_sample_stamp") - number(r, "joint_sample_stamp")) < 1e-10
               and 0 <= number(r, "timestamp") - number(r, "imu_sample_stamp") <= .020]
    h_seen = {}
    for r in aligned:
        h_seen.setdefault(number(r, "imu_sample_stamp"), momentum(r)[-1])
    th = np.array(sorted(h_seen))
    hh = np.array([h_seen[s] for s in th])
    feedback = {}
    for key in ("hip_effort_left", "knee_effort_left"):
        if key in rows[0]:
            values = [number(r, key) for r in effort_rows]
            feedback[key] = {"samples": len(values), "min": min(values), "max": max(values),
                             "all_zero": all(abs(v) <= 1e-9 for v in values),
                             "physical_measurement_validated": False}
    feedback_series = {}
    if args.feedback_csv:
        with args.feedback_csv.open(newline="") as stream:
            for r in csv.DictReader(stream):
                if r.get("effort_valid") == "1":
                    feedback_series.setdefault(r["joint"], []).append(
                        (float(r["sample_stamp_s"]), float(r["reported_effort_nm"])))
    stats = []
    for stage, stage_rows in (("THRUST", [r for r in effort_rows if r["state_name"] == "THRUST"]),
                              ("ARREST", [r for r in effort_rows if r["state_name"] == "FLIGHT" and r["flight_subphase"] == "0"]),
                              ("TUCK", [r for r in effort_rows if r["state_name"] == "FLIGHT" and r["flight_subphase"] == "1"]),
                              ("EXTEND", [r for r in effort_rows if r["state_name"] == "FLIGHT" and r["flight_subphase"] == "2"]),
                              ("PROTECTIVE", [r for r in effort_rows if r["state_name"] == "FLIGHT" and r["flight_subphase"] == "3"])):
        for j, key in enumerate(COMMANDS):
            if not stage_rows:
                continue
            low = min(stage_rows, key=lambda r: number(r, key))
            high = max(stage_rows, key=lambda r: number(r, key))
            limit_key = "hip_effort_limit" if j % 2 == 0 else "knee_effort_limit"
            saturation = sum(abs(number(r, key)) >= number(r, limit_key) - 1e-3 for r in stage_rows)
            stats.append({"stage": stage, "joint": JOINTS[j], "min_command_Nm": number(low, key),
                          "min_sim_s": number(low, "timestamp"), "max_command_Nm": number(high, key),
                          "max_sim_s": number(high, "timestamp"), "limit_samples": saturation,
                          "samples": len(stage_rows)})
    summary = {"source_log": str(args.log.resolve()), "source_sha256": hashlib.sha256(args.log.read_bytes()).hexdigest(),
               "command_semantics": "final published command; actual_tau columns are not torque sensors",
               "effort_mode_filter": "effort_mode_active == 1; controller_mode is not an actuator-mode field",
               "origin_sim_s": thrust, "events_sim_s": events, "feedback": feedback,
               "statistics": stats, "source_notes": source_notes,
               "physical_acceptance": "not inferred from torque charts; consult independent contact and landing audits"}
    (output/"torque_metrics.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
    with (output/"phase_torque_statistics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(stats[0]))
        writer.writeheader(); writer.writerows(stats)
    fields = ("timestamp", "joint_sample_stamp", "imu_sample_stamp", "state_name", "flight_subphase",
              "effort_mode_active", *COMMANDS, *VELOCITIES, "pitch_rate_raw",
              "hip_effort_left", "knee_effort_left", "hip_effort_limit", "knee_effort_limit")
    with (output/"torque_signals.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(effort_rows)

    plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10,
                         "axes.unicode_minus": False, "axes.spines.top": False,
                         "axes.spines.right": False, "svg.fonttype": "none"})
    colors = {"卸力开始": "#d97706", "真实离地": "#15803d", "控制器腾空": "#0369a1",
              "保护部署": "#be123c", "首次轮接触": "#15803d", "非轮碰撞": "#7e22ce", "失稳保护": "#be123c"}
    def decorate(ax, start, end, detail):
        ax.set_xlim(start, end); ax.grid(alpha=.2)
        ax.axhline(0, color="#94a3b8", linewidth=.7)
        for name, absolute in events.items():
            if absolute is not None and start <= absolute-thrust <= end:
                ax.axvline(absolute-thrust, color=colors[name], linestyle=":" if name == "卸力开始" else "--", linewidth=.9, alpha=.7)
        if events["非轮碰撞"] is not None and events["非轮碰撞"]-thrust < end:
            ax.axvspan(events["非轮碰撞"]-thrust, end, color="#f3e8ff", alpha=.55)
    def torque_axis(ax, offset, end):
        mask = (t >= thrust) & (t <= thrust+end)
        ax.step(t[mask]-thrust, torque[mask, offset], where="post", color="#1d4ed8", linewidth=1.6, label="左侧指令")
        ax.step(t[mask]-thrust, torque[mask, offset+2], where="post", color="#ea580c", linestyle="--", linewidth=1.2, label="右侧指令")
        for joint, label in (("link_002_joint" if offset == 0 else "link_003_joint", "左侧 reported effort"),
                             ("link_005_joint" if offset == 0 else "link_006_joint", "右侧 reported effort")):
            if joint in feedback_series:
                arr = np.array(feedback_series[joint])
                arr = arr[(arr[:,0] >= thrust) & (arr[:,0] <= thrust+end)]
                ax.plot(arr[:,0]-thrust, arr[:,1], alpha=.6, label=label)
        ax.set_ylabel(("髋" if offset == 0 else "膝")+"指令力矩 (N·m)")
        ax.legend(loc="upper left", ncol=2, frameon=False, fontsize=9)
    event_caption = " | ".join(f"{name} {absolute-thrust:.3f}s" for name,absolute in events.items() if absolute is not None)
    zero_feedback = bool(feedback) and all(v["all_zero"] for v in feedback.values())
    feedback_note = ("注意：主曲线为下发指令；原记录反馈 effort 全0，尚无有效实测扭矩。"
                     if zero_feedback else "注意：主曲线为下发指令；reported effort 需核实物理来源。")
    takeoff_mask = (t >= thrust) & (t <= thrust+.34)
    if np.any(takeoff_mask) and np.max(np.abs(torque[takeoff_mask,:2]-torque[takeoff_mask,2:])) < .02:
        feedback_note += " 左右曲线在本次起跳段基本重合。"
    fig, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=True)
    torque_axis(axes[0],0,.34); torque_axis(axes[1],1,.34)
    qmask = (tq >= thrust) & (tq <= thrust+.34)
    for j,color in ((0,"#1d4ed8"),(1,"#ea580c")):
        axes[2].plot(tq[qmask]-thrust,qdot[qmask,j],color=color,label=JOINTS[j],linewidth=1.3)
        axes[2].plot(tq[qmask]-thrust,qdot[qmask,j+2],color=color,linestyle="--",linewidth=.8,label=JOINTS[j+2])
    axes[2].set_ylabel("关节角速度 (rad/s)");axes[2].legend(ncol=4,frameon=False,fontsize=9)
    if native_t is not None:
        nmask = (native_t >= thrust) & (native_t <= thrust+.34)
        axes[3].plot(native_t[nmask]-thrust,native_rate[nmask],color="#334155",linewidth=1.4,label="原生1ms姿态差分")
    else:
        ti, rates=series(effort_rows,"imu_sample_stamp",("pitch_rate_raw",))
        imask = (ti >= thrust) & (ti <= thrust+.34)
        axes[3].plot(ti[imask]-thrust,rates[imask,0],label="IMU原始角速度")
    axes[3].set_ylabel("机身角速度 (rad/s)");axes[3].legend(frameon=False,fontsize=9)
    hmask = (th >= thrust) & (th <= thrust+.34)
    axes[4].plot(th[hmask]-thrust,hh[hmask],color="#7c3aed",marker=".",markersize=3,label="同stamp CAD估计")
    h_target = next((number(r,"thrust_ground_angular_momentum_target") for r in rows
                     if r["state_name"] == "THRUST" and "thrust_ground_angular_momentum_target" in r), None)
    if h_target is not None:
        axes[4].axhline(h_target,color="#94a3b8",linestyle="--",label=f"本次目标 {h_target:.2f}")
    axes[4].set_ylabel("总角动量 (kg·m²/s)");axes[4].legend(frameon=False,fontsize=9)
    for ax in axes:decorate(ax,0,.34,True)
    axes[-1].set_xlabel(f"相对推地开始的仿真时间 (s)；零点 = {thrust:.3f}s")
    fig.suptitle("起跳阶段：髋、膝指令力矩与运动变化",fontsize=16,x=.08,ha="left",y=.982)
    fig.text(.08,.95,feedback_note,fontsize=10,color="#9a3412")
    fig.text(.08,.926," | ".join(f"{n} {v-thrust:.3f}s" for n,v in events.items() if v is not None and v-thrust<=.34),fontsize=9)
    fig.subplots_adjust(top=.895,bottom=.065,left=.1,right=.96,hspace=.16)
    save_figure(fig,output/"takeoff_torque_detail")
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    torque_axis(axes[0],0,1.0);torque_axis(axes[1],1,1.0)
    if native_t is not None:
        nmask = (native_t >= thrust) & (native_t <= thrust+1.)
        axes[2].plot(native_t[nmask]-thrust,np.rad2deg(native_pitch[nmask]),color="#334155",linewidth=1.4)
    else:
        pmask = (t >= thrust) & (t <= thrust+1.)
        pitch = np.array([math.degrees(number(r,"pitch")) for r in effort_rows])
        axes[2].plot(t[pmask]-thrust,pitch[pmask],color="#334155")
    axes[2].set_ylabel("机身俯仰角 (°)\n正=前倾，负=后倾")
    for ax in axes:decorate(ax,0,1.0,False)
    axes[-1].set_xlabel(f"相对推地开始的仿真时间 (s)；零点 = {thrust:.3f}s")
    fig.suptitle("本次单跳：扭矩指令与机身姿态",fontsize=16,x=.08,ha="left",y=.982)
    fig.text(.08,.94,feedback_note,fontsize=10,color="#9a3412")
    fig.text(.08,.906,event_caption,fontsize=8)
    fig.subplots_adjust(top=.85,bottom=.09,left=.1,right=.96,hspace=.2)
    save_figure(fig,output/"jump_torque_overview")
    print(json.dumps({"output_dir":str(output.resolve()),"events_sim_s":events,
                      "feedback":feedback,"statistics":stats},ensure_ascii=False))


if __name__ == "__main__":
    main()
