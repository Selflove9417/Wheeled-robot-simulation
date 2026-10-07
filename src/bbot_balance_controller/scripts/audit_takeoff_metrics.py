#!/usr/bin/env python3
"""Offline takeoff-metric audit for flat-ground jump trials (read-only).

Separates real-motion evidence from latched/summary bookkeeping:

* com_world_vz is the 20 ms backward difference of com_world_z
  (verified: (z(8.20)-z(8.18))/0.02 == vz(8.20) on archived logs), i.e. a
  window MEAN, while the controller latch threshold is instantaneous
  (0.95 * target). A peak living inside one window can fail the latch.
* Physical takeoff time is NOT determinable (contact wrench missing), so all
  apex deltas are reported per labelled proxy event, never as one number.

Usage:
  python3 audit_takeoff_metrics.py trial_dir_or_log.csv [more...]
"""
import csv
import math
import sys
from pathlib import Path

DT = 0.02          # COM estimator period (s)
G = 9.81
VZ_LO, VZ_HI = 1.83, 2.13
APEX_LO, APEX_HI = 0.17, 0.23
VX_LO, VX_HI = 0.35, 0.55


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def load(path):
    with open(path, newline="", encoding="utf-8", errors="replace") as stream:
        return list(csv.DictReader(stream))


def unique_com_samples(rows):
    """(stamp, z, vz, t_first_seen) for each distinct com_sample_stamp."""
    seen = {}
    order = []
    for row in rows:
        stamp = number(row.get("com_sample_stamp"))
        if stamp is None or stamp <= 0:
            continue
        z = number(row.get("com_world_z"))
        vz = number(row.get("com_world_vz"))
        if z is None or vz is None:
            continue
        if stamp not in seen:
            seen[stamp] = (stamp, z, vz, number(row.get("timestamp")))
            order.append(stamp)
    return [seen[s] for s in sorted(order)]


def first_where(rows, predicate):
    for row in rows:
        if predicate(row):
            return row
    return None


def audit(log_path):
    rows = load(log_path)
    if not rows:
        return None
    samples = unique_com_samples(rows)
    if len(samples) < 3:
        print(f"{log_path}: no usable COM samples")
        return None

    # ---- real-motion evidence -------------------------------------------
    peak = max(samples, key=lambda s: s[2])
    apex = max(samples, key=lambda s: s[1])
    # vz estimator = (z[stamp] - z[stamp-0.02]) / 0.02  -> window mean over
    # [stamp-DT, stamp].  True instantaneous peak >= window mean; <= mean plus
    # one window of pre-peak rise (g per window after release).
    peak_lo = peak[2]
    peak_hi = peak[2] + G * DT

    # takeoff proxies (labelled, non-physical):
    proxies = {}
    blend1 = first_where(rows, lambda r: number(r.get("thrust_release_blend")) is not None
                         and number(r.get("thrust_release_blend")) >= 1.0)
    if blend1 is not None:
        proxies["blend=1"] = (number(blend1.get("timestamp")), number(blend1.get("com_world_z")))
    clearance = first_where(rows, lambda r: (number(r.get("wheel_clearance")) or -1) > 0)
    if clearance is not None:
        proxies["clearance>0"] = (number(clearance.get("timestamp")),
                                  number(clearance.get("com_world_z")))
    last_contact_stamp = None
    contacts_path = Path(str(log_path).replace("_log.csv", "_contacts.csv"))
    if contacts_path.is_file():
        last_stamp = None
        with open(contacts_path, newline="", encoding="utf-8", errors="replace") as stream:
            for row in csv.DictReader(stream):
                stamp = number(row.get("stamp") or row.get("header_stamp") or row.get("sim_stamp"))
                if stamp is not None:
                    last_stamp = stamp
        if last_stamp is not None:
            # existence cue only; com z at that moment from nearest sample
            nearest = min(samples, key=lambda s: abs(s[0] - last_stamp))
            proxies["last_contact_msg"] = (last_stamp, nearest[1])

    # ---- latched / summary bookkeeping ----------------------------------
    summary = {}
    summary_path = Path(str(log_path).replace("_log.csv", "_summary.csv"))
    if summary_path.is_file():
        with open(summary_path, newline="", encoding="utf-8", errors="replace") as stream:
            entries = list(csv.DictReader(stream))
        if entries:
            summary = entries[-1]
    latched = number(rows[-1].get("com_takeoff_latched_vz"))
    confirmed = number(rows[-1].get("com_takeoff_confirmed_vz"))
    apex_delta_summary = number(summary.get("apex_com_z_delta"))
    takeoff_vz_summary = number(summary.get("takeoff_com_vz"))
    takeoff_vx_summary = number(summary.get("takeoff_com_vx"))
    target_vz = number(summary.get("target_takeoff_velocity")) or 1.98091
    latch_threshold = 0.95 * target_vz

    print(f"\n=== {Path(log_path).name} ===")
    print(f"latch: threshold={latch_threshold:.5f}  peak_est_sample={peak[2]:.5f} "
          f"(miss={latch_threshold - peak[2]:+.5f})  latched_vz={latched} "
          f"confirmed_vz={confirmed}")
    print(f"summary: takeoff_vz={takeoff_vz_summary} apex_delta={apex_delta_summary} "
          f"takeoff_vx={takeoff_vx_summary}")
    print("REAL MOTION (offline, raw COM samples):")
    print(f"  peak vz window-mean {peak[2]:.3f} at stamp {peak[0]:.3f}; "
          f"instantaneous in [{peak_lo:.3f}, {peak_hi:.3f}]  "
          f"band [{VZ_LO},{VZ_HI}]: "
          f"{'PASS(robust)' if peak_lo >= VZ_LO and peak_hi <= VZ_HI else 'AMBIGUOUS/FAIL'}")
    print(f"  apex com_z {apex[1]:.3f} at stamp {apex[0]:.3f}")
    print(f"  ballistic apex delta from vz bound: "
          f"[{peak_lo**2/(2*G):.3f}, {peak_hi**2/(2*G):.3f}]  "
          f"band [{APEX_LO},{APEX_HI}]: "
          f"{'PASS(robust)' if peak_lo**2/(2*G) >= APEX_LO and peak_hi**2/(2*G) <= APEX_HI else 'AMBIGUOUS/FAIL'}")
    for name, (stamp, z) in proxies.items():
        delta = apex[1] - z
        # sampling bound on proxy z: within one window z moves <= vz*DT
        local_vz = min(samples, key=lambda s: abs(s[0] - stamp))[2]
        bound = abs(local_vz) * DT
        verdict = ("PASS(robust)" if delta - bound >= APEX_LO and delta + bound <= APEX_HI
                   else ("FAIL(robust)" if delta + bound < APEX_LO or delta - bound > APEX_HI
                         else "AMBIGUOUS"))
        print(f"  proxy {name:17s} t={stamp:.3f} z={z:.3f} "
              f"apex_delta={delta:.3f} ±{bound:.3f} [{delta-bound:.3f},{delta+bound:.3f}] "
              f"{verdict}")
    fwd = takeoff_vx_summary
    if fwd is not None:
        print(f"  takeoff vx (summary channel): {fwd:.3f} band [{VX_LO},{VX_HI}]: "
              f"{'PASS' if VX_LO <= fwd <= VX_HI else 'FAIL'}")
    print("  NOTE: physical takeoff instant undeterminable (contact wrench missing); "
          "proxy deltas only.")
    return {"peak": peak, "peak_lo": peak_lo, "peak_hi": peak_hi,
            "apex": apex, "summary": summary, "latched": latched}


def main():
    paths = []
    for arg in sys.argv[1:]:
        path = Path(arg)
        if path.is_dir():
            paths.extend(sorted(path.glob("*_log.csv")))
        elif path.is_file():
            paths.append(path)
    if not paths:
        print("no logs", file=sys.stderr)
        return 1
    for path in paths:
        audit(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
