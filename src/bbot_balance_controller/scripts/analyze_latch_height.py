#!/usr/bin/env python3
"""Compare the legacy latch-based apex rise with offline diagnostic references.

The interpolation is between two 20 ms COM-velocity estimates. It is not a
physical liftoff measurement and never replaces the acceptance metric.
"""

import argparse
import csv
import math
from pathlib import Path


def finite(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"missing numeric field: {key}") from None
    if not math.isfinite(value):
        raise ValueError(f"non-finite field: {key}")
    return value


def compute(rows, summary):
    if not rows or not summary:
        raise ValueError("missing controller log or summary")
    latch_index = next((i for i, row in enumerate(rows)
                        if row.get("state_name") == "THRUST"
                        and row.get("velocity_reached") == "1"), None)
    if latch_index is None:
        raise ValueError("95% speed latch not observed in THRUST")
    latch = rows[latch_index]
    if latch.get("com_velocity_valid") != "1":
        raise ValueError("latch COM velocity is invalid")
    latch_stamp = finite(latch, "com_sample_stamp")
    latch_time = finite(latch, "timestamp")
    if latch_stamp <= 0 or not 0 <= latch_time - latch_stamp <= 0.080:
        raise ValueError("latch COM sample is stale or in the future")

    # Keep the first control row for each source stamp. Repeated 200 Hz rows
    # holding a 50 Hz sample must never shift the previous sample's phase.
    samples = {}
    for row in rows[:latch_index + 1]:
        if row.get("state_name") != "THRUST" or row.get("com_velocity_valid") != "1":
            continue
        stamp = finite(row, "com_sample_stamp")
        if stamp > 0:
            samples.setdefault(stamp, row)
    prior_stamps = [stamp for stamp in samples if stamp < latch_stamp]
    if not prior_stamps:
        raise ValueError("no independent COM sample before latch")
    prior_stamp = max(prior_stamps)
    prior = samples[prior_stamp]
    gap = latch_stamp - prior_stamp
    if not 0.015 <= gap <= 0.025:
        raise ValueError(f"unexpected COM sample gap: {gap:.3f} s")
    prior_time = finite(prior, "timestamp")
    if not 0 <= prior_time - prior_stamp <= 0.080:
        raise ValueError("prior COM sample is stale or in the future")

    threshold = 0.95 * finite(latch, "target_takeoff_velocity")
    v0 = finite(prior, "com_world_vz")
    v1 = finite(latch, "com_world_vz")
    if not v0 < threshold <= v1:
        raise ValueError("samples do not bracket the 95% threshold")
    fraction = (threshold - v0) / (v1 - v0)
    z0 = finite(prior, "com_world_z")
    z1 = finite(latch, "com_world_z")
    interpolated_z = z0 + fraction * (z1 - z0)
    interpolated_stamp = prior_stamp + fraction * gap

    legacy_height = finite(summary, "apex_com_z_delta")
    recorded_latch_z = finite(summary, "takeoff_com_z")
    if abs(recorded_latch_z - z1) > 0.002:
        raise ValueError("summary latch height disagrees with controller log")
    apex_z = recorded_latch_z + legacy_height
    flight = next((row for row in rows if row.get("state_name") == "FLIGHT"), None)
    if flight is None:
        raise ValueError("FLIGHT entry missing")
    flight_z = finite(flight, "com_world_z")
    flight_stamp = finite(flight, "com_sample_stamp")
    flight_age = finite(flight, "timestamp") - flight_stamp
    if not 0 <= flight_age <= 0.080:
        raise ValueError("FLIGHT-entry COM sample is stale or in the future")
    flight_max = max(finite(row, "com_world_z") for row in rows
                     if row.get("state_name") == "FLIGHT"
                     and row.get("com_velocity_valid") == "1")
    if abs(apex_z - flight_max) > 0.005:
        raise ValueError("summary apex disagrees with FLIGHT COM samples")

    return {
        "latch_stamp_s": latch_stamp,
        "prior_stamp_s": prior_stamp,
        "sample_gap_ms": 1000 * gap,
        "threshold_vz_mps": threshold,
        "prior_vz_mps": v0,
        "latch_vz_mps": v1,
        "latch_z_m": recorded_latch_z,
        "interpolated_stamp_s": interpolated_stamp,
        "interpolated_z_m": interpolated_z,
        "flight_entry_z_m": flight_z,
        "apex_z_m": apex_z,
        "legacy_rise_m": legacy_height,
        "interpolated_rise_m": apex_z - interpolated_z,
        "flight_entry_rise_m": apex_z - flight_z,
    }


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def analyze(path):
    summary_path = path.with_name(path.name.replace("_log.csv", "_summary.csv"))
    if not summary_path.is_file():
        raise ValueError("matching summary CSV is missing")
    summaries = read_csv(summary_path)
    if len(summaries) != 1:
        raise ValueError("summary CSV must contain exactly one data row")
    return compute(read_csv(path), summaries[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="controller *_log.csv files")
    args = parser.parse_args()
    print("| Trial | Legacy rise (m) | Interpolated-reference rise (m) | FLIGHT-entry-reference rise (m) | Latch z (m) | Interpolated z (m) | Absolute apex z (m) | Status |")
    print("|---|---:|---:|---:|---:|---:|---:|---|")
    for path in args.logs:
        if not path.name.endswith("_log.csv"):
            parser.error(f"not a controller log: {path}")
        try:
            result = analyze(path)
            print(f"| {path.parent.name}/{path.stem} | "
                  f"{result['legacy_rise_m']:.4f} | "
                  f"{result['interpolated_rise_m']:.4f} | "
                  f"{result['flight_entry_rise_m']:.4f} | "
                  f"{result['latch_z_m']:.4f} | "
                  f"{result['interpolated_z_m']:.4f} | "
                  f"{result['apex_z_m']:.4f} | OK |")
        except (OSError, ValueError) as exc:
            print(f"| {path.parent.name}/{path.stem} | — | — | — | — | — | — | UNAVAILABLE: {exc} |")


if __name__ == "__main__":
    main()
