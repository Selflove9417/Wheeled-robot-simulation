#!/usr/bin/env python3
"""Non-overwriting PID measurements for the journal-paper GS-LQR comparison.

The commissioning/acceptance CSV files remain untouched. Each condition is
recorded to a dedicated publication_campaign directory, and the complete
pass/fail manifest is kept beside those CSVs.
"""

import argparse
import importlib.util
import json
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
ACCEPTANCE = SCRIPT_DIR / "run_torque_pid_acceptance.py"
OUTPUT = SCRIPT_DIR.parent / "src" / "data_logs" / "pid_publication_campaign"


def load_acceptance():
    spec = importlib.util.spec_from_file_location("pid_acceptance", ACCEPTANCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.TRIALS_DIR = str(OUTPUT)
    return module


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", choices=("static", "lift", "push", "all"), default="all")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--height", type=float, choices=(0.30, 0.40, 0.50))
    parser.add_argument("--force", type=int, choices=(-20, 20))
    parser.add_argument("--rep", type=int)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")

    OUTPUT.mkdir(parents=True, exist_ok=True)
    manifest_path = OUTPUT / "manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
    else:
        manifest = {"protocol": "nominal three-loop PID, k_x=0, Hdot=0.05 m/s", "trials": {}}
    acc = load_acceptance()

    def run_once(key, filename, fn):
        path = OUTPUT / filename
        if key in manifest["trials"] or path.exists():
            print(f"[Skip] {key}: pre-existing result/log; no overwrite")
            return
        ok, metrics = fn()
        manifest["trials"][key] = {"accepted": bool(ok), "metrics": metrics, "file": filename}
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        print(f"[Saved] {key}: accepted={ok}")

    if args.job in ("static", "all"):
        heights = (args.height,) if args.height else (0.30, 0.40, 0.50)
        for height in heights:
            start = 0.36
            for rep in range(1, args.repeats + 1):
                key = f"static_h{int(height*100):03d}_rep{rep}"
                filename = key + ".csv"
                run_once(key, filename, lambda h=height, s=start, name=filename:
                         acc.run_static_test(1, h, s, name))

    if args.job in ("lift", "all"):
        for rep in range(1, args.repeats + 1):
            key = f"lift_rep{rep}"
            filename = key + ".csv"
            run_once(key, filename, lambda name=filename:
                     acc.run_test_sweep(log_filename=name))

    if args.job in ("push", "all"):
        heights = (args.height,) if args.height else (0.40, 0.50)
        for height in heights:
            start = 0.36
            forces = (float(args.force),) if args.force is not None else (+20.0, -20.0)
            for force in forces:
                prefix = f"push_h{int(height*100):03d}_{'pos' if force > 0 else 'neg'}_"
                for rep in range(1, args.repeats + 1):
                    if args.rep is not None and rep != args.rep:
                        continue
                    if args.rep is None and sum(
                            item["accepted"] for key, item in manifest["trials"].items()
                            if key.startswith(prefix)) >= 3:
                        print(f"[Complete] {prefix}: three accepted runs")
                        break
                    key = f"push_h{int(height*100):03d}_{'pos' if force > 0 else 'neg'}_rep{rep}"
                    filename = key + ".csv"
                    run_once(key, filename, lambda f=force, h=height, s=start, name=filename:
                             acc.run_push_test(f, 5 if f > 0 else 6, name,
                                               target_height=h, startup_height=s,
                                               late_bridge=True))


if __name__ == "__main__":
    main()
