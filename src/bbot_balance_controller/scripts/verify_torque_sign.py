#!/usr/bin/env python3
"""
verify_torque_sign.py
Quick test to verify whether tau_each = -0.5 * u or +0.5 * u stabilizes the robot.
"""
import os
import sys
import time
import subprocess
import signal

WS_ROOT = "/home/admin/bbot_ws_new"

def cleanup():
    subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ign gazebo"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros_gz_bridge"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "torque_cascade_pid_controller"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "spawner"], stderr=subprocess.DEVNULL)
    time.sleep(1.5)

def run_test(duration=8.0):
    cleanup()
    log_path = f"{WS_ROOT}/src/bbot_balance_controller/src/data_logs/torque_sign_test.csv"
    if os.path.exists(log_path):
        os.remove(log_path)

    cmd = (
        f"source {WS_ROOT}/setup_env.sh && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"controller_type:=torque_cascade_pid "
        f"world:=empty.sdf "
        f"torque_pid_log_path:={log_path} "
        f"torque_pid_startup_height:=0.36 "
        f"torque_pid_target_height:=0.36 "
    )

    print("[SignTest] Launching test...")
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid, text=True)

    t0 = time.time()
    last_log_check = time.time()
    started = False
    try:
        while time.time() - t0 < duration:
            time.sleep(0.5)
            if os.path.exists(log_path) and os.path.getsize(log_path) > 100:
                if not started:
                    print("[SignTest] Log data detected, controller is publishing.")
                    started = True
    finally:
        cleanup()

    if os.path.exists(log_path) and os.path.getsize(log_path) > 100:
        import csv
        with open(log_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = [r for r in reader if len(r) == len(reader.fieldnames) and r['pitch'] is not None]
        print(f"[SignTest] Collected {len(rows)} valid samples.")
        if not rows:
            print("[SignTest] No valid samples found.")
            return False
        pitches = [float(r['pitch']) for r in rows]
        u_vals = [float(r['u_clamped']) for r in rows]
        tau_vals = [float(r['tau_left']) for r in rows]
        v_vals = [float(r['v']) for r in rows]
        min_p, max_p = min(pitches), max(pitches)
        print(f"  Pitch range: [{min_p:.4f}, {max_p:.4f}] rad ([{min_p*57.3:.2f}, {max_p*57.3:.2f}] deg)")
        print(f"  u_clamped range: [{min(u_vals):.2f}, {max(u_vals):.2f}] Nm")
        print(f"  tau_left range: [{min(tau_vals):.2f}, {max(tau_vals):.2f}] Nm")
        print(f"  v range: [{min(v_vals):.3f}, {max(v_vals):.3f}] m/s")
        max_abs_pitch = max(abs(min_p), abs(max_p))
        if max_abs_pitch > 0.45:
            print(f"[SignTest] FAIL: Robot fell! Max |pitch| = {max_abs_pitch:.4f} rad ({max_abs_pitch*57.3:.2f} deg)")
            return False
        else:
            print(f"[SignTest] SUCCESS: Robot remained upright! Max |pitch| = {max_abs_pitch:.4f} rad ({max_abs_pitch*57.3:.2f} deg)")
            return True
    else:
        print("[SignTest] No CSV data produced.")
        return False

if __name__ == "__main__":
    run_test()
