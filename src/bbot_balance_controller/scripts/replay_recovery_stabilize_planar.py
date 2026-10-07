#!/usr/bin/env python3
"""Planar replay of the RECOVERY_EFFORT_STABILIZE wheel law (read-only).

Model: rigid inverted pendulum (COM height h above ground contact) on a
rolling wheel whose axle velocity tracks the published command with the
controller's slew limit and command caps. Odometry velocity == axle velocity
(rolling). The law, gains, deadbands, capture caps and slew limits are
transcribed from bbot_velocity_jump_controller.cpp (2026-09-29 @ 873c325):
  cmd = 0.043*(0.65*k_theta*lean + 0.22*k_theta_dot*lean_rate)
        + 0.85*deadband(x_dot,0.03) + 0.35*clamp(x-x_ref,0.75,db 0.02)
  capture caps 0.75..1.10, slew 4.0 (slow) / 7.5 m/s^2, 200 Hz.
Sign calibration from trial_01_20260929_233154 catch phase:
  cmd +1.94 -> wheels +28 rad/s -> axle moves backward; so positive cmd =
  axle backward; negative cmd = forward chase.
"""
import math

DT = 0.005
G = 9.81
H = 0.387            # COM above ground contact (0.317 above axle + 0.07 wheel)
CMD_SCALE = 0.043
K_THETA = -233.4004
K_THETA_DOT = -62.6391
WHEEL_R = 0.07


def deadband(v, db):
    return 0.0 if abs(v) < db else math.copysign(abs(v) - db, v)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Params:
    att_theta = 0.65
    att_rate = 0.22
    kv = 0.85
    kx = 0.35
    x_dot_alpha = 0.08          # per 5 ms tick
    base_cap = 0.75
    cap_gain = 1.75
    cap_extra_max = 0.20
    capture_threshold = 0.030
    slow_vel = 0.20
    slow_capture = 0.12
    accel_slow = 4.0
    accel_fast = 7.5
    x_ref = 0.0


def simulate(theta0, theta_dot0, v0, x0, dur, p=Params, k_theta=K_THETA,
             k_theta_dot=K_THETA_DOT, cmd_scale=CMD_SCALE, wheel_cap=None):
    theta, theta_dot, v, x = theta0, theta_dot0, v0, x0
    cmd_x = -v0
    x_dot = v
    trace = []
    n = int(dur / DT)
    for i in range(n):
        lean_err = theta
        lean_rate = theta_dot
        p_term = p.att_theta * k_theta * lean_err
        d_term = p.att_rate * k_theta_dot * lean_rate
        attitude = cmd_scale * (p_term + d_term)
        velocity_error = deadband(x_dot, 0.03)
        raw_pos = x - p.x_ref
        pos_err = deadband(clamp(raw_pos, -0.75, 0.75), 0.02)
        cmd_target = attitude + p.kv * velocity_error + p.kx * pos_err

        capture_height = clamp(H - WHEEL_R, 0.15, 0.55)
        capture_omega = math.sqrt(G / capture_height)
        capture_state = lean_err + lean_rate / capture_omega
        min_fwd, max_bwd = -p.base_cap, p.base_cap
        if lean_err < 0.0 and capture_state < -p.capture_threshold:
            guard = clamp(-capture_state - p.capture_threshold, 0.0, p.cap_extra_max)
            max_bwd = clamp(p.base_cap + p.cap_gain * guard, p.base_cap, p.base_cap + 0.35)
        elif lean_err > 0.0 and capture_state > p.capture_threshold:
            guard = clamp(capture_state - p.capture_threshold, 0.0, p.cap_extra_max)
            min_fwd = -clamp(p.base_cap + p.cap_gain * guard, p.base_cap, p.base_cap + 0.35)
        if wheel_cap is not None:
            min_fwd, max_bwd = -wheel_cap, wheel_cap
        if raw_pos > 0.75 and capture_state < 0.12:
            cmd_target = max(cmd_target, 0.0)
        elif raw_pos < -0.75 and capture_state > -0.12:
            cmd_target = min(cmd_target, 0.0)
        final = clamp(cmd_target, min_fwd, max_bwd)

        slow = abs(x_dot) < p.slow_vel and abs(capture_state) < p.slow_capture
        step = (p.accel_slow if slow else p.accel_fast) * DT
        cmd_x += clamp(final - cmd_x, -step, step)

        # Forward axle velocity has the opposite sign to the wheel command.
        # x_dot/x must use the same forward coordinate as lean and theta_ddot.
        v_prev = v
        v = -cmd_x
        x_dot += p.x_dot_alpha * (v - x_dot)   # per-tick low pass (200 Hz)
        x += v * DT
        # Pendulum acceleration uses forward axle acceleration directly.
        a = (v - v_prev) / DT
        theta_ddot = (G * math.sin(theta) - math.cos(theta) * a) / H
        theta_dot += theta_ddot * DT
        theta += theta_dot * DT
        trace.append((i * DT, theta, theta_dot, v, cmd_x))
    return trace


if __name__ == "__main__":
    import sys
    # Stabilize-entry state from trial_01_20260929_233154 (candidate run2 trial 1)
    # theta0 = com_lean 15.002 -> +0.0053; theta_dot0 = com_lean_rate;
    # v0 = x_dot 0.0375; x0-x_ref = 1.0760-1.0800 = -0.004.
    # Baseline reference divergence: lean 0.005 -> 0.265 in ~1.2 s.
    p = Params()
    p.x_ref = 0.0
    trace = simulate(0.0053, 0.10, 0.0375, -0.004, 1.6, p)
    print(" t     theta   theta_dot   v      cmd")
    for t, th, thd, v, c in trace[::40]:
        print(f"{t:5.2f}  {th:+.4f}  {thd:+.4f}  {v:+.3f}  {c:+.3f}")
    reach = next((row for row in trace if abs(row[1]) > 0.26), None)
    print("time to |theta|=0.26:", f"{reach[0]:.2f}s" if reach else "never")
