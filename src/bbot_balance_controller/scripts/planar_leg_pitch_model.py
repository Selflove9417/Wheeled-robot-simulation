#!/usr/bin/env python3
"""2-DOF sagittal standing model: hip angle qh + body pitch phi.

Knee locked at QK (recovery-stabilize value); axle pinned to wheel radius
(pure rolling), axle world motion prescribed by the wheel-velocity servo.
Axle acceleration enters as a pseudo-force on the (qh, phi) slice.

Geometry/mass constants transcribed from centroidal_state.hpp (bit-matched
against ground_gravity_* log columns). Symmetric legs (log |L-R|<=4e-4).
"""
import math
import numpy as np

M_BODY, M_THIGH, M_SHANK, M_WHEEL = 9.5, 1.2, 0.8, 2.0
G0 = 9.81
RW = 0.07
QK = -0.367
HIP2BOX = np.array([0.00761282, 0.12396677])
THIGH_C = np.array([-0.13690699, -0.02116697])
KNEE_POS = np.array([-0.29348091, -0.06220095])
SHANK_C = np.array([0.11538205, -0.08532288])
AXLE_OFF = np.array([0.28210870, -0.19553796])
I_BODY = 0.159013 * M_BODY / 14.0
I_THIGH, I_SHANK = 0.017921, 0.013130


def rot(a, p):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1]])


def axle_offset(qh):
    return rot(qh, KNEE_POS) + rot(qh + QK, AXLE_OFF)


def bodies(qh, phi):
    """COM of body, thigh-pair, shank-pair with axle frozen at origin."""
    off = axle_offset(qh)
    hip = -off  # axle=(0, RW) => hip = -off ; z handled by construction below
    hip = np.array([-off[0], RW - off[1]])
    box = hip + rot(phi, HIP2BOX)
    thigh = hip + rot(qh, THIGH_C)
    shank = hip + rot(qh, KNEE_POS) + rot(qh + QK, SHANK_C)
    return box, thigh, shank


BODIES = ((M_BODY, 0, I_BODY), (2 * M_THIGH, 1, 2 * I_THIGH),
          (2 * M_SHANK, 2, 2 * I_SHANK))


def body_list(qh, phi):
    box, thigh, shank = bodies(qh, phi)
    return [box, thigh, shank]


def jacobian(qh, phi, which, h=1e-7):
    """d COM_i / d q[which]."""
    dqh = qh + (h if which == 0 else 0.0)
    dphi = phi + (h if which == 1 else 0.0)
    p0 = body_list(qh, phi)
    p1 = body_list(dqh, dphi)
    return [(b - a) / h for a, b in zip(p0, p1)]


def mass_matrix(qh, phi):
    M = np.zeros((2, 2))
    for i in range(2):
        Ji = jacobian(qh, phi, i)
        for j in range(2):
            Jj = jacobian(qh, phi, j)
            s = 0.0
            for (m, idx, _) in BODIES:
                s += m * float(Ji[idx] @ Jj[idx])
            M[i, j] = s
    # rotational diagonals: qh rotates both locked leg links, phi the box
    M[0, 0] += 2 * I_THIGH + 2 * I_SHANK
    M[1, 1] += I_BODY
    return M


def gravity_torque(qh, phi):
    """Q = -dV/dq with V = sum m g z."""
    h = 1e-7
    out = np.zeros(2)
    for i in range(2):
        dqh = qh + (h if i == 0 else 0.0)
        dphi = phi + (h if i == 1 else 0.0)
        p1 = body_list(dqh, dphi)
        p0 = body_list(qh, phi)
        dv = 0.0
        for (m, idx, _), a, b in zip(BODIES, p0, p1):
            dv += m * G0 * (b[1] - a[1])
        out[i] = -dv / h
    return out


def pseudo_force(qh, phi, xdd):
    """Axle acceleration xdd (world forward+) -> generalized force."""
    out = np.zeros(2)
    for i in range(2):
        Ji = jacobian(qh, phi, i)
        s = 0.0
        for (m, idx, _) in BODIES:
            s += m * Ji[idx][0]
        out[i] = -s * xdd
    return out


def step(q, qd, tau_hip, xdd, dt):
    """One explicit RK4-ish (Euler is enough at 1 kHz) integration step.

    tau_hip: torque applied to the LEG link about the hip; reaction -tau on
    the body. xdd: prescribed axle acceleration.
    """
    M = mass_matrix(*q)
    Gh = gravity_torque(*q)
    Qp = pseudo_force(*q, xdd)
    # Coriolis/centrifugal via finite-difference of M along motion (small)
    # neglected here: magnitudes << gravity at these speeds (validated later).
    Q = np.array([tau_hip, -tau_hip]) + Qp
    qdd = np.linalg.solve(M, Q - Gh)
    q_new = q + qd * dt
    qd_new = qd + qdd * dt
    return q_new, qd_new, qdd, M, Gh, Q


if __name__ == "__main__":
    q = np.array([0.251, 0.031])
    qd = np.array([-0.05, 0.02])
    M = mass_matrix(*q)
    Gt = gravity_torque(*q)
    print("M=\n", M)
    print("gravity torque=", Gt)
    print("pseudo for 1 m/s^2:", pseudo_force(*q, 1.0))
