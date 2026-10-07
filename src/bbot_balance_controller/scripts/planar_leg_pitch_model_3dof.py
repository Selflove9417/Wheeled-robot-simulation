#!/usr/bin/env python3
"""Sagittal fixed-axle model in measured coordinates (hip, knee, IMU pitch).

The URDF joint rotations are relative to the torso. The logged IMU pitch has
opposite sign to URDF X-axis rotation, so world link angles are
  torso=-pitch, thigh=-pitch+hip, shank=-pitch+hip+knee.
The wheel axle is the reference point. Its forward acceleration and wheel spin
acceleration are prescribed inputs, not inferred contact forces. Torques are
per leg; symmetric left/right motors contribute twice. Since hip and knee are
*relative* generalized coordinates, motor virtual work is tau_h*dhip+
 tau_k*dknee: there is no separate -tau_h on the pitch coordinate.

This is an analysis model, not a validated contact/actuator model. Log effort
fields echo commanded values; contact wrench is missing. Validate kinematics
and measured response before using its dynamics to select a control change.
"""
import argparse
import csv
import math
import numpy as np

G = 9.81
R_WHEEL = 0.07
MASS_BODY = 9.5
MASS_THIGH_PAIR = 2.4
MASS_SHANK_PAIR = 1.6
MASS_WHEEL_PAIR = 4.0
INERTIA_BODY = 0.159013 * MASS_BODY / 14.0
INERTIA_THIGH_PAIR = 2 * 0.017921
INERTIA_SHANK_PAIR = 2 * 0.013130
INERTIA_WHEEL_PAIR = 2 * 0.006481
BODY_COM_FROM_HIP = np.array([0.00761282, 0.12396677])
THIGH_COM_FROM_HIP = np.array([-0.13690699, -0.02116697])
KNEE_FROM_HIP = np.array([-0.29348091, -0.06220095])
SHANK_COM_FROM_KNEE = np.array([0.11538205, -0.08532288])
AXLE_FROM_KNEE = np.array([0.28210870, -0.19553796])
MASSES = np.array([MASS_BODY, MASS_THIGH_PAIR, MASS_SHANK_PAIR,
                   MASS_WHEEL_PAIR])
ANGULAR_JACOBIAN = np.array([[0., 0., -1.], [1., 0., -1.],
                             [1., 1., -1.], [1., 1., -1.]])
INERTIAS = np.array([INERTIA_BODY, INERTIA_THIGH_PAIR,
                     INERTIA_SHANK_PAIR, INERTIA_WHEEL_PAIR])


def rotate(angle, point):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * point[0] - s * point[1],
                     s * point[0] + c * point[1]])


def positions(q):
    """World (forward, up) COM positions with wheel axle at (0, R_WHEEL)."""
    hip_angle, knee_angle, pitch = q
    torso_angle = -pitch
    thigh_angle = torso_angle + hip_angle
    shank_angle = thigh_angle + knee_angle
    knee_offset = rotate(thigh_angle, KNEE_FROM_HIP)
    axle_offset = knee_offset + rotate(shank_angle, AXLE_FROM_KNEE)
    hip = np.array([0., R_WHEEL]) - axle_offset
    torso = hip + rotate(torso_angle, BODY_COM_FROM_HIP)
    thigh = hip + rotate(thigh_angle, THIGH_COM_FROM_HIP)
    shank = hip + knee_offset + rotate(shank_angle, SHANK_COM_FROM_KNEE)
    return np.array([torso, thigh, shank, [0., R_WHEEL]])


def centroidal(q):
    p = (MASSES[:, None] * positions(q)).sum(axis=0) / MASSES.sum()
    forward, height = p[0], p[1] - R_WHEEL
    return forward, height, math.atan2(forward, height)


def position_jacobian(q, step=1e-6):
    out = np.empty((4, 2, 3))
    for i in range(3):
        delta = np.zeros(3)
        delta[i] = step
        out[:, :, i] = (positions(q + delta) - positions(q - delta)) / (2*step)
    return out


def mass_matrix(q):
    jac = position_jacobian(q)
    mass = np.einsum('b,bai,baj->ij', MASSES, jac, jac)
    for inertia, angular in zip(INERTIAS, ANGULAR_JACOBIAN):
        mass += inertia * np.outer(angular, angular)
    return mass


def potential(q):
    return float(G * np.dot(MASSES, positions(q)[:, 1]))


def gravity(q, step=1e-6):
    """dV/dq; the dynamics use M*qdd + C + gravity = Q."""
    out = np.empty(3)
    for i in range(3):
        delta = np.zeros(3)
        delta[i] = step
        out[i] = (potential(q + delta) - potential(q - delta)) / (2*step)
    return out


def coriolis(q, qdot, step=1e-5):
    dmass = np.empty((3, 3, 3))
    for k in range(3):
        delta = np.zeros(3)
        delta[k] = step
        dmass[:, :, k] = (mass_matrix(q + delta) - mass_matrix(q - delta)) / (2*step)
    out = np.zeros(3)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                gamma = 0.5 * (dmass[i, j, k] + dmass[i, k, j] - dmass[j, k, i])
                out[i] += gamma * qdot[j] * qdot[k]
    return out


def generalized_force(q, qdot, hip_torque, knee_torque, axle_acceleration,
                      wheel_spin_acceleration=0.0):
    """Commanded motor torque plus known kinematic inputs; no contact force."""
    motor = np.array([2*hip_torque, 2*knee_torque, 0.])
    jac = position_jacobian(q)
    acceleration_force = -axle_acceleration * np.einsum('b,bi->i',
                                                        MASSES, jac[:, 0, :])
    rotor_reaction = -INERTIA_WHEEL_PAIR * wheel_spin_acceleration * ANGULAR_JACOBIAN[3]
    passive = np.array([-1.0*qdot[0], -1.0*qdot[1], 0.])
    return motor + acceleration_force + rotor_reaction + passive


def acceleration(q, qdot, hip_torque, knee_torque, axle_acceleration,
                 wheel_spin_acceleration=0.0):
    force = generalized_force(q, qdot, hip_torque, knee_torque,
                              axle_acceleration, wheel_spin_acceleration)
    return np.linalg.solve(mass_matrix(q), force - coriolis(q, qdot) - gravity(q))



def rolling_acceleration(q, qdot, hip_torque, knee_torque,
                         axle_acceleration, step=1e-5):
    """No-slip planar response with measured axle acceleration imposed.

    Forward wheel travel corresponds to negative wheel rotation in these log
    coordinates. Hence absolute wheel angle = -axle_x/R_WHEEL and relative
    motor angle = -axle_x/R_WHEEL - thigh_angle - knee_angle. The unmeasured
    wheel motor torque is eliminated using the axle equation. This remains a
    hypothesis: local rolling velocity agreement does not prove no slip or
    reveal actual wheel/contact effort.
    """
    jac = position_jacobian(q)
    coupling = np.einsum('b,bi->i', MASSES, jac[:, 0, :])
    coupling_gradient = np.empty((3, 3))
    for k in range(3):
        delta = np.zeros(3)
        delta[k] = step
        jplus = position_jacobian(q + delta)
        jminus = position_jacobian(q - delta)
        coupling_gradient[:, k] = np.einsum(
            'b,bi->i', MASSES,
            (jplus[:, 0, :] - jminus[:, 0, :]) / (2*step))
    axle_coriolis = float(qdot @ coupling_gradient @ qdot)

    motor = np.array([2*hip_torque, 2*knee_torque, 0.])
    shank_angle_jac = ANGULAR_JACOBIAN[3]
    # Wheel absolute spin follows rolling; its inertia belongs to axle only.
    body_mass = mass_matrix(q) - INERTIA_WHEEL_PAIR * np.outer(
        shank_angle_jac, shank_angle_jac)
    axle_mass = MASSES.sum() + INERTIA_WHEEL_PAIR / R_WHEEL**2
    lhs = body_mass - R_WHEEL * np.outer(shank_angle_jac, coupling)
    rhs = (motor - coriolis(q, qdot) - gravity(q)
           + (R_WHEEL * axle_mass * shank_angle_jac - coupling)
           * axle_acceleration
           + R_WHEEL * axle_coriolis * shank_angle_jac)
    return np.linalg.solve(lhs, rhs)

def validate_log(path):
    errors = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row['com_balance_valid'] != '1':
                continue
            try:
                q = np.array([float(row['hip_pos_left']),
                              float(row['knee_pos_left']), float(row['pitch'])])
                observed = np.array([float(row['com_forward_from_axle']),
                                     float(row['com_height_above_axle']),
                                     float(row['com_lean'])])
            except (ValueError, KeyError):
                continue
            if np.all(np.isfinite(q)) and np.all(np.isfinite(observed)):
                errors.append(np.array(centroidal(q)) - observed)
    if not errors:
        raise ValueError('No valid centroidal rows in '+path)
    e = np.abs(np.array(errors))
    print('rows', len(e), 'mean_abs [forward,height,lean]', e.mean(axis=0),
          'max_abs', e.max(axis=0))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='Controller CSV for centroidal check')
    args = parser.parse_args()
    sample = np.array([.25, -.36, .04])
    print('sample_centroidal', centroidal(sample))
    print('sample_mass', mass_matrix(sample))
    print('sample_gravity', gravity(sample))
    if args.log:
        validate_log(args.log)
