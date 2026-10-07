#!/usr/bin/env python3
"""Regression checks for the offline wheel/pendulum coordinate convention.

These checks validate the diagnostic model, not full robot stability.
"""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import replay_recovery_stabilize_planar as replay

class PlanarReplayTest(unittest.TestCase):
    def test_command_and_forward_velocity_are_opposite(self):
        trace = replay.simulate(.0053, .1, .0375, -.004, 1.6)
        for _, _, _, velocity, command in trace:
            self.assertAlmostEqual(velocity, -command, places=12)
        self.assertGreater(trace[-1][1], .26,
                           'old velocity-servo hold falsely appears stable again')

    def test_increasing_attitude_gain_delays_but_does_not_capture(self):
        crossings = []
        for scale in (1., 1.4, 3.):
            trace = replay.simulate(.0053, .1, .0375, -.004, 5.,
                k_theta=replay.K_THETA*scale,
                k_theta_dot=replay.K_THETA_DOT*scale)
            crossing = next(t for t, angle, _, _, _ in trace if abs(angle) >= .15)
            crossings.append(crossing)
        self.assertLess(crossings[0], crossings[1])
        self.assertLess(crossings[1], crossings[2])

    def test_independent_velocity_servo_linearization_has_gravity_mode(self):
        # h*theta_dd = g*theta - axle_accel;
        # tau*axle_accel = A*theta + D*theta_dot - (1+V)*axle_velocity.
        for scale in (1., 1.4, 3.):
            a = .043*.65*233.4004*scale
            d = .043*.22*62.6391*scale
            for tau in (.01, .025, .05):
                h, v = .387, .85
                plant = np.array([[0., 1., 0.],
                    [(9.81-a/tau)/h, -d/(h*tau), (1+v)/(h*tau)],
                    [a/tau, d/tau, -(1+v)/tau]])
                poles = np.linalg.eigvals(plant)
                self.assertEqual(sum(p.real > 0 for p in poles), 1)

    def test_deadband_removes_margin_symmetrically(self):
        self.assertEqual(replay.deadband(.02, .03), 0.)
        self.assertAlmostEqual(replay.deadband(.10, .03), .07)
        self.assertAlmostEqual(replay.deadband(-.10, .03), -.07)

if __name__ == '__main__':
    unittest.main()
