#!/usr/bin/env python3
"""Behavior checks for the optional constraints of the shared IK solver."""

import unittest

import numpy as np

from aerial_motion_planning.ik import IKProblem


class TestWholeBodyIK(unittest.TestCase):
    def test_masked_coordinate_stays_fixed_in_both_modes(self):
        problem = IKProblem(lambda x: x - [2.0, 3.0], [0.0, 0.0], [4.0, 4.0], mask=[True, False])
        np.testing.assert_allclose(problem.offline([0.0, 1.0]), [2.0, 1.0], atol=1e-5)
        np.testing.assert_allclose(problem.incremental([0.0, 1.0], iterations=2)[0], [2.0, 1.0], atol=1e-5)

    def test_optional_contact_overrides_tip_in_incremental_mode(self):
        free = IKProblem(lambda x: x - [2.0], [0.0], [4.0])
        contact = IKProblem(lambda x: x - [2.0], [0.0], [4.0], equality=lambda x: x - [1.0])
        self.assertAlmostEqual(free.incremental([0.0])[0][0], 2.0, places=5)
        solution, softened, _ = contact.incremental([0.0])
        self.assertAlmostEqual(solution[0], 1.0, places=5)
        self.assertFalse(softened)

    def test_collision_check_can_block_a_step(self):
        collisions = lambda x: {("arm", "body")} if x[0] > 0.8 else set()
        problem = IKProblem(lambda x: x - [2.0], [0.0], [4.0], collision=collisions)
        solution, _, blocked = problem.incremental([0.0])
        self.assertLessEqual(solution[0], 0.8)
        self.assertEqual(blocked, [("arm", "body")])


if __name__ == "__main__":
    unittest.main()
