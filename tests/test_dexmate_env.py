"""Validate action contracts and the sustained lift/release success condition."""

import unittest

import numpy as np

from Isaacsim_tactile_env.dexmate_env import GraspProgress, validate_action


class EnvironmentContractTests(unittest.TestCase):
    def test_invalid_actions_rejected(self):
        lo, hi = np.array([-1., 0.]), np.array([1., .4])
        for value in ([0], [0, np.nan], [0, np.inf], [1.1, .1], [0, -.1]):
            with self.assertRaises(ValueError):
                validate_action(value, lo, hi)
        accepted = validate_action([.2, .3], lo, hi)
        np.testing.assert_allclose(accepted, [.2, .3])

    def test_contact_without_lift_cannot_succeed(self):
        task = GraspProgress(.84)
        for _ in range(300):
            task.update(np.array([0, 0, .84]), np.zeros(3), [True, True], np.ones((4, 12, 32)), [.1, .1])
        self.assertFalse(task.held)

    def test_interrupted_hold_does_not_accumulate(self):
        task = GraspProgress(.84)
        for _ in range(239):
            task.update(np.array([0, 0, .89]), np.zeros(3), [True, True], np.ones((4, 12, 32)), [.1, .1])
        task.update(np.array([0, 0, .89]), np.zeros(3), [False, False], np.ones((4, 12, 32)), [.1, .1])
        task.update(np.array([0, 0, .89]), np.zeros(3), [True, True], np.ones((4, 12, 32)), [.1, .1])
        self.assertFalse(task.held)

    def test_success_requires_hold_and_stable_open_release(self):
        task = GraspProgress(.84)
        for _ in range(240):
            self.assertFalse(task.update(np.array([0, 0, .89]), np.zeros(3), [True, True],
                                         np.ones((4, 12, 32)), [.1, .1]))
        self.assertTrue(task.held)
        for _ in range(59):
            self.assertFalse(task.update(np.array([0, 0, .84]), np.zeros(3), [False, False],
                                         np.zeros((4, 12, 32)), [.4, .4]))
        self.assertTrue(task.update(np.array([0, 0, .84]), np.zeros(3), [False, False],
                                    np.zeros((4, 12, 32)), [.4, .4]))
