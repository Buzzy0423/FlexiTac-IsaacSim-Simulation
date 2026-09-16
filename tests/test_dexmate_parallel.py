"""Parallel throughput must use wall time and count only complete, accepted episodes."""

import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from benchmark_dexmate_parallel import physx_errors, summarize_workers  # noqa: E402


class ParallelSummaryTests(unittest.TestCase):
    def test_native_physics_error_invalidates_otherwise_successful_grasps(self):
        text = ("[Error] [omni.physx.plugin] PhysX error: simulation will miss interactions\n"
                "[Warning] [omni.usd] Warning in _ReportErrors\n")
        errors = {"worker_00": physx_errors(text)}
        self.assertEqual(len(errors["worker_00"]), 1)
        result = summarize_workers(self.reports(), errors)
        self.assertEqual(result["physical_passed_count"], 2)
        self.assertEqual(result["passed_count"], 0)
        self.assertEqual(result["successful_episodes_per_minute"], 0)
        self.assertIsNone(result["collection_seconds_per_successful_episode"])

    def reports(self):
        episode = {"reference_complete": True, "episode": {"passed": True},
                   "full_rate_comparison": {"tactile": {"max_abs_difference": 0}}}
        return [{"collection_started_monotonic": start, "collection_finished_monotonic": finish,
                 "episodes": [copy.deepcopy(episode)], "timings": {"env_step": {"seconds": 20}}}
                for start, finish in ((10, 40), (11, 50))]

    def test_concurrent_time_uses_slowest_worker_not_sum_or_mean(self):
        result = summarize_workers(self.reports())
        self.assertEqual(result["collection_wall_seconds"], 40)
        self.assertEqual(result["successful_episodes_per_minute"], 3)
        self.assertEqual(result["collection_seconds_per_successful_episode"], 20)
        self.assertEqual(result["start_skew_seconds"], 1)

    def test_early_exit_is_not_counted_as_complete(self):
        reports = self.reports()
        reports[1]["episodes"][0]["reference_complete"] = False
        result = summarize_workers(reports)
        self.assertEqual(result["passed_count"], 1)
        self.assertFalse(result["all_numeric_samples_identical"])

    def test_physics_success_and_exact_numeric_equality_are_separate(self):
        reports = self.reports()
        reports[1]["episodes"][0]["full_rate_comparison"]["tactile"]["max_abs_difference"] = .001
        result = summarize_workers(reports)
        self.assertEqual(result["passed_count"], 2)
        self.assertFalse(result["all_numeric_samples_identical"])
