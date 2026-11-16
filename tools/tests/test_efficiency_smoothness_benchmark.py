"""Comfort regression tests; run without CARLA using unittest discovery."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location(
    "efficiency_smoothness_benchmark",
    Path(__file__).resolve().parents[1] / "efficiency_smoothness_benchmark.py",
)
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def metric_inputs(yaw_rates):
    """Stationary translation with angular velocity supplied in rad/s."""
    n = len(yaw_rates)
    angular_velocity = np.zeros((n, 3))
    angular_velocity[:, 2] = yaw_rates
    return dict(
        acceleration=np.zeros((n, 3)),
        angular_velocity=angular_velocity,
        forward_vector=np.tile([1.0, 0.0, 0.0], (n, 1)),
        right_vector=np.tile([0.0, 1.0, 0.0], (n, 1)),
        location=np.zeros((n, 3)),
        rotation=np.zeros((n, 3)),
    )


class ComfortYawTests(unittest.TestCase):
    def test_constant_yaw_rate_has_zero_yaw_acceleration(self):
        for rate in (0.0, -0.6, 0.6):
            with self.subTest(rate=rate):
                self.assertTrue(benchmark.compute_comfort_metric(
                    **metric_inputs(np.full(20, rate))))

    def test_rapid_change_fails_below_yaw_rate_limit(self):
        rates = np.r_[np.full(10, -0.6), np.full(10, 0.6)]
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.assertFalse(benchmark.compute_comfort_metric(
                    **metric_inputs(sign * rates), time_interval=0.1))

    def test_yaw_acceleration_scales_with_sampling_interval(self):
        data = metric_inputs(np.r_[np.full(10, -0.6), np.full(10, 0.6)])
        self.assertFalse(benchmark.compute_comfort_metric(**data, time_interval=0.1))
        self.assertTrue(benchmark.compute_comfort_metric(**data, time_interval=0.2))

    def test_constant_rate_above_rate_limit_still_fails(self):
        self.assertFalse(benchmark.compute_comfort_metric(
            **metric_inputs(np.full(20, 1.0))))

    def test_segmented_metric_propagates_sampling_interval(self):
        rates = np.r_[np.full(10, -0.6), np.full(10, 0.6)]
        for repeats in (1, 2):
            with self.subTest(repeats=repeats):
                data = metric_inputs(np.tile(rates, repeats))
                self.assertEqual(benchmark.seg_compute_comfort_metric(
                    **data, time_interval=0.1), 0.0)
                self.assertEqual(benchmark.seg_compute_comfort_metric(
                    **data, time_interval=0.2), 1.0)

    def test_carla_metric_json_converts_degrees_per_second(self):
        data = metric_inputs(np.full(20, 0.5))
        # AutonomousAgent.get_metric_info stores Actor.get_angular_velocity,
        # whose units are degrees/s (unlike the IMU sensor's rad/s).
        data["angular_velocity"] = np.degrees(data["angular_velocity"])
        frames = {str(i): {k: v[i].tolist() for k, v in data.items()}
                  for i in range(20)}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            route = root / "route"
            route.mkdir()
            (route / "metric_info.json").write_text(json.dumps(frames))
            results = root / "results.json"
            results.write_text(json.dumps({"_checkpoint": {"records": [{
                "save_name": "route", "infractions": {"min_speed_infractions": []}
            }]}}))
            records, _ = benchmark.read_from_json(str(results), str(root))
        np.testing.assert_allclose(records[0]["angular_velocity"][:, 2], 0.5)
        self.assertTrue(benchmark.seg_compute_comfort_metric(**records[0]))


if __name__ == "__main__":
    unittest.main()
