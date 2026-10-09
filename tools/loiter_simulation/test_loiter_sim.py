import unittest

from loiter_sim import FuzzyL1Tuner, SimulationConfig, simulate, trapezoid_membership


class FuzzyL1TunerTests(unittest.TestCase):
    def test_trapezoid_membership_handles_triangle_and_shoulders(self):
        self.assertEqual(trapezoid_membership(1.0, (0.0, 1.0, 1.0, 2.0)), 1.0)
        self.assertEqual(trapezoid_membership(0.5, (0.0, 1.0, 1.0, 2.0)), 0.5)
        self.assertEqual(trapezoid_membership(0.0, (0.0, 0.0, 1.0, 2.0)), 1.0)

    def test_fuzzy_period_is_clamped_and_delta_error_is_signed(self):
        tuner = FuzzyL1Tuner(20.0, 10.0, 30.0)
        period, rate = tuner.update(100.0, 0.05)
        self.assertTrue(10.0 <= period <= 30.0)
        self.assertEqual(rate, 0.0)
        period, rate = tuner.update(101.0, 0.05)
        self.assertTrue(10.0 <= period <= 30.0)
        self.assertAlmostEqual(rate, 20.0)
        period, rate = tuner.update(100.0, 0.05)
        self.assertTrue(10.0 <= period <= 30.0)
        self.assertAlmostEqual(rate, -20.0)

    def test_rule_base_matches_proposal(self):
        self.assertEqual(
            FuzzyL1Tuner.RULES,
            ((2, 1, 0), (1, 1, 0), (1, 0, 0)),
        )


class SimulationTests(unittest.TestCase):
    def test_fixed_and_fuzzy_runs_have_finite_data_and_expected_periods(self):
        config = SimulationConfig(duration_s=20.0, dt_s=0.05)
        fixed = simulate(config, fuzzy_enabled=False, tuning_profile="balanced")
        fuzzy = simulate(config, fuzzy_enabled=True, tuning_profile="balanced")
        self.assertTrue((fixed.period_s == config.base_period_s).all())
        self.assertTrue((fuzzy.period_s >= config.min_period_s).all())
        self.assertTrue((fuzzy.period_s <= config.max_period_s).all())
        self.assertTrue(__import__("numpy").isfinite(fuzzy.radial_error_m).all())
        self.assertGreater(fixed.metrics(config.loiter_radius_m)["radial_error_rms_m"], 0.0)

    def test_firmware_profile_is_available_for_reproduction(self):
        result = simulate(SimulationConfig(duration_s=5.0), fuzzy_enabled=True,
                          tuning_profile="firmware")
        self.assertTrue((result.period_s >= 10.0).all())

    def test_lqr_controller_is_available(self):
        result = simulate(SimulationConfig(duration_s=5.0), fuzzy_enabled=False,
                          controller="lqr")
        self.assertEqual(result.name, "LQR navigasi")
        self.assertTrue(__import__("numpy").isfinite(result.radial_error_m).all())


if __name__ == "__main__":
    unittest.main()