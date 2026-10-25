#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证离线时序模型的解析样条、命令限幅和所选相位的走停差异。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from validate_command_horizon import sample, simulate


class HorizonTests(unittest.TestCase):
    def test_constant_velocity_cubic_and_endpoint_hold(self):
        segment = (0,100,0.,.1,.01,.1)
        p,v = sample(segment,50)
        self.assertAlmostEqual(.005,p)
        self.assertAlmostEqual(.1,v)
        self.assertEqual((.01,0.),sample(segment,101))

    def test_command_acceleration_is_not_interpolation_acceleration(self):
        short,_ = simulate(50)
        self.assertLessEqual(short['max_qp_like_command_acceleration_rad_s2'],.2512+1e-12)
        self.assertGreater(short['cruise_peak_fd_acceleration_rad_s2'],1.)
        self.assertGreater(short['cruise_zero_fraction'],.5)

    def test_longer_horizon_only_covers_nominal_callback_interval(self):
        long,_ = simulate(140)
        delayed,_ = simulate(140,spin_ms=200)
        self.assertEqual(0.,long['cruise_zero_fraction'])
        self.assertGreater(delayed['cruise_zero_fraction'],.1)

    def test_interruption_results_remain_measured_model_outputs(self):
        results = {}
        for h in (50,140):
            result,trace = simulate(h,cutoff_ms=4000)
            results[h] = result
            self.assertEqual(0.,trace[-1]['velocity_rad_s'])
        # 选定相位下，短轨迹停发时已处于等待区，剩余旧点也可能直接过期。
        self.assertEqual(0.,results[50]['max_displacement_after_cutoff_rad'])
        self.assertGreater(results[140]['max_displacement_after_cutoff_rad'],0.)
        self.assertGreater(results[140]['full_peak_fd_acceleration_rad_s2'],.2512)


if __name__ == '__main__':
    unittest.main()
