"""Checks for period boundaries, frame conversion and unbiased energy integration."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from mission_profile import Mission, read_period
from prepare_missions import prepare
from analyze_energy import integral


def write_test_wave(path, period=40, amplitude=2):
    t = np.linspace(0, period, 801)
    w = 2*np.pi/period
    vx = 13.67+.25*np.sin(w*t)
    x = 13.67*t+.25/w*(1-np.cos(w*t))
    z = 130+amplitude*(1-np.cos(w*t))
    vz = amplitude*w*np.sin(w*t)
    v = np.hypot(vx, vz)
    gamma = np.rad2deg(np.arctan2(vz, vx))
    with Path(path).open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s','X_m','Z_m','gamma_deg','V_mps'])
        writer.writerows(zip(t,x,z,gamma,v))


class EnergyTests(unittest.TestCase):
    def test_five_periods_and_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_test_wave(root/'wave.csv')
            prepare(root/'wave.csv', root/'pair', periods=5, warmup=1)
            periodic, steady = [Mission(root/'pair'/f'{k}.json') for k in ('periodic','steady')]
            self.assertEqual(periodic.duration, 240)
            self.assertEqual(periodic.duration-periodic.score_start, 200)
            for boundary in (40,80,120,160,200):
                a, b = periodic.longitudinal(boundary-1e-7), periodic.longitudinal(boundary+1e-7)
                np.testing.assert_allclose(a[:4], b[:4], atol=1e-5)
            self.assertAlmostEqual(periodic.longitudinal(240)[0], steady.longitudinal(240)[0])
            sp, _ = periodic.sample(0, (10,20,np.pi/2), -.1)
            np.testing.assert_allclose(sp[:6], [10,20,-130.1,0,13.67,0], atol=1e-8)
            # A discontinuous cycle must fail instead of teleporting the reference.
            with (root/'wave.csv').open() as f:
                rows = list(csv.reader(f))
            rows[-1][2] = '135'
            with (root/'bad.csv').open('w', newline='') as f: csv.writer(f).writerows(rows)
            with self.assertRaisesRegex(ValueError, 'Nonperiodic'):
                read_period(root/'bad.csv')

    def test_irregular_samples_and_window_clipping(self):
        t = np.array([0,.1,.6,1.2,2.7,3.])
        self.assertAlmostEqual(integral(t, t*10+20, .3, 2.3), 66.)
        self.assertAlmostEqual(integral(t, np.ones(len(t))*60, 0, 3), 180.)
        with self.assertRaises(ValueError): integral(t, t, 0, 4)


if __name__ == '__main__':
    unittest.main()
