"""Time-driven longitudinal references; never adjust phase to the flown position."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicHermiteSpline


class Mission:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.config = json.loads(self.path.read_text())
        c = self.config
        self.kind = c['kind']
        if self.kind not in ('steady', 'periodic'):
            raise ValueError('Mission kind must be steady or periodic')
        source = self.path.parent / c['source_csv']
        if hashlib.sha256(source.read_bytes()).hexdigest() != c['source_sha256']:
            raise ValueError('Source CSV changed after mission preparation')
        self.data = read_period(source)
        self.period = float(self.data['time_s'][-1])
        self.distance = float(self.data['X_m'][-1])
        self.speed = self.distance / self.period
        self.periods = int(c['periods'])
        self.warmup = int(c['warmup_periods'])
        if self.periods < 1 or self.warmup < 0:
            raise ValueError('Require positive scored periods and nonnegative warmup')
        self.duration = (self.periods + self.warmup) * self.period
        self.score_start = self.warmup * self.period
        self.altitude = float(self.data['Z_m'][0])
        t, v, gamma = self.data['time_s'], self.data['V_mps'], np.deg2rad(self.data['gamma_deg'])
        self.xcurve = CubicHermiteSpline(t, self.data['X_m'], v*np.cos(gamma))
        self.zcurve = CubicHermiteSpline(t, self.data['Z_m'], v*np.sin(gamma))
        self.control_mode = c.get('control_mode', 'position')
        if self.control_mode not in ('position', 'attitude_thrust'):
            raise ValueError('control_mode must be position or attitude_thrust')

    def longitudinal(self, seconds):
        t = max(0.0, float(seconds))
        if self.kind == 'steady':
            return np.array([self.speed*t, self.altitude, self.speed, 0., 0., 0., self.speed])
        cycle = math.floor(t/self.period)
        phase = t-cycle*self.period
        return np.array([cycle*self.distance+float(self.xcurve(phase)),
                         float(self.zcurve(phase)), float(self.xcurve(phase, 1)),
                         float(self.zcurve(phase, 1)), float(self.xcurve(phase, 2)),
                         float(self.zcurve(phase, 2)),
                         float(np.interp(phase, self.data['time_s'], self.data['V_mps']))])

    def sample(self, seconds, entry, origin_down):
        x, z, vx, vz, ax, az, speed = self.longitudinal(seconds)
        n, e, heading = entry
        co, si = math.cos(heading), math.sin(heading)
        return (n+x*co, e+x*si, origin_down-z, vx*co, vx*si, -vz,
                ax*co, ax*si, -az), speed

    def controls(self, seconds, aircraft):
        if self.kind == 'steady':
            v = self.speed
            q = .5*aircraft['rho']*v*v
            cl = aircraft['m']*aircraft['g']/(q*aircraft['S'])
            alpha = (cl-aircraft['CL0'])/aircraft['CLa']
            drag = q*aircraft['S']*(aircraft['CD0']+cl*cl/(math.pi*aircraft['e']*aircraft['AR']))
            return alpha, drag, 0., v
        phase = max(0., float(seconds)) % self.period
        t = self.data['time_s']
        return tuple(float(np.interp(phase, t, self.data[name])) for name in
                     ('alpha_rad', 'thrust_N', 'gamma_deg', 'V_mps'))


def read_period(path):
    data = np.genfromtxt(path, delimiter=',', names=True, encoding='utf-8-sig')
    required = ('time_s', 'X_m', 'Z_m', 'gamma_deg', 'V_mps')
    if data.ndim != 1 or len(data) < 4 or not set(required).issubset(data.dtype.names or ()):
        raise ValueError('CSV requires time_s,X_m,Z_m,gamma_deg,V_mps and at least four rows')
    if any(not np.isfinite(data[k]).all() for k in required):
        raise ValueError('Nonfinite trajectory data')
    data['time_s'] -= data['time_s'][0]
    data['X_m'] -= data['X_m'][0]
    if np.any(np.diff(data['time_s']) <= 0) or np.any(np.diff(data['X_m']) <= 0):
        raise ValueError('Time and forward distance must increase strictly')
    for name, tolerance in [('Z_m', .02), ('V_mps', .02), ('gamma_deg', .02)]:
        if abs(data[name][-1]-data[name][0]) > tolerance:
            raise ValueError(f'Nonperiodic endpoint: {name}')
    if data['Z_m'].min() < 10 or data['V_mps'].min() < 12.3 or data['V_mps'].max() > 28:
        raise ValueError('Trajectory outside configured altitude/airspeed bounds')
    dt = np.diff(data['time_s'])
    gamma = np.deg2rad(data['gamma_deg'])
    for coordinate, velocity in [('X_m', data['V_mps']*np.cos(gamma)),
                                 ('Z_m', data['V_mps']*np.sin(gamma))]:
        residual = np.diff(data[coordinate])/dt - .5*(velocity[1:]+velocity[:-1])
        if np.max(np.abs(residual)) > .25:
            raise ValueError(f'{coordinate} inconsistent with V/gamma or sampling too coarse')
    return data
