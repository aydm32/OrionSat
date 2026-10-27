"""Small signal filters for the IMU pipeline. All work on numpy vectors (3 axes at once).

Each filter is called as  y = f(x, dt)  with dt in seconds.

  ma      moving average        (FIR, boxcar of n samples)
  median  running median        (non-linear, removes spikes, keeps edges)
  lp      1st-order low-pass    (IIR, cutoff fc in Hz)

Stages can be chained, e.g. "median,lp".
"""
import math
from collections import deque

import numpy as np


class MovingAverage:
    def __init__(self, n=5):
        self.buf = deque(maxlen=max(1, n))

    def __call__(self, x, dt):
        self.buf.append(np.asarray(x, float))
        return np.mean(self.buf, axis=0)


class Median:
    def __init__(self, n=5):
        self.buf = deque(maxlen=max(1, n))

    def __call__(self, x, dt):
        self.buf.append(np.asarray(x, float))
        return np.median(self.buf, axis=0)


class LowPass:
    """y[k] = y[k-1] + alpha * (x[k] - y[k-1]),  alpha = dt / (dt + 1/(2*pi*fc))"""

    def __init__(self, fc=10.0):
        self.fc = fc
        self.y = None

    def __call__(self, x, dt):
        x = np.asarray(x, float)
        if self.y is None:
            self.y = x.copy()
            return self.y
        alpha = dt / (dt + 1.0 / (2.0 * math.pi * self.fc))
        self.y = self.y + alpha * (x - self.y)
        return self.y


class Chain:
    def __init__(self, stages):
        self.stages = stages

    def __call__(self, x, dt):
        for s in self.stages:
            x = s(x, dt)
        return np.asarray(x, float)


def build(spec, n=5, fc=10.0):
    """spec: 'none' or comma list of ma / median / lp"""
    stages = []
    for name in (s.strip().lower() for s in spec.split(",")):
        if name in ("", "none"):
            continue
        if name == "ma":
            stages.append(MovingAverage(n))
        elif name == "median":
            stages.append(Median(n))
        elif name == "lp":
            stages.append(LowPass(fc))
        else:
            raise ValueError(f"unknown filter '{name}' (use none, ma, median, lp)")
    return Chain(stages)


def deadband(v, thr):
    """Zero out values smaller than thr (kills residual gyro noise at rest)."""
    v = np.asarray(v, float)
    return np.where(np.abs(v) < thr, 0.0, v)
