#!/usr/bin/env python3
"""
E-Cube 3D viewer: a 1U CubeSat that moves exactly like the real board.

Reads the CSV stream from the E-Cube firmware:
    t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,pres,temp,alt     (missing sensors = nan)
and estimates attitude with the best filter the available sensors allow:
    gyro + accel + magnetometer -> Mahony MARG   (absolute heading from the compass)
    gyro + accel                -> Mahony IMU    (yaw drifts slowly; press z to zero it)
    accel (+ magnetometer)      -> tilt + compass, smoothed (no gyro on the board)

Usage:
    python ecube_viewer.py --demo                       # no hardware, synthetic tumbling cube
    python ecube_viewer.py --port /dev/ttyUSB0          # live
    python ecube_viewer.py --port COM5 --mag-map -y,x,z # if the compass axes differ from the IMU

Keys:  m  start/stop magnetometer calibration (rotate the cube through ALL orientations)
       z  zero the yaw       space  start/stop CSV recording       q  quit
"""
import argparse
import csv
import json
import math
import os
import sys
import textwrap
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

FIELDS = "t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,pres,temp,alt"


# ------------------------------------------------------------------- math
def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_to_matrix(q):
    """Body -> world rotation. World: X = magnetic north, Y = west, Z = up."""
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]])


def matrix_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def euler_deg(q):
    q0, q1, q2, q3 = q
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def axis_map(spec):
    """'-y,x,z' -> 3x3 matrix: new_x = -old_y, new_y = old_x, new_z = old_z"""
    toks = [t.strip().lower() for t in spec.split(",")]
    if len(toks) != 3:
        raise ValueError(f"bad axis map '{spec}'")
    M = np.zeros((3, 3))
    for i, t in enumerate(toks):
        sign = -1.0 if t.startswith("-") else 1.0
        ax = t.lstrip("+-")
        if ax not in ("x", "y", "z"):
            raise ValueError(f"bad axis '{t}' in '{spec}'")
        M[i, "xyz".index(ax)] = sign
    return M


# --------------------------------------------------------------- attitude
class Attitude:
    def __init__(self, kp=2.0, kp_init=20.0, init_s=2.0, use_mag=True):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.kp, self.kp_init, self.init_s = kp, kp_init, init_s
        self.use_mag = use_mag
        self.t = 0.0
        self.mode = "waiting"
        self.lock = threading.Lock()

    def update(self, a, g_dps, m, dt):
        """a in g, g_dps in deg/s, m in uT (any may be NaN)."""
        has_g = bool(np.all(np.isfinite(g_dps)))
        has_a = bool(np.all(np.isfinite(a))) and 0.7 < np.linalg.norm(a) < 1.3
        has_m = self.use_mag and m is not None and bool(np.all(np.isfinite(m))) and np.linalg.norm(m) > 1e-3
        with self.lock:
            self.t += dt
            if has_g:
                self.mode = "Mahony MARG (gyro+accel+mag)" if has_m else "Mahony IMU (gyro+accel)"
                self._mahony(a if has_a else None, g_dps, m if has_m else None, dt)
            elif bool(np.all(np.isfinite(a))):
                self.mode = "tilt + compass (no gyro)"
                self._static(a, m if has_m else None, dt)

    def _mahony(self, a, g_dps, m, dt):
        q0, q1, q2, q3 = self.q
        gx, gy, gz = np.radians(g_dps)
        ex = ey = ez = 0.0
        if a is not None:
            ax, ay, az = a / np.linalg.norm(a)
            vx = 2 * (q1 * q3 - q0 * q2)
            vy = 2 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
            ex += ay * vz - az * vy
            ey += az * vx - ax * vz
            ez += ax * vy - ay * vx
        if m is not None:
            mx, my, mz = m / np.linalg.norm(m)
            hx = 2 * (mx * (0.5 - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2))
            hy = 2 * (mx * (q1 * q2 + q0 * q3) + my * (0.5 - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1))
            bx = math.hypot(hx, hy)
            bz = 2 * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5 - q1 * q1 - q2 * q2))
            wx = 2 * (bx * (0.5 - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2))
            wy = 2 * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3))
            wz = 2 * (bx * (q0 * q2 + q1 * q3) + bz * (0.5 - q1 * q1 - q2 * q2))
            ex += my * wz - mz * wy
            ey += mz * wx - mx * wz
            ez += mx * wy - my * wx
        kp = self.kp_init if self.t < self.init_s else self.kp
        gx += kp * ex
        gy += kp * ey
        gz += kp * ez
        dq = 0.5 * np.array([-q1 * gx - q2 * gy - q3 * gz,
                             q0 * gx + q2 * gz - q3 * gy,
                             q0 * gy - q1 * gz + q3 * gx,
                             q0 * gz + q1 * gy - q2 * gx])
        q = self.q + dq * dt
        self.q = q / np.linalg.norm(q)

    def _static(self, a, m, dt):
        na = np.linalg.norm(a)
        if na < 0.3:
            return
        up = a / na
        north = None
        if m is not None:
            mh = m - np.dot(m, up) * up
            if np.linalg.norm(mh) > 1e-6:
                north = mh / np.linalg.norm(mh)
        if north is None:  # no compass: heading is arbitrary, tie it to the body X axis
            for ref in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
                mh = ref - np.dot(ref, up) * up
                if np.linalg.norm(mh) > 1e-6:
                    north = mh / np.linalg.norm(mh)
                    break
        west = np.cross(up, north)
        qt = matrix_to_quat(np.vstack([north, west, up]))
        if np.dot(qt, self.q) < 0:
            qt = -qt
        alpha = 1.0 - math.exp(-dt / 0.15)
        q = (1 - alpha) * self.q + alpha * qt
        self.q = q / np.linalg.norm(q)

    def snapshot(self):
        with self.lock:
            return self.q.copy()

    def zero_yaw(self):
        with self.lock:
            yaw = euler_deg(self.q)[2]
            h = -math.radians(yaw) / 2
            q = qmul(np.array([math.cos(h), 0, 0, math.sin(h)]), self.q)
            self.q = q / np.linalg.norm(q)


# -------------------------------------------------------- mag calibration
class MagCal:
    """Hard-iron offset + soft-iron axis scale from a min/max sweep. Saved to JSON."""

    def __init__(self, path, map_spec):
        self.path, self.map_spec = path, map_spec
        self.offset = np.zeros(3)
        self.scale = np.ones(3)
        self.calibrated = False
        self.collecting = False
        self.message = "mag not calibrated (press m)"
        self.mn = np.full(3, np.inf)
        self.mx = np.full(3, -np.inf)
        self._load()

    def _load(self):
        try:
            with open(self.path) as f:
                d = json.load(f)
            if d.get("map") == self.map_spec:
                self.offset = np.array(d["offset"])
                self.scale = np.array(d["scale"])
                self.calibrated = True
                self.message = f"mag calibration loaded ({self.path})"
        except (OSError, ValueError, KeyError):
            pass

    def start(self):
        self.collecting = True
        self.mn = np.full(3, np.inf)
        self.mx = np.full(3, -np.inf)
        self.message = "CALIBRATING: rotate the cube through all orientations, press m again"

    def feed(self, m):
        if np.all(np.isfinite(m)):
            self.mn = np.minimum(self.mn, m)
            self.mx = np.maximum(self.mx, m)

    def finish(self):
        self.collecting = False
        span = self.mx - self.mn
        if not np.all(np.isfinite(span)) or np.any(span < 20.0):
            self.message = "calibration failed: rotate through ALL orientations (each axis needs >20 uT range)"
            return False
        self.offset = (self.mx + self.mn) / 2
        r = span / 2
        self.scale = r.mean() / r
        self.calibrated = True
        with open(self.path, "w") as f:
            json.dump({"map": self.map_spec, "offset": self.offset.tolist(),
                       "scale": self.scale.tolist()}, f, indent=1)
        self.message = f"calibrated, saved to {self.path}"
        return True

    def toggle(self):
        self.finish() if self.collecting else self.start()

    def apply(self, m):
        return (m - self.offset) * self.scale


# ------------------------------------------------------------ processing
class Processor:
    def __init__(self, args):
        self.att = Attitude(kp=args.kp, use_mag=not args.no_mag)
        self.magcal = MagCal(args.mag_cal, args.mag_map)
        self.imu_map = axis_map(args.imu_map)
        self.mag_map = axis_map(args.mag_map)
        self.lock = threading.Lock()
        self.tel = {"pres": np.nan, "temp": np.nan, "alt": np.nan, "a": np.nan, "m": np.nan}
        self.info = ""
        self.status = "starting"
        self.last_ms = None
        self.arrivals = deque(maxlen=200)
        self.rec = None
        self.rec_writer = None
        self.rec_path = None
        self.rec_n = 0
        self.true_q = None  # filled by demo mode (for testing)

    def rate(self):
        if len(self.arrivals) < 2 or time.perf_counter() - self.arrivals[-1] > 1.0:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    def feed(self, rec):
        ms = rec["t_ms"]
        if self.last_ms is None or ms < self.last_ms:
            self.last_ms = ms
            return
        dt = (ms - self.last_ms) / 1000.0
        self.last_ms = ms
        if not 0 < dt < 0.5:
            return

        v = lambda *k: np.array([rec.get(x, np.nan) for x in k], float)
        a = self.imu_map @ v("ax", "ay", "az")
        g = self.imu_map @ v("gx", "gy", "gz")
        m = v("mx", "my", "mz")
        if np.all(np.isfinite(m)):
            m = self.mag_map @ m
            if self.magcal.collecting:
                self.magcal.feed(m)
            m = self.magcal.apply(m)
        self.att.update(a, g, m, dt)

        q = self.att.snapshot()
        r, p, y = euler_deg(q)
        with self.lock:
            self.tel.update(pres=rec.get("pres", np.nan), temp=rec.get("temp", np.nan),
                            alt=rec.get("alt", np.nan), a=float(np.linalg.norm(a)),
                            m=float(np.linalg.norm(m)) if np.all(np.isfinite(m)) else np.nan)
        self.arrivals.append(time.perf_counter())
        if self.rec_writer:
            self.rec_writer.writerow([f"{rec.get(k, np.nan):.5g}" for k in FIELDS.split(",")]
                                     + [f"{x:.6f}" for x in q] + [f"{r:.2f}", f"{p:.2f}", f"{y:.2f}"])
            self.rec_n += 1

    def toggle_record(self):
        if self.rec:
            self.rec.close()
            print(f"[REC] saved {self.rec_n} samples to {self.rec_path}")
            self.rec = self.rec_writer = None
        else:
            os.makedirs("recordings", exist_ok=True)
            self.rec_path = os.path.join("recordings", datetime.now().strftime("ecube_%Y%m%d_%H%M%S.csv"))
            self.rec = open(self.rec_path, "w", newline="")
            self.rec_writer = csv.writer(self.rec)
            self.rec_writer.writerow(FIELDS.split(",") + ["q0", "q1", "q2", "q3", "roll", "pitch", "yaw"])
            self.rec_n = 0
            print(f"[REC] recording to {self.rec_path}")


# ----------------------------------------------------------------- sources
def serial_reader(proc, port, baud, fields, stop):
    import serial
    names = [f.strip() for f in fields.split(",")]
    while not stop.is_set():
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
        except (serial.SerialException, OSError) as e:
            proc.status = f"waiting for {port}: {e}"
            stop.wait(1.0)
            continue
        proc.status = f"connected {port}@{baud}"
        proc.last_ms = None
        try:
            while not stop.is_set():
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("ascii", errors="ignore").strip()
                if text.startswith("#"):
                    t = text.lstrip("# ").strip()
                    if t.startswith("found"):
                        proc.info = t.replace("found ;", "").strip(";").replace(";", "  ")
                    continue
                parts = text.split(",")
                if len(parts) != len(names):
                    continue
                try:
                    proc.feed(dict(zip(names, (float(p) for p in parts))))
                except ValueError:
                    continue
        except (serial.SerialException, OSError):
            proc.status = "disconnected, retrying..."
        finally:
            ser.close()


def demo_source(proc, stop, rate=100.0):
    """Synthetic tumbling cube with gravity, earth field (+ hard-iron) and pressure."""
    rng = np.random.default_rng(3)
    q = np.array([1.0, 0, 0, 0])
    m_earth = np.array([20.0, 0.0, -40.0])      # north 20 uT, down 40 uT (world X north, Z up)
    hard_iron = np.array([8.0, -5.0, 12.0])
    period, t0, n = 1.0 / rate, time.perf_counter(), 0
    proc.status = "demo (synthetic data)"
    proc.info = "DEMO: MPU6050  QMC5883L  BMP180 (simulated)"
    while not stop.is_set():
        t = n * period
        wait = t0 + t - time.perf_counter()
        if wait > 0 and stop.wait(wait):
            return
        w_dps = np.array([30 * math.sin(0.5 * t), 20 * math.cos(0.35 * t), 25 * math.sin(0.2 * t + 1)])
        w = np.radians(w_dps)
        q = q + 0.5 * qmul(q, np.array([0, *w])) * period
        q /= np.linalg.norm(q)
        R = quat_to_matrix(q)
        a = R.T @ np.array([0, 0, 1.0]) + rng.normal(0, 0.01, 3)
        m = R.T @ m_earth + hard_iron + rng.normal(0, 0.3, 3)
        g = w_dps + rng.normal(0, 0.3, 3)
        proc.true_q = q.copy()
        proc.feed({"t_ms": t * 1000, "ax": a[0], "ay": a[1], "az": a[2], "gx": g[0], "gy": g[1],
                   "gz": g[2], "mx": m[0], "my": m[1], "mz": m[2],
                   "pres": 1013.25 - 0.1 * math.sin(0.1 * t), "temp": 25.0 + 0.2 * math.sin(0.05 * t),
                   "alt": 0.8 * math.sin(0.1 * t)})
        n += 1


# --------------------------------------------------------------- 3D model
def build_model():
    """1U CubeSat: aluminium frame faces, 2x2 solar cells on the four sides, PCB-green top."""
    h = 0.5
    faces = {
        "+x": [(h, -h, -h), (h, h, -h), (h, h, h), (h, -h, h)],
        "-x": [(-h, -h, -h), (-h, h, -h), (-h, h, h), (-h, -h, h)],
        "+y": [(-h, h, -h), (h, h, -h), (h, h, h), (-h, h, h)],
        "-y": [(-h, -h, -h), (h, -h, -h), (h, -h, h), (-h, -h, h)],
        "+z": [(-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h)],
        "-z": [(-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h)]}
    body_col = {"+x": "#c9ced6", "-x": "#c9ced6", "+y": "#c9ced6", "-y": "#c9ced6",
                "+z": "#2e8b57", "-z": "#4a4f57"}
    polys, cols, edges = [], [], []
    for k, p in faces.items():
        polys.append(np.array(p, float))
        cols.append(body_col[k])
        edges.append("#333333")
    m, gap, off = 0.07, 0.02, h + 0.004
    cell = (2 * h - 2 * m) / 2
    for axis in (0, 1):                      # panels on +-X and +-Y faces
        for s in (-1, 1):
            for i in range(2):
                for j in range(2):
                    u0 = -h + m + i * cell + gap / 2
                    v0 = -h + m + j * cell + gap / 2
                    u1, v1 = u0 + cell - gap, v0 + cell - gap
                    quad = []
                    for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1)):
                        pt = [0.0, 0.0, v]
                        pt[axis] = s * off
                        pt[1 - axis] = u
                        quad.append(pt)
                    polys.append(np.array(quad, float))
                    cols.append("#1f3a93")
                    edges.append("#8fa6ff")
    return np.array(polys), cols, edges


class Viewer:
    def __init__(self, proc, window_title="E-Cube viewer", demo=False):
        self.proc = proc
        self.demo = demo
        self.P, cols, edges = build_model()
        self.fig = plt.figure(window_title, figsize=(12, 7))
        gs = self.fig.add_gridspec(1, 2, width_ratios=[3, 1.25])
        self.ax = self.fig.add_subplot(gs[0], projection="3d")
        self.ax.set_xlim(-1.3, 1.3)
        self.ax.set_ylim(-1.3, 1.3)
        self.ax.set_zlim(-1.3, 1.3)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.set_axis_off()
        self.ax.view_init(elev=22, azim=-58)
        self.poly = Poly3DCollection(list(self.P), facecolors=cols, edgecolors=edges, linewidths=0.8)
        self.ax.add_collection3d(self.poly)

        # fixed world reference: north arrow on the "floor"
        self.ax.plot([0, 1.1], [0, 0], [-1.25, -1.25], c="k", lw=2)
        self.ax.text(1.2, 0, -1.25, "N", fontsize=11, weight="bold")
        self.ax.plot([-1.1, 1.1], [0, 0], [-1.25, -1.25], c="0.8", lw=0.8)
        self.ax.plot([0, 0], [-1.1, 1.1], [-1.25, -1.25], c="0.8", lw=0.8)

        self.antenna, = self.ax.plot([0, 0], [0, 0], [0.5, 1.2], c="#222", lw=2, marker="o", markevery=[1])
        self.arrows, self.labels = [], []
        for c, n in (("r", "X"), ("g", "Y"), ("b", "Z")):
            ln, = self.ax.plot([0, 0], [0, 0], [0, 0], c=c, lw=2.5)
            self.arrows.append(ln)
            self.labels.append(self.ax.text(0, 0, 0, n, color=c, fontsize=10, weight="bold"))

        self.panel = self.fig.add_subplot(gs[1])
        self.panel.axis("off")
        self.text = self.panel.text(0.0, 1.0, "", va="top", family="monospace", fontsize=9,
                                    transform=self.panel.transAxes)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.subplots_adjust(left=0.01, right=0.99, top=0.97, bottom=0.03)

    def _on_key(self, e):
        if e.key == "m":
            self.proc.magcal.toggle()
        elif e.key == "z":
            self.proc.att.zero_yaw()
        elif e.key == " ":
            self.proc.toggle_record()

    def update(self, _=None):
        p = self.proc
        q = p.att.snapshot()
        R = quat_to_matrix(q)
        self.poly.set_verts(list(self.P @ R.T))
        top = R @ np.array([0, 0, 0.5])
        tip = R @ np.array([0, 0, 1.2])
        self.antenna.set_data_3d([top[0], tip[0]], [top[1], tip[1]], [top[2], tip[2]])
        for i, (ln, lb) in enumerate(zip(self.arrows, self.labels)):
            e = R[:, i] * 1.0
            ln.set_data_3d([0, e[0]], [0, e[1]], [0, e[2]])
            lb.set_position((e[0] * 1.1, e[1] * 1.1))
            lb.set_3d_properties(e[2] * 1.1)

        r, pi, y = euler_deg(q)
        with p.lock:
            t = dict(p.tel)
        mag_used = "MARG" in p.att.mode
        heading = f"{(-y) % 360:6.1f} deg" if mag_used else "  n/a (no compass)"
        mc = p.magcal
        fmt = lambda x, f: "   n/a" if not np.isfinite(x) else f.format(x)
        wrap = lambda txt, ind="": textwrap.fill(txt, 40, subsequent_indent=ind)
        lines = [
            "E-CUBE" + ("  [DEMO]" if self.demo else ""),
            wrap(f"sensors : {p.info or 'detecting...'}", " " * 10),
            wrap(f"link    : {p.status}", " " * 10),
            f"rate    : {p.rate():5.1f} Hz",
            f"filter  : {p.att.mode}",
            "",
            f"roll    {r:+7.1f} deg",
            f"pitch   {pi:+7.1f} deg",
            f"yaw     {y:+7.1f} deg",
            f"heading {heading}",
            "",
            f"|accel| {fmt(t['a'], '{:6.2f} g')}",
            f"|B|     {fmt(t['m'], '{:6.1f} uT')}",
            f"press   {fmt(t['pres'], '{:7.1f} hPa')}",
            f"temp    {fmt(t['temp'], '{:6.1f} C')}",
            f"rel alt {fmt(t['alt'], '{:+6.1f} m')}",
            "",
            wrap(mc.message),
            wrap(f"REC {p.rec_path}") if p.rec else "",
            "",
            "m mag-cal | z zero yaw",
            "space record | q quit",
        ]
        self.text.set_text("\n".join(lines))
        return []

    def run(self):
        self.anim = FuncAnimation(self.fig, self.update, interval=40, blit=False, cache_frame_data=False)
        plt.show()


# -------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description="E-Cube 3D CubeSat viewer")
    ap.add_argument("--port", help="serial port (default: first one found)")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--fields", default=FIELDS)
    ap.add_argument("--demo", action="store_true", help="synthetic data, no hardware")
    ap.add_argument("--no-mag", action="store_true", help="ignore the magnetometer")
    ap.add_argument("--kp", type=float, default=2.0, help="filter correction gain")
    ap.add_argument("--imu-map", default="x,y,z", help="remap accel+gyro axes, e.g. -y,x,z")
    ap.add_argument("--mag-map", default="x,y,z", help="remap magnetometer axes, e.g. -y,x,z")
    ap.add_argument("--mag-cal", default="mag_cal.json", help="magnetometer calibration file")
    ap.add_argument("--record", action="store_true", help="start recording immediately")
    args = ap.parse_args(argv)

    proc = Processor(args)
    stop = threading.Event()
    if args.demo:
        threading.Thread(target=demo_source, args=(proc, stop), daemon=True).start()
    else:
        port = args.port
        if not port:
            from serial.tools import list_ports
            ports = list_ports.comports()
            if not ports:
                sys.exit("No serial port found. Plug in the E-Cube, pass --port, or try --demo.")
            port = ports[0].device
        threading.Thread(target=serial_reader, args=(proc, port, args.baud, args.fields, stop),
                         daemon=True).start()
    if args.record:
        proc.toggle_record()
    viewer = Viewer(proc, demo=args.demo)
    try:
        viewer.run()
    finally:
        stop.set()
        if proc.rec:
            proc.toggle_record()


if __name__ == "__main__":
    main()
