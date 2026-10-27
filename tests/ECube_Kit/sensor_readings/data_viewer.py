#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import filters

HEADER = ["t_ms", "ax", "ay", "az", "gx", "gy", "gz",
          "fax", "fay", "faz", "fgx", "fgy", "fgz",
          "q0", "q1", "q2", "q3", "roll", "pitch", "yaw", "mag", "alt"]

def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])

class Mahony:
    def __init__(self, kp_run=2.0, kp_init=20.0, init_time=2.0):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.kp_run, self.kp_init, self.init_time = kp_run, kp_init, init_time
        self.t = 0.0
        self.lock = threading.Lock()

    def update(self, acc_g, gyro_dps, dt):
        with self.lock:
            q0, q1, q2, q3 = self.q
            gx, gy, gz = np.radians(gyro_dps)
            self.t += dt

            n = np.linalg.norm(acc_g)
            if 0.7 < n < 1.3:
                ax, ay, az = acc_g / n
                vx = 2 * (q1 * q3 - q0 * q2)
                vy = 2 * (q0 * q1 + q2 * q3)
                vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
                ex = ay * vz - az * vy
                ey = az * vx - ax * vz
                ez = ax * vy - ay * vx
                kp = self.kp_init if self.t < self.init_time else self.kp_run
                gx += kp * ex
                gy += kp * ey
                gz += kp * ez

            dq = 0.5 * np.array([
                -q1 * gx - q2 * gy - q3 * gz,
                q0 * gx + q2 * gz - q3 * gy,
                q0 * gy - q1 * gz + q3 * gx,
                q0 * gz + q1 * gy - q2 * gx,
            ])
            q = self.q + dq * dt
            self.q = q / np.linalg.norm(q)

    def snapshot(self):
        with self.lock:
            return self.q.copy()

    def zero_yaw(self):
        with self.lock:
            q0, q1, q2, q3 = self.q
            yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
            h = -yaw / 2
            qz = np.array([math.cos(h), 0.0, 0.0, math.sin(h)])
            q = qmul(qz, self.q)
            self.q = q / np.linalg.norm(q)

def quat_to_matrix(q):
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
    ])

def quat_to_euler_deg(q):
    q0, q1, q2, q3 = q
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class Recorder:
    def __init__(self, folder="recordings"):
        self.folder = folder
        self.f = None
        self.w = None
        self.path = None
        self.n = 0
        self.lock = threading.Lock()

    @property
    def active(self):
        return self.f is not None

    def start(self):
        with self.lock:
            if self.f: return
            os.makedirs(self.folder, exist_ok=True)
            name = datetime.now().strftime("cubesat_%Y%m%d_%H%M%S.csv")
            self.path = os.path.join(self.folder, name)
            self.f = open(self.path, "w", newline="")
            self.w = csv.writer(self.f)
            self.w.writerow(HEADER)
            self.n = 0
            print(f"[REC] recording to {self.path}")

    def stop(self):
        with self.lock:
            if not self.f: return
            self.f.close()
            print(f"[REC] saved {self.n} samples to {self.path}")
            self.f = None
            self.w = None

    def toggle(self):
        self.stop() if self.active else self.start()

    def write(self, row):
        with self.lock:
            if not self.w: return
            self.w.writerow(row)
            self.n += 1
            if self.n % 100 == 0: self.f.flush()

class Processor:
    def __init__(self, accel_f, gyro_f, deadband_dps, kp, recorder, hist_len=1500, trail_len=300):
        self.accel_f, self.gyro_f = accel_f, gyro_f
        self.deadband = deadband_dps
        self.fusion = Mahony(kp_run=kp)
        self.recorder = recorder
        self.lock = threading.Lock()
        self.hist = deque(maxlen=hist_len)
        self.trail = deque(maxlen=trail_len)
        self.last_ms = None
        self.t0 = None
        self.mag = 0.0
        self.alt = 0.0

    def feed(self, ms, acc, gyro, mag, alt):
        if self.last_ms is None:
            self.last_ms = self.t0 = ms
            return
        dt = (ms - self.last_ms) / 1000.0
        self.last_ms = ms
        if dt <= 0 or dt > 0.5: return

        fa = self.accel_f(acc, dt)
        fg = filters.deadband(self.gyro_f(gyro, dt), self.deadband)
        self.fusion.update(fa, fg, dt)

        q = self.fusion.snapshot()
        r, p, y = quat_to_euler_deg(q)
        tip = quat_to_matrix(q)[:, 0] * 1.4
        t = (ms - self.t0) / 1000.0
        
        with self.lock:
            self.hist.append((t, r, p, y))
            self.trail.append(tip)
            self.mag = mag
            self.alt = alt

        if self.recorder.active:
            row = ([int(ms)] + [f"{v:.4f}" for v in (*acc, *gyro, *fa, *fg)] +
                   [f"{v:.6f}" for v in q] + [f"{v:.2f}" for v in (r, p, y)] +
                   [f"{mag:.2f}", f"{alt:.2f}"])
            self.recorder.write(row)

    def snapshot(self):
        with self.lock:
            return np.array(self.hist), np.array(self.trail), self.mag, self.alt

def serial_reader(ser, proc, stop):
    while not stop.is_set():
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            break
        parts = line.split(",")
        # We expect exactly 10 columns from the ESP32
        if len(parts) != 10:
            continue
        try:
            v = [float(p) for p in parts]
        except ValueError:
            continue
        proc.feed(v[0], np.array(v[1:4]), np.array(v[4:7]), v[7], v[8])

def replay_reader(path, proc, speed, stop):
    with open(path, newline="") as f:
        rows = [(float(r["t_ms"]),
                 np.array([float(r[k]) for k in ("ax", "ay", "az")]),
                 np.array([float(r[k]) for k in ("gx", "gy", "gz")]),
                 float(r["mag"]), float(r["alt"]))
                for r in csv.DictReader(f)]
    if not rows:
        print("Replay file is empty.")
        return
    start, t0 = time.perf_counter(), rows[0][0]
    for ms, a, g, mag, alt in rows:
        if stop.is_set(): return
        delay = (ms - t0) / 1000.0 / speed - (time.perf_counter() - start)
        if delay > 0: time.sleep(delay)
        proc.feed(ms, a, g, mag, alt)
    print("Replay finished.")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay", metavar="CSV")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--accel-filter", default="median,lp")
    ap.add_argument("--gyro-filter", default="none")
    ap.add_argument("--ma-n", type=int, default=5)
    ap.add_argument("--fc", type=float, default=10.0)
    ap.add_argument("--deadband", type=float, default=0.1)
    ap.add_argument("--kp", type=float, default=2.0)
    args = ap.parse_args()

    recorder = Recorder()
    proc = Processor(
        filters.build(args.accel_filter, args.ma_n, args.fc),
        filters.build(args.gyro_filter, args.ma_n, args.fc),
        args.deadband, args.kp, recorder)

    stop = threading.Event()
    ser = None
    if args.replay:
        threading.Thread(target=replay_reader, args=(args.replay, proc, args.speed, stop), daemon=True).start()
    else:
        port = args.port or (list_ports.comports()[0].device if list_ports.comports() else sys.exit("No ports."))
        print(f"Opening {port} @ {args.baud}...")
        ser = serial.Serial(port, args.baud, timeout=1)
        threading.Thread(target=serial_reader, args=(ser, proc, stop), daemon=True).start()

    if args.record: recorder.start()

    # CubeSat Dimensions (1U form factor: 1x1x1 unit)
    sx, sy, sz = 1.0, 1.0, 1.0 
    base = np.array([[x * sx, y * sy, z * sz] for z in (-1, 1) for y in (-1, 1) for x in (-1, 1)], float)
    faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    colors = ["#c0392b", "#27ae60", "#2980b9", "#2980b9", "#8e44ad", "#f39c12"]

    fig = plt.figure("CubeSat Real-Time Telemetry", figsize=(12, 6))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.5, 1.5); ax.set_zlim(-1.5, 1.5)
    ax.set_box_aspect((1, 1, 1)); ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    
    poly = Poly3DCollection([base[f] for f in faces], facecolors=colors, edgecolors="k", alpha=0.9)
    ax.add_collection3d(poly)
    axis_lines = [ax.plot([0, 0], [0, 0], [0, 0], c=c, lw=3)[0] for c in ("r", "g", "b")]
    trail_line, = ax.plot([], [], [], c="orange", lw=1, alpha=0.8)
    title = ax.set_title("")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_ylim(-190, 190); ax2.set_xlabel("time (s)"); ax2.set_ylabel("degrees")
    ax2.grid(True, alpha=0.3)
    hist_lines = [ax2.plot([], [], label=n, lw=1.2)[0] for n in ("roll", "pitch", "yaw")]
    ax2.legend(loc="upper left")

    def update(_):
        q = proc.fusion.snapshot()
        R = quat_to_matrix(q)
        verts = base @ R.T
        poly.set_verts([verts[f] for f in faces])
        
        for i, ln in enumerate(axis_lines):
            end = R[:, i] * 1.4
            ln.set_data_3d([0, end[0]], [0, end[1]], [0, end[2]])

        hist, trail, mag, alt = proc.snapshot()
        if len(trail) > 1: trail_line.set_data_3d(trail[:, 0], trail[:, 1], trail[:, 2])
        if len(hist) > 1:
            for i, ln in enumerate(hist_lines):
                ln.set_data(hist[:, 0], hist[:, i + 1])
            ax2.set_xlim(max(0.0, hist[-1, 0] - 10.0), max(10.0, hist[-1, 0]))

        r, p, y = quat_to_euler_deg(q)
        rec = "   [REC]" if recorder.active else ""
        title.set_text(f"Roll {r:6.1f}  Pitch {p:6.1f}  Yaw {y:6.1f}  |  Mag {mag:5.1f}°  Alt {alt:5.1f}m{rec}")
        title.set_color("red" if recorder.active else "black")
        return []

    def on_key(event):
        if event.key == "z": proc.fusion.zero_yaw()
        elif event.key == " ": recorder.toggle()

    fig.canvas.mpl_connect("key_press_event", on_key)
    anim = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    finally:
        stop.set()
        recorder.stop()
        if ser: ser.close()

if __name__ == "__main__":
    main()
