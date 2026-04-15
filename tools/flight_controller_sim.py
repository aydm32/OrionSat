#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════╗
║         FLIGHT CONTROLLER SIMULATOR  v1.0            ║
║  Real-time 3D visualization · Bidirectional Control  ║
╠══════════════════════════════════════════════════════╣
║  MOUSE (in 3D viewport)                              ║
║    Left  drag  →  Yaw (←→)  +  Pitch (↑↓)          ║
║    Right drag  →  Roll (←→)  +  Altitude (↑↓)       ║
║    Middle drag →  Translate X / Y                    ║
║    Scroll      →  Zoom view                          ║
║                                                      ║
║  KEYBOARD                                            ║
║    R  →  Reset drone to origin                       ║
║    V  →  Reset camera angle                          ║
╚══════════════════════════════════════════════════════╝

Dependencies: pip install matplotlib numpy
"""

import tkinter as tk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# ── Color Palette ─────────────────────────────────────────────────────────────
BG     = '#0d1117'
BG2    = '#161b22'
BORDER = '#30363d'
TEXT   = '#e6edf3'
MUTED  = '#8b949e'
BLUE   = '#58a6ff'
RED    = '#ff7675'
GREEN  = '#55efc4'
ORANGE = '#ffa657'
YELLOW = '#e3b341'
PURPLE = '#bc8cff'
CYAN   = '#79c0ff'


class FlightControllerSim:

    # ── DOF limits ────────────────────────────────────────────────────────────
    LIMITS = {
        'x':     (-10.0,  10.0),
        'y':     (-10.0,  10.0),
        'z':     (-10.0,  10.0),
        'roll':  (-180.0, 180.0),
        'pitch': ( -90.0,  90.0),
        'yaw':   (-180.0, 180.0),
    }

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("✈  Flight Controller Simulator  v1.0")
        self.root.configure(bg=BG)
        self.root.geometry('1350x820')
        self.root.minsize(950, 620)

        # ── Internal state (single source of truth) ───────────────────────────
        self.pos = [0.0, 0.0, 0.0]   # X, Y, Z  (meters)
        self.rot = [0.0, 0.0, 0.0]   # Roll, Pitch, Yaw  (degrees)

        # Camera (for orbit-style view control)
        self._cam_elev = 22.0
        self._cam_azim = -55.0

        # ── Widget maps ───────────────────────────────────────────────────────
        self.sliders:  dict[str, tk.Scale]     = {}
        self.entries:  dict[str, tk.Entry]     = {}
        self.str_vars: dict[str, tk.StringVar] = {
            k: tk.StringVar(value='0.00') for k in self.LIMITS
        }
        self.telem_labels: dict[str, tk.Label] = {}

        # ── Mouse drag state ──────────────────────────────────────────────────
        self._mx = self._my = None
        self._btn = None

        # Guard against recursive updates
        self._busy = False

        self._build_ui()
        self._redraw()

    # ═════════════════════════════════════════════════════════════════════════
    #  Rotation math
    # ═════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _rot_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        """ZYX Euler angles → 3×3 rotation matrix (aerospace NED convention)."""
        r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        Rx = np.array([[1,  0,   0 ], [0,  cr, -sr], [0,  sr, cr]])
        Ry = np.array([[cp, 0,   sp], [0,  1,   0 ], [-sp, 0, cp]])
        Rz = np.array([[cy,-sy,  0 ], [sy, cy,  0 ], [0,  0,  1 ]])
        return Rz @ Ry @ Rx

    # ═════════════════════════════════════════════════════════════════════════
    #  UI construction
    # ═════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        # Left control panel
        left = tk.Frame(self.root, bg=BG, width=320)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(12, 5), pady=12)
        left.pack_propagate(False)

        # Right 3-D viewport
        right = tk.Frame(self.root, bg=BG)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 12), pady=12)

        self._build_left_panel(left)
        self._build_3d_panel(right)

        # Keyboard shortcuts
        self.root.bind('<r>', lambda _: self._reset_drone())
        self.root.bind('<R>', lambda _: self._reset_drone())
        self.root.bind('<v>', lambda _: self._reset_camera())
        self.root.bind('<V>', lambda _: self._reset_camera())

    # ── Left panel ────────────────────────────────────────────────────────────

    def _build_left_panel(self, parent: tk.Frame):
        # Header
        tk.Label(parent, text='✈  FLIGHT CONTROLLER', bg=BG, fg=BLUE,
                 font=('Consolas', 14, 'bold')).pack(pady=(12, 2))
        tk.Label(parent, text='SIMULATOR  v 1.0', bg=BG, fg=MUTED,
                 font=('Consolas', 9)).pack()
        self._hr(parent, top=10)

        # Position section
        self._control_section(parent, '⊕  POSITION', [
            ('X', 'x',    'm',  RED),
            ('Y', 'y',    'm',  GREEN),
            ('Z', 'z',    'm',  BLUE),
        ])
        self._hr(parent)

        # Rotation section
        self._control_section(parent, '↻  ROTATION', [
            ('Roll',  'roll',  '°', RED),
            ('Pitch', 'pitch', '°', GREEN),
            ('Yaw',   'yaw',   '°', BLUE),
        ])
        self._hr(parent)

        # Buttons row
        btn_frame = tk.Frame(parent, bg=BG)
        btn_frame.pack(fill=tk.X, padx=14, pady=6)
        self._create_button(btn_frame, '⟳ RESET DRONE', RED,    self._reset_drone).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self._create_button(btn_frame, '⊙ RESET CAM',  MUTED,  self._reset_camera).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

        # Mouse control help
        self._hr(parent)
        help_txt = (
            "MOUSE  ( in 3D viewport )\n"
            "─────────────────────────\n"
            "Left  drag  →  Yaw / Pitch\n"
            "Right drag  →  Roll / Alt\n"
            "Middle drag →  X / Y pos\n\n"
            "KEYBOARD\n"
            "─────────────────────────\n"
            "  R  →  Reset drone\n"
            "  V  →  Reset camera"
        )
        info = tk.Label(parent, text=help_txt, bg=BG2, fg=MUTED,
                        font=('Consolas', 8), justify=tk.LEFT,
                        padx=12, pady=10, relief='flat')
        info.pack(fill=tk.X, padx=14, pady=4)

        # Telemetry
        self._hr(parent)
        tk.Label(parent, text='📡  TELEMETRY', bg=BG, fg=ORANGE,
                 font=('Consolas', 10, 'bold')).pack(pady=(6, 4), padx=14, anchor='w')

        telem_box = tk.Frame(parent, bg=BG2)
        telem_box.pack(fill=tk.X, padx=14, pady=(0, 8))

        telem_rows = [
            ('Altitude', 'z',     'm'),
            ('Roll',     'roll',  '°'),
            ('Pitch',    'pitch', '°'),
            ('Heading',  'yaw',   '°'),
            ('X pos',    'x',     'm'),
            ('Y pos',    'y',     'm'),
        ]
        for name, key, unit in telem_rows:
            row = tk.Frame(telem_box, bg=BG2)
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(row, text=f'{name}:', bg=BG2, fg=MUTED,
                     font=('Consolas', 8), width=9, anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text='  0.00', bg=BG2, fg=YELLOW,
                           font=('Consolas', 9, 'bold'), width=8, anchor='e')
            lbl.pack(side=tk.LEFT)
            tk.Label(row, text=unit, bg=BG2, fg=MUTED,
                     font=('Consolas', 8), width=2).pack(side=tk.LEFT)
            self.telem_labels[key] = lbl

    def _hr(self, parent: tk.Frame, top: int = 5):
        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=14, pady=(top, 5))

    @staticmethod
    def _create_button(parent, label, color, cmd) -> tk.Button:
        return tk.Button(parent, text=label, bg='#21262d', fg=color,
                         font=('Consolas', 8, 'bold'), relief='flat',
                         activebackground=BORDER, cursor='hand2',
                         padx=4, pady=5, command=cmd)

    def _control_section(self, parent: tk.Frame, title: str, rows):
        tk.Label(parent, text=title, bg=BG, fg=CYAN,
                 font=('Consolas', 10, 'bold')).pack(pady=(8, 4), padx=14, anchor='w')

        for label, key, unit, color in rows:
            lo, hi = self.LIMITS[key]

            # Label + Entry + Unit
            row = tk.Frame(parent, bg=BG)
            row.pack(fill=tk.X, padx=14, pady=(2, 0))

            tk.Label(row, text=f'{label:6}', bg=BG, fg=color,
                     font=('Consolas', 10, 'bold'), width=7, anchor='w').pack(side=tk.LEFT)

            entry = tk.Entry(row, textvariable=self.str_vars[key],
                             bg='#21262d', fg=CYAN, insertbackground=TEXT,
                             font=('Consolas', 10), width=8, relief='flat', bd=4)
            entry.pack(side=tk.LEFT, padx=4)
            entry.bind('<Return>',   lambda e, k=key: self._entry_changed(k))
            entry.bind('<FocusOut>', lambda e, k=key: self._entry_changed(k))
            self.entries[key] = entry

            tk.Label(row, text=unit, bg=BG, fg=MUTED,
                     font=('Consolas', 9), width=2).pack(side=tk.LEFT)

            # Slider
            res = 0.01 if 'roll' not in key and 'pitch' not in key and 'yaw' not in key else 0.1
            slider = tk.Scale(parent, from_=lo, to=hi, orient=tk.HORIZONTAL,
                              resolution=res, bg=BG, fg=MUTED,
                              troughcolor='#21262d', activebackground=color,
                              highlightthickness=0, bd=0, showvalue=False,
                              command=lambda v, k=key: self._slider_changed(k, float(v)))
            slider.pack(fill=tk.X, padx=14, pady=(0, 3))
            self.sliders[key] = slider

    # ── 3-D panel ─────────────────────────────────────────────────────────────

    def _build_3d_panel(self, parent: tk.Frame):
        tk.Label(parent, text='3D VIEWPORT  —  Drag to control drone',
                 bg=BG, fg=MUTED, font=('Consolas', 8)).pack(anchor='w', padx=4)

        self.fig = plt.Figure(facecolor=BG, tight_layout=True)
        self.ax  = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(BG2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Disable built-in 3-D mouse rotation (we replace it)
        try:
            self.ax.disable_mouse_rotation()
        except AttributeError:
            try:
                self.ax.mouse_init(rotate_btn=None, zoom_btn=None)
            except Exception:
                pass

        # Custom mouse events on the 3-D canvas
        self.canvas.mpl_connect('button_press_event',   self._mouse_press)
        self.canvas.mpl_connect('motion_notify_event',  self._mouse_drag)
        self.canvas.mpl_connect('button_release_event', self._mouse_release)
        self.canvas.mpl_connect('scroll_event',         self._mouse_scroll)

    # ═════════════════════════════════════════════════════════════════════════
    #  Event handlers
    # ═════════════════════════════════════════════════════════════════════════

    def _entry_changed(self, key: str):
        if self._busy:
            return
        try:
            val = float(self.str_vars[key].get())
            self._set(key, val)
        except ValueError:
            self._sync_widgets()  # revert bad input

    def _slider_changed(self, key: str, val: float):
        if self._busy:
            return
        self._set(key, val, skip_slider=key)

    def _mouse_press(self, event):
        if event.inaxes == self.ax:
            self._mx, self._my, self._btn = event.x, event.y, event.button

    def _mouse_drag(self, event):
        if self._mx is None:
            return
        dx = event.x - self._mx
        dy = event.y - self._my
        self._mx, self._my = event.x, event.y

        ROT = 0.45   # deg / pixel
        POS = 0.04   # m   / pixel

        if self._btn == 1:       # LEFT  → Yaw + Pitch
            self.rot[2] = self._clamp(self.rot[2] + dx * ROT, *self.LIMITS['yaw'])
            self.rot[1] = self._clamp(self.rot[1] + dy * ROT, *self.LIMITS['pitch'])

        elif self._btn == 3:     # RIGHT → Roll + Altitude
            self.rot[0] = self._clamp(self.rot[0] + dx * ROT, *self.LIMITS['roll'])
            self.pos[2] = self._clamp(self.pos[2] - dy * POS, *self.LIMITS['z'])

        elif self._btn == 2:     # MIDDLE → Translate X / Y
            self.pos[0] = self._clamp(self.pos[0] + dx * POS, *self.LIMITS['x'])
            self.pos[1] = self._clamp(self.pos[1] - dy * POS, *self.LIMITS['y'])

        self._sync_widgets()
        self._redraw()

    def _mouse_release(self, _event):
        self._mx = self._my = self._btn = None

    def _mouse_scroll(self, event):
        """Zoom by adjusting camera distance via view_init limits."""
        delta = -0.3 if event.button == 'up' else 0.3
        self._cam_elev = self._clamp(self._cam_elev + delta * 5, -89, 89)
        self._redraw()

    # ═════════════════════════════════════════════════════════════════════════
    #  State management
    # ═════════════════════════════════════════════════════════════════════════

    def _set(self, key: str, val: float, skip_slider: str = ''):
        """Apply a single DOF value, clamp, then sync everything."""
        val = self._clamp(val, *self.LIMITS[key])
        idx = {'x': 0, 'y': 1, 'z': 2}
        ridx = {'roll': 0, 'pitch': 1, 'yaw': 2}
        if key in idx:  self.pos[idx[key]]  = val
        else:           self.rot[ridx[key]] = val
        self._sync_widgets(skip_slider=skip_slider)
        self._redraw()

    def _sync_widgets(self, skip_slider: str = ''):
        """Push internal state → all widgets + telemetry."""
        self._busy = True
        kv = {
            'x': self.pos[0], 'y': self.pos[1], 'z': self.pos[2],
            'roll': self.rot[0], 'pitch': self.rot[1], 'yaw': self.rot[2],
        }
        for k, v in kv.items():
            self.str_vars[k].set(f'{v:.2f}')
            if k != skip_slider:
                self.sliders[k].set(v)
            if k in self.telem_labels:
                self.telem_labels[k].config(text=f'{v:+7.2f}')
        self._busy = False

    def _reset_drone(self):
        self.pos = [0.0, 0.0, 0.0]
        self.rot = [0.0, 0.0, 0.0]
        self._sync_widgets()
        self._redraw()

    def _reset_camera(self):
        self._cam_elev = 22.0
        self._cam_azim = -55.0
        self._redraw()

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    # ═════════════════════════════════════════════════════════════════════════
    #  3-D rendering
    # ═════════════════════════════════════════════════════════════════════════

    def _redraw(self):
        ax = self.ax
        ax.cla()
        ax.set_facecolor(BG2)

        # Pane style
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor(BORDER)
        ax.grid(True, color=BORDER, linestyle='--', alpha=0.35, linewidth=0.5)
        ax.tick_params(colors=MUTED, labelsize=7)
        ax.set_xlabel('X (m)', color=RED,   labelpad=8, fontsize=8)
        ax.set_ylabel('Y (m)', color=GREEN, labelpad=8, fontsize=8)
        ax.set_zlabel('Z (m)', color=BLUE,  labelpad=8, fontsize=8)

        # Camera
        ax.view_init(elev=self._cam_elev, azim=self._cam_azim)

        # Viewport limits (follow drone)
        lim = 5.0
        cx, cy, cz = self.pos
        ax.set_xlim(cx - lim, cx + lim)
        ax.set_ylim(cy - lim, cy + lim)
        ax.set_zlim(cz - lim, cz + lim)

        # Ground plane (Z = 0)
        if cz - lim < 0 < cz + lim:
            gx = np.linspace(cx - lim, cx + lim, 8)
            gy = np.linspace(cy - lim, cy + lim, 8)
            gxx, gyy = np.meshgrid(gx, gy)
            ax.plot_surface(gxx, gyy, np.zeros_like(gxx),
                            alpha=0.07, color=BLUE, zorder=0)

        # World origin arrows
        orig = 0.9
        ax.quiver(0, 0, 0, orig, 0, 0, color=RED,   alpha=0.5, arrow_length_ratio=0.25, linewidth=1.2)
        ax.quiver(0, 0, 0, 0, orig, 0, color=GREEN, alpha=0.5, arrow_length_ratio=0.25, linewidth=1.2)
        ax.quiver(0, 0, 0, 0, 0, orig, color=BLUE,  alpha=0.5, arrow_length_ratio=0.25, linewidth=1.2)

        # Drone
        R = self._rot_matrix(*self.rot)
        center = np.array(self.pos, dtype=float)
        self._draw_drone(ax, center, R)

        # Ground shadow
        shadow_z = max(cz - lim + 0.05, cz - lim)
        if cz > shadow_z:
            ax.plot([cx], [cy], [shadow_z], 'o',
                    color='#333', markersize=22, alpha=0.2, zorder=1)

        # Status overlay
        ax.set_title(
            f"POS   X:{self.pos[0]:+7.2f}  Y:{self.pos[1]:+7.2f}  Z:{self.pos[2]:+7.2f} m\n"
            f"ROT   R:{self.rot[0]:+7.1f}°  P:{self.rot[1]:+7.1f}°  Yw:{self.rot[2]:+7.1f}°",
            color=TEXT, fontsize=8, fontfamily='monospace', pad=8
        )

        self.canvas.draw_idle()

    def _draw_drone(self, ax, center: np.ndarray, R: np.ndarray):
        """Render a realistic quadcopter (X-configuration)."""
        arm = 1.25

        # Motor positions in body frame
        # FR=front-right, FL=front-left, RL=rear-left, RR=rear-right
        motors_body = np.array([
            [ arm,  arm, 0.0],   # FR  (CW  → red)
            [-arm,  arm, 0.0],   # FL  (CCW → green)
            [-arm, -arm, 0.0],   # RL  (CW  → red)
            [ arm, -arm, 0.0],   # RR  (CCW → green)
        ])
        motors_w = (R @ motors_body.T).T + center   # world-frame positions
        m_colors = [RED, GREEN, RED, GREEN]

        # ── Arms ────────────────────────────────────────────────────────────
        for mw, mc in zip(motors_w, m_colors):
            ax.plot([center[0], mw[0]], [center[1], mw[1]], [center[2], mw[2]],
                    color='#636e72', linewidth=3.5, zorder=5, solid_capstyle='round')

        # ── Rotors ───────────────────────────────────────────────────────────
        for mw, mc in zip(motors_w, m_colors):
            θ = np.linspace(0, 2 * np.pi, 48)
            rr = 0.46
            disk_b = np.array([rr * np.cos(θ), rr * np.sin(θ), np.zeros(48)])
            disk_w = (R @ disk_b).T + mw
            ax.plot(disk_w[:, 0], disk_w[:, 1], disk_w[:, 2],
                    color=mc, linewidth=2.0, alpha=0.9, zorder=6)

            # Blades (2 × cross)
            for φ in (0.0, np.pi / 2):
                blade_b = np.array([[rr * np.cos(φ), -rr * np.cos(φ)],
                                    [rr * np.sin(φ), -rr * np.sin(φ)],
                                    [0.0, 0.0]])
                bw = (R @ blade_b).T + mw
                ax.plot(bw[:, 0], bw[:, 1], bw[:, 2],
                        color=mc, linewidth=2.2, alpha=0.75, zorder=7)

            # Motor dome
            ax.scatter(*mw, color=mc, s=45, zorder=9, depthshade=False)

        # ── Body plate ───────────────────────────────────────────────────────
        bx = 0.30
        body_b = np.array([
            [-bx, -bx, -0.08], [ bx, -bx, -0.08],
            [ bx,  bx, -0.08], [-bx,  bx, -0.08],
            [-bx, -bx,  0.12], [ bx, -bx,  0.12],
            [ bx,  bx,  0.12], [-bx,  bx,  0.12],
        ])
        body_w = (R @ body_b.T).T + center
        body_edges = [(0,1),(1,2),(2,3),(3,0),
                      (4,5),(5,6),(6,7),(7,4),
                      (0,4),(1,5),(2,6),(3,7)]
        for i, j in body_edges:
            ax.plot([body_w[i, 0], body_w[j, 0]],
                    [body_w[i, 1], body_w[j, 1]],
                    [body_w[i, 2], body_w[j, 2]],
                    color='#b2bec3', linewidth=1.1, alpha=0.85, zorder=5)

        # Center hub
        ax.scatter(*center, color=ORANGE, s=70, zorder=10, depthshade=False)

        # ── Body-frame coordinate axes ────────────────────────────────────────
        axl = 1.7
        for col, direction in zip([RED, GREEN, BLUE], R.T):
            d = direction * axl
            ax.quiver(center[0], center[1], center[2],
                      d[0], d[1], d[2],
                      color=col, linewidth=2.0, arrow_length_ratio=0.22, zorder=9)

        # ── Forward nose indicator ────────────────────────────────────────────
        nose_b = np.array([0.0, arm * 0.85, 0.18])
        nose_w = R @ nose_b + center
        ax.scatter(*nose_w, color=YELLOW, s=90, marker='^', zorder=11, depthshade=False)

    # ═════════════════════════════════════════════════════════════════════════
    #  Entry point
    # ═════════════════════════════════════════════════════════════════════════

    def run(self):
        self._sync_widgets()
        self.root.mainloop()


if __name__ == '__main__':
    sim = FlightControllerSim()
    sim.run()
