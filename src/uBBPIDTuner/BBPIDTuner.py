"""
   BBPIDTuner — pBBPID live tuner + scope (pymoos2 AppCastingMOOSApp).
   Author: Karan Mahesh

   A native MOOS app (runs under pAntler) presenting a Tk GUI that:
     * live-edits every pBBPID parameter via slider AND text entry
       (gains, limits, filters, feedforward coeffs) over a single
       BBPID_SET "key=value" channel,
     * resets all parameters to the plug-file defaults
       (pBBPID publishes them as BBPID_PARAMS_DEFAULT),
     * edits the speed gain schedule,
     * plots desired vs measured speed/heading/yaw rate, and the
       feedforward surfaces over the expected (speed, yaw-rate) range.

   The Tk loop is pumped from iterate(), so the GUI lives inside the
   AppCastingMOOSApp run loop. Config (ProcessConfig = uBBPIDTuner):
     history        = 30      // plot window, seconds
     publish_suffix = _ALL    // shoreside -> vehicle via uFldShoreBroker qbridge
"""

from collections import deque

from pymoos2 import AppCastingMOOSApp, ACTable, moos_time

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk

# Scope plots: (title, desired_var, measured_var)
PLOTS = [
    ("Speed  (m/s)",      "DESIRED_SPEED",         "NAV_SPEED"),
    ("Heading  (deg)",    "DESIRED_HEADING",       "NAV_HEADING"),
    ("Yaw rate  (rad/s)", "BBPID_DESIRED_YAWRATE", "BBPID_MEAS_YAWRATE"),
]
# "Actuators" tab: each plot = (title, [(var, color, label), ...])
EXTRA_PLOTS = [
    ("Feedforward contribution [%]", [
        ("BBPID_FF_THRUST", "tab:red",    "forward FF (-> thrust)"),
        ("BBPID_FF_RUDDER", "tab:blue",   "yaw FF (-> rudder)"),
    ]),
    ("Left / Right thrust [%]", [
        ("DESIRED_THRUST_L", "tab:green",  "left"),
        ("DESIRED_THRUST_R", "tab:purple", "right"),
    ]),
]
PLOT_VARS = sorted(
    {v for _, a, b in PLOTS for v in (a, b)}
    | {var for _, lines in EXTRA_PLOTS for var, _, _ in lines})

# Triplet groups -> sent as "key=a,b,c"
GAIN_GROUPS = [
    ("Speed PID  (Kp,Ki,Kd)",   "speed_pid",   [("Kp", 0, 10), ("Ki", 0, 2), ("Kd", 0, 2)]),
    ("Heading PID  (Kp,Ki,Kd)", "heading_pid", [("Kp", 0, 5),  ("Ki", 0, 2), ("Kd", 0, 2)]),
    ("YawRate PID  (Kp,Ki,Kd)", "yawrate_pid", [("Kp", 0, 400), ("Ki", 0, 100), ("Kd", 0, 100)]),
]
FF_GROUPS = [
    ("FF speed  (c0,cv,crr)", "ff_speed", [("c0", -50, 50), ("cv", -50, 50), ("crr", -400, 400)]),
    ("FF yaw  (d0,dr,dvr)",   "ff_yaw",   [("d0", -50, 50), ("dr", 0, 400),  ("dvr", -100, 100)]),
]
# Scalars -> sent as "key=v"
LIMIT_SCALARS = [
    ("max_thrust",      "max_thrust",             0, 100),
    ("max_rudder",      "max_rudder",             0, 100),
    ("max_yawrate[r/s]", "max_yawrate",           0, 1.5),
    ("speed_ilim",      "speed_integral_limit",   0, 200),
    ("yawrate_ilim",    "yawrate_integral_limit", 0, 200),
    ("des_yaw_filt[s]", "des_yawrate_filter",     0, 2),
    ("yawrate_filt",    "yawrate_filter",         0, 1),
]
FF_SCALARS = [
    ("ff_rudder_scale", "ff_rudder_scale", 0, 3),
]

# Initial display values (overwritten by the plug snapshot when it arrives)
INIT = {
    "speed_pid": [3.0, 0.5, 0.0], "heading_pid": [1.3, 0.2, 0.0],
    "yawrate_pid": [229.18, 28.648, 0.0],
    "max_thrust": 100, "max_rudder": 100, "max_yawrate": 0.43633,
    "speed_integral_limit": 100, "yawrate_integral_limit": 50,
    "des_yawrate_filter": 0.3, "yawrate_filter": 0.5,
    "ff_speed": [20.0, 15.0, -164.14], "ff_yaw": [2.5806, 235.65, 0.0],
    "ff_rudder_scale": 1.0, "ff_enable": True,
    "ff_speed_enable": True, "ff_yaw_enable": True,
}

DEFAULT_SCHEDULE = [   # speed [m/s], kp, ki, kd (rad/s), max_yawrate [rad/s]
    (0.5, 171.89, 5.7296, 0.0,    0.5236),
    (1.5, 114.59, 5.7296, 0.0,    0.34907),
    (3.0, 68.755, 2.8648, 11.459, 0.20944),
]
SPEED_RANGE = (0.0, 2.5)   # expected speed range for the FF map [m/s]


# ==========================================================================
class ParamControl:
    """A label + slider + numeric entry, kept in sync. Typing beyond the
    slider range extends it."""

    def __init__(self, parent, row, label, lo, hi, default, on_change):
        self.lo, self.hi, self.on_change = lo, hi, on_change
        self._silent = False
        ttk.Label(parent, text=label, width=16, anchor="w").grid(
            row=row, column=0, sticky="w", pady=1)
        self.var = tk.DoubleVar(value=default)
        self.scale = ttk.Scale(parent, from_=lo, to=hi, variable=self.var,
                               orient="horizontal", length=150, command=self._slide)
        self.scale.grid(row=row, column=1, padx=4, sticky="ew")
        self.evar = tk.StringVar(value=f"{default:.4g}")
        self.entry = ttk.Entry(parent, width=8, textvariable=self.evar)
        self.entry.grid(row=row, column=2)
        self.entry.bind("<Return>", self._typed)
        self.entry.bind("<FocusOut>", self._typed)

    def _slide(self, _=None):
        if self._silent:
            return
        self.evar.set(f"{self.var.get():.4g}")
        self.on_change()

    def _typed(self, _=None):
        try:
            v = float(self.evar.get())
        except ValueError:
            return
        if v < self.lo:
            self.scale.configure(from_=v)
        if v > self.hi:
            self.scale.configure(to=v)
        self._silent = True
        self.var.set(v)
        self._silent = False
        self.on_change()

    def get(self):
        return self.var.get()

    def set_silent(self, v):
        """Set value without triggering on_change (for init / reset display)."""
        self._silent = True
        if v < self.lo:
            self.scale.configure(from_=v)
        if v > self.hi:
            self.scale.configure(to=v)
        self.var.set(v)
        self.evar.set(f"{v:.4g}")
        self._silent = False


# ==========================================================================
class TunerGUI:
    def __init__(self, app, history):
        self.app = app
        self.history = float(history)
        self.alive = True
        self.groups = {}      # cfgkey -> [ParamControl]
        self.scalars = {}     # cfgkey -> ParamControl
        self.defaults = {}    # cfgkey -> value/list (from plug snapshot)
        self._defaults_applied = False

        self.root = tk.Tk()
        self.root.title("pBBPID Tuner")
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main)
        left.pack(side="left", fill="y", padx=(0, 8))
        self._build_topbar(left)
        self._build_controls(left)

        self._build_plots(main)
        self._refresh_ff_map()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- top bar: reset + save + ff_enable ------------------------------
    def _build_topbar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(0, 4))
        ttk.Button(bar, text="⟲ Reset to plug defaults",
                   command=self._reset).pack(side="left")
        ttk.Button(bar, text="💾 Save config",
                   command=self._save_config).pack(side="left", padx=(6, 0))
        ttk.Button(bar, text="∫ Reset integrators",
                   command=lambda: self.app.set_param("reset_integrators", "1")
                   ).pack(side="left", padx=(6, 0))
        self.ff_enable = tk.BooleanVar(value=INIT["ff_enable"])
        ttk.Checkbutton(bar, text="FF enabled", variable=self.ff_enable,
                        command=lambda: self.app.set_param(
                            "ff_enable", "true" if self.ff_enable.get() else "false")
                        ).pack(side="left", padx=(10, 0))
        self.ff_speed_enable = tk.BooleanVar(value=INIT["ff_speed_enable"])
        ttk.Checkbutton(bar, text="speed FF", variable=self.ff_speed_enable,
                        command=lambda: self.app.set_param(
                            "ff_speed_enable",
                            "true" if self.ff_speed_enable.get() else "false")
                        ).pack(side="left", padx=(6, 0))
        self.ff_yaw_enable = tk.BooleanVar(value=INIT["ff_yaw_enable"])
        ttk.Checkbutton(bar, text="yaw FF", variable=self.ff_yaw_enable,
                        command=lambda: self.app.set_param(
                            "ff_yaw_enable",
                            "true" if self.ff_yaw_enable.get() else "false")
                        ).pack(side="left", padx=(6, 0))
        self._save_status = ttk.Label(parent, text="", foreground="gray30",
                                      wraplength=360, justify="left")
        self._save_status.pack(fill="x", pady=(0, 4))

    # ---- save current values as a droppable pBBPID config block ---------
    def _save_config(self):
        import datetime
        import os

        def grp(key):
            return ", ".join(f"{c.get():.5g}" for c in self.groups[key])

        def sca(key):
            return f"{self.scalars[key].get():.5g}"

        L = ["//---------------------------------------------------------",
             "// pBBPID tuned config  (uBBPIDTuner save)",
             "// Drop into plug_pBBPID.moos. XMODE-specific params",
             "// (yawrate_source, nav_yawrate_var, yawrate_scale,",
             "//  rudder_polarity, allow_reverse) are NOT included.",
             "//---------------------------------------------------------",
             "ProcessConfig = pBBPID", "{"]
        L.append(f"  speed_pid   = {grp('speed_pid')}")
        L.append(f"  heading_pid = {grp('heading_pid')}")
        L.append(f"  yawrate_pid = {grp('yawrate_pid')}")
        L.append("")
        L.append(f"  des_yawrate_filter     = {sca('des_yawrate_filter')}")
        L.append(f"  yawrate_filter         = {sca('yawrate_filter')}")
        L.append("")
        for k in ("max_thrust", "max_rudder", "max_yawrate",
                  "speed_integral_limit", "yawrate_integral_limit"):
            L.append(f"  {k:<22} = {sca(k)}")
        L.append("")
        L.append(f"  enable_gain_schedule   = "
                 f"{'true' if self.sched_on.get() else 'false'}")
        for it in self.tree.get_children():
            sp, kp, ki, kd, mxy = self.tree.item(it, "values")
            L.append(f"  schedule_point = speed={sp}, kp={kp}, ki={ki}, "
                     f"kd={kd}, max_yawrate={mxy}")
        L.append("")
        L.append(f"  ff_enable              = "
                 f"{'true' if self.ff_enable.get() else 'false'}")
        L.append(f"  ff_speed_enable = "
                 f"{'true' if self.ff_speed_enable.get() else 'false'}")
        L.append(f"  ff_yaw_enable   = "
                 f"{'true' if self.ff_yaw_enable.get() else 'false'}")
        L.append(f"  ff_speed        = {grp('ff_speed')}")
        L.append(f"  ff_yaw          = {grp('ff_yaw')}")
        L.append(f"  ff_rudder_scale = {sca('ff_rudder_scale')}")
        L.append("}")
        text = "\n".join(L) + "\n"

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.abspath(f"bbpid_tuned_{ts}.moos")
        try:
            with open(path, "w") as f:
                f.write(text)
            self._save_status.config(text=f"saved → {path}", foreground="green")
            self.app.report_event(f"saved tuned config -> {path}")
        except OSError as e:
            self._save_status.config(text=f"save failed: {e}", foreground="red")

    # ---- control notebook ----------------------------------------------
    def _build_controls(self, parent):
        nb = ttk.Notebook(parent)
        nb.pack(fill="both", expand=True)

        tab_g = ttk.Frame(nb, padding=6); nb.add(tab_g, text="Gains")
        for label, key, comps in GAIN_GROUPS:
            self._group(tab_g, label, key, comps)
        self._build_schedule(tab_g)

        tab_l = ttk.Frame(nb, padding=6); nb.add(tab_l, text="Limits & Filters")
        self._scalars(tab_l, LIMIT_SCALARS)

        tab_f = ttk.Frame(nb, padding=6); nb.add(tab_f, text="Feedforward")
        for label, key, comps in FF_GROUPS:
            self._group(tab_f, label, key, comps)
        self._scalars(tab_f, FF_SCALARS)

    def _group(self, parent, label, key, comps):
        box = ttk.LabelFrame(parent, text=label, padding=4)
        box.pack(fill="x", pady=3)
        box.columnconfigure(1, weight=1)
        ctrls = []
        init = INIT.get(key, [0] * len(comps))
        for i, (cn, lo, hi) in enumerate(comps):
            pc = ParamControl(box, i, cn, lo, hi, init[i],
                              lambda k=key: self._send_group(k))
            ctrls.append(pc)
        self.groups[key] = ctrls

    def _scalars(self, parent, specs):
        box = ttk.Frame(parent)
        box.pack(fill="x")
        box.columnconfigure(1, weight=1)
        for i, (label, key, lo, hi) in enumerate(specs):
            pc = ParamControl(box, i, label, lo, hi, INIT.get(key, 0),
                              lambda k=key: self._send_scalar(k))
            self.scalars[key] = pc

    # ---- publishing -----------------------------------------------------
    def _send_group(self, key):
        vals = [c.get() for c in self.groups[key]]
        self.app.set_param(key, ",".join(f"{v:.5g}" for v in vals))
        if key.startswith("ff"):
            self._refresh_ff_map()

    def _send_scalar(self, key):
        self.app.set_param(key, f"{self.scalars[key].get():.5g}")
        if key in ("ff_rudder_scale", "max_yawrate"):
            self._refresh_ff_map()

    # ---- plug-default snapshot -> init controls + reset baseline --------
    def on_defaults(self, snapshot):
        d = {}
        for part in snapshot.split(";"):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k, v = k.strip(), v.strip()
            if "," in v:
                try: d[k] = [float(x) for x in v.split(",")]
                except ValueError: pass
            elif v in ("true", "false"):
                d[k] = (v == "true")
            else:
                try: d[k] = float(v)
                except ValueError: pass
        self.defaults = d
        if not self._defaults_applied:
            self._apply_defaults(publish=False)   # first snapshot: just display
            self._defaults_applied = True

    def _apply_defaults(self, publish):
        for key, ctrls in self.groups.items():
            if key in self.defaults:
                for c, v in zip(ctrls, self.defaults[key]):
                    c.set_silent(v)
                if publish:
                    self._send_group(key)
        for key, c in self.scalars.items():
            if key in self.defaults:
                c.set_silent(self.defaults[key])
                if publish:
                    self._send_scalar(key)
        for key, var in (("ff_enable", self.ff_enable),
                         ("ff_speed_enable", self.ff_speed_enable),
                         ("ff_yaw_enable", self.ff_yaw_enable)):
            if key in self.defaults:
                var.set(bool(self.defaults[key]))
                if publish:
                    self.app.set_param(key, "true" if var.get() else "false")
        self._refresh_ff_map()

    def _reset(self):
        if self.defaults:
            self._apply_defaults(publish=True)

    # ---- gain schedule (unchanged channel) ------------------------------
    def _build_schedule(self, parent):
        box = ttk.LabelFrame(parent, text="Gain schedule (yaw-rate loop)", padding=4)
        box.pack(fill="x", pady=3)
        self.sched_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="enabled", variable=self.sched_on,
                        command=lambda: self.app.schedule_enable(self.sched_on.get())
                        ).grid(row=0, column=0, columnspan=4, sticky="w")
        cols = ("speed", "kp", "ki", "kd", "max_yawrate")
        self.tree = ttk.Treeview(box, columns=cols, show="headings", height=4)
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=54, anchor="center")
        self.tree.grid(row=1, column=0, columnspan=4, pady=3)
        for r in DEFAULT_SCHEDULE:
            self.tree.insert("", "end", values=tuple(f"{x:g}" for x in r))
        self.sentries = {}
        er = ttk.Frame(box); er.grid(row=2, column=0, columnspan=4, sticky="w")
        for i, c in enumerate(cols):
            ttk.Label(er, text=c, width=5).grid(row=0, column=i)
            e = ttk.Entry(er, width=6); e.grid(row=1, column=i, padx=1)
            self.sentries[c] = e
        b = ttk.Frame(box); b.grid(row=3, column=0, columnspan=4, sticky="w", pady=(3, 0))
        ttk.Button(b, text="Add", command=self._sched_add).pack(side="left")
        ttk.Button(b, text="Remove", command=self._sched_rm).pack(side="left")
        ttk.Button(b, text="Apply ▶", command=self._sched_apply).pack(side="left", padx=(8, 0))

    def _sched_add(self):
        try:
            vals = tuple(f"{float(self.sentries[c].get()):g}"
                         for c in ("speed", "kp", "ki", "kd", "max_yawrate"))
        except ValueError:
            return
        self.tree.insert("", "end", values=vals)

    def _sched_rm(self):
        for i in self.tree.selection():
            self.tree.delete(i)

    def _sched_apply(self):
        self.app.schedule_clear()
        rows = [tuple(float(x) for x in self.tree.item(i, "values"))
                for i in self.tree.get_children()]
        for sp, kp, ki, kd, mxy in sorted(rows):
            self.app.schedule_point(sp, kp, ki, kd, mxy)
        self.app.schedule_enable(self.sched_on.get())

    # ---- plots ----------------------------------------------------------
    def _build_plots(self, parent):
        nb = ttk.Notebook(parent)
        nb.pack(side="right", fill="both", expand=True)

        sc = ttk.Frame(nb); nb.add(sc, text="Scope")
        self.fig = Figure(figsize=(6.5, 6.0), dpi=100)
        self.axes, self.lines = [], []
        for i, (title, dvar, mvar) in enumerate(PLOTS):
            ax = self.fig.add_subplot(3, 1, i + 1)
            ax.set_ylabel(title); ax.grid(True, alpha=0.3)
            ld, = ax.plot([], [], lw=1.6, color="tab:red", label="desired")
            lm, = ax.plot([], [], lw=1.4, color="tab:blue", label="measured")
            ax.legend(loc="upper right", fontsize=8)
            self.axes.append(ax); self.lines.append((ld, dvar, lm, mvar))
        self.axes[-1].set_xlabel("time [s]")
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=sc)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Actuators tab: FF contributions + left/right thrust (time series)
        ac = ttk.Frame(nb); nb.add(ac, text="Actuators")
        self.extra_fig = Figure(figsize=(6.5, 6.0), dpi=100)
        self.extra_axes = []     # [(ax, [(line2d, var), ...]), ...]
        n = len(EXTRA_PLOTS)
        for i, (title, lines) in enumerate(EXTRA_PLOTS):
            ax = self.extra_fig.add_subplot(n, 1, i + 1)
            ax.set_ylabel(title); ax.grid(True, alpha=0.3)
            lns = []
            for var, color, label in lines:
                ln, = ax.plot([], [], lw=1.4, color=color, label=label)
                lns.append((ln, var))
            ax.legend(loc="upper right", fontsize=8)
            self.extra_axes.append((ax, lns))
        self.extra_axes[-1][0].set_xlabel("time [s]")
        self.extra_fig.tight_layout()
        self.extra_canvas = FigureCanvasTkAgg(self.extra_fig, master=ac)
        self.extra_canvas.get_tk_widget().pack(fill="both", expand=True)

        fm = ttk.Frame(nb); nb.add(fm, text="FF map")
        self.ff_fig = Figure(figsize=(6.5, 6.0), dpi=100)
        self.ff_ax_r = self.ff_fig.add_subplot(2, 1, 1)
        self.ff_ax_t = self.ff_fig.add_subplot(2, 1, 2)
        self.ff_fig.tight_layout()
        self.ff_canvas = FigureCanvasTkAgg(self.ff_fig, master=fm)
        self.ff_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._ff_cbars = []

    def _refresh_ff_map(self):
        c = [self.groups["ff_speed"][i].get() for i in range(3)]   # c0,cv,crr
        d = [self.groups["ff_yaw"][i].get() for i in range(3)]     # d0,dr,dvr
        rmax = self.scalars["max_yawrate"].get() if "max_yawrate" in self.scalars else 25.0
        rmax = max(rmax, 1.0)
        v = np.linspace(SPEED_RANGE[0], SPEED_RANGE[1], 60)
        r = np.linspace(-rmax, rmax, 60)
        V, R = np.meshgrid(v, r)
        ff_rud = d[0] + d[1] * R + d[2] * V * R
        ff_thr = c[0] + c[1] * V + c[2] * R * R
        for cb in self._ff_cbars:
            try: cb.remove()
            except Exception: pass
        self._ff_cbars = []
        for ax, Z, title in [(self.ff_ax_r, ff_rud, "rudder FF: d0+dr*r+dvr*v*r  [%]"),
                             (self.ff_ax_t, ff_thr, "thrust FF: c0+cv*v+crr*r^2  [%]")]:
            ax.clear()
            cf = ax.contourf(V, R, Z, levels=18, cmap="RdBu_r")
            ax.set(xlabel="desired speed v* [m/s]",
                   ylabel="desired yaw rate r* [rad/s]", title=title)
            self._ff_cbars.append(self.ff_fig.colorbar(cf, ax=ax))
        self.ff_canvas.draw_idle()

    def _update_axis(self, ax, line_vars, t_lo, now):
        ymin = ymax = None
        for line, var in line_vars:
            buf = self.app.buf[var]
            xs = [t for t, _ in buf if t >= t_lo]
            ys = [val for t, val in buf if t >= t_lo]
            line.set_data(xs, ys)
            if ys:
                lo, hi = min(ys), max(ys)
                ymin = lo if ymin is None else min(ymin, lo)
                ymax = hi if ymax is None else max(ymax, hi)
        ax.set_xlim(max(0.0, t_lo), max(self.history, now))
        if ymin is not None:
            pad = max(0.5, 0.1 * (ymax - ymin))
            ax.set_ylim(ymin - pad, ymax + pad)

    def refresh(self):
        now = moos_time() - self.app.t0
        t_lo = now - self.history
        for ax, (ld, dvar, lm, mvar) in zip(self.axes, self.lines):
            self._update_axis(ax, ((ld, dvar), (lm, mvar)), t_lo, now)
        self.canvas.draw_idle()
        for ax, lns in self.extra_axes:
            self._update_axis(ax, lns, t_lo, now)
        self.extra_canvas.draw_idle()

    def pump(self):
        if self.alive:
            self.root.update()

    def _on_close(self):
        self.alive = False
        self.app.request_stop()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


# ==========================================================================
class BBPIDTuner(AppCastingMOOSApp):

    SYNOPSIS = "Live pBBPID tuner (gains/limits/filters/FF) + scope + FF map."
    VERSION = "0.3.0"

    SUBSCRIPTIONS = [
        "DESIRED_SPEED / NAV_SPEED, DESIRED_HEADING / NAV_HEADING,",
        "BBPID_DESIRED_YAWRATE / BBPID_MEAS_YAWRATE   — scope (double)",
        "BBPID_PARAMS_DEFAULT                         — plug defaults (string)",
    ]
    PUBLICATIONS = [
        "BBPID_SET            — 'key=value' config line (gains/limits/filters/FF)",
        "BBPID_SCHEDULE_*     — gain schedule",
        "(a 'publish_suffix' such as _ALL is appended for shoreside qbridge)",
    ]
    EXAMPLE_CONFIG = """\
ProcessConfig = uBBPIDTuner
{
    AppTick   = 20
    CommsTick = 10
    history        = 30
    publish_suffix = _ALL
}"""

    def __init__(self):
        super().__init__()
        self.history = 30.0
        self.suffix = ""
        self.t0 = 0.0
        self.buf = {v: deque(maxlen=20000) for v in PLOT_VARS}
        self.gui = None

    def on_startup(self):
        self.history = self.config("history", default=30.0, type=float)
        self.suffix = self.config("publish_suffix", default="")
        self.t0 = moos_time()
        self.gui = TunerGUI(self, self.history)
        self.report_event(f"tuner GUI up (suffix='{self.suffix or '(none)'}')")

    def register_variables(self):
        for v in PLOT_VARS:
            self.register(v)
        self.register("BBPID_PARAMS_DEFAULT")

    def on_new_mail(self, mail):
        now = moos_time() - self.t0
        for msg in mail:
            if msg.name in self.buf and msg.is_double():
                self.buf[msg.name].append((now, msg.double()))
            elif msg.name == "BBPID_PARAMS_DEFAULT" and msg.is_string() and self.gui:
                self.gui.on_defaults(msg.string())

    def iterate(self):
        if self.gui and self.gui.alive:
            self.gui.refresh()
            self.gui.pump()
        return True

    def build_report(self):
        self.report(f"pBBPID Tuner  |  suffix '{self.suffix or '(none)'}'  |  "
                    f"GUI {'open' if (self.gui and self.gui.alive) else 'closed'}  |  "
                    f"defaults {'loaded' if (self.gui and self.gui.defaults) else 'pending'}")
        tab = ACTable(["Plot var", "samples", "latest"])
        for v in PLOT_VARS:
            b = self.buf[v]
            tab.add(v, str(len(b)), f"{b[-1][1]:.3f}" if b else "-")
        self.report(tab)
        return True

    def request_stop(self):
        self._running = False

    # ---- publishing (suffix routes shoreside->vehicle via qbridge) -------
    def set_param(self, key, value):
        self.notify("BBPID_SET" + self.suffix, f"{key}={value}")

    def schedule_enable(self, on):
        self.notify("BBPID_SCHEDULE_ENABLE" + self.suffix, "true" if on else "false")

    def schedule_clear(self):
        self.notify("BBPID_SCHEDULE_CLEAR" + self.suffix, 1.0)

    def schedule_point(self, speed, kp, ki, kd, mxy):
        self.notify("BBPID_SCHEDULE_POINT" + self.suffix,
                    f"speed={speed},kp={kp},ki={ki},kd={kd},max_yawrate={mxy}")
