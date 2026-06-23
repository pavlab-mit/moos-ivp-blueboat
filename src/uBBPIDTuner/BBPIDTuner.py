"""
   BBPIDTuner — pBBPID live tuner + scope (pymoos2 AppCastingMOOSApp).
   Author: Karan Mahesh

   A native MOOS app (runs under pAntler) presenting a Tk GUI that:
     * live-edits the pBBPID PID gains   (publishes BBPID_<LOOP>_<K{P,I,D}>)
     * live-edits the speed gain schedule (BBPID_SCHEDULE_ENABLE/_POINT/_CLEAR)
     * plots desired vs measured speed, heading, and yaw rate.

   The Tk event loop is pumped from iterate(), so the GUI lives entirely
   inside the AppCastingMOOSApp run loop (no second mainloop / thread).

   Config (ProcessConfig = uBBPIDTuner):
     history        = 30      // plot window, seconds
     publish_suffix = _ALL    // "_ALL" on shoreside so uFldShoreBroker
                              // qbridge broadcasts to the vehicle(s)
"""

from collections import deque

from pymoos2 import AppCastingMOOSApp, ACTable, moos_time

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk

# What we plot:  (title, desired_var, measured_var)
PLOTS = [
    ("Speed  (m/s)",      "DESIRED_SPEED",         "NAV_SPEED"),
    ("Heading  (deg)",    "DESIRED_HEADING",       "NAV_HEADING"),
    ("Yaw rate  (deg/s)", "BBPID_DESIRED_YAWRATE", "BBPID_MEAS_YAWRATE"),
]
PLOT_VARS = sorted({v for _, a, b in PLOTS for v in (a, b)})

# PID loops:  (label, db_token, [(component, default), ...])
LOOPS = [
    ("Speed",    "SPEED",   [("Kp", 1.0), ("Ki", 0.0), ("Kd", 0.0)]),
    ("Heading",  "HEADING", [("Kp", 1.3), ("Ki", 0.0), ("Kd", 0.0)]),
    ("Yaw rate", "YAWRATE", [("Kp", 2.0), ("Ki", 0.1), ("Kd", 0.0)]),
]
COMP_RANGE = {"Kp": (0.0, 10.0, 0.05), "Ki": (0.0, 2.0, 0.01), "Kd": (0.0, 2.0, 0.01)}

# Default gain-schedule rows: (speed, kp, ki, kd, max_yawrate)
DEFAULT_SCHEDULE = [
    (0.5, 3.0, 0.10, 0.0, 30.0),
    (1.5, 2.0, 0.10, 0.0, 20.0),
    (3.0, 1.2, 0.05, 0.2, 12.0),
]


# ==========================================================================
class TunerGUI:
    """Tk window. Publishing goes through the owning MOOS app (app.notify)."""

    def __init__(self, app, history):
        self.app = app
        self.history = float(history)
        self.alive = True

        self.root = tk.Tk()
        self.root.title("pBBPID Tuner")
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        controls = ttk.Frame(main)
        controls.pack(side="left", fill="y", padx=(0, 8))
        self._build_gains(controls)
        self._build_schedule(controls)
        self._build_plots(main)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- PID gain sliders ------------------------------------------------
    def _build_gains(self, parent):
        box = ttk.LabelFrame(parent, text="PID gains (live)", padding=6)
        box.pack(fill="x")
        for label, token, comps in LOOPS:
            ttk.Label(box, text=label, font=("TkDefaultFont", 10, "bold")) \
                .grid(sticky="w", columnspan=3, pady=(6, 0))
            for comp, default in comps:
                self._gain_row(box, token, comp, default)

    def _gain_row(self, parent, token, comp, default):
        lo, hi, res = COMP_RANGE[comp]
        r = parent.grid_size()[1]
        ttk.Label(parent, text=comp, width=3).grid(row=r, column=0, sticky="w")
        var = tk.DoubleVar(value=default)
        readout = ttk.Label(parent, width=6, text=f"{default:.3g}")

        def on_move(_=None):
            v = round(var.get() / res) * res
            readout.config(text=f"{v:.3g}")
            self.app.set_gain(token, comp, v)

        scale = ttk.Scale(parent, from_=lo, to=hi, variable=var,
                          orient="horizontal", length=150, command=on_move)
        scale.grid(row=r, column=1, sticky="ew", padx=4)
        readout.grid(row=r, column=2, sticky="e")

    # ---- gain schedule ---------------------------------------------------
    def _build_schedule(self, parent):
        box = ttk.LabelFrame(parent, text="Gain schedule (yaw-rate loop)", padding=6)
        box.pack(fill="x", pady=(8, 0))

        self.sched_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="enabled", variable=self.sched_on,
                        command=self._apply_enable).grid(row=0, column=0,
                                                         columnspan=4, sticky="w")
        cols = ("speed", "kp", "ki", "kd", "max_yawrate")
        self.tree = ttk.Treeview(box, columns=cols, show="headings", height=5)
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=58, anchor="center")
        self.tree.grid(row=1, column=0, columnspan=4, pady=4)
        for row in DEFAULT_SCHEDULE:
            self.tree.insert("", "end", values=tuple(f"{x:g}" for x in row))

        self.entries = {}
        er = ttk.Frame(box)
        er.grid(row=2, column=0, columnspan=4, sticky="w")
        for i, c in enumerate(cols):
            ttk.Label(er, text=c, width=5).grid(row=0, column=i)
            e = ttk.Entry(er, width=6)
            e.grid(row=1, column=i, padx=1)
            self.entries[c] = e

        btns = ttk.Frame(box)
        btns.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))
        ttk.Button(btns, text="Add",     command=self._add_point).pack(side="left")
        ttk.Button(btns, text="Remove",  command=self._remove_point).pack(side="left")
        ttk.Button(btns, text="Apply ▶", command=self._apply_schedule).pack(side="left", padx=(8, 0))

    def _add_point(self):
        try:
            vals = tuple(f"{float(self.entries[c].get()):g}"
                         for c in ("speed", "kp", "ki", "kd", "max_yawrate"))
        except ValueError:
            return
        self.tree.insert("", "end", values=vals)

    def _remove_point(self):
        for item in self.tree.selection():
            self.tree.delete(item)

    def _apply_enable(self):
        self.app.schedule_enable(self.sched_on.get())

    def _apply_schedule(self):
        self.app.schedule_clear()
        rows = [tuple(float(x) for x in self.tree.item(i, "values"))
                for i in self.tree.get_children()]
        for speed, kp, ki, kd, mxy in sorted(rows):
            self.app.schedule_point(speed, kp, ki, kd, mxy)
        self.app.schedule_enable(self.sched_on.get())

    # ---- plots -----------------------------------------------------------
    def _build_plots(self, parent):
        self.fig = Figure(figsize=(6.5, 6.0), dpi=100)
        self.axes, self.lines = [], []
        for i, (title, dvar, mvar) in enumerate(PLOTS):
            ax = self.fig.add_subplot(3, 1, i + 1)
            ax.set_ylabel(title)
            ax.grid(True, alpha=0.3)
            ld, = ax.plot([], [], lw=1.6, color="tab:red", label="desired")
            lm, = ax.plot([], [], lw=1.4, color="tab:blue", label="measured")
            ax.legend(loc="upper right", fontsize=8)
            self.axes.append(ax)
            self.lines.append((ld, dvar, lm, mvar))
        self.axes[-1].set_xlabel("time (s)")
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(side="right", fill="both", expand=True)

    def refresh(self):
        now = moos_time() - self.app.t0
        t_lo = now - self.history
        for ax, (ld, dvar, lm, mvar) in zip(self.axes, self.lines):
            ymin = ymax = None
            for line, var in ((ld, dvar), (lm, mvar)):
                buf = self.app.buf[var]
                xs = [t for t, _ in buf if t >= t_lo]
                ys = [v for t, v in buf if t >= t_lo]
                line.set_data(xs, ys)
                if ys:
                    lo, hi = min(ys), max(ys)
                    ymin = lo if ymin is None else min(ymin, lo)
                    ymax = hi if ymax is None else max(ymax, hi)
            ax.set_xlim(max(0.0, t_lo), max(self.history, now))
            if ymin is not None:
                pad = max(0.5, 0.1 * (ymax - ymin))
                ax.set_ylim(ymin - pad, ymax + pad)
        self.canvas.draw_idle()

    def pump(self):
        """Service Tk events once (called from the MOOS iterate loop)."""
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

    SYNOPSIS = "Live pBBPID tuner + desired/measured scope (Tk GUI)."
    VERSION = "0.2.0"

    SUBSCRIPTIONS = [
        "DESIRED_SPEED / NAV_SPEED            — speed plot (double)",
        "DESIRED_HEADING / NAV_HEADING        — heading plot (double)",
        "BBPID_DESIRED_YAWRATE / BBPID_MEAS_YAWRATE — yaw-rate plot (double)",
    ]
    PUBLICATIONS = [
        "BBPID_<SPEED|HEADING|YAWRATE>_<KP|KI|KD>  — live gains (double)",
        "BBPID_SCHEDULE_ENABLE|_POINT|_CLEAR        — gain schedule",
        "(a 'publish_suffix' such as _ALL is appended for shoreside qbridge)",
    ]
    EXAMPLE_CONFIG = """\
ProcessConfig = uBBPIDTuner
{
    AppTick   = 20      // high tick -> smooth GUI (iterate pumps Tk)
    CommsTick = 10

    history        = 30     // plot window, seconds
    publish_suffix = _ALL   // shoreside: broadcast to vehicle via qbridge
}"""

    def __init__(self):
        super().__init__()
        self.history = 30.0
        self.suffix = ""
        self.t0 = 0.0
        self.buf = {v: deque(maxlen=20000) for v in PLOT_VARS}
        self.gui = None

    # ---- MOOS lifecycle --------------------------------------------------
    def on_startup(self):
        self.history = self.config("history", default=30.0, type=float)
        self.suffix = self.config("publish_suffix", default="")
        self.t0 = moos_time()
        self.gui = TunerGUI(self, self.history)
        self.report_event(f"tuner GUI up (suffix='{self.suffix or '(none)'}')")

    def register_variables(self):
        for v in PLOT_VARS:
            self.register(v)

    def on_new_mail(self, mail):
        now = moos_time() - self.t0
        for msg in mail:
            if msg.name in self.buf and msg.is_double():
                self.buf[msg.name].append((now, msg.double()))

    def iterate(self):
        if self.gui and self.gui.alive:
            self.gui.refresh()
            self.gui.pump()
        return True

    def build_report(self):
        self.report(f"pBBPID Tuner  |  publish suffix: '{self.suffix or '(none)'}'"
                    f"  |  GUI: {'open' if (self.gui and self.gui.alive) else 'closed'}")
        tab = ACTable(["Plot var", "samples", "latest"])
        for v in PLOT_VARS:
            b = self.buf[v]
            tab.add(v, str(len(b)), f"{b[-1][1]:.3f}" if b else "-")
        self.report(tab)
        return True

    def request_stop(self):
        """Called by the GUI on window close -> end the MOOS run loop."""
        self._running = False

    # ---- publishing (suffix routes shoreside->vehicle via qbridge) -------
    def set_gain(self, token, comp, value):
        self.notify(f"BBPID_{token}_{comp.upper()}{self.suffix}", float(value))

    def schedule_enable(self, on):
        self.notify("BBPID_SCHEDULE_ENABLE" + self.suffix, "true" if on else "false")

    def schedule_clear(self):
        self.notify("BBPID_SCHEDULE_CLEAR" + self.suffix, 1.0)

    def schedule_point(self, speed, kp, ki, kd, mxy):
        self.notify("BBPID_SCHEDULE_POINT" + self.suffix,
                    f"speed={speed},kp={kp},ki={ki},kd={kd},max_yawrate={mxy}")
