"""
   DiffThrustTuner — shoreside web dashboard for live-tuning pDiffThrustPID_v2.

   Bridges MOOS <-> a browser:
     - subscribes PDIFF_THRUST_STATE (controller's current params) + desired/actual signals
     - serves a small web UI (stdlib http.server, no extra deps, works offline)
     - "Apply" in the UI -> PDIFF_THRUST_UPDATES back to the controller
   The controller is always the source of truth: every field shows the value it
   echoed in PDIFF_THRUST_STATE, so you never lose track while tuning.

   Author: Raymond Turrisi
"""
import os
import json
import queue
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymoos2 import AppCastingMOOSApp, ACTable, moos_time

HERE = os.path.dirname(os.path.abspath(__file__))

# desired/actual signals plotted in the dashboard
SIGNALS = ["DESIRED_SPEED", "NAV_SPEED",
           "DESIRED_HEADING", "NAV_HEADING",
           "DESIRED_THRUST_L", "DESIRED_THRUST_R"]


def _parse_kv(s):
    """'a=1;b=2;t=0,0:1,2' -> {'a':'1','b':'2','t':'0,0:1,2'} (controller's STATE format)."""
    out = {}
    for pair in s.split(";"):
        pair = pair.strip()
        if "=" in pair:
            k, v = pair.split("=", 1)
            out[k.strip()] = v.strip()
    return out


class DiffThrustTuner(AppCastingMOOSApp):

    SYNOPSIS = "Shoreside web dashboard to live-tune pDiffThrustPID_v2."
    VERSION = "0.1.0"
    SUBSCRIPTIONS = ["PDIFF_THRUST_STATE  — controller's current params (string)"] + \
                    [f"{s}  — plotted signal (double)" for s in SIGNALS]
    PUBLICATIONS = ["PDIFF_THRUST_UPDATES — param updates to the controller (string)"]
    EXAMPLE_CONFIG = """\
ProcessConfig = pDiffThrustTuner
{
    AppTick   = 8
    CommsTick = 8

    web_port = 8080      // dashboard at http://localhost:8080
    history  = 90        // seconds of signal history to plot
}"""

    def __init__(self):
        super().__init__()
        self.web_port = 8080
        self.history = 90.0
        self.publish_suffix = ""               # "_ALL" -> uFldShoreBroker qbridge to vehicle
        self.speed_update_var = "LEGRUN_UPDATE"  # behavior updates var for desired speed
        self.params = {}                       # latest PDIFF_THRUST_STATE
        self.params_ts = 0.0
        self.buffers = {s: deque() for s in SIGNALS}
        self.lock = threading.Lock()
        self.apply_q = queue.Queue()           # web thread -> MOOS thread; items are (var, value)

    def on_startup(self):
        self.web_port = self.config("web_port", 8080, int)
        self.history = self.config("history", 90.0, float)
        self.publish_suffix = self.config("publish_suffix", "")
        self.speed_update_var = self.config("speed_update_var", "LEGRUN_UPDATE")
        self._start_server()
        self.report_event(f"dashboard on http://localhost:{self.web_port}")

    def register_variables(self):
        self.register("PDIFF_THRUST_STATE")
        for s in SIGNALS:
            self.register(s)

    def on_new_mail(self, mail):
        now = moos_time()
        with self.lock:
            for msg in mail:
                if msg.name == "PDIFF_THRUST_STATE":
                    self.params = _parse_kv(msg.string())
                    self.params_ts = now
                elif msg.name in self.buffers and msg.is_double():
                    self.buffers[msg.name].append((now, msg.double()))
            cutoff = now - self.history
            for buf in self.buffers.values():
                while buf and buf[0][0] < cutoff:
                    buf.popleft()

    def iterate(self):
        # publish queued applies from the UI (only the MOOS thread touches the DB)
        while True:
            try:
                var, val = self.apply_q.get_nowait()
            except queue.Empty:
                break
            self.notify(var + self.publish_suffix, val)
            self.report_event(f"sent {var}={val}")
        # ask the controller to announce itself until we've heard from it
        if not self.params and (self.m_iteration % 8 == 0):
            self.notify("PDIFF_THRUST_UPDATES" + self.publish_suffix, "query")

    def build_report(self):
        tab = ACTable(["Controller param", "Value"])
        if self.params:
            for k, v in self.params.items():
                tab.add(k, v)
        else:
            tab.add("(waiting for", "PDIFF_THRUST_STATE)")
        self.report(tab)
        self.report(f"\nDashboard:  http://localhost:{self.web_port}")
        return True

    # ----------------------------------------------------------------- web
    def _snapshot(self):
        with self.lock:
            age = round(moos_time() - self.params_ts, 1) if self.params_ts else None
            return {
                "params": dict(self.params),
                "params_age": age,
                "signals": {s: list(self.buffers[s]) for s in SIGNALS},
                "now": moos_time(),
                "history": self.history,
                "speed_var": self.speed_update_var,
            }

    def _start_server(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    with open(os.path.join(HERE, "dashboard.html"), "rb") as f:
                        self._send(200, "text/html", f.read())
                elif self.path == "/state":
                    self._send(200, "application/json",
                               json.dumps(app._snapshot()).encode())
                else:
                    self._send(404, "text/plain", b"not found")

            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(n).decode().strip() if n else ""
                if self.path == "/apply":                       # controller params
                    if body:
                        app.apply_q.put(("PDIFF_THRUST_UPDATES", body))
                    self._send(200, "application/json", b'{"ok":true}')
                elif self.path == "/speed":                     # leg speed setpoint
                    if body:
                        app.apply_q.put((app.speed_update_var, "speed=" + body))
                    self._send(200, "application/json", b'{"ok":true}')
                else:
                    self._send(404, "text/plain", b"not found")

        server = ThreadingHTTPServer(("0.0.0.0", self.web_port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
