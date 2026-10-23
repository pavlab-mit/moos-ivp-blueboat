#!/usr/bin/env python3
"""Score a handling_block run from the back-seat .alog.

Reads the MANEUVER_PHASE markers the block's uTimerScript posts and
computes the plan §8.3 acceptance metrics plus, per checkpoint-brief
§4, the EVIDENCE COLUMN for every tunable in the control stack:

    steps/zigs   t63, overshoot, peak yaw, settled     -> heading_pid
                 rate-tracking RMS, rudder-sat frac    -> yawrate_pid, max_yawrate
                 governor min speed fraction           -> yaw_priority_*
    pivots       sustained rate p50/p90                -> yawrate_pid, ff_yaw
                 common-mode body-surge drift          -> MOT_THST_ASYM
                 CW/CCW rate split                     -> motor match
    straight     cross-track RMS                       -> heading_pid
                 standing speed error, FF thrust share -> speed_pid, ff_speed
    speed steps  speed t63, standing error, FF share   -> speed_pid, ff_speed
    run summary  mixer saturation frac + heading-error
                 correlation                           -> MOT_STR_THR_MIX
                 integral health (|I| vs limits),
                 gate timeline                         -> lifecycle fixes

Usage:
    analysis/handling_block.py <backseat.alog>              score one run
    analysis/handling_block.py --compare <A.alog> <B.alog>  A/B two runs
    analysis/handling_block.py <alog> --ledger tuning.jsonl append record
    analysis/handling_block.py <alog> --json                full JSON dump

The report header carries parameter PROVENANCE: the pBBPID block from
the run's ._moos snapshot, overlaid with any BBPID_PARAMS_ACTIVE
republications (live tuning during the run). A ledger line is one
self-describing record: params + every metric, so evidence accumulates
across sessions ("change one thing -> repeat", brief §4).

Notes on the inputs:
  * NAV_HEADING is the back seat's EKF heading (~COG underway).
  * GYRO_Z_LVL_IMU is rad/s, bridged from the front seat; reported
    as |deg/s| to sidestep the sign-convention caveat (plan §2.7).
  * Phases separated by RC takeovers score correctly: samples where
    autonomy was not driving are excluded using BB_CMD_AUTHORITY.
  * Vars absent from older logs (ITERMs, BBPID_GATE,
    BB_MIX_SATURATION) degrade to "--" rather than failing.
"""

import argparse
import datetime
import json
import math
import os
import sys


# ----------------------------------------------------------------- util

def wrap180(d):
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d


def pct(sorted_vals, p):
    if not sorted_vals: return float("nan")
    return sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * p))]


def mean(vals):
    vals = list(vals)
    return sum(vals) / len(vals) if vals else float("nan")


def rms(vals):
    vals = list(vals)
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def fnum(v, fmtstr="{:6.2f}"):
    """Format a possibly-NaN metric for the table."""
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "    --"
    return fmtstr.format(v)


def latest_before(series, t):
    """Last (t0, v) in series with t0 <= t, else None."""
    lo = None
    for (t0, v) in series:
        if t0 > t: break
        lo = v
    return lo


def pair_nearest(a, b, tol=0.5):
    """Pair series a with nearest-in-time samples of b -> [(t, va, vb)].

    Advances the pointer on TIME (<= t), then picks the nearer of the
    two neighbors: the classic advance-while-strictly-closer pointer
    deadlocks forever on duplicate timestamps (two samples of b at the
    same instant), which real logs contain.
    """
    out, j = [], 0
    for (t, va) in a:
        while j + 1 < len(b) and b[j + 1][0] <= t:
            j += 1
        k = j
        if j + 1 < len(b) and abs(b[j + 1][0] - t) < abs(b[j][0] - t):
            k = j + 1
        if b and abs(b[k][0] - t) <= tol:
            out.append((t, va, b[k][1]))
    return out


# ----------------------------------------------------------------- load

NUM_VARS = {
    "NAV_HEADING", "NAV_SPEED", "NAV_X", "NAV_Y", "GYRO_Z_LVL_IMU",
    "DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_THRUST", "DESIRED_RUDDER",
    "BBPID_DESIRED_YAWRATE", "BBPID_MEAS_YAWRATE",
    "BBPID_FF_THRUST", "BBPID_FF_RUDDER", "BBPID_GOV_SPEED",
    "BBPID_SPEED_ITERM", "BBPID_HEADING_ITERM", "BBPID_YAWRATE_ITERM",
    "BBPID_HEADING_ERROR", "BB_MIX_SATURATION",
}
STR_VARS = {"BB_CMD_AUTHORITY", "BBPID_GATE", "BBPID_PARAMS_ACTIVE"}


def load(alog):
    """Return (phases, series): phases [(t, kv)], series var -> [(t, val)]."""
    phases = []
    series = {v: [] for v in NUM_VARS | STR_VARS}
    for line in open(alog, errors="replace"):
        parts = line.split(None, 3)
        if len(parts) < 4 or parts[0].startswith("%"):
            continue
        try:
            t = float(parts[0])
        except ValueError:
            continue
        var, val = parts[1], parts[3].strip()
        if var == "MANEUVER_PHASE":
            kv = dict(p.split("=", 1) for p in val.split(",") if "=" in p)
            phases.append((t, kv))
        elif var in STR_VARS:
            series[var].append((t, val))
        elif var in NUM_VARS:
            try:
                series[var].append((t, float(val)))
            except ValueError:
                pass
    return phases, series


def load_params(alog, series):
    """Parameter provenance: ._moos snapshot pBBPID block, overlaid with
    BBPID_PARAMS_ACTIVE republications (live tuning during the run)."""
    params, source = {}, None
    moos = alog.rsplit(".alog", 1)[0] + "._moos"
    if os.path.exists(moos):
        inside = False
        for line in open(moos, errors="replace"):
            s = line.strip()
            if s.startswith("ProcessConfig") and "pBBPID" in s:
                inside = True
                continue
            if inside:
                if s.startswith("}"):
                    break
                s = s.split("//")[0].strip()
                if "=" in s and not s.startswith("{"):
                    k, v = s.split("=", 1)
                    params[k.strip().lower()] = v.strip()
        source = "._moos"
    snaps = series.get("BBPID_PARAMS_ACTIVE", [])
    if snaps:
        # Last snapshot wins; count changes for the report.
        for kv in snaps[-1][1].split(";"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                params[k.strip().lower()] = v.strip()
        source = (source + " + " if source else "") + \
                 f"BBPID_PARAMS_ACTIVE x{len(snaps)}"
    return params, source


# ----------------------------------------------------------- windowing

def in_window(series, t0, t1):
    return [(t, v) for (t, v) in series if t0 <= t < t1]


def autonomy_windows(auth, t0, t1):
    """Sub-intervals of [t0,t1) where BB_CMD_AUTHORITY == AUTONOMY."""
    out, cur = [], None
    prior = [v for (t, v) in auth if t <= t0]
    state = prior[-1] if prior else "NONE"
    if state == "AUTONOMY": cur = t0
    for (t, v) in auth:
        if t < t0 or t >= t1:
            if t >= t1: break
            continue
        if v == "AUTONOMY" and cur is None:
            cur = t
        elif v != "AUTONOMY" and cur is not None:
            out.append((cur, t)); cur = None
    if cur is not None: out.append((cur, t1))
    return out


def filt(samples, windows):
    return [(t, v) for (t, v) in samples
            if any(a <= t < b for (a, b) in windows)]


# ------------------------------------------------------------- scoring

def rate_tracking(des, meas, t0, horizon):
    """RMS of (desired - measured) yaw rate [deg/s] over the transient."""
    pairs = pair_nearest([(t, v) for (t, v) in des if t < t0 + horizon],
                         meas)
    return rms([(a - b) * 180.0 / math.pi for (_, a, b) in pairs])


def sat_fraction(rudder, thresh=99.0):
    vals = [abs(v) for (_, v) in rudder]
    if not vals: return float("nan")
    return sum(1 for v in vals if v >= thresh) / len(vals)


def score_step(kv, t0, t1, S):
    tgt = float(kv.get("hdg", "nan"))
    hdg, gyr = S["hdg"], S["gyr"]
    m = {"kind": "step", "target": tgt}
    if len(hdg) < 5:
        m["note"] = "insufficient data"
        return m
    err0 = wrap180(tgt - hdg[0][1])
    m["step_deg"] = err0
    if abs(err0) < 5:
        m["note"] = "step too small from start"
        return m
    sgn = 1.0 if err0 > 0 else -1.0
    t63 = crossed_t = None
    overshoot, settled = 0.0, None
    for (t, h) in hdg:
        e = wrap180(tgt - h)
        if t63 is None and abs(e) <= 0.37 * abs(err0):
            t63 = t - t0
        if crossed_t is None and e * sgn <= 0:
            crossed_t = t
        if crossed_t is not None:
            overshoot = max(overshoot, -e * sgn)
        settled = e
    m["t63"] = t63
    m["overshoot"] = overshoot
    m["settled"] = settled
    m["peak_yaw"] = max((abs(v) for (_, v) in gyr), default=float("nan")) \
        * 180.0 / math.pi
    # Inner-loop evidence: how well the rate loop served the command,
    # and whether the rudder railed doing it.
    m["rate_rms"] = rate_tracking(S["des_rate"], S["meas_rate"], t0,
                                  min(10.0, t1 - t0))
    m["rud_sat_frac"] = sat_fraction(S["rud"])
    # Governor evidence: how much speed was spent on the turn.
    cmd_spd = float(kv.get("spd", "nan"))
    gov = [v for (_, v) in S["gov"]]
    m["gov_min_frac"] = (min(gov) / cmd_spd) if gov and cmd_spd > 0 \
        else float("nan")
    return m


def score_pivot(kv, t0, t1, S):
    gyr = [(t, v) for (t, v) in S["gyr"] if t > t0 + 2.0]   # skip spin-up
    m = {"kind": "pivot", "target": float(kv.get("hdg", "nan"))}
    rates = sorted(abs(v) * 180.0 / math.pi for (_, v) in gyr)
    if not rates:
        m["note"] = "no gyro data"
        return m
    m["rate_p50"] = pct(rates, .5)
    m["rate_p90"] = pct(rates, .9)
    m["rot_sign"] = 1.0 if mean(v for (_, v) in gyr) >= 0 else -1.0
    m["mean_rudder"] = mean(abs(v) for (_, v) in S["rud"])
    # Thrust-asymmetry evidence: a pivot in place should not translate.
    xy = S["xy"]
    if len(xy) >= 2:
        (x0, y0), (x1, y1) = xy[0][1], xy[-1][1]
        m["drift_m"] = math.hypot(x1 - x0, y1 - y0)
    # SIGNED body-frame surge drift -- the observable analysis/rc_cal.py
    # inverts for thrust_asymmetry, reported here so a block run is the
    # null test for whatever session 0 set. Under a constant body-surge
    # force the velocity rotates with the hull, so this is constant
    # through a pivot; a world-fixed disturbance (wind, current)
    # oscillates instead and averages out over a whole turn.
    hdg = S["hdg"]
    surges = []
    for i in range(2, len(xy) - 2):
        ta, (xa, ya) = xy[i - 2]
        tb, (xb, yb) = xy[i + 2]
        span = tb - ta
        h = latest_before(hdg, xy[i][0])
        if span <= 0 or h is None:
            continue
        th = math.radians(h)
        surges.append(((xb - xa) / span) * math.sin(th)
                      + ((yb - ya) / span) * math.cos(th))
    if surges:
        m["drift_surge"] = mean(surges)
    return m


def score_straight(kv, t0, t1, S):
    tgt = float(kv.get("hdg", "nan"))
    m = {"kind": "straight", "target": tgt}
    xy = S["xy"]
    if len(xy) < 10:
        m["note"] = "insufficient data"
        return m
    ax, ay = xy[min(len(xy) - 1, 5 * 10)][1]     # anchor ~5 s in
    th = math.radians(tgt)
    ux, uy = math.sin(th), math.cos(th)          # marine convention
    xte = [ (x - ax) * uy - (y - ay) * ux for (_, (x, y)) in xy ]
    m["xte_rms"] = rms(xte)
    m["xte_max"] = max(abs(v) for v in xte)
    _speed_metrics(kv, t0, t1, S, m, settle=5.0)
    return m


def score_speed(kv, t0, t1, S, prev_spd):
    cmd = float(kv.get("spd", "nan"))
    m = {"kind": "speed", "cmd": cmd, "prev": prev_spd}
    spd = S["spd"]
    if len(spd) < 5:
        m["note"] = "insufficient data"
        return m
    # t63 of the speed step from the previous phase's command.
    if prev_spd is not None and not math.isnan(cmd) and abs(cmd - prev_spd) > 0.05:
        thresh = prev_spd + 0.63 * (cmd - prev_spd)
        up = cmd > prev_spd
        for (t, v) in spd:
            if (up and v >= thresh) or (not up and v <= thresh):
                m["t63"] = t - t0
                break
    _speed_metrics(kv, t0, t1, S, m, settle=max(5.0, (t1 - t0) * 0.5))
    return m


def _speed_metrics(kv, t0, t1, S, m, settle):
    """Shared speed-loop evidence over the settled tail of a phase."""
    cmd = float(kv.get("spd", "nan"))
    tail = lambda series: [v for (t, v) in series if t >= t0 + settle]
    sp = tail(S["spd"])
    if sp and not math.isnan(cmd):
        m["spd_mean"] = mean(sp)
        m["spd_err"] = cmd - mean(sp)            # standing error -> speed Ki
    th = tail(S["thrust"])
    ff = tail(S["ff_thrust"])
    if th:
        m["thrust_mean"] = mean(th)
        if ff and abs(mean(th)) > 1e-6:
            m["ff_share"] = mean(ff) / mean(th)  # FF vs PID share -> ff_speed
    it = tail(S["it_speed"])
    if it:
        m["iterm_speed"] = mean(it)              # what FF fails to give, I supplies


def phase_series(s, xy, t0, t1, wins):
    """All per-phase series, autonomy-filtered."""
    g = lambda var: filt(in_window(s[var], t0, t1), wins)
    return {
        "hdg": g("NAV_HEADING"), "gyr": g("GYRO_Z_LVL_IMU"),
        "spd": g("NAV_SPEED"), "rud": g("DESIRED_RUDDER"),
        "thrust": g("DESIRED_THRUST"),
        "ff_thrust": g("BBPID_FF_THRUST"),
        "des_rate": g("BBPID_DESIRED_YAWRATE"),
        "meas_rate": g("BBPID_MEAS_YAWRATE"),
        "gov": g("BBPID_GOV_SPEED"),
        "it_speed": g("BBPID_SPEED_ITERM"),
        "xy": filt(in_window(xy, t0, t1), wins),
    }


# ------------------------------------------------------------- summary

def run_summary(s, params, t_lo, t_hi):
    m = {}
    # Mixer saturation and its cost in heading error (MOT_STR_THR_MIX).
    sat = in_window(s["BB_MIX_SATURATION"], t_lo, t_hi)
    if sat:
        m["mix_sat_frac"] = sum(1 for (_, v) in sat if v > 1.0) / len(sat)
        herr = in_window(s["BBPID_HEADING_ERROR"], t_lo, t_hi)
        on, off = [], []
        for (t, e) in herr:
            q = latest_before(sat, t)
            if q is None: continue
            (on if q > 1.0 else off).append(abs(e) * 180.0 / math.pi)
        m["hdg_err_sat"] = mean(on)
        m["hdg_err_unsat"] = mean(off)
        m["sat_samples"] = len(on)
    # Integral health vs configured limits (windup fixes verification).
    def ihealth(var, limit_key):
        vals = [abs(v) for (_, v) in in_window(s[var], t_lo, t_hi)]
        if not vals: return None
        lim = float(params.get(limit_key, "nan"))
        return {"max": max(vals),
                "limit": lim,
                "at_limit_frac": (sum(1 for v in vals if v >= 0.98 * lim)
                                  / len(vals)) if lim == lim and lim > 0
                                 else float("nan")}
    for var, key, name in (("BBPID_SPEED_ITERM", "speed_integral_limit", "i_speed"),
                           ("BBPID_YAWRATE_ITERM", "yawrate_integral_limit", "i_yawrate")):
        h = ihealth(var, key)
        if h: m[name] = h
    # Gate timeline.
    gate = in_window(s["BBPID_GATE"], t_lo, t_hi)
    if gate:
        m["gate_events"] = [(round(t - t_lo, 1), v) for (t, v) in gate]
    return m


def pivot_asymmetry(metrics):
    """CW vs CCW sustained-rate and drift comparison (MOT_THST_ASYM)."""
    cw = [p for p in metrics if p.get("rot_sign") == 1.0 and "rate_p50" in p]
    ccw = [p for p in metrics if p.get("rot_sign") == -1.0 and "rate_p50" in p]
    if not cw or not ccw:
        return None
    r_cw = mean(p["rate_p50"] for p in cw)
    r_ccw = mean(p["rate_p50"] for p in ccw)
    return {
        "rate_p50_cw": r_cw,
        "rate_p50_ccw": r_ccw,
        "drift_cw": mean(p.get("drift_m", float("nan")) for p in cw),
        "drift_ccw": mean(p.get("drift_m", float("nan")) for p in ccw),
        # COMMON MODE -> thrust_asymmetry. A wrong A pushes the boat the
        # same way in both directions, so it shows up in the mean.
        "surge_common": mean(p.get("drift_surge", float("nan"))
                             for p in cw + ccw),
        # SPLIT -> motor / prop / ESC mismatch, NOT A. Kept separate
        # because conflating the two is what made the 27 Aug pivot line
        # read as asymmetry evidence when the phase was simply broken.
        "rate_split": (abs(r_cw - r_ccw) / max(r_cw, r_ccw)
                       if max(r_cw, r_ccw) > 1e-6 else float("nan")),
    }


# ------------------------------------------------------------ full run

def score_run(alog):
    phases, s = load(alog)
    if not phases:
        raise SystemExit(
            "No MANEUVER_PHASE markers found -- is this a handling_block log?")
    params, psource = load_params(alog, s)

    # Pair NAV_X/NAV_Y by nearest-in-time (they arrive separately).
    xy = [(t, (x, y)) for (t, x, y)
          in pair_nearest(s["NAV_X"], s["NAV_Y"])]

    end_t = max(t for (t, _) in s["NAV_HEADING"]) if s["NAV_HEADING"] \
        else phases[-1][0]
    out = {"alog": alog, "params": params, "params_source": psource,
           "phases": [], "n_phases": len(phases)}
    prev_spd = None
    for i, (t0, kv) in enumerate(phases):
        t1 = phases[i + 1][0] if i + 1 < len(phases) else end_t
        name = kv.get("phase", "?")
        wins = autonomy_windows(s["BB_CMD_AUTHORITY"], t0, t1)
        S = phase_series(s, xy, t0, t1, wins)
        if name.startswith(("stepA", "stepB", "zig")):
            m = score_step(kv, t0, t1, S)
        elif name.startswith("pivot"):
            m = score_pivot(kv, t0, t1, S)
        elif name == "straight":
            m = score_straight(kv, t0, t1, S)
        elif name.startswith("speed"):
            m = score_speed(kv, t0, t1, S, prev_spd)
        else:
            m = {"kind": "other", "dur": t1 - t0}
        m["name"] = name
        m["windows"] = len(wins)
        out["phases"].append(m)
        try:
            prev_spd = float(kv.get("spd", "nan"))
        except ValueError:
            prev_spd = None
    out["summary"] = run_summary(s, params, phases[0][0], end_t)
    asym = pivot_asymmetry([p for p in out["phases"] if p["kind"] == "pivot"])
    if asym:
        out["summary"]["pivot_asymmetry"] = asym
    return out


# ---------------------------------------------------------- formatting

def fmt_phase(m):
    n = m["name"]
    paused = f"  [{m['windows']} autonomy windows]" if m["windows"] > 1 else ""
    if "note" in m:
        return f"{n:12s}  ({m['note']})" + paused
    k = m["kind"]
    if k == "step":
        return (f"{n:12s}  step {m['step_deg']:+6.1f}  "
                f"t63 {fnum(m.get('t63'), '{:5.1f}')}s  "
                f"ovs {fnum(m.get('overshoot'), '{:5.1f}')}  "
                f"settled {fnum(m.get('settled'), '{:+5.1f}')}  "
                f"peak {fnum(m.get('peak_yaw'), '{:5.1f}')}d/s  "
                f"rateRMS {fnum(m.get('rate_rms'), '{:5.1f}')}  "
                f"rudSat {fnum(m.get('rud_sat_frac'), '{:4.0%}')}  "
                f"govMin {fnum(m.get('gov_min_frac'), '{:4.0%}')}" + paused)
    if k == "pivot":
        return (f"{n:12s}  rate p50 {fnum(m.get('rate_p50'), '{:5.1f}')} "
                f"p90 {fnum(m.get('rate_p90'), '{:5.1f}')} d/s  "
                f"dir {'CW' if m.get('rot_sign', 0) > 0 else 'CCW'}  "
                f"drift {fnum(m.get('drift_m'), '{:5.1f}')} m  "
                f"|rud| {fnum(m.get('mean_rudder'), '{:5.1f}')}" + paused)
    if k == "straight":
        return (f"{n:12s}  XTE rms {fnum(m.get('xte_rms'), '{:5.2f}')} "
                f"max {fnum(m.get('xte_max'), '{:5.2f}')} m  "
                f"spdErr {fnum(m.get('spd_err'), '{:+5.2f}')}  "
                f"thrust {fnum(m.get('thrust_mean'), '{:5.1f}')}  "
                f"ffShare {fnum(m.get('ff_share'), '{:4.0%}')}" + paused)
    if k == "speed":
        return (f"{n:12s}  {fnum(m.get('prev'), '{:3.1f}')}->"
                f"{m['cmd']:3.1f} m/s  "
                f"t63 {fnum(m.get('t63'), '{:5.1f}')}s  "
                f"spdErr {fnum(m.get('spd_err'), '{:+5.2f}')}  "
                f"thrust {fnum(m.get('thrust_mean'), '{:5.1f}')}  "
                f"ffShare {fnum(m.get('ff_share'), '{:4.0%}')}  "
                f"I {fnum(m.get('iterm_speed'), '{:5.1f}')}" + paused)
    return f"{n:12s}  ({m.get('dur', 0):.0f}s)" + paused


def print_report(r):
    print(f"handling_block: {r['n_phases']} phases   {r['alog']}")
    if r["params_source"]:
        gains = {k: r["params"].get(k, "?")
                 for k in ("speed_pid", "heading_pid", "yawrate_pid")}
        print(f"params [{r['params_source']}]: " +
              "  ".join(f"{k}={v}" for k, v in gains.items()))
    print("-" * 100)
    for m in r["phases"]:
        print(fmt_phase(m))
    print("-" * 100)
    s = r["summary"]
    print("knob evidence:")
    if "mix_sat_frac" in s:
        print(f"  MOT_STR_THR_MIX : mixer saturated {s['mix_sat_frac']:.1%} of ticks; "
              f"|hdg err| {fnum(s.get('hdg_err_sat'), '{:.2f}')} deg saturated "
              f"vs {fnum(s.get('hdg_err_unsat'), '{:.2f}')} unsaturated "
              f"(n={s.get('sat_samples', 0)})")
    else:
        print("  MOT_STR_THR_MIX : BB_MIX_SATURATION not in log (bridge it)")
    a = s.get("pivot_asymmetry")
    if a:
        sc = a.get("surge_common", float("nan"))
        verdict = ""
        if sc == sc:
            verdict = ("  -> raise A" if sc > 0.05 else
                       "  -> lower A" if sc < -0.05 else "  -> null, A is good")
        print(f"  MOT_THST_ASYM   : common-mode surge drift "
              f"{fnum(sc, '{:+.3f}')} m/s{verdict}")
        print(f"  motor match     : pivot rate CW {a['rate_p50_cw']:.1f} vs "
              f"CCW {a['rate_p50_ccw']:.1f} deg/s "
              f"(split {fnum(100*a['rate_split'], '{:.0f}')}%"
              + ("; >10%, NOT thrust_asymmetry -- motor/prop/ESC)"
                 if a['rate_split'] == a['rate_split'] and a['rate_split'] > 0.10
                 else ")"))
    for key, label in (("i_speed", "speed integral "),
                       ("i_yawrate", "yawrate integral")):
        h = s.get(key)
        if h:
            print(f"  {label:16s}: max |I| {h['max']:.1f} of limit "
                  f"{h['limit']:.0f}  (at-limit {fnum(h['at_limit_frac'], '{:.1%}')})")
    g = s.get("gate_events")
    if g is not None:
        print(f"  integrate gate  : {len(g)} transitions: " +
              " ".join(f"{v}@{t:.0f}s" for (t, v) in g[:12]) +
              (" ..." if len(g) > 12 else ""))
    print("-" * 100)


NUMERIC_KEYS = ["t63", "overshoot", "settled", "peak_yaw", "rate_rms",
                "rud_sat_frac", "rate_p50", "rate_p90", "drift_m",
                "xte_rms", "spd_err", "ff_share", "iterm_speed"]


def print_compare(ra, rb):
    print(f"A: {ra['alog']}\nB: {rb['alog']}")
    pa = {m["name"]: m for m in ra["phases"]}
    pb = {m["name"]: m for m in rb["phases"]}
    print("-" * 100)
    print(f"{'phase':12s} {'metric':13s} {'A':>9s} {'B':>9s} {'delta':>9s}")
    for name, ma in pa.items():
        mb = pb.get(name)
        if not mb: continue
        for k in NUMERIC_KEYS:
            va, vb = ma.get(k), mb.get(k)
            if va is None or vb is None: continue
            if isinstance(va, float) and math.isnan(va): continue
            if isinstance(vb, float) and math.isnan(vb): continue
            print(f"{name:12s} {k:13s} {va:9.2f} {vb:9.2f} {vb - va:+9.2f}")
    print("-" * 100)
    # Param diff -- the "one knob per run" audit.
    keys = sorted(set(ra["params"]) | set(rb["params"]))
    diffs = [(k, ra["params"].get(k, "--"), rb["params"].get(k, "--"))
             for k in keys if ra["params"].get(k) != rb["params"].get(k)]
    if diffs:
        print("param changes A -> B:")
        for k, va, vb in diffs:
            print(f"  {k:24s} {va}  ->  {vb}")
    else:
        print("param changes A -> B: none detected")


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("alog", nargs="?")
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"))
    ap.add_argument("--ledger", metavar="FILE",
                    help="append this run's record as one JSON line")
    ap.add_argument("--json", action="store_true",
                    help="dump the full record as JSON instead of the table")
    args = ap.parse_args()

    if args.compare:
        print_compare(score_run(args.compare[0]), score_run(args.compare[1]))
        return
    if not args.alog:
        print(__doc__); sys.exit(1)

    r = score_run(args.alog)
    if args.json:
        print(json.dumps(r, indent=2, default=str))
    else:
        print_report(r)
    if args.ledger:
        r["scored_at"] = datetime.datetime.now().isoformat(timespec="seconds")
        with open(args.ledger, "a") as f:
            f.write(json.dumps(r, default=str) + "\n")
        print(f"appended to {args.ledger}")


if __name__ == "__main__":
    main()
