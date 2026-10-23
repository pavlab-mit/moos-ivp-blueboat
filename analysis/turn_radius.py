#!/usr/bin/env python3
"""Score turning-capability trials from a back-seat .alog.

For every BHV_FixedTurn execution (helm MODE enters *FIX_TURNING, ends
at the behavior's FT_REPORT or when MODE leaves) this reports the
quantities a capability table needs, split into the entry transient
and the steady part of the turn (after the first --settle degrees of
heading change):

    target speed / direction          from FT_REPORT and BHV settings
    R_fit    (m)   least-squares circle through NAV_X/NAV_Y, steady part
    R_kin    (m)   mean speed / mean |yaw rate|, steady part
    R_ft     (m)   BHV_FixedTurn's own avg [min-max] radius (opposite-
                   heading point pairs; drift-sensitive, cross-check)
    speed    (m/s) mean hull-frame surge (NAV_SURGE, else NAV_SPEED),
                   steady part, plus min and the value at turn entry
    yawM     (deg/s) mean turn rate from NAV_HEADING over the steady part
    rudder   (%)   p50 / p90 |DESIRED_RUDDER| and the saturated fraction
    mixer    (%)   fraction of ticks the front-seat mixer reported saturation
    reverse  (%)   fraction of ticks either motor was driven negative
    autonomy (%)   fraction of the window under BB_CMD_AUTHORITY=AUTONOMY;
                   below ~98% the trial had an RC takeover: distrust it
    turn time (s)

and the profile the controller was running (max_yawrate from the
._moos snapshot overlaid with BBPID_PARAMS_ACTIVE).

Usage:
    analysis/turn_radius.py <backseat.alog> [--settle 60] [--json out.json]
    analysis/turn_radius.py <alog> --legrun       score BHV_LegRun end turns
                                                  (LR_MODE turn1/turn2) instead
    analysis/turn_radius.py <alog> --sim          sim log: no front-seat vars
"""

import argparse
import json
import math
import os
import sys

NUM_VARS = {
    "NAV_X", "NAV_Y", "NAV_HEADING", "NAV_SPEED", "NAV_SURGE",
    "DESIRED_RUDDER", "DESIRED_THRUST", "DESIRED_SPEED",
    "BBPID_MEAS_YAWRATE", "BBPID_DESIRED_YAWRATE", "GYRO_Z_LVL_IMU",
    "BB_MIX_SATURATION", "NVGR_APPLIED_LEFT", "NVGR_APPLIED_RIGHT",
}
STR_VARS = {"MODE", "FT_REPORT", "LR_MODE", "BB_CMD_AUTHORITY",
            "BBPID_PARAMS_ACTIVE", "BHV_SETTINGS"}


# ----------------------------------------------------------------- util

def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    return s[min(len(s) - 1, int(len(s) * p))]


def mean(vals):
    vals = list(vals)
    return sum(vals) / len(vals) if vals else float("nan")


def wrap180(d):
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d


def in_window(series, t0, t1):
    return [(t, v) for (t, v) in series if t0 <= t < t1]


def kv(s):
    out = {}
    for f in s.split(","):
        if "=" in f:
            k, v = f.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def fit_circle(points):
    """Algebraic (Kasa) least-squares circle. Returns (cx, cy, r, rmse)."""
    n = len(points)
    if n < 8:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    # centre the data for conditioning
    u = [p[0] - mx for p in points]
    v = [p[1] - my for p in points]
    suu = sum(a * a for a in u); svv = sum(b * b for b in v); suv = sum(a * b for a, b in zip(u, v))
    suuu = sum(a ** 3 for a in u); svvv = sum(b ** 3 for b in v)
    suvv = sum(a * b * b for a, b in zip(u, v)); svuu = sum(b * a * a for a, b in zip(u, v))
    det = suu * svv - suv * suv
    if abs(det) < 1e-9:
        return None
    rhs1 = 0.5 * (suuu + suvv); rhs2 = 0.5 * (svvv + svuu)
    uc = (rhs1 * svv - rhs2 * suv) / det
    vc = (suu * rhs2 - suv * rhs1) / det
    r = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    rmse = math.sqrt(mean([(math.hypot(a - uc, b - vc) - r) ** 2 for a, b in zip(u, v)]))
    return (uc + mx, vc + my, r, rmse)


# ----------------------------------------------------------------- load

def load(alog):
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
        if var in STR_VARS:
            series[var].append((t, val))
        elif var in NUM_VARS:
            try:
                series[var].append((t, float(val)))
            except ValueError:
                pass
    return series


def load_params(alog, series):
    """pBBPID params: ._moos snapshot overlaid with BBPID_PARAMS_ACTIVE."""
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
                    params[k.strip().lower()] = v.strip()   # last line wins, as pBBPID does
        source = "._moos"
    snaps = series.get("BBPID_PARAMS_ACTIVE", [])
    if snaps:
        for item in snaps[-1][1].split(";"):
            if "=" in item:
                k, v = item.split("=", 1)
                params[k.strip().lower()] = v.strip()
        source = (source + " + " if source else "") + f"BBPID_PARAMS_ACTIVE x{len(snaps)}"
    return params, source


def profile_label(params):
    try:
        mxy = float(params.get("max_yawrate", "nan"))
    except ValueError:
        return "?", float("nan")
    if mxy >= 1.0:
        return "agile", mxy
    if mxy <= 0.2:
        return "slow", mxy
    return "stock", mxy


# ------------------------------------------------------------ windows

def fixed_turn_windows(series):
    """[(t0, t1, ft_report_kv or None)] from MODE transitions + FT_REPORT."""
    wins, start = [], None
    modes = series["MODE"]
    fts = series["FT_REPORT"]
    for (t, m) in modes:
        turning = m.upper().endswith("FIX_TURNING")
        if turning and start is None:
            start = t
        elif not turning and start is not None:
            wins.append([start, t, None])
            start = None
    if start is not None:
        last_t = max(t for (t, _) in modes)
        wins.append([start, last_t + 1.0, None])
    # Attach the FT_REPORT posted inside (or within 2 s after) each window.
    for w in wins:
        cands = [(t, v) for (t, v) in fts if w[0] <= t <= w[1] + 2.0]
        if cands:
            w[2] = kv(cands[-1][1])
            w[1] = min(w[1], cands[-1][0] + 0.01)
    return [tuple(w) for w in wins]


def legrun_windows(series):
    wins, start, label = [], None, None
    for (t, m) in series["LR_MODE"]:
        if m.startswith("turn") and start is None:
            start, label = t, m
        elif not m.startswith("turn") and start is not None:
            wins.append((start, t, {"lr": label}))
            start = None
    return wins


# -------------------------------------------------------------- score

def unwrap_heading(hs):
    """[(t, hdg)] -> [(t, cumulative signed change)]."""
    out, acc, prev = [], 0.0, None
    for (t, h) in hs:
        if prev is not None:
            acc += wrap180(h - prev)
        prev = h
        out.append((t, acc))
    return out


def score_window(s, t0, t1, ft, settle_deg, sim):
    m = {"t0": t0, "t1": t1, "turn_time": t1 - t0}
    if ft:
        for k in ("spd", "mod_hdg", "rudder_avg", "avg_rad", "min_rad", "max_rad"):
            if k in ft:
                try: m["ft_" + k] = float(ft[k])
                except ValueError: pass

    hdg = in_window(s["NAV_HEADING"], t0, t1)
    if len(hdg) < 4:
        m["error"] = "no heading data"
        return m
    cum = unwrap_heading(hdg)
    total = cum[-1][1]
    m["hdg_change_deg"] = total
    m["turn_dir"] = "star" if total > 0 else "port"
    # steady part: after |settle| degrees of change, until the end
    t_settle = next((t for (t, c) in cum if abs(c) >= settle_deg), None)
    if t_settle is None:
        m["warning"] = f"turned only {abs(total):.0f} deg; no steady segment"
        t_settle = t1
    m["t_settle"] = t_settle

    # authority (field only): fraction under AUTONOMY, and the last
    # contiguous AUTONOMY segment, which is the only part of an
    # interrupted turn worth measuring (the behavior restarts on hand-back).
    g0, g1 = t_settle, t1          # geometry window
    auth = s["BB_CMD_AUTHORITY"]
    if auth and not sim:
        prior = [v for (t, v) in auth if t <= t0]
        state = prior[-1] if prior else "NONE"
        segs, cur = [], (t0 if state == "AUTONOMY" else None)
        for (t, v) in auth:
            if t <= t0: continue
            if t >= t1: break
            if v == "AUTONOMY" and cur is None: cur = t
            elif v != "AUTONOMY" and cur is not None: segs.append((cur, t)); cur = None
            state = v
        if cur is not None: segs.append((cur, t1))
        aut = sum(b - a for a, b in segs)
        m["autonomy_frac"] = aut / (t1 - t0) if t1 > t0 else float("nan")
        if segs:
            a0, a1 = segs[-1]
            if len(segs) > 1 or a0 > t0 + 0.5:
                m["warning"] = (m.get("warning", "") + f" RC takeover; scored last autonomy segment "
                                f"{a0-t0:.0f}-{a1-t0:.0f} s").strip()
                # re-derive the settle point inside that segment
                cum_seg = unwrap_heading(in_window(s["NAV_HEADING"], a0, a1))
                ts = next((t for (t, c) in cum_seg if abs(c) >= settle_deg), None)
                g0, g1 = (ts if ts else a0), a1
        else:
            m["warning"] = (m.get("warning", "") + " no autonomy in window").strip()
    m["geom_t0"], m["geom_t1"] = g0, g1

    # geometry
    xs = in_window(s["NAV_X"], g0, g1); ys = in_window(s["NAV_Y"], g0, g1)
    pts = pair_xy(xs, ys)
    fit = fit_circle(pts)
    if fit:
        m["r_fit"], m["r_fit_rmse"], m["fit_n"] = fit[2], fit[3], len(pts)
    pts_all = pair_xy(in_window(s["NAV_X"], t0, t1), in_window(s["NAV_Y"], t0, t1))
    fit_all = fit_circle(pts_all)
    if fit_all:
        m["r_fit_full"] = fit_all[2]

    # speed
    spd_var = "NAV_SURGE" if (s["NAV_SURGE"] and not sim) else "NAV_SPEED"
    spd = in_window(s[spd_var], g0, g1)
    spd_entry = in_window(s[spd_var], t0, min(t0 + 2.0, t1))
    m["speed_var"] = spd_var
    if spd:
        m["speed_mean"] = mean(v for _, v in spd)
        m["speed_min"] = min(v for _, v in spd)
    if spd_entry:
        m["speed_entry"] = mean(v for _, v in spd_entry)
    des = in_window(s["DESIRED_SPEED"], t0, t1)
    if des:
        m["speed_cmd"] = pct([v for _, v in des], 0.5)

    # yaw rate. Mean rate from the heading itself over the geometry
    # window (total change / duration) drives R_kin; the controller's
    # measured rate (rad/s -> deg/s) is reported as p50/p90 beside it.
    hseg = unwrap_heading(in_window(s["NAV_HEADING"], g0, g1))
    if len(hseg) >= 2 and hseg[-1][0] > hseg[0][0]:
        m["yawrate_mean"] = abs(hseg[-1][1] - hseg[0][1]) / (hseg[-1][0] - hseg[0][0])
        if "speed_mean" in m and m["yawrate_mean"] > 0:
            m["r_kin"] = m["speed_mean"] / (m["yawrate_mean"] * math.pi / 180.0)
    yr = [abs(v) * 180.0 / math.pi for _, v in in_window(s["BBPID_MEAS_YAWRATE"], g0, g1)]
    if yr:
        m["yawrate_p50"] = pct(yr, 0.5)
        m["yawrate_p90"] = pct(yr, 0.9)
    elif "yawrate_mean" in m:
        m["yawrate_p50"] = m["yawrate_mean"]

    rud = [abs(v) for _, v in in_window(s["DESIRED_RUDDER"], g0, g1)]
    if rud:
        m["rudder_p50"] = pct(rud, 0.5); m["rudder_p90"] = pct(rud, 0.9)
        m["rudder_sat_frac"] = sum(1 for v in rud if v >= 99.0) / len(rud)
    thr = [v for _, v in in_window(s["DESIRED_THRUST"], t0, t1)]
    if thr:
        m["thrust_neg_frac"] = sum(1 for v in thr if v < 0) / len(thr)

    if not sim:
        sat = [v for _, v in in_window(s["BB_MIX_SATURATION"], t0, t1)]
        if sat:
            m["mix_sat_frac"] = sum(1 for v in sat if v > 1.0) / len(sat)
        l = in_window(s["NVGR_APPLIED_LEFT"], t0, t1); r = in_window(s["NVGR_APPLIED_RIGHT"], t0, t1)
        if l or r:
            neg = sum(1 for _, v in l if v < -1.0) + sum(1 for _, v in r if v < -1.0)
            m["motor_reverse_frac"] = neg / (len(l) + len(r))
    return m


def pair_xy(xs, ys, tol=0.3):
    out, j = [], 0
    for (t, x) in xs:
        while j + 1 < len(ys) and ys[j + 1][0] <= t:
            j += 1
        if ys and abs(ys[j][0] - t) <= tol:
            out.append((x, ys[j][1]))
    return out


# -------------------------------------------------------------- report

def f(v, w=6, d=1):
    if v is None or (isinstance(v, float) and v != v):
        return "--".rjust(w)
    return f"{v:{w}.{d}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("alog")
    ap.add_argument("--settle", type=float, default=60.0,
                    help="deg of heading change before the steady segment starts")
    ap.add_argument("--legrun", action="store_true", help="score LR_MODE turn windows")
    ap.add_argument("--sim", action="store_true", help="sim log (no front-seat vars)")
    ap.add_argument("--json", help="write all metrics to this file")
    a = ap.parse_args()

    s = load(a.alog)
    params, src = load_params(a.alog, s)
    prof, mxy = profile_label(params)
    wins = legrun_windows(s) if a.legrun else fixed_turn_windows(s)
    if not wins:
        sys.exit("turn_radius: no turn windows found "
                 + ("(LR_MODE turn*)" if a.legrun else "(MODE *FIX_TURNING)"))

    rows = [score_window(s, t0, t1, ft, a.settle, a.sim) for (t0, t1, ft) in wins]
    for i, r in enumerate(rows):
        r["turn"] = i + 1

    print(f"turn_radius: {os.path.basename(a.alog)}")
    print(f"  profile {prof}  (max_yawrate = {mxy} rad/s"
          f"{'' if mxy != mxy else f' = {mxy*180/math.pi:.1f} deg/s'};"
          f" yaw_priority_gain = {params.get('yaw_priority_gain', '?')};"
          f" allow_reverse = {params.get('allow_reverse', '?')})  [{src}]")
    print(f"  {len(rows)} turn window(s); steady segment starts after {a.settle:.0f} deg of heading change\n")

    hdr = (f"  {'#':>2} {'cmd':>5} {'dir':>4} {'R_fit':>6} {'R_kin':>6} {'R_ft':>6} {'ft[min-max]':>12} "
           f"{'v_mean':>6} {'v_min':>6} {'v_ent':>6} {'yawM':>6} {'rud50':>6} {'rud90':>6} "
           f"{'sat%':>5} {'mix%':>5} {'rev%':>5} {'aut%':>5} {'time':>6} {'turn':>6}")
    print(hdr)
    for r in rows:
        ftr = (f"[{r['ft_min_rad']:.1f}-{r['ft_max_rad']:.1f}]"
               if "ft_min_rad" in r and "ft_max_rad" in r else "--")
        pc = lambda k: f(100 * r[k], 5, 0) if k in r else "--".rjust(5)
        print(f"  {r['turn']:>2} {f(r.get('ft_spd', r.get('speed_cmd')), 5, 2)} {r.get('turn_dir','--'):>4} "
              f"{f(r.get('r_fit'))} {f(r.get('r_kin'))} {f(r.get('ft_avg_rad'))} {ftr:>12} "
              f"{f(r.get('speed_mean'), 6, 2)} {f(r.get('speed_min'), 6, 2)} {f(r.get('speed_entry'), 6, 2)} "
              f"{f(r.get('yawrate_mean', r.get('yawrate_p50')))} {f(r.get('rudder_p50'), 6, 0)} {f(r.get('rudder_p90'), 6, 0)} "
              f"{pc('rudder_sat_frac')} {pc('mix_sat_frac')} {pc('motor_reverse_frac')} "
              f"{pc('autonomy_frac')} {f(r.get('turn_time'), 6, 0)} {f(r.get('hdg_change_deg'), 6, 0)}"
              + (f"   ! {r['warning']}" if "warning" in r else "")
              + (f"   ! {r['error']}" if "error" in r else ""))

    # per-speed summary
    by = {}
    for r in rows:
        k = r.get("ft_spd", r.get("speed_cmd"))
        if k is None or "r_fit" not in r: continue
        by.setdefault(round(k, 2), []).append(r)
    if by:
        print(f"\n  capability table ({prof}):")
        print(f"  {'setpoint':>8} {'n':>2} {'R_fit mean':>10} {'[min-max]':>12} {'v_mean':>7} {'yawM':>6} {'rud90':>6} {'rev%':>5}")
        for k in sorted(by):
            rs = by[k]
            rf = [r["r_fit"] for r in rs]
            rev = [r["motor_reverse_frac"] for r in rs if "motor_reverse_frac" in r]
            print(f"  {k:>8.2f} {len(rs):>2} {mean(rf):>10.1f} {f'[{min(rf):.1f}-{max(rf):.1f}]':>12} "
                  f"{mean(r['speed_mean'] for r in rs if 'speed_mean' in r):>7.2f} "
                  f"{mean(r.get('yawrate_mean', r.get('yawrate_p50')) for r in rs if 'yawrate_mean' in r or 'yawrate_p50' in r):>6.1f} "
                  f"{mean(r['rudder_p90'] for r in rs if 'rudder_p90' in r):>6.0f} "
                  f"{(f'{100*mean(rev):.0f}' if rev else '--'):>5}")

    if a.json:
        with open(a.json, "w") as fh:
            json.dump({"alog": a.alog, "profile": prof, "params": params, "turns": rows}, fh, indent=1)
        print(f"\n  wrote {a.json}")

    bad = [r for r in rows if "error" in r]
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
