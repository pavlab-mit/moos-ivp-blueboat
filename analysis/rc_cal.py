#!/usr/bin/env python3
"""Score a session-0 open-loop RC calibration run (playbook section 4).

Identifies pThrustMix's `thrust_asymmetry` (MOT_THST_ASYM) from a single
RC-flown log, using the boat's own drag curve as the calibrated ruler.
No autonomy, no controller, no parameter sweep -- `thrust_asymmetry` is
front-seat config and is not live-settable, so it gets measured once and
set once.

THE IDENTITY
    A side commanded FORWARD with effort c delivers c.
    A side commanded REVERSE with effort c delivers c/A_true, because
    reverse is the weaker direction -- that is what A means.
    The mixer has already multiplied the reverse side by A_set, so the
    net per-side mean forward-equivalent effort actually delivered is

        m_eff(A_true) = ( L/[A_true if L<0 else 1]
                        + R/[A_true if R<0 else 1] ) / 2

    On a STRAIGHT run both sides are forward, nothing is compensated,
    and m_eff is just the commanded effort -- so the straight segments
    calibrate effort against steady speed (the drag curve).
    On a PIVOT the two sides fight, and the residual m_eff shows up as
    fore/aft drift. Read that drift back through the drag curve and
    solve the equation above for A_true.

    Drift FORWARD  -> reverse weaker than assumed -> raise A.
    Drift AFT      -> lower A.
    A CW-vs-CCW SPLIT is a different fault -- motor, prop or ESC
    mismatch, not A -- and is reported separately.

THE GPS LEVER ARM (measure this, do not fit it)
    NAV_X/NAV_Y follow the GPS antenna, not the centre of rotation. On a
    catamaran the antenna sits on a pontoon, offset d to starboard of the
    turn centre, and a boat spinning at omega carries it around a circle:

        surge_measured = surge_true - omega * d

    omega flips sign with turn direction, so this error ADDS to one
    direction and SUBTRACTS from the other -- it masquerades exactly as a
    left/right motor imbalance. At 40 deg/s and d = 0.35 m it is 0.24 m/s,
    which is the same size as the drift being measured. Pass the measured
    offset with --gps-offset. Without it the tool solves d from matched
    CW/CCW pairs, which works but spends the data that would otherwise
    cross-check the answer.

    The GYRO's offset does not matter: angular velocity is the same at
    every point of a rigid body.

WHY BODY-FRAME SURGE IS THE RIGHT DRIFT MEASURE
    Under a constant body-frame surge force the velocity vector rotates
    with the hull, so body-frame surge velocity is CONSTANT during a
    steady pivot (the track is a circle of radius v/omega). A world-fixed
    disturbance -- wind, current -- oscillates in the body frame instead
    and averages out over a whole number of turns. Hence the run card
    asks for >= 360 deg per pivot.

Usage:
    analysis/rc_cal.py <backseat.alog>
    analysis/rc_cal.py <backseat.alog> --asym 1.6     # declared A_set
    analysis/rc_cal.py <backseat.alog> --json

Reads only back-seat variables: NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED,
NVGR_APPLIED_LEFT/RIGHT, BB_CMD_AUTHORITY, and RC_MARK if bridged.
"""

import argparse
import json
import math
import sys


GPS_OFFSET = [None]     # set from --gps-offset; None = solve it from the data
CONFIRM = [False]       # --confirm: A is already set, just check drift ~ 0


# ----------------------------------------------------------------- util

def load(alog, want):
    """{var: [(t, value)]} for the vars we need, numeric where possible."""
    out = {v: [] for v in want}
    with open(alog) as f:
        for line in f:
            if line.startswith("%"):
                continue
            p = line.split(None, 3)
            if len(p) < 4:
                continue
            var = p[1]
            if var not in out:
                continue
            try:
                t = float(p[0])
            except ValueError:
                continue
            raw = p[3].strip()
            try:
                out[var].append((t, float(raw)))
            except ValueError:
                out[var].append((t, raw))
    for v in out:
        out[v].sort(key=lambda r: r[0])
    return out


def latest(series, t):
    lo, hi, r = 0, len(series) - 1, None
    while lo <= hi:
        m = (lo + hi) // 2
        if series[m][0] <= t:
            r = series[m][1]
            lo = m + 1
        else:
            hi = m - 1
    return r


def mean(v):
    v = list(v)
    return sum(v) / len(v) if v else float("nan")


def wrap180(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def fnum(v, f="{:7.2f}"):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "     --"
    return f.format(v)


# ------------------------------------------------------- resampled frame

def build_frame(S, hz=5.0):
    """One time-aligned record per tick: efforts, heading, position, and
    the derived body-frame surge velocity and yaw rate."""
    xs, ys = S["NAV_X"], S["NAV_Y"]
    if len(xs) < 20 or len(ys) < 20:
        return []
    t0, t1 = xs[0][0], xs[-1][0]
    dt = 1.0 / hz
    rows, t = [], t0
    while t <= t1:
        x, y = latest(xs, t), latest(ys, t)
        h = latest(S["NAV_HEADING"], t)
        L, R = latest(S["NVGR_APPLIED_LEFT"], t), latest(S["NVGR_APPLIED_RIGHT"], t)
        if None in (x, y, h, L, R):
            t += dt
            continue
        rows.append({"t": t, "x": x, "y": y, "hdg": h, "L": L, "R": R,
                     "auth": latest(S["BB_CMD_AUTHORITY"], t)})
        t += dt

    # Central differences for world velocity and yaw rate.
    for i, r in enumerate(rows):
        a, b = rows[max(0, i - 2)], rows[min(len(rows) - 1, i + 2)]
        span = b["t"] - a["t"]
        if span <= 0:
            r["surge"] = r["yawrate"] = float("nan")
            continue
        vx, vy = (b["x"] - a["x"]) / span, (b["y"] - a["y"]) / span
        th = math.radians(r["hdg"])
        # MOOS local grid: x east, y north; heading 0 = north, CW positive.
        r["surge"] = vx * math.sin(th) + vy * math.cos(th)
        r["yawrate"] = wrap180(b["hdg"] - a["hdg"]) / span
    return rows


# --------------------------------------------------------- segmentation

def segment(rows, min_len=8.0):
    """Split the run into straight and pivot segments by applied effort.

    straight: both sides forward, matched within 5% -> drag-curve point
    pivot:    sides opposed, both meaningful -> asymmetry evidence
    """
    def kind(r):
        L, R = r["L"], r["R"]
        if abs(L) < 3.0 and abs(R) < 3.0:
            return "idle"
        if L > 0 and R > 0 and abs(L - R) <= 5.0:
            return "straight"
        if L * R < 0 and min(abs(L), abs(R)) > 8.0:
            return "pivot"
        return "other"

    segs, cur = [], None
    for r in rows:
        k = kind(r)
        # Effort must also be steady within a segment, or a slow sweep
        # through the stick range would read as one long segment.
        if cur and k == cur["kind"] and abs(r["L"] - cur["L0"]) < 6.0 \
                and abs(r["R"] - cur["R0"]) < 6.0:
            cur["rows"].append(r)
            continue
        if cur and cur["rows"][-1]["t"] - cur["rows"][0]["t"] >= min_len:
            segs.append(cur)
        cur = {"kind": k, "rows": [r], "L0": r["L"], "R0": r["R"]}
    if cur and cur["rows"][-1]["t"] - cur["rows"][0]["t"] >= min_len:
        segs.append(cur)
    return [s for s in segs if s["kind"] in ("straight", "pivot")]


def settled(seg, skip=4.0):
    """Rows after the transient at the head of a segment."""
    t0 = seg["rows"][0]["t"]
    return [r for r in seg["rows"] if r["t"] >= t0 + skip]


# ----------------------------------------------------------- drag curve

def fit_drag(straights):
    """Least-squares m_eff = a*v + b*v^2 through the origin.

    Both sides are forward on a straight run, so the commanded per-side
    effort IS the delivered forward-equivalent effort -- no dependence on
    A. That is what makes this an independent ruler.
    """
    pts = []
    for s in straights:
        rows = settled(s)
        if len(rows) < 10:
            continue
        v = mean(r["surge"] for r in rows)
        m = mean((r["L"] + r["R"]) / 2.0 for r in rows)
        if v > 0.15:
            pts.append((v, m, len(rows)))
    if len(pts) < 2:
        return None, pts
    s11 = sum(v * v for v, _, _ in pts)
    s12 = sum(v ** 3 for v, _, _ in pts)
    s22 = sum(v ** 4 for v, _, _ in pts)
    b1 = sum(v * m for v, m, _ in pts)
    b2 = sum(v * v * m for v, m, _ in pts)
    det = s11 * s22 - s12 * s12
    if abs(det) < 1e-12:
        a = b1 / s11 if s11 else float("nan")
        return (a, 0.0), pts
    a = (b1 * s22 - b2 * s12) / det
    b = (s11 * b2 - s12 * b1) / det
    return (a, b), pts


def effort_for(drag, v):
    """Signed forward-equivalent effort that holds surge velocity v."""
    a, b = drag
    return a * v + b * v * abs(v)


# ------------------------------------------------------ the A_true solve

def m_eff(L, R, A):
    le = L if L >= 0 else L / A
    re = R if R >= 0 else R / A
    return (le + re) / 2.0


def solve_asym(L, R, target, lo=1.0, hi=8.0):
    """Bisect A_true so that m_eff(L, R, A) == target.

    Monotone increasing in A (a bigger A means the reverse side is
    credited with less delivered thrust, so the net rises), so bisection
    is safe. Returns None when the target is outside the reachable range.
    """
    if m_eff(L, R, lo) > target or m_eff(L, R, hi) < target:
        return None
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if m_eff(L, R, mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def score_pivot(seg, drag, gps_offset=0.0):
    rows = settled(seg)
    if len(rows) < 10:
        return None
    L, R = mean(r["L"] for r in rows), mean(r["R"] for r in rows)
    yr = mean(r["yawrate"] for r in rows)
    m = {
        "t0": rows[0]["t"], "dur": rows[-1]["t"] - rows[0]["t"],
        "L": L, "R": R,
        "u": max(abs(L), abs(R)) if L * R < 0 else float("nan"),
        # REVERSE side over FORWARD side -- not right/left, which would
        # invert on CCW pivots. With the throttle stick centred this ratio
        # is exactly A_set, because that is the compensation the mixer
        # applied to the reversing side and nothing else touched it.
        "applied_ratio": (max(abs(L), abs(R)) / min(abs(L), abs(R)))
                         if min(abs(L), abs(R)) > 1e-6 else float("nan"),
        "yawrate": yr,
        "dir": "CW" if yr > 0 else "CCW",
        "turns": abs(yr) * (rows[-1]["t"] - rows[0]["t"]) / 360.0,
        "drift": mean(r["surge"] for r in rows),
    }
    # Decompose: a body-fixed force traces a circle (no net world
    # displacement over whole turns); wind or current adds a straight
    # world-frame translation on top. Reporting the world part separately
    # is what lets a run exonerate -- or convict -- the environment.
    span = lambda i: rows[min(len(rows)-1, i+2)]["t"] - rows[max(0, i-2)]["t"]
    dx = lambda i, k: (rows[min(len(rows)-1, i+2)][k] - rows[max(0, i-2)][k])
    vv = [(dx(i, "x") / span(i), dx(i, "y") / span(i))
          for i in range(len(rows)) if span(i) > 0]
    if vv:
        m["world_drift"] = math.hypot(mean(a for a, _ in vv),
                                      mean(b for _, b in vv))
    # Is the straight-line drag curve even applicable? The rotation drives
    # flow over the hull at ~omega*halflength. When that is comparable to
    # the drift speed, axial drag during a pivot is nothing like axial drag
    # in a straight line and the ruler does not apply.
    # Remove the GPS lever-arm term before anything downstream sees it.
    m["lever"] = -math.radians(yr) * gps_offset
    m["drift_corr"] = m["drift"] - m["lever"]
    if abs(m["drift_corr"]) > 1e-6:
        m["spin_ratio"] = abs(math.radians(yr) * 0.6 / m["drift_corr"])
    if drag:
        m["m_eff_meas"] = effort_for(drag, m["drift_corr"])
        m["A_true"] = solve_asym(L, R, m["m_eff_meas"])
    return m


# ------------------------------------------------------------------ run

def _g(c, A):
    return c if c >= 0 else c / A


def joint_fit(piv, drag):
    """Best single A across ALL pivots, and how badly it misfits.

    Every pivot must be explained by the same A. Fitting them jointly and
    reporting the residual is the difference between a measurement and an
    average of disagreeing numbers.
    """
    if not drag or not piv:
        return None
    obs = [(p["L"], p["R"], effort_for(drag, p.get("drift_corr", p["drift"])))
           for p in piv]

    def sse(A):
        return sum((( _g(L, A) + _g(R, A)) / 2.0 - t) ** 2 for L, R, t in obs)

    best = min(((sse(1.0 + i * 0.005), 1.0 + i * 0.005) for i in range(1400)))
    return {"A": best[1], "rms": math.sqrt(best[0] / len(obs)),
            "n": len(obs)}


def linearity(piv, drag, asym_set=None):
    """A FIXED A predicts net thrust proportional to command magnitude.

    Compare same-direction pivots: if the thrust ratio tracks the command
    ratio, one scalar A can describe the boat. If it runs away, reverse
    efficiency is falling with command and no single number will do.
    """
    if not drag:
        return None
    # A trimmed null ladder deliberately walks the applied ratio away from
    # A_set, so "more command" is not comparable across rungs. Restrict to
    # untrimmed pivots, where the ratio still equals A_set.
    if asym_set:
        piv = [p for p in piv
               if p["applied_ratio"] == p["applied_ratio"]
               and abs(p["applied_ratio"] - asym_set) / asym_set < 0.05]
    out = []
    for d in ("CW", "CCW"):
        g = sorted((p for p in piv if p["dir"] == d),
                   key=lambda p: max(abs(p["L"]), abs(p["R"])))
        if len(g) < 2:
            continue
        b = g[0]
        bcmd = max(abs(b["L"]), abs(b["R"]))
        bthr = effort_for(drag, b.get("drift_corr", b["drift"]))
        if abs(bthr) < 1e-6:
            continue
        for p in g[1:]:
            out.append({
                "dir": d,
                "cmd_ratio": max(abs(p["L"]), abs(p["R"])) / bcmd,
                "thr_ratio": effort_for(drag, p.get("drift_corr", p["drift"])) / bthr,
            })
    return out


def null_continuous(rows, gps_offset, tau=None, direction=None,
                    with_current=True, hz=5.0):
    """The null, estimated from EVERY opposed-effort sample in the run.

    Segment-and-average assumes the operator holds discrete rungs at
    constant effort. Flying two sticks on a spinning boat, they do not:
    the applied ratio wanders continuously, so a "segment" spans many
    ratios and averaging over it is meaningless. This treats the run as
    what it is -- a continuous sweep -- and regresses drift on ratio.

    Two corrections make that honest:

    * LAG. Surge drift is the boat's dynamic response to the force, not
      an instantaneous readout: it takes several seconds to build. So
      drift at time t is paired with the ratio averaged over the
      PRECEDING tau seconds. tau is scanned, and the null's insensitivity
      to it is itself evidence the fit is real.
    * CURRENT. A world-fixed set oscillates in the body frame as the boat
      spins. Over whole turns it averages out, but it inflates scatter --
      and over partial turns it biases. Fitting sin/cos of heading
      absorbs it and reports it.

    Null = the ratio at which drift crosses zero, where the two sides'
    delivered thrust cancels and A_true = |R|/|L| exactly.
    """
    raw = []
    for r in rows:
        L, R = r["L"], r["R"]
        if L * R >= 0 or min(abs(L), abs(R)) < 8.0:
            continue
        if r["surge"] != r["surge"] or r["yawrate"] != r["yawrate"]:
            continue
        if direction == "CW" and r["yawrate"] <= 0:
            continue
        if direction == "CCW" and r["yawrate"] >= 0:
            continue
        raw.append((r["t"], max(abs(L), abs(R)) / min(abs(L), abs(R)),
                    r["surge"] + math.radians(r["yawrate"]) * gps_offset,
                    r["yawrate"], r["hdg"]))
    if len(raw) < 100:
        return None

    def build(t_lag):
        n = max(1, int(t_lag * hz))
        out = []
        for i, (t, _, dr, yr, h) in enumerate(raw):
            w = [q for q in raw[max(0, i - n):i + 1] if t - q[0] <= t_lag + 0.01]
            if t_lag > 0 and len(w) < n * 0.8:
                continue
            out.append((sum(q[1] for q in w) / len(w), dr, yr, h))
        return out

    def solve(d, wc):
        cols = [lambda q: q[0], lambda q: 1.0]
        if wc:
            cols += [lambda q: math.sin(math.radians(q[3])),
                     lambda q: math.cos(math.radians(q[3]))]
        k = len(cols)
        M = [[sum(cols[i](q) * cols[j](q) for q in d) for j in range(k)]
             for i in range(k)]
        v = [sum(cols[i](q) * q[1] for q in d) for i in range(k)]
        for i in range(k):
            pv = max(range(i, k), key=lambda rr: abs(M[rr][i]))
            M[i], M[pv] = M[pv], M[i]
            v[i], v[pv] = v[pv], v[i]
            if abs(M[i][i]) < 1e-12:
                return None
            for rr in range(i + 1, k):
                f = M[rr][i] / M[i][i]
                for c in range(i, k):
                    M[rr][c] -= f * M[i][c]
                v[rr] -= f * v[i]
        x = [0.0] * k
        for i in reversed(range(k)):
            x[i] = (v[i] - sum(M[i][j] * x[j] for j in range(i + 1, k))) / M[i][i]
        if abs(x[0]) < 1e-9:
            return None
        yb = sum(q[1] for q in d) / len(d)
        pred = lambda q: sum(x[i] * cols[i](q) for i in range(k))
        sstot = sum((q[1] - yb) ** 2 for q in d)
        r2 = (1 - sum((q[1] - pred(q)) ** 2 for q in d) / sstot) if sstot > 1e-12 else float("nan")
        return {"A": -x[1] / x[0], "slope": x[0], "r2": r2, "n": len(d),
                "current": (math.hypot(x[2], x[3]) if wc else None)}

    def solve_local(d, wc):
        """The null is a LOCAL property, so fit it locally.

        drift is not linear in ratio: m_eff = (L/2)*(1 - ratio/A) and L
        itself falls as ratio rises, then drift = drag^-1(m_eff) bends on
        top of that. A straight line across the whole ladder is therefore
        pulled by its far end -- measured on synthetic data as up to 0.3
        of error. Fit globally once to locate the crossing, then re-fit
        using only samples near it, twice.
        """
        f = solve(d, wc)
        if not f:
            return None
        for _ in range(3):
            w = [q for q in d if abs(q[0] - f["A"]) <= 0.6]
            if len(w) < 60:
                break
            g = solve(w, wc)
            if not g or abs(g["A"] - f["A"]) < 0.005:
                f = g or f
                break
            f = g
        loc = [q for q in d if abs(q[0] - f["A"]) <= 0.6]
        f["n_local"] = len(loc)
        # Is the null actually SURROUNDED by data, or sitting at the edge
        # of what the sticks reached? A one-sided local window means the
        # crossing is extrapolated, and the fit will read biased without
        # ever looking uncertain.
        if len(loc) < 60:
            # The crossing landed outside -- or at the very edge of -- the
            # ratios the sticks actually reached. Default-to-fine here is
            # how a fully extrapolated answer would slip through looking
            # confident, so it is an explicit failure.
            f["balance"] = f["drift_balance"] = float("nan")
            f["bracketed"] = False
        else:
            above = sum(1 for q in loc if q[0] > f["A"])
            f["balance"] = above / len(loc)
            pos = sum(1 for q in loc if q[1] > 0)
            f["drift_balance"] = pos / len(loc)
            f["bracketed"] = 0.15 <= f["balance"] <= 0.85 and \
                             0.15 <= f["drift_balance"] <= 0.85
        return f

    taus = [tau] if tau is not None else [0, 2, 4, 6, 8, 10, 12]
    scan = []
    for t_lag in taus:
        d = build(t_lag)
        if len(d) < 50:
            continue
        f = solve_local(d, with_current)
        if f:
            f["tau"] = t_lag
            scan.append(f)
    if not scan:
        return None
    best = max(scan, key=lambda f: f["r2"])
    best["scan"] = scan
    best["seconds"] = len(raw) / hz
    # Spread of the null across the lag scan: if the answer barely moves,
    # the lag is not doing the work and the fit is real.
    As = [f["A"] for f in scan if f["tau"] >= 2]
    best["A_spread"] = (max(As) - min(As)) if As else float("nan")
    return best


def confirm_null(rows, gps_offset, hz=5.0):
    """Confirmation pass: A is already set, so just ask whether the drift
    is actually zero.

    Nothing to regress here -- a confirmation run is flown at ZERO throttle
    trim, so the applied ratio is pinned at A_set and a fit would be
    degenerate. The question is simply whether the lever-corrected surge
    drift is zero, split by direction and by how hard the turn was, since
    a residual that changes sign with direction is a motor mismatch rather
    than a wrong A.
    """
    buckets = {}
    for r in rows:
        L, R = r["L"], r["R"]
        if L * R >= 0 or min(abs(L), abs(R)) < 8.0:
            continue
        if r["surge"] != r["surge"] or r["yawrate"] != r["yawrate"]:
            continue
        cmd = max(abs(L), abs(R))
        key = ("CW" if r["yawrate"] > 0 else "CCW",
               "hard" if cmd >= 80.0 else "moderate")
        drift = r["surge"] + math.radians(r["yawrate"]) * gps_offset
        ratio = cmd / min(abs(L), abs(R))
        buckets.setdefault(key, []).append((drift, ratio, r["yawrate"]))
    out = []
    for (d, band), v in sorted(buckets.items()):
        if len(v) < 40:
            continue
        out.append({"dir": d, "band": band, "n": len(v),
                    "seconds": len(v) / hz,
                    "drift": mean(x[0] for x in v),
                    "ratio": mean(x[1] for x in v),
                    "rate": mean(abs(x[2]) for x in v)})
    return out or None


def null_crossing(piv, direction=None):
    """Where does drift cross zero as a function of the applied ratio?

    THE NULL METHOD (playbook section 4). At zero drift the two sides'
    delivered thrust cancels exactly:

        L + R/A_true = 0     ->     A_true = |R| / |L|

    which holds for ANY opposed pair regardless of how the mixer produced
    it, needs no drag curve, and carries no spin-drag term (zero velocity
    is zero drag whatever the drag law). So instead of inverting an
    absolute drift measurement we only have to find where drift changes
    sign, and INTERPOLATE the ratio there.

    Straight-line least squares of drift against ratio; the intercept is A.
    """
    g = [p for p in piv if direction is None or p["dir"] == direction]
    g = [p for p in g
         if p.get("drift_corr") is not None and p["applied_ratio"] == p["applied_ratio"]]
    if len(g) < 3:
        return None
    xs = [p["applied_ratio"] for p in g]
    ys = [p["drift_corr"] for p in g]
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx, sxy = sum(x * x for x in xs), sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    b = (n * sxy - sx * sy) / den
    a = (sy - b * sx) / n
    if abs(b) < 1e-9:
        return None
    # The global line is only a sanity check. drift-vs-ratio is CURVED
    # (m_eff is linear in ratio only at fixed L, and drift = drag^-1(m_eff)
    # bends on top of that), so a straight line through the whole ladder
    # lands biased. The null is a LOCAL question: interpolate between the
    # two adjacent rungs that actually straddle zero.
    A_global = -a / b
    A_local = None
    order = sorted(zip(xs, ys))
    for (x0, y0), (x1, y1) in zip(order, order[1:]):
        if (y0 <= 0.0 <= y1) or (y1 <= 0.0 <= y0):
            if abs(y1 - y0) > 1e-9:
                A_local = x0 + (x1 - x0) * (0.0 - y0) / (y1 - y0)
            break
    ybar = sy / n
    sstot = sum((y - ybar) ** 2 for y in ys)
    ssres = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    return {
        "A": A_local if A_local is not None else A_global,
        "A_local": A_local, "A_global": A_global, "slope": b, "n": n,
        "r2": (1 - ssres / sstot) if sstot > 1e-12 else float("nan"),
        "ratio_lo": min(xs), "ratio_hi": max(xs),
        "drift_lo": min(ys), "drift_hi": max(ys),
        # The whole point is bracketing. Extrapolating to a sign change we
        # never observed is exactly the failure the drag-curve method had.
        "bracketed": min(ys) < 0.0 < max(ys),
    }


def solve_offset(piv):
    """Solve the GPS lateral offset from matched-command CW/CCW pairs.

    At the same command magnitude the true surge drift is the same in both
    directions, so whatever splits them is the lever arm:
        v_cw - v_ccw = -(w_cw - w_ccw) * d
    """
    cw = [p for p in piv if p["L"] > 0]
    ccw = [p for p in piv if p["L"] < 0]
    if not cw or not ccw:
        return None, []
    cmd = lambda p: max(abs(p["L"]), abs(p["R"]))
    ds = []
    for a in cw:
        b = min(ccw, key=lambda q: abs(cmd(q) - cmd(a)))
        if abs(cmd(b) - cmd(a)) / cmd(a) > 0.08:
            continue
        wa, wb = math.radians(a["yawrate"]), math.radians(b["yawrate"])
        if abs(wa - wb) < 1e-6:
            continue
        ds.append({"cmd": cmd(a), "d": (b["drift"] - a["drift"]) / (wa - wb)})
    return (mean(x["d"] for x in ds) if ds else None), ds


def score(alog, asym_declared):
    want = ["NAV_X", "NAV_Y", "NAV_HEADING", "NAV_SPEED", "BB_CMD_AUTHORITY",
            "NVGR_APPLIED_LEFT", "NVGR_APPLIED_RIGHT", "RC_MARK", "RC_CH6"]
    S = load(alog, want)
    rows = build_frame(S)
    if not rows:
        return {"error": "no NAV_X/NAV_Y in log -- is this the back-seat alog?"}

    auton = [r for r in rows if r["auth"] == "AUTONOMY"]
    segs = segment(rows)
    straights = [s for s in segs if s["kind"] == "straight"]
    pivots = [s for s in segs if s["kind"] == "pivot"]

    drag, drag_pts = fit_drag(straights)
    # Two passes: the first only to solve the lever arm if it was not given.
    piv0 = [p for p in (score_pivot(s, drag) for s in pivots) if p]
    solved, solved_pts = solve_offset(piv0)
    off = GPS_OFFSET[0]
    off_source = "measured (--gps-offset)"
    if off is None:
        off, off_source = (solved or 0.0), ("SOLVED from CW/CCW pairs"
                                            if solved else "assumed 0")
    piv = [p for p in (score_pivot(s, drag, off) for s in pivots) if p]

    # A_set inferred from the data: on a pure pivot the mixer's applied
    # ratio IS A_set, so this cross-checks the declared value AND
    # confirms the operator's throttle stick was actually centred.
    ratios = [p["applied_ratio"] for p in piv
              if p["applied_ratio"] == p["applied_ratio"]]
    inferred = mean(ratios) if ratios else float("nan")

    good = [p for p in piv if p.get("A_true")]
    cw = [p for p in good if p["dir"] == "CW"]
    ccw = [p for p in good if p["dir"] == "CCW"]

    return {
        "alog": alog,
        "duration": rows[-1]["t"] - rows[0]["t"],
        "autonomy_samples": len(auton),
        # RC_MARK is a counter and the front-seat bridge retransmits it at
        # 10 Hz, so the log holds thousands of rows of a handful of values.
        # Presses = distinct values, not rows.
        "marks": len({v for _, v in S["RC_MARK"]}),
        "drag": drag, "drag_pts": drag_pts,
        "gps_offset": off, "gps_offset_source": off_source,
        "gps_offset_pts": solved_pts,
        "drag_vmin": min((v for v, _, _ in drag_pts), default=None),
        "pivots": piv,
        "asym_declared": asym_declared,
        "asym_inferred": inferred,
        "A_true": mean(p["A_true"] for p in good) if good else None,
        "A_true_cw": mean(p["A_true"] for p in cw) if cw else None,
        "A_true_ccw": mean(p["A_true"] for p in ccw) if ccw else None,
        "rate_cw": mean(abs(p["yawrate"]) for p in cw) if cw else None,
        "rate_ccw": mean(abs(p["yawrate"]) for p in ccw) if ccw else None,
        "confirm": confirm_null(rows, off),
        "cont": {"both": null_continuous(rows, off),
                 "CW": null_continuous(rows, off, direction="CW"),
                 "CCW": null_continuous(rows, off, direction="CCW")},
        "null": {"both": null_crossing(piv),
                 "CW": null_crossing(piv, "CW"),
                 "CCW": null_crossing(piv, "CCW")},
        "joint": joint_fit(piv, drag),
        "linearity": linearity(piv, drag, asym_declared),
        "drift_all_same_sign": (all(p["drift_corr"] > 0 for p in piv)
                                or all(p["drift_corr"] < 0 for p in piv)) if piv else False,
        "drift_sign": (1 if piv and mean(p["drift_corr"] for p in piv) > 0 else -1),
    }


def report(r):
    if "error" in r:
        print("rc_cal: " + r["error"])
        return 1
    print(f"rc_cal (session 0, open loop)   {r['alog']}")
    print(f"  {r['duration']:.0f} s logged, {r['marks']} RC marks, "
          f"declared thrust_asymmetry = {r['asym_declared']}")
    print("-" * 88)

    # --- guard rails first: a run flown wrong should say so loudly.
    fatal = []
    if r["autonomy_samples"]:
        fatal.append(f"{r['autonomy_samples']} samples with "
                     "BB_CMD_AUTHORITY=AUTONOMY -- this must be an RC-only "
                     "run; relaunch with --rc-cal and leave CH6 in RC")
    if len(r["drag_pts"]) < 2:
        fatal.append(f"only {len(r['drag_pts'])} usable straight segment(s); "
                     "the drag curve needs >= 2 distinct speeds (3 is better)")
    if not r["pivots"]:
        fatal.append("no pivot segments found (need opposed L/R efforts "
                     "held >= 8 s)")

    print("drag curve  (straight runs: both sides forward, no compensation)")
    for v, m, n in sorted(r["drag_pts"]):
        print(f"    v {v:5.2f} m/s   effort {m:6.2f}%   ({n} samples)")
    if r["drag"]:
        a, b = r["drag"]
        print(f"    fit: effort = {a:.2f}*v + {b:.2f}*v|v|")
    else:
        print("    fit: INSUFFICIENT DATA")
    print()

    print(f"GPS lever arm: d = {r['gps_offset']:+.3f} m "
          f"({abs(r['gps_offset']):.2f} m to "
          f"{'port' if r['gps_offset'] < 0 else 'starboard'})  [{r['gps_offset_source']}]")
    for x in r["gps_offset_pts"]:
        print(f"    from the cmd {x['cmd']:.0f} CW/CCW pair: d = {x['d']:+.3f} m")
    if "SOLVED" in r["gps_offset_source"]:
        print("    ^ FITTED, not measured. Tape-measure the antenna offset from")
        print("      the centre of rotation and pass --gps-offset: it is a")
        print("      constant, and fitting it spends the CW/CCW cross-check.")
    print()

    print("pivots      (opposed efforts: the residual shows up as drift)")
    print("    dir  turns   cmd%    L%      R%   rate d/s   raw   lever    corr"
          "   spin   A_true")
    for p in r["pivots"]:
        cmd = max(abs(p["L"]), abs(p["R"]))
        print(f"    {p['dir']:>3s} {p['turns']:6.2f} {cmd:6.1f} "
              f"{fnum(p['L'],'{:7.1f}')} {fnum(p['R'],'{:7.1f}')} "
              f"{fnum(p['yawrate'],'{:9.1f}')} "
              f"{fnum(p.get('drift'),'{:6.2f}')} "
              f"{fnum(p.get('lever'),'{:7.2f}')} "
              f"{fnum(p.get('drift_corr'),'{:7.2f}')} "
              f"{fnum(p.get('spin_ratio'),'{:6.2f}')} "
              f"{fnum(p.get('A_true'),'{:8.2f}')}")
        if p["turns"] < 1.0:
            print(f"         ^ only {p['turns']:.2f} turns: wind/current does "
                  "not average out below 1 full rotation")
        if r["drag_vmin"] and abs(p.get("drift_corr", 0)) < r["drag_vmin"]:
            print(f"         ^ drift {abs(p.get('drift_corr',0)):.2f} m/s is BELOW the "
                  f"slowest straight ({r['drag_vmin']:.2f} m/s): "
                  "drag curve extrapolated")
    print()

    print("-" * 88)
    inf, dec = r["asym_inferred"], r["asym_declared"]
    if inf == inf:
        flag = ""
        if abs(inf - dec) / dec > 0.05:
            flag = ("   <-- MISMATCH: throttle stick was not centred, or "
                    "A_set is not what was declared")
        print(f"  A_set inferred from applied ratio : {inf:.2f}  "
              f"(declared {dec:.2f}){flag}")

    cf = r.get("confirm")
    if cf and CONFIRM[0]:
        print("  CONFIRMATION (A already set; is the drift actually zero?)")
        print(f"    {'dir':>4} {'band':>9} {'ratio':>6} {'rate d/s':>9} "
              f"{'secs':>6} {'drift m/s':>10}")
        worst = 0.0
        for b in cf:
            print(f"    {b['dir']:>4} {b['band']:>9} {b['ratio']:6.2f} "
                  f"{b['rate']:9.1f} {b['seconds']:6.0f} {b['drift']:+10.3f}")
            worst = max(worst, abs(b["drift"]))
        print()
        # +/-0.15 m/s of residual drift corresponds to roughly +/-0.1 in A
        # on this hull, so that is the useful pass line.
        if worst <= 0.15:
            print(f"    PASS: worst residual {worst:.3f} m/s -> A is right to "
                  "about +/-0.1")
        else:
            print(f"    residual {worst:.3f} m/s exceeds 0.15 -> A is off by "
                  "roughly "
                  f"{0.1*worst/0.15:.1f}; sign says "
                  + ("raise it" if mean(b['drift'] for b in cf) > 0 else "lower it"))
        ds = [b["drift"] for b in cf if b["dir"] == "CW"]
        cs = [b["drift"] for b in cf if b["dir"] == "CCW"]
        if ds and cs and abs(mean(ds) - mean(cs)) > 0.15:
            print(f"    NOTE: CW {mean(ds):+.3f} vs CCW {mean(cs):+.3f} -- a residual "
                  "that flips with")
            print("          direction is a motor/prop/ESC mismatch, not A. Check "
                  "--gps-offset first.")
        print()

    cb = (r.get("cont") or {}).get("both")
    if cb:
        print("  NULL METHOD, continuous fit over every opposed-effort sample")
        print(f"  ({cb['seconds']:.0f} s of pivoting; no drag curve, no spin-drag term):")
        cw, ccw = r["cont"].get("CW"), r["cont"].get("CCW")
        for lbl, k in (("both", cb), ("CW", cw), ("CCW", ccw)):
            if k:
                print(f"    {lbl:>4s}: null A = {k['A']:.2f}   "
                      f"R^2 {k['r2']:.3f}  n={k['n']}  lag {k['tau']:.0f}s")
        print(f"    lag scan {' '.join(f'{f["tau"]:.0f}s:{f["A"]:.2f}' for f in cb['scan'] if f['tau'] >= 2)}")
        if not cb.get("bracketed", True):
            print(f"    *** NULL NOT BRACKETED ({cb.get('n_local', 0)} samples near it; "
                  f"{fnum(100*cb.get('drift_balance', float('nan')), '{:.0f}')}% drifted forward)")
            print("        The crossing sits at or beyond the EDGE of the ratios the")
            print("        sticks actually reached, so this number is EXTRAPOLATED.")
            print("        Extend the throttle range until the boat clearly drifts")
            print("        BOTH ways, and re-fly.")
        print(f"      -> null moves {cb['A_spread']:.2f} across the whole lag scan"
              + ("  (insensitive: the fit is real)" if cb["A_spread"] < 0.15
                 else "  <-- lag-sensitive, treat with care"))
        if cb.get("current") is not None:
            print(f"    world-fixed set (current + wind): {cb['current']:.3f} m/s "
                  "-- fitted out, not assumed away")
        if cw and ccw:
            print(f"    CW vs CCW: {abs(cw['A']-ccw['A']):.2f} apart"
                  + ("  (the lever arm cancels between them -- good cross-check)"
                     if abs(cw["A"] - ccw["A"]) < 0.25 else
                     "  <-- disagree; check --gps-offset and motor match"))
        print()
        print(f"  ==> set thrust_asymmetry = {cb['A']:.2f}")
        print("      plug_pThrustMix.moos, front seat -- not live, next start.")
        print()

    nb = (r.get("null") or {}).get("both")
    if nb and not cb:
        print("  NULL METHOD (no drag curve, no spin-drag term):")
        for lbl in ("both", "CW", "CCW"):
            k = (r.get("null") or {}).get(lbl)
            if not k:
                continue
            tag = "" if k.get("A_local") else "  [extrapolated, no bracket]"
            print(f"    {lbl:>4s}: drift = 0 at ratio {k['A']:.2f}{tag}   "
                  f"(n={k['n']}, R^2 {k['r2']:.3f}, "
                  f"ratios {k['ratio_lo']:.2f}-{k['ratio_hi']:.2f}, "
                  f"drift {k['drift_lo']:+.2f}..{k['drift_hi']:+.2f} m/s)"
                  + ("" if k["bracketed"] else "   <-- NOT BRACKETED"))
        if not nb["bracketed"]:
            print("    ^ drift never changed sign: the crossing is EXTRAPOLATED.")
            print("      Extend the throttle ladder until the boat visibly drifts")
            print("      the other way -- that bracket is the whole method.")
        cw, ccw = (r["null"].get("CW"), r["null"].get("CCW"))
        if cw and ccw:
            print(f"    CW/CCW agreement: {abs(cw['A']-ccw['A']):.2f}"
                  + ("  (the lever arm cancels between them -- good cross-check)"
                     if abs(cw["A"] - ccw["A"]) < 0.25 else
                     "  <-- disagree; check --gps-offset and motor match"))
        if nb["bracketed"]:
            print()
            print(f"  ==> set thrust_asymmetry = {nb['A']:.2f}")
            print("      plug_pThrustMix.moos, front seat -- not live, next start.")
            print("      NOTE: raising A shrinks the mixer feasible region")
            print("      (max steering at zero throttle = 1/A), so peak pivot rate")
            print("      falls. Check it still clears the 25 deg/s bar.")
        print()

    # ---- what is robust vs what is not
    print()
    print("  ROBUST (independent of the drag curve):")
    if r["drift_all_same_sign"]:
        way = "FORWARD" if r["drift_sign"] > 0 else "AFT"
        print(f"    * every pivot drifts {way} -> "
              + ("reverse is WEAKER than the mixer assumes: A is too LOW"
                 if r["drift_sign"] > 0 else
                 "reverse is STRONGER than assumed: A is too HIGH"))
    else:
        print("    * pivot drift changes sign between pivots -- no clear verdict")
    if r["A_true_cw"] and r["A_true_ccw"]:
        print(f"    * after lever-arm removal, CW {r['A_true_cw']:.2f} vs "
              f"CCW {r['A_true_ccw']:.2f}"
              + ("   (agree -> motors matched)"
                 if abs(r['A_true_cw'] - r['A_true_ccw']) < 0.4 else
                 "   (still split -> real motor/prop/ESC mismatch)"))
    if r["rate_cw"] and r["rate_ccw"]:
        sp = abs(r["rate_cw"] - r["rate_ccw"]) / max(r["rate_cw"], r["rate_ccw"])
        print(f"    * pivot rate CW {r['rate_cw']:.1f} vs CCW {r['rate_ccw']:.1f} deg/s "
              f"(split {100*sp:.0f}%)")

    lin = r.get("linearity") or []
    runaway = [x for x in lin if x["cmd_ratio"] > 1.0
               and x["thr_ratio"] / x["cmd_ratio"] > 1.4]
    print()
    print("  MODEL FIT:")
    j = r.get("joint")
    if j:
        print(f"    * best single A across all {j['n']} pivots: {j['A']:.2f}  "
              f"(RMS residual {j['rms']:.2f}% effort)")
    for x in lin:
        print(f"    * {x['dir']}: command x{x['cmd_ratio']:.2f} -> "
              f"required thrust x{x['thr_ratio']:.2f}"
              + ("   <-- a fixed A predicts these equal" if
                 x["thr_ratio"] / max(x["cmd_ratio"], 1e-6) > 1.4 else ""))
    if runaway:
        print("    * required thrust outruns command. Either reverse efficiency")
        print("      genuinely falls with stick, OR the low-drift pivots are")
        print("      under-served by the drag curve. `spin` decides: it is")
        print("      omega*halflength / drift, i.e. how much of the flow over")
        print("      the hull is SPIN rather than translation. A pivot with")
        print("      spin >> 1 has far more drag than the straight-line curve")
        print("      predicts, so its A is UNDER-estimated. Trust low-spin rows.")
        best = [p for p in r["pivots"]
                if p.get("spin_ratio") and p["spin_ratio"] < 0.8 and p.get("A_true")]
        if best:
            print(f"    * lowest-spin rows (most trustworthy): A = "
                  + ", ".join(f"{p['A_true']:.2f}" for p in best))

    print()
    if r["A_true"]:
        lo = min(p["A_true"] for p in r["pivots"] if p.get("A_true"))
        hi = max(p["A_true"] for p in r["pivots"] if p.get("A_true"))
        confident = (not runaway) and (hi - lo) < 0.5
        print(f"  ==> thrust_asymmetry: per-pivot estimates span {lo:.2f}-{hi:.2f}"
              f", best single fit {j['A']:.2f}" if j else "")
        if confident:
            print(f"      set thrust_asymmetry = {r['A_true']:.2f}")
        else:
            print("      SPREAD IS TOO WIDE TO SET FROM THIS RUN ALONE. The sign")
            print("      is certain, the magnitude is not. Move A toward the")
            print("      high-command estimates and NULL-TEST it: zero drift means")
            print("      zero net force whatever the drag curve says.")
        print("      plug_pThrustMix.moos, front seat -- not live, next start.")
    else:
        print("  ==> thrust_asymmetry: NOT DETERMINED")

    if fatal:
        print()
        print("  PROBLEMS WITH THIS RUN:")
        for f in fatal:
            print(f"    * {f}")
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("alog")
    ap.add_argument("--asym", type=float, default=1.6,
                    help="thrust_asymmetry the front seat was running "
                         "(default 1.6; pThrustMix config is not in the "
                         "back-seat ._moos snapshot)")
    ap.add_argument("--gps-offset", type=float, default=None,
                    help="MEASURED lateral offset of the GPS antenna from the "
                         "centre of rotation, metres, + = starboard. Omit and "
                         "it is solved from matched CW/CCW pivot pairs.")
    ap.add_argument("--confirm", action="store_true",
                    help="confirmation run: A is already set and the run was "
                         "flown at zero throttle trim, so report whether the "
                         "drift is actually zero rather than fitting a null.")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    GPS_OFFSET[0] = a.gps_offset
    CONFIRM[0] = a.confirm
    r = score(a.alog, a.asym)
    if a.json:
        print(json.dumps(r, indent=2, default=str))
        return 0
    return report(r)


if __name__ == "__main__":
    sys.exit(main())
