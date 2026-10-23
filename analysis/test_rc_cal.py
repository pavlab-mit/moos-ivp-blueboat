#!/usr/bin/env python3
"""Self-test for rc_cal.py: can it recover a KNOWN thrust asymmetry?

Synthesises back-seat alogs from a forward model -- pick A_true and a
drag curve, integrate the trajectory the boat would actually fly, write
the log, and check rc_cal reads A_true back out. The forward model here
and the inverse in rc_cal.py were written to the same identity, so this
checks the algebra and the segmentation, not the physics; the physics
assumption (pivot surge drag == straight-line surge drag) is the ~20%
caveat documented in the playbook.

    analysis/test_rc_cal.py
"""

import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DRAG = (8.0, 4.0)                       # effort = 8v + 4v|v|


def v_for(m):
    lo, hi = -5.0, 5.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if DRAG[0] * mid + DRAG[1] * mid * abs(mid) < m:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def m_eff(L, R, A):
    return ((L if L >= 0 else L / A) + (R if R >= 0 else R / A)) / 2


def synth(path, a_set, a_true, gps_d=0.0):
    """gps_d: lateral offset of the antenna from the turn centre, + = stbd.
    The logged NAV_X/NAV_Y follow the ANTENNA, so the synthetic track has
    to carry it around the circle exactly as the real boat does."""
    rows, st = [], {"t": 10.0, "x": 0.0, "y": 0.0, "h": 0.0}

    def run(L, R, yr, dur):
        v, dt = v_for(m_eff(L, R, a_true)), 0.05
        for _ in range(int(dur / dt)):
            th = math.radians(st["h"])
            st["x"] += v * math.sin(th) * dt
            st["y"] += v * math.cos(th) * dt
            st["h"] = (st["h"] + yr * dt) % 360
            st["t"] += dt
            # antenna sits gps_d to starboard of the turn centre
            ah = math.radians(st["h"])
            ax = st["x"] + gps_d * math.cos(ah)
            ay = st["y"] - gps_d * math.sin(ah)
            rows.append((st["t"], ax, ay, st["h"], L, R))

    for eff in (15.0, 25.0, 40.0):
        run(eff, eff, 0.0, 25)
        run(0.0, 0.0, 0.0, 6)
    for u, yr in ((40.0, 30.0), (55.0, 38.0)):
        run(u, -u * a_set, yr, 22)
        run(0.0, 0.0, 0.0, 6)
        run(-u * a_set, u, -yr, 22)
        run(0.0, 0.0, 0.0, 6)

    out = [f"%% synthetic  a_set={a_set} a_true={a_true}"]
    for (t, x, y, h, L, R) in rows[::2]:
        for var, val in (("NAV_X", x), ("NAV_Y", y), ("NAV_HEADING", h),
                         ("NVGR_APPLIED_LEFT", L), ("NVGR_APPLIED_RIGHT", R)):
            out.append(f"{t:.3f}   {var}  sim  {val:.5f}")
        out.append(f"{t:.3f}   BB_CMD_AUTHORITY  sim  RC")
    open(path, "w").write("\n".join(out) + "\n")


def offset_recovered(out):
    for line in out.splitlines():
        if line.strip().startswith("GPS lever arm"):
            return float(line.split("=")[1].split("m")[0].strip())
    return None


def recovered(path, a_set, solve_offset=False):
    r = subprocess.run(
        [sys.executable, os.path.join(HERE, "rc_cal.py"), path,
         "--asym", str(a_set)]
        + ([] if solve_offset else ["--gps-offset", "0"]),
        capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if "best single A across" in line:
            return float(line.split(":")[1].split("(")[0].strip()), r.stdout
    return None, r.stdout + r.stderr


def mixer(surge, yaw, A, mixp=0.6):
    """Port of bb::allocate, so the synthetic ladder is the one the boat flies."""
    ss, ts = max(-100, min(100, yaw)) * .01, max(-100, min(100, surge)) * .01
    A = max(A, 1.0); low = -1.0 / A; best = (1.0 + low) * .5
    rng = (max(ts, 0.0) - low) if ts < best else (1.0 - best)
    ss = max(-rng, min(rng, ss)); ts = max(low, min(1.0, ts))
    mx, mn = ts + abs(ss), ts - abs(ss)
    sv = mx if mx > (mn / low) else (mn / low)
    if sv > 1.0:
        f = 1.0 / sv
        if mixp >= 0.5:
            ss *= f + (1.0 - f) * ((mixp - 0.5) / 0.5)
            ts = (1.0 - abs(ss)) if ts >= best else (abs(ss) + low)
    L, R = ts + ss, ts - ss
    if R < 0: R *= A
    if L < 0: L *= A
    return 100 * L, 100 * R


def synth_null(path, a_set, a_true, gps_d=0.0, steer=28.0,
               ladder=(8, 5, 3, 0, -3, -5)):
    """A null-ladder run: steering held, throttle stepped through the null."""
    rows, st = [], {"t": 10.0, "x": 0.0, "y": 0.0, "h": 0.0}

    def run(L, R, yr, dur):
        v, dt = v_for(m_eff(L, R, a_true)), 0.05
        for _ in range(int(dur / dt)):
            th = math.radians(st["h"])
            st["x"] += v * math.sin(th) * dt
            st["y"] += v * math.cos(th) * dt
            st["h"] = (st["h"] + yr * dt) % 360
            st["t"] += dt
            ah = math.radians(st["h"])
            rows.append((st["t"], st["x"] + gps_d * math.cos(ah),
                         st["y"] - gps_d * math.sin(ah), st["h"], L, R))

    for eff in (10.0, 20.0, 35.0):
        run(eff, eff, 0.0, 25); run(0.0, 0.0, 0.0, 6)
    for sgn in (+1, -1):
        for t in ladder:
            L, R = mixer(t, sgn * steer, a_set)
            yr = sgn * 32.0
            run(L, R, yr, 22)
            run(0.0, 0.0, 0.0, 6)

    out = [f"%% synthetic null  a_set={a_set} a_true={a_true} d={gps_d}"]
    for (t, x, y, h, L, R) in rows[::2]:
        for var, val in (("NAV_X", x), ("NAV_Y", y), ("NAV_HEADING", h),
                         ("NVGR_APPLIED_LEFT", L), ("NVGR_APPLIED_RIGHT", R)):
            out.append(f"{t:.3f}   {var}  sim  {val:.5f}")
        out.append(f"{t:.3f}   BB_CMD_AUTHORITY  sim  RC")
    open(path, "w").write("\n".join(out) + "\n")


def synth_wander(path, a_set, a_true, gps_d=0.0, current=(0.0, 0.0),
                 tau_surge=6.0):
    """A realistically-flown pivot: the operator wanders the sticks, the
    boat's surge LAGS the force, and a world-fixed current sets underneath.
    Nothing here is held constant, which is the point."""
    rows, st = [], {"t": 10.0, "x": 0.0, "y": 0.0, "h": 0.0, "v": 0.0}
    dt = 0.05

    def step(L, R, yr, dur):
        # first-order surge: the force is m_eff, the boat catches up slowly
        target = v_for(m_eff(L, R, a_true))
        for _ in range(int(dur / dt)):
            st["v"] += (target - st["v"]) * dt / tau_surge
            th = math.radians(st["h"])
            st["x"] += (st["v"] * math.sin(th) + current[0]) * dt
            st["y"] += (st["v"] * math.cos(th) + current[1]) * dt
            st["h"] = (st["h"] + yr * dt) % 360
            st["t"] += dt
            ah = math.radians(st["h"])
            rows.append((st["t"], st["x"] + gps_d * math.cos(ah),
                         st["y"] - gps_d * math.sin(ah), st["h"], L, R))

    for eff in (10.0, 20.0, 35.0):
        step(eff, eff, 0.0, 25); step(0.0, 0.0, 0.0, 6)
    # sticks wander continuously through the ladder, both directions
    for sgn in (+1, -1):
        for i in range(28):
            t = 8.0 - i * 0.5 + (2.0 if i % 3 == 0 else -1.0)   # no fixed rungs
            steer = 28.0 + (12.0 if i % 4 < 2 else 0.0)
            L, R = mixer(t, sgn * steer, a_set)
            step(L, R, sgn * 32.0, 6.0)
        step(0.0, 0.0, 0.0, 6)

    out = [f"%% synthetic wander a_set={a_set} a_true={a_true}"]
    for (t, x, y, h, L, R) in rows[::2]:
        for var, val in (("NAV_X", x), ("NAV_Y", y), ("NAV_HEADING", h),
                         ("NVGR_APPLIED_LEFT", L), ("NVGR_APPLIED_RIGHT", R)):
            out.append(f"{t:.3f}   {var}  sim  {val:.5f}")
        out.append(f"{t:.3f}   BB_CMD_AUTHORITY  sim  RC")
    open(path, "w").write("\n".join(out) + "\n")


CASES = [
    # (a_set, a_true, what it represents)
    (1.6, 2.2, "reverse weaker than assumed -> forward drift, raise A"),
    (1.6, 1.6, "already correct -> the NULL, drift ~0"),
    (2.4, 1.8, "A too high -> aft drift, lower A"),
    (1.0, 1.9, "no compensation at all"),
]

def main():
    fails = 0
    for a_set, a_true, note in CASES:
        with tempfile.NamedTemporaryFile(suffix=".alog", delete=False) as f:
            path = f.name
        synth(path, a_set, a_true)
        got, out = recovered(path, a_set)
        os.unlink(path)
        if got is None:
            print(f"FAIL  A_set={a_set} A_true={a_true}: no A_true in output")
            print(out)
            fails += 1
            continue
        err = abs(got - a_true)
        ok = err <= 0.03
        print(f"{'ok  ' if ok else 'FAIL'}  A_set={a_set}  A_true={a_true}  "
              f"recovered={got:.2f}  err={err:.3f}   ({note})")
        if not ok:
            fails += 1
    # --- the GPS lever arm: it masquerades as a motor imbalance, so the
    # --- solver must pull it back out and still land on the right A.
    print()
    for d_true in (-0.35, +0.40):
        with tempfile.NamedTemporaryFile(suffix=".alog", delete=False) as f:
            path = f.name
        synth(path, 1.6, 2.2, gps_d=d_true)
        got, out = recovered(path, 1.6, solve_offset=True)
        d_got = offset_recovered(out)
        ok = (got is not None and abs(got - 2.2) <= 0.05
              and d_got is not None and abs(d_got - d_true) <= 0.03)
        os.unlink(path)
        print(f"{'ok  ' if ok else 'FAIL'}  gps_d={d_true:+.2f} A_true=2.2  "
              f"recovered d={d_got if d_got is None else round(d_got,3)} "
              f"A={got if got is None else round(got,2)}   "
              "(lever arm solved from CW/CCW pairs)")
        if not ok:
            fails += 1
    # --- the NULL ladder: hold steering, step the throttle, find the ratio
    # --- where drift crosses zero. Must recover A_true with no drag curve.
    print()
    for a_set, a_true, d_true in ((2.8, 2.55, -0.356), (2.8, 3.05, -0.356)):
        with tempfile.NamedTemporaryFile(suffix=".alog", delete=False) as f:
            path = f.name
        synth_null(path, a_set, a_true, gps_d=d_true)
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, "rc_cal.py"), path,
             "--asym", str(a_set), "--gps-offset", str(d_true)],
            capture_output=True, text=True)
        got = brack = None
        for line in r.stdout.splitlines():
            ln = line.strip()
            if ln.startswith("both: null A ="):        # continuous fit
                got = float(ln.split("=")[1].split()[0]); brack = True
            elif ln.startswith("both:") and "ratio" in ln and got is None:
                got = float(ln.split("ratio")[1].split()[0])
                brack = "NOT BRACKETED" not in ln
        os.unlink(path)
        ok = got is not None and abs(got - a_true) <= 0.06 and brack
        print(f"{'ok  ' if ok else 'FAIL'}  NULL ladder A_set={a_set} A_true={a_true}  "
              f"recovered={got if got is None else round(got,2)}  "
              f"bracketed={brack}")
        if not ok:
            fails += 1
            print(r.stdout[-1500:])
    # --- CONTINUOUS null: the operator does not hold discrete rungs, the
    # --- ratio wanders, and surge is a LAGGED response to the force.
    print()
    for a_set, a_true, cur in ((2.8, 1.82, (0.10, -0.05)),
                               (1.6, 2.00, (0.0, 0.0))):
        with tempfile.NamedTemporaryFile(suffix=".alog", delete=False) as f:
            path = f.name
        synth_wander(path, a_set, a_true, gps_d=-0.356, current=cur)
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, "rc_cal.py"), path,
             "--asym", str(a_set), "--gps-offset", "-0.356"],
            capture_output=True, text=True)
        got = None
        for line in r.stdout.splitlines():
            if line.strip().startswith("both: null A ="):
                got = float(line.split("=")[1].split()[0])
        os.unlink(path)
        ok = got is not None and abs(got - a_true) <= 0.10
        print(f"{'ok  ' if ok else 'FAIL'}  CONTINUOUS wander A_set={a_set} "
              f"A_true={a_true} current={math.hypot(*cur):.2f} m/s  "
              f"recovered={got if got is None else round(got,2)}")
        if not ok:
            fails += 1
            print(r.stdout[-1200:])
    # a ladder that does NOT bracket the null must SAY SO, not quietly
    # report the biased edge value
    with tempfile.NamedTemporaryFile(suffix=".alog", delete=False) as f:
        path = f.name
    synth_wander(path, 1.6, 3.6, gps_d=-0.356)      # unreachable at A_set 1.6
    r = subprocess.run(
        [sys.executable, os.path.join(HERE, "rc_cal.py"), path,
         "--asym", "1.6", "--gps-offset", "-0.356"],
        capture_output=True, text=True)
    os.unlink(path)
    flagged = "NULL NOT BRACKETED" in r.stdout
    print(f"{'ok  ' if flagged else 'FAIL'}  unbracketed ladder (A_true=3.6 "
          f"out of reach at A_set=1.6) -> flagged={flagged}")
    if not flagged:
        fails += 1
    total = len(CASES) + 7
    print()
    print("PASS" if not fails else f"{fails}/{total} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
