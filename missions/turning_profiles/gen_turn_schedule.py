#!/usr/bin/env python3
"""Generate the turning_profiles behavior schedule and op-region plugs.

One BHV_LegRun block (the racetrack that carries the boat to every
turn at the trial speed) and one BHV_FixedTurn block (one turn_spec
per trial: 355 deg at a hard heading offset, the configured speed,
a timeout, and the flag that hands control back to the leg run).
The LAST turn's flags end the mission: LEG_RUN=false, RETURN=true.

The number of leg speeds equals the number of turns, so BHV_LegRun
fires exactly one mid-leg FIX_TURN per trial and the fixed-turn
schedule can never be exhausted while legs are still running (an
exhausted schedule would leave FIX_TURN latched true and the helm
with nothing to do).

Also writes the op-region plug from the same polygon used for the
clearance check below, so the box the boat is held in and the box
the geometry was checked against are one and the same.

Geometry check (refuses to write if it fails): every fixed-turn circle
(radius --check-radius, the largest radius the profile can produce)
and every leg-run end turn must sit inside the op-region polygon with
at least --margin metres to spare.

Usage (the launcher does this):
  gen_turn_schedule.py --speeds 0.75,1.0,1.25,1.5 --trials 2 --dir star \
      --timeout 75 --turn-rad 7 --check-radius 6 --out plugs/plug_turn_schedule.bhv
"""

import argparse
import math
import sys

DEFAULT_LEG = "45,-40,40,65"           # x, y, length, compass heading
# blueboat_dynamics op region after "widen ope region for new points"
# (commit 8632255): the track moved toward the dock, away from sailors.
DEFAULT_POLY = "90,40: -75.54,-54.26: -22,-167: 142,-77"


def parse_poly(s):
    pts = []
    for p in s.split(":"):
        x, y = p.split(",")
        pts.append((float(x), float(y)))
    return pts


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def inside(p, poly):
    x, y = p
    c = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def clearance(p, poly):
    """Distance from p to the nearest op-region edge; negative if outside."""
    d = min(seg_dist(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))
    return d if inside(p, poly) else -d


def unit(hdg_deg):
    h = math.radians(hdg_deg)
    return math.sin(h), math.cos(h)          # compass heading -> (east, north)


def side(hdg_deg, which):
    """Unit vector to starboard (or port) of a compass heading."""
    off = 90.0 if which == "star" else -90.0
    return unit(hdg_deg + off)


def geometry(leg, turn_rad, dir_, check_r):
    """Return list of (label, center, radius) circles the boat will trace."""
    cx, cy, length, ang = leg
    ux, uy = unit(ang)
    p1 = (cx - 0.5 * length * ux, cy - 0.5 * length * uy)   # leg start (heading ang)
    p2 = (cx + 0.5 * length * ux, cy + 0.5 * length * uy)
    sx, sy = side(ang, dir_)
    circles = []
    # Fixed turns start at the leg midpoint on BOTH legs: outbound (hdg ang)
    # and return (hdg ang+180). Turning to the same hand puts the two
    # circles on opposite sides of the leg.
    circles.append(("fixed turn, outbound leg", (cx + check_r * sx, cy + check_r * sy), check_r))
    circles.append(("fixed turn, return leg", (cx - check_r * sx, cy - check_r * sy), check_r))
    # Leg-run end turns: turn2 (at p2, arriving on hdg ang) to starboard,
    # turn1 (at p1, arriving on hdg ang+180) to port -> both on the
    # starboard side of the outbound heading, an oval on one side.
    circles.append(("leg-run end turn at p2", (p2[0] + turn_rad * sx, p2[1] + turn_rad * sy), turn_rad))
    circles.append(("leg-run end turn at p1", (p1[0] + turn_rad * sx, p1[1] + turn_rad * sy), turn_rad))
    return p1, p2, circles


def fmt(v):
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speeds", default="0.75,1.0,1.25,1.5",
                    help="comma list of trial speeds, m/s (speed-major order)")
    ap.add_argument("--trials", type=int, default=2, help="turns per speed")
    ap.add_argument("--dir", choices=("star", "port"), default="star")
    ap.add_argument("--mod-hdg", type=float, default=90.0,
                    help="heading offset commanded during the turn (deg)")
    ap.add_argument("--fix", type=float, default=355.0,
                    help="heading change that completes a turn (deg)")
    ap.add_argument("--timeout", type=float, default=75.0,
                    help="per-turn timeout (s); the turn completes and reports anyway")
    ap.add_argument("--leg", default=DEFAULT_LEG, help="x,y,len,ang of the leg")
    ap.add_argument("--turn-rad", type=float, default=7.0,
                    help="BHV_LegRun end-turn radius (m)")
    ap.add_argument("--mid-leg-pct", type=float, default=50.0)
    ap.add_argument("--check-radius", type=float, default=6.0,
                    help="largest fixed-turn radius this profile can produce (m)")
    ap.add_argument("--poly", default=DEFAULT_POLY, help="op-region polygon")
    ap.add_argument("--margin", type=float, default=12.0,
                    help="required clearance from every circle to the op region (m)")
    ap.add_argument("--out", default="plugs/plug_turn_schedule.bhv")
    ap.add_argument("--out-opregion", default="plugs/plug_opregion.bhv")
    ap.add_argument("--force", action="store_true",
                    help="write even if the geometry check fails")
    a = ap.parse_args()

    speeds = [float(s) for s in a.speeds.split(",") if s.strip()]
    if not speeds or a.trials < 1:
        sys.exit("gen_turn_schedule: need >=1 speed and >=1 trial")
    leg = [float(v) for v in a.leg.split(",")]
    if len(leg) != 4:
        sys.exit("gen_turn_schedule: --leg wants x,y,len,ang")
    poly = parse_poly(a.poly)

    # ------------------------------------------------ geometry check
    p1, p2, circles = geometry(leg, a.turn_rad, a.dir, a.check_radius)
    worst = None
    print("gen_turn_schedule: geometry vs op region")
    print(f"  leg  p1=({fmt(p1[0])},{fmt(p1[1])})  p2=({fmt(p2[0])},{fmt(p2[1])})  "
          f"hdg {fmt(leg[3])}  len {fmt(leg[2])}")
    for label, c, r in circles:
        clr = clearance(c, poly) - r
        flag = "" if clr >= a.margin else "   <-- TOO CLOSE"
        print(f"  {label:26s} centre ({fmt(c[0])},{fmt(c[1])}) R {fmt(r):>5}  "
              f"clearance {clr:6.1f} m{flag}")
        worst = clr if worst is None else min(worst, clr)
    for label, p in (("p1", p1), ("p2", p2)):
        clr = clearance(p, poly)
        if clr < a.margin:
            print(f"  leg end {label} clearance {clr:.1f} m   <-- TOO CLOSE")
        worst = min(worst, clr)
    if worst < a.margin:
        msg = (f"gen_turn_schedule: geometry check FAILED "
               f"(worst clearance {worst:.1f} m < margin {a.margin} m)")
        if not a.force:
            sys.exit(msg + "; move --leg, shrink --check-radius/--turn-rad, or --force")
        print(msg + " -- writing anyway (--force)")

    # ------------------------------------------------ schedule
    turns = [(s, i + 1) for s in speeds for i in range(a.trials)]
    n = len(turns)
    nominal = sum((a.fix / 57.2958) * max(a.check_radius, 1.0) / s for s, _ in turns)
    print(f"  {n} turns; nominal turning time at R={a.check_radius} m: {nominal:.0f} s "
          f"(+ legs); per-turn timeout {a.timeout:.0f} s")

    leg_spds = ",".join(f"{a.trials}:{fmt(s)}" for s in speeds)
    lines = []
    w = lines.append
    w("//----------------------------------------------------------")
    w("// GENERATED by gen_turn_schedule.py -- do not edit by hand.")
    w(f"//   speeds {a.speeds}  trials {a.trials}  dir {a.dir}  mod_hdg {fmt(a.mod_hdg)}"
      f"  fix {fmt(a.fix)}  timeout {fmt(a.timeout)}")
    w(f"//   leg {a.leg}  turn_rad {fmt(a.turn_rad)}  check_radius {fmt(a.check_radius)}")
    w("//----------------------------------------------------------")
    w("")
    w("Behavior = BHV_LegRun")
    w("{")
    w("  name      = legs")
    w("  pwt       = 100")
    w("  condition = MODE == LEG_RUNNING")
    w("  updates   = LEG_UPDATE")
    w("  perpetual = true")
    w("")
    w("            lead = 6")
    w("     lead_damper = 1")
    w(f"           speed = {fmt(speeds[0])}")
    w("    capture_line = true")
    w("  capture_radius = 2")
    w("     slip_radius = 6")
    w("        patience = 90")
    w("")
    w(f"             leg = x={fmt(leg[0])},y={fmt(leg[1])},len={fmt(leg[2])},ang={fmt(leg[3])}")
    w("      turn1_bias = 0")
    w("      turn2_bias = 0")
    w("       turn1_dir = port")
    w("       turn2_dir = star")
    w(f"        turn_rad = {fmt(a.turn_rad)}")
    w("   init_leg_mode = far_turn   // first leg runs to the FAR end: a full approach leg")
    w("          repeat = 10000   // ended by the last turn's flags, not by count")
    w("")
    w("  // One leg per turn. The leg before each turn runs at that turn's")
    w("  // speed; end turns keep the most recent leg speed (onturn=true) so")
    w("  // the boat arrives on the next leg already settled.")
    w(f"        leg_spds = {leg_spds}")
    w("  leg_spds_onturn = true")
    w("  leg_spds_repeat = false")
    w(f"     mid_leg_pct = {fmt(a.mid_leg_pct)}")
    w("    mid_leg_flag = FIX_TURN=true")
    w("")
    w("  visual_hints = nextpt_color=yellow, nextpt_vertex_size=8, nextpt_lcolor=gray70")
    w("  visual_hints = vertex_color=dodger_blue, edge_color=white, vertex_size=6, edge_size=1")
    w("  visual_hints = turn_edge_color=gray80, turn_vertex_color=green")
    w("}")
    w("")
    w("Behavior = BHV_FixedTurn")
    w("{")
    w("  name      = fturn")
    w("  pwt       = 100")
    w("  condition = MODE == FIX_TURNING")
    w("#ifdef XMODE BBOAT")
    w("  // Field only: an RC takeover idles the turn instead of letting")
    w("  // RC-driven heading change count toward it. On hand-back the")
    w("  // behavior restarts from 'stem' and the turn begins again.")
    w("  condition = BB_CMD_AUTHORITY = AUTONOMY")
    w("#endif")
    w("  updates   = FTURN_UPDATE")
    w("  perpetual = true")
    w("")
    w(f"  fix_turn  = {fmt(a.fix)}")
    w(f"  mod_hdg   = {fmt(a.mod_hdg)}")
    w(f"  turn_dir  = {a.dir}")
    w(f"  timeout   = {fmt(a.timeout)}")
    w("  turn_delay = 0")
    w("  stale_nav_thresh = 5")
    w("  schedule_repeat  = false")
    w("")
    w("  endflag = TURNING_TIME = $[TURN_TIME]")
    w("  endflag = TURNING_DIST = $[TURN_DIST]")
    w("")
    w("  // One turn_spec per trial, speed-major. Every turn hands control")
    w("  // back to the leg run; the last one ends the mission.")
    for k, (s, trial) in enumerate(turns):
        spec = (f"spd={fmt(s)}, mhdg={fmt(a.mod_hdg)}, fix={fmt(a.fix)}, "
                f"turn={a.dir}, timeout={fmt(a.timeout)}, flag=FIX_TURN=false")
        if k == n - 1:
            spec += ", flag=LEG_RUN=false, flag=RETURN=true"
        w(f"  turn_spec = {spec}   // {k+1:2d}/{n}: {fmt(s)} m/s trial {trial}")
    w("}")
    with open(a.out, "w") as f:
        f.write("\n".join(lines) + "\n")

    op = []
    w = op.append
    w("//----------------------------------------------------------")
    w("// GENERATED by gen_turn_schedule.py (same polygon the schedule's")
    w("// geometry was checked against) -- do not edit by hand.")
    w("//----------------------------------------------------------")
    w("Behavior = BHV_OpRegionV24")
    w("{")
    w("  name      = opreg")
    w("  pwt       = 300")
    w("  condition = MODE == ACTIVE")
    w("  updates   = RECOVER_UPDATES")
    w("")
    w(f"  core_poly = pts={{{a.poly}}}")
    w("  dynamic_region_var = RESCUE_REGION")
    w("")
    w("  save_dist = 5")
    w("  halt_dist = 10")
    w("  visual_hints = edge_size=1, save_edge_color=gray30, halt_edge_color=gray40")
    w("}")
    with open(a.out_opregion, "w") as f:
        f.write("\n".join(op) + "\n")
    print(f"  wrote {a.out} ({n} turns) and {a.out_opregion}")


if __name__ == "__main__":
    main()
