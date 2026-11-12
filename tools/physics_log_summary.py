#!/usr/bin/env python3
"""Summarise a physics_check console capture for the car-day checks.

Usage:
    python3 tools/physics_log_summary.py LOGFILE [--from S] [--to S] [--checks]

LOGFILE is the raw console capture. Only lines starting with "B4," (CSV
rows) and "PHYS_S," (5 s counters) are used, so other text is ignored. A leading
"[SEV](task) " prefix is dropped first.
--from / --to limit the rows to that many seconds after the first row.
--checks reads the "PHYS_G," phase markers of the guided run of physics_check and
prints a verdict for checks 1 to 3 of "Why the hump state may always read FLAT"
(artifacts/notes/imu-lift-overturn.md). Check 4 needs the wheels (docs/bench.md).

Standard library only. Column order must match log_header() in
src/physics_task.c.
"""

import argparse
import math
import re
import statistics
import sys

COLUMNS = [
    "t_us", "ax_mg", "ay_mg", "az_mg", "mx_raw", "my_raw", "mz_raw",
    "mag_new", "b2_valid", "vl_mm_s", "vr_mm_s", "ds_um", "acc_mm_s2",
    "pitch_cdeg", "state", "motion", "yaw_ddps", "cal", "theta0_cdeg",
    "word",
]
STATES = {-1: "NO_DATA", 0: "FLAT", 1: "CLIMB", 2: "CREST", 3: "DESCEND"}
MOTIONS = {0: "UNKNOWN", 1: "STATIONARY", 2: "MOVING", 3: "ACCELERATING",
           4: "TURNING", 5: "CLIMBING", 6: "DESCENDING", 7: "LIFTED"}
FAULTS = {1: "I2C", 2: "STALE_MOTION", 4: "TILT_RANGE"}
MAG_XY_LSB_PER_G = 1100.0
MAG_Z_LSB_PER_G = 980.0


def parse(path):
    rows = []
    stats = []
    phases = []
    last_t_us = None
    with open(path, "r", errors="replace") as handle:
        for line in handle:
            # Console lines carry a "[SEV](task) " prefix (D100). Drop it.
            line = re.sub(r"^\[[A-Z]+\]\([^)]*\)\s*", "", line.strip())
            if line.startswith("B4,"):
                parts = line.split(",")[1:]
                if len(parts) != len(COLUMNS):
                    continue
                try:
                    row = {name: int(value, 16) if name == "word" else int(value)
                           for name, value in zip(COLUMNS, parts)}
                except ValueError:
                    continue
                rows.append(row)
                last_t_us = row["t_us"]
            elif line.startswith("PHYS_S,"):
                stats.append(line)
            elif line.startswith("PHYS_G,"):
                parts = line.split(",", 3)
                if len(parts) == 4 and parts[2].isdigit():
                    phases.append((int(parts[2]), last_t_us, parts[3]))
    return rows, stats, phases


def describe(values):
    if not values:
        return "no data"
    spread = statistics.pstdev(values) if len(values) > 1 else 0.0
    return "mean %8.1f  std %6.1f  min %7d  max %7d" % (
        statistics.fmean(values), spread, min(values), max(values))


# Settle time at the start of a guided phase before its rows count (s).
PHASE_SETTLE_S = 2.0
# Smallest change of a board axis that proves a lift of about 10 degrees (mg).
AXIS_MOVED_MG = 100
# A hump state starts above this pitch in the car's configuration (cdeg).
HUMP_ENTER_CDEG = 400


def phase_rows(rows, phases, index):
    """Rows of guided phase `index`, without the settle time. [] if absent."""
    starts = {p[0]: p[1] for p in phases if p[1] is not None}
    if index not in starts:
        return []
    begin = starts[index]
    later = [t for i, t in starts.items() if i > index]
    end = min(later) if later else None
    out = []
    for r in rows:
        dt = ((r["t_us"] - begin) & 0xFFFFFFFF) / 1e6
        if dt < PHASE_SETTLE_S or dt > 1e3:
            continue
        if end is not None and ((r["t_us"] - end) & 0xFFFFFFFF) < 0x80000000:
            continue
        out.append(r)
    return out


def median(values):
    return statistics.median(values) if values else 0.0


def axis_delta(rest, moved):
    """Axis with the largest change from rest to moved, and the change (mg)."""
    best = ("", 0.0)
    for axis in ("ax_mg", "ay_mg", "az_mg"):
        delta = median([r[axis] for r in moved]) - median([r[axis] for r in rest])
        if abs(delta) > abs(best[1]):
            best = (axis, delta)
    return best


def run_checks(rows, phases):
    """Checks 1 to 3 of the FLAT list, from the guided run of physics_check."""
    print()
    print("Guided checks (hump state always FLAT)")
    if not phases:
        print("  No PHYS_G markers in the file. Run the guided physics_check and save the whole console.")
        return
    rest = phase_rows(rows, phases, 0) + phase_rows(rows, phases, 2)
    good = [r for r in rest if r["state"] != -1]
    cal = [r for r in good if r["cal"] == 1]
    print("  1 calibrated after the still phase:  %s" %
          ("PASS (theta_0 %.2f deg)" % (cal[-1]["theta0_cdeg"] / 100.0) if cal else
           "FAIL - keep the car still and level for 4 s at the start"))
    nose = [r for r in phase_rows(rows, phases, 1) if r["state"] != -1]
    side = [r for r in phase_rows(rows, phases, 3) if r["state"] != -1]
    if not good or not nose:
        print("  2 forward axis and sign:             NO DATA")
    else:
        axis, delta = axis_delta(good, nose)
        pitch = median([r["pitch_cdeg"] for r in nose])
        ok = abs(delta) >= AXIS_MOVED_MG and pitch >= 500
        print("  2 nose lift moved %s by %+d mg, median pitch %+.0f cdeg: %s" %
              (axis or "nothing", delta, pitch,
               "PASS (pitch rises on nose-up)" if ok else
               "FAIL - pitch must rise to about +1000 on a 10 degree nose-up"))
        if axis:
            print("      forward axis seen: %s, sign of the reading on nose-up: %s."
                  " The car config must have fwd_axis on this axis and fwd_sign so that"
                  " pitch rises on nose-up." % (axis[:2].upper(), "+" if delta > 0 else "-"))
    if good and side:
        axis, delta = axis_delta(good, side)
        print("      side lift moved %s by %+d mg (the lateral axis; it must differ from the forward axis)"
              % (axis or "nothing", delta))
    ramp = [r for r in phase_rows(rows, phases, 5) if r["state"] != -1]
    if not ramp:
        print("  3 ramp peak:                         NO DATA")
    else:
        peak = max(r["pitch_cdeg"] for r in ramp)
        trough = min(r["pitch_cdeg"] for r in ramp)
        states = sorted({r["state"] for r in ramp})
        print("  3 ramp: pitch peak %+d cdeg, trough %+d cdeg, states %s: %s" %
              (peak, trough, ",".join(STATES.get(x, str(x)) for x in states),
               "PASS (the ramp is steeper than the enter threshold)" if peak >= HUMP_ENTER_CDEG
               else "FAIL - the peak is below the enter threshold %d cdeg; the ramp is too gentle"
                    " or the limit too high" % HUMP_ENTER_CDEG))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--from", dest="start", type=float, default=0.0)
    parser.add_argument("--to", dest="end", type=float, default=None)
    parser.add_argument("--checks", action="store_true")
    args = parser.parse_args()

    rows, stats, phases = parse(args.logfile)
    if not rows:
        print("No B4 rows found. Was log_every set and the console open?")
        return 1

    t0 = rows[0]["t_us"]

    def seconds(row):
        return ((row["t_us"] - t0) & 0xFFFFFFFF) / 1e6

    rows = [r for r in rows
            if seconds(r) >= args.start
            and (args.end is None or seconds(r) <= args.end)]
    if len(rows) < 2:
        print("Fewer than 2 rows in the selected time range.")
        return 1

    duration = seconds(rows[-1]) - seconds(rows[0])
    gaps = [((b["t_us"] - a["t_us"]) & 0xFFFFFFFF) for a, b in zip(rows, rows[1:])]
    long_gaps = [g for g in gaps if g > 25000]
    print("Rows %d over %.1f s (%.1f rows/s). Gaps > 25 ms: %d (largest %.1f ms)"
          % (len(rows), duration, (len(rows) - 1) / duration if duration else 0.0,
             len(long_gaps), max(gaps) / 1000.0))

    good = [r for r in rows if r["state"] != -1]
    print("Rows with no accelerometer data: %d" % (len(rows) - len(good)))
    print()
    for axis in ("ax_mg", "ay_mg", "az_mg"):
        print("%-6s %s" % (axis, describe([r[axis] for r in good])))
    mags = [math.sqrt(r["ax_mg"] ** 2 + r["ay_mg"] ** 2 + r["az_mg"] ** 2)
            for r in good]
    print("|a| mg %s" % describe([int(round(m)) for m in mags]))
    print("pitch  %s  (centidegrees)" % describe([r["pitch_cdeg"] for r in good]))
    cal_rows = [r for r in good if r["cal"] == 1]
    if cal_rows:
        print("Calibrated from %.2f s, theta_0 = %.2f deg"
              % (seconds(cal_rows[0]), cal_rows[-1]["theta0_cdeg"] / 100.0))
    else:
        print("Never calibrated in this range (car not still for 1 s?)")

    mag_rows = [r for r in rows if r["mag_new"] == 1]
    print()
    if mag_rows:
        gauss = [math.sqrt((r["mx_raw"] / MAG_XY_LSB_PER_G) ** 2
                           + (r["my_raw"] / MAG_XY_LSB_PER_G) ** 2
                           + (r["mz_raw"] / MAG_Z_LSB_PER_G) ** 2)
                 for r in mag_rows]
        print("Magnetometer rows %d, |m| mean %.3f G (min %.3f, max %.3f)"
              % (len(mag_rows), statistics.fmean(gauss), min(gauss), max(gauss)))
        for axis, lsb in (("mx_raw", MAG_XY_LSB_PER_G), ("my_raw", MAG_XY_LSB_PER_G),
                          ("mz_raw", MAG_Z_LSB_PER_G)):
            values = [r[axis] for r in mag_rows]
            offset = (max(values) + min(values)) / 2.0
            print("  %s min %6d max %6d  hard-iron offset estimate %+.3f G "
                  "(valid only after a full slow circle)"
                  % (axis, min(values), max(values), offset / lsb))
    else:
        print("No magnetometer rows.")

    yaw = [r["yaw_ddps"] / 10.0 for r in good]
    print("Yaw rate deg/s: min %.1f max %.1f (positive = turning left)"
          % (min(yaw), max(yaw)))

    print()
    seen_states = sorted({r["state"] for r in rows})
    seen_motion = sorted({r["motion"] for r in rows})
    print("States seen:  " + ", ".join(STATES.get(s, str(s)) for s in seen_states))
    print("Motions seen: " + ", ".join(MOTIONS.get(m, str(m)) for m in seen_motion))
    transitions = []
    for a, b in zip(rows, rows[1:]):
        if a["state"] != b["state"]:
            transitions.append("%.2fs %s->%s" % (seconds(b), STATES.get(a["state"]),
                                                  STATES.get(b["state"])))
    if transitions:
        print("State changes: " + "; ".join(transitions[:20])
              + (" ..." if len(transitions) > 20 else ""))
    # Motion class 7 is LIFTED: the attitude module saw the car lifted or
    # overturned. The IMU no longer reports collisions (D140).
    lifts = ["%.2fs" % seconds(b) for a, b in zip(rows, rows[1:])
             if b["motion"] == 7 and a["motion"] != 7]
    print("Lift or overturn events: %d" % len(lifts)
          + ((" at " + ", ".join(lifts[:10])) if lifts else ""))
    fault_bits = 0
    for r in rows:
        fault_bits |= (r["word"] >> 5) & 0x7
    names = [name for bit, name in FAULTS.items() if fault_bits & bit]
    print("Fault bits seen: " + (", ".join(names) if names else "none"))

    if stats:
        print()
        print("Last counters: " + stats[-1])
    if args.checks:
        run_checks(rows, phases)
    return 0


if __name__ == "__main__":
    sys.exit(main())
