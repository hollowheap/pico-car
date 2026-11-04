#!/usr/bin/env python3
"""Summarise a buddy4_imu_check console capture for the car-day checks.

Usage:
    python3 tools/b4_log_summary.py LOGFILE [--from S] [--to S]

LOGFILE is the raw console capture. Only lines starting with "B4," (CSV
rows) and "B4S," (5 s counters) are used, so other text is ignored.
--from / --to limit the rows to that many seconds after the first row.

Standard library only. Column order must match log_header() in
src/b4_task.c.
"""

import argparse
import math
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
           4: "TURNING", 5: "CLIMBING", 6: "DESCENDING", 7: "IMPACT"}
FAULTS = {1: "I2C", 2: "STALE_MOTION", 4: "TILT_RANGE"}
MAG_XY_LSB_PER_G = 1100.0
MAG_Z_LSB_PER_G = 980.0


def parse(path):
    rows = []
    stats = []
    with open(path, "r", errors="replace") as handle:
        for line in handle:
            line = line.strip()
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
            elif line.startswith("B4S,"):
                stats.append(line)
    return rows, stats


def describe(values):
    if not values:
        return "no data"
    spread = statistics.pstdev(values) if len(values) > 1 else 0.0
    return "mean %8.1f  std %6.1f  min %7d  max %7d" % (
        statistics.fmean(values), spread, min(values), max(values))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--from", dest="start", type=float, default=0.0)
    parser.add_argument("--to", dest="end", type=float, default=None)
    args = parser.parse_args()

    rows, stats = parse(args.logfile)
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
    impacts = sum(1 for a, b in zip(rows, rows[1:])
                  if b["motion"] == 7 and a["motion"] != 7)
    print("Impact events: %d" % impacts)
    fault_bits = 0
    for r in rows:
        fault_bits |= (r["word"] >> 5) & 0x7
    names = [name for bit, name in FAULTS.items() if fault_bits & bit]
    print("Fault bits seen: " + (", ".join(names) if names else "none"))

    if stats:
        print()
        print("Last counters: " + stats[-1])
    return 0


if __name__ == "__main__":
    sys.exit(main())
