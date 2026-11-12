#!/usr/bin/env python3
"""Summarise a console capture of car_bench and wdt_check.

Usage:
    python3 tools/bench_summary.py LOGFILE

LOGFILE is the raw console capture. The lines that count start with a tag in
capitals (after the "[SEV](task:module) " prefix): ENC, STEP, STEP_RESULT, SENS,
SENS_RESULT, LINE_RESULT, DRV, DRV_RESULT and W1_PINS. Other text is ignored.
Standard library only. docs/bench.md says what each item measures.
"""

import re
import statistics
import sys

PROBE = {0: "OK", 1: "DEAD (no edges)", 2: "REVERSED (counts backward)",
         3: "CROSSED (the other encoder moved)", 4: "UNCLEAR"}
LOOP_MS = 10
RESET = {0: "power", 1: "watchdog", 2: "brown-out", 3: "software"}


def tagged(path):
    out = []
    with open(path, "r", errors="replace") as handle:
        for line in handle:
            line = re.sub(r"^\[[A-Z]+\]\([^)]*\)\s*", "", line.strip())
            match = re.match(r"^([A-Z][A-Z0-9_]+),(.*)$", line)
            if match:
                out.append((match.group(1), match.group(2)))
    return out


def fields(text):
    result = {}
    for part in text.split(","):
        if "=" in part:
            key, value = part.split("=", 1)
            result[key] = value
    return result


def step(rows):
    data = [[int(x) for x in text.split(",")] for tag, text in rows if tag == "STEP"]
    if not data:
        return
    print("STEP: wheel step response (the speed updates every 100 ms, so times are coarse)")
    on = next((i for i, r in enumerate(data) if r[1] > 5 or r[2] > 5), None)
    for name, col in (("left", 1), ("right", 2)):
        speeds = [r[col] for r in data]
        if on is None:
            print("  %s: the wheel never moved" % name)
            continue
        end = max(on + 3, len(speeds) // 2)
        final = statistics.fmean(speeds[max(on, end - 15):end])
        t63 = next((i for i in range(on, len(speeds)) if speeds[i] >= 0.63 * final), None)
        t90 = next((i for i in range(on, len(speeds)) if speeds[i] >= 0.90 * final), None)
        print("  %s: final %.0f mm/s, 63%% after %s ms, 90%% after %s ms, duty at the end %d permille" % (
            name, final, (t63 - on) * LOOP_MS if t63 is not None else "never",
            (t90 - on) * LOOP_MS if t90 is not None else "never", data[end - 1][3 if col == 1 else 4]))
    print("  The model assumes 60 ms to rise (SIM_LAG_UP_S). Put the 63% time in Q72.")


def line(rows):
    results = [fields(text) for tag, text in rows if tag == "LINE_RESULT"]
    if not results:
        return
    print("LINE: one run of each law on a straight line")
    per = {}
    for r in results:
        secs = int(r["secs_x10"]) / 10.0
        edges = int(r["edges"]) / secs
        raw = int(r["raw_edges"]) / secs
        stops = int(r["inner_stops"]) / secs
        per[r["law"]] = edges
        print("  %-8s %.1f s, %d mm, sensor changes %.1f/s (raw %.1f/s), inner-wheel stops %.1f/s, "
              "heading swing %.1f deg, searching %s ms" % (
                  r["law"], secs, int(r["travel_mm"]), edges, raw, stops,
                  int(r["heading_swing_deci"]) / 10.0, r["searching_ms"]))
    if "legacy" in per and "cascade" in per and per["legacy"] > 0:
        ratio = per["cascade"] / per["legacy"]
        print("  cascade / legacy = %.2f (the model gives 0.46). %s" % (
            ratio, "Better." if ratio < 0.65 else "Not better: tune LINE_KH, LINE_RATE_HIT, or the spacing."))


def sens(rows):
    data = [text.split(",") for tag, text in rows if tag == "SENS"]
    result = [fields(text) for tag, text in rows if tag == "SENS_RESULT"]
    if not data:
        return
    print("SENS: %d changes of the line sensors" % len(data))
    last = None
    for t, left, right, code in data[:12]:
        print("  %6s ms  left %s right %s code %s%s" % (
            t, left, right, code, "" if last is None else "  (+%d ms)" % (int(t) - last)))
        last = int(t)
    if result:
        print("  filtered edges %s, raw edges %s: a big gap means comparator chatter" % (
            result[-1].get("filtered_edges"), result[-1].get("raw_edges")))


def drv(rows):
    data = [[int(x) for x in text.split(",")] for tag, text in rows if tag == "DRV"]
    if not data:
        return
    print("DRV: duty and speed while the car was driven, lifted at 6 s and set down at 10 s")
    phases = (("on the floor", 1000, 5500), ("in the air", 7000, 9500), ("set down", 11000, 14000))
    for name, a, b in phases:
        rows_in = [r for r in data if a <= r[0] <= b]
        if rows_in:
            print("  %-13s duty %4.0f / %4.0f permille, speed %4.0f / %4.0f mm/s, cue set in %d of %d samples" % (
                name, statistics.fmean(r[1] for r in rows_in), statistics.fmean(r[2] for r in rows_in),
                statistics.fmean(r[3] for r in rows_in), statistics.fmean(r[4] for r in rows_in),
                sum(r[7] for r in rows_in), len(rows_in)))
    print("  The cue must be set in the air and clear on the floor. If the duty does not fall in the air, the cue cannot work.")


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    rows = tagged(sys.argv[1])
    if not rows:
        print("No tagged lines found. Was the whole console copied?")
        return 1
    for tag, text in rows:
        if tag == "ENC":
            parts = text.split(",")
            info = fields(",".join(parts[1:]))
            print("ENC: %s wheel alone: driven %s mm, other %s mm -> %s" % (
                parts[0], info.get("driven_mm"), info.get("other_mm"), PROBE.get(int(info.get("verdict", "4")))))
        elif tag == "W1_PINS":
            info = fields(text)
            print("W1: motor pins at start GP8=%s GP9=%s GP10=%s GP11=%s, last reset: %s" % (
                info.get("GP8"), info.get("GP9"), info.get("GP10"), info.get("GP11"),
                RESET.get(int(info.get("reset_reason", "0")), "?")))
    step(rows)
    sens(rows)
    line(rows)
    drv(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
