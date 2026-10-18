/** @file b4_test.h
 *
 * @brief B4 Task 4: pass/fail criteria + fault table (testable).
 *
 * How to run (no Navigation needed):
 *  1. Known humps 20/30/40 mm, 300 mm long ramps, flat approach 1 m.
 *  2. Constant 0.15 m/s (hand-push at encoder-verified speed or B2
 *     cruise). 10 trials per height.
 *  3. Log via b4_log over USB, compute detection + peak error offline.
 *
 * Pass criteria freeze for Tribunal 1:
 *  - B4-T1 detection >=9/10 per height, 0 false positives on 5 m flat
 *    linoleum + 5 m carpet at 0.15 m/s.
 *  - B4-T2 peak error mean <=10 mm, max <=15 mm per height.
 *  - B4-T3 10-min vibration run (motors on blocks): no I2C lockup,
 *    stale-count 0, fault_count 0.
 *  - B4-T4 impact test (tap chassis): MOTION_IMPACT raised within
 *    40 ms, no latched fault.
 *
 * Fault table (detection -> latch -> clear):
 *  F1 I2C NACK/timeout 3x in a row -> fault=I2C_NACK, hold last good
 *     pitch, report fault to Telemetry, keep polling. Clear after 5
 *     good reads. Never stops motors directly (Navigation decides).
 *  F2 Stale speed (>50 ticks same seq, ~1 s) -> fault=STALE_DATA,
 *     freeze height integration, report. Clear on new seq.
 *  F3 Tilt invalid (|a| <0.4g or >2.5g) -> fault=INVALID_TILT,
 *     skip update. Clear on next valid sample. Covers unplugged /
 *     free-fall.
 *  F4 Stack/heap: B4 uses only static ctx + stack locals, no malloc,
 *     no VLA, no recursion. Check with `pico-sdk` stack guard + 10-min
 *     run, require >=20% headroom per NFR7.
 */

#ifndef B4_TEST_H
#define B4_TEST_H

#define B4_T1_TRIALS_PER_HEIGHT  10u
#define B4_T1_MIN_DETECT         9u
#define B4_T1_FLAT_RUN_M         5u
#define B4_T1_MAX_FALSE_POS      0u

#define B4_T2_MEAN_ERR_MM        10.0f
#define B4_T2_MAX_ERR_MM         15.0f

#define B4_T3_SOAK_MIN           10u
#define B4_T4_IMPACT_MS          40u

#define B4_FAULT_NACK_TRIPS      3u
#define B4_FAULT_CLEAR_READS     5u
#define B4_STALE_TRIP_TICKS      50u

#endif /* B4_TEST_H */

/*** end of file ***/
