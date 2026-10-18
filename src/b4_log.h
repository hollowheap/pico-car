/** @file b4_log.h
 *
 * @brief B4 Task 2: USB CSV logging harness for early prototyping.
 *
 * Use this week before Navigation exists: flash a build that calls
 * b4_log_sample() every 20 ms while pushing the car over humps by
 * hand. Capture with `screen /dev/ttyACM0` to CSV, then tune
 * HUMP_PITCH_* thresholds offline.
 *
 * Columns: ms, ax_g, ay_g, az_g, pitch_raw, pitch_filt, speed_mps,
 * state, win_mm, run_peak_mm, motion, fault
 */

#ifndef B4_LOG_H
#define B4_LOG_H

#include <stdbool.h>

#include "hump.h"

/*!
 * @brief Print CSV header to USB stdio (call once after stdio init).
 */
void b4_log_header (void);

/*!
 * @brief Print one CSV row for the current sample.
 *
 * @param[in] now_ms Timestamp in ms.
 * @param[in] p_accel Calibrated accel in g.
 * @param[in] pitch_raw Unfiltered pitch in deg.
 * @param[in] pitch_filt Filtered pitch in deg.
 * @param[in] p_speed Speed snapshot used.
 * @param[in] p_res Hump result for this tick.
 */
void b4_log_sample (unsigned long now_ms, const imu_accel_g_t * p_accel,
                    float pitch_raw, float pitch_filt,
                    const speed_snapshot_t * p_speed,
                    const hump_result_t * p_res);

#endif /* B4_LOG_H */

/*** end of file ***/
