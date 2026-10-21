/** @file encoder_math.h
 *
 * @brief Checked integer conversions for wheel encoder measurements.
 *
 * Inputs are edge counts, measured wheel travel per revolution in micrometres,
 * and elapsed time in microseconds. Calibration must use the same edge-counting
 * mode as capture. These functions do not infer direction or physical rest.
 *
 * Functions keep no shared state and do not configure GPIO or motors. Callers
 * must obtain coherent count/time samples using the kernel's SMP-safe rules.
 * The arithmetic API supports the full uint32_t range; it does not define the
 * final packed motion-message format or an operating speed limit.
 */

#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* Modulo-2^32 difference for an increasing edge-event counter, not a signed
 * quadrature position. The actual increment must be less than one full counter
 * cycle between samples; multiple wraps cannot be recovered. */
uint32_t encoder_math_count_delta (uint32_t previous, uint32_t current);

/* Return floor(count * wheel_travel_um / edges_per_revolution). Calibration
 * values must be nonzero. A null output or a result above UINT32_MAX fails.
 * The output is unchanged on failure. For total odometry, convert the total
 * count rather than repeatedly accumulating rounded interval distances. */
bool encoder_math_distance_um (uint32_t count,
                              uint32_t wheel_travel_um,
                              uint32_t edges_per_revolution,
                              uint32_t * p_distance_um);

/* Return floor(count * wheel_travel_um * 1000000 /
 *              (edges_per_revolution * elapsed_us)).
 * This keeps the fractional distance until the final division. A zero time,
 * invalid calibration, null output or out-of-range result fails without
 * changing the output. A successful zero speed is not proof of physical rest.
 * Run conversions in task context; they perform bounded 64-bit arithmetic. */
bool encoder_math_speed_um_s (uint32_t count,
                             uint32_t elapsed_us,
                             uint32_t wheel_travel_um,
                             uint32_t edges_per_revolution,
                             uint32_t * p_speed_um_s);

#endif /* ENCODER_MATH_H */

/*** end of file ***/
