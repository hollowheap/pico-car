/** @file odometry.h
 * @brief Task-context, forward-normalised wheel speed and displacement.
 */
#ifndef MOTION_SAMPLER_H
#define MOTION_SAMPLER_H

#include "quadrature.h"

typedef struct
{
    uint32_t wheel_travel_um;
    uint32_t edges_per_revolution;
    int8_t   forward_sign; /* +1 or -1 relative to quadrature positive. */
} motion_scale_t;

typedef struct
{
    motion_scale_t wheel[ENCODER_WHEELS];
    uint32_t       max_interval_us; /* Nonzero and <= INT32_MAX. */
} motion_config_t;

typedef struct
{
    uint32_t              timestamp_us;
    quadrature_counts_t wheel[ENCODER_WHEELS];
} motion_input_t;

typedef struct
{
    int64_t net_edges;
    int64_t
        displacement_um;    /* Signed displacement, not absolute path length. */
    int32_t  velocity_um_s; /* Signed interval average, positive car-forward. */
    uint32_t invalid_delta;
    bool     speed_valid;
    bool     odometry_valid; /* This sample only; same as speed_valid. */
    uint8_t  invalid_samples; /* Samples without speed_valid since init; wraps. */
} motion_wheel_sample_t;

typedef struct
{
    uint32_t              timestamp_us;
    uint32_t              elapsed_us;
    motion_wheel_sample_t wheel[ENCODER_WHEELS];
} motion_sample_t;

typedef struct
{
    motion_config_t config;
    motion_input_t  previous;
    int64_t         net_edges[ENCODER_WHEELS];
    uint8_t         invalid_samples[ENCODER_WHEELS];
    bool            initialised;
} odometry_t;

/* Single task owns this state. Supply a coherent capture snapshot. Re-init
 * explicitly after a capture reset or a calibration change; displacement
 * then starts at zero. Failure leaves state unchanged. */
bool odometry_init (odometry_t *      p_sampler,
                          const motion_config_t * p_config,
                          const motion_input_t *  p_baseline);

/* Failure (null/uninitialised input, zero dt or arithmetic overflow) changes
 * neither state nor output. Success may publish invalid quality flags:
 * a capture invalid or an excessive dt makes that interval's speed and
 * odometry not valid, and the next clean interval is valid again.
 * invalid_samples counts such intervals per wheel (wraps at 256), so a
 * reader that polls less often than the samples arrive can still see one.
 * Observed net counts remain available, without guessed missing edges.
 * Conversions truncate toward zero; total edges preserve fractional travel.
 * Speed/displacement magnitudes must fit INT32_MAX/INT64_MAX respectively.
 * Each event counter must increase by < 2^32 per interval; timestamps must
 * be sampled within one 2^32-us cycle. Resets/multiple wraps are not inferred.
 * A valid zero speed does not prove physical rest or encoder health.
 * Any shared publication needs caller-supplied synchronisation. */
bool odometry_update (odometry_t *     p_sampler,
                            const motion_input_t * p_input,
                            motion_sample_t *      p_output);

/* Caller checks freshness before consuming cached data. A fresh timestamp
 * does not override per-wheel quality or provisional-scale flags. */
bool motion_sample_fresh (const motion_sample_t * p_sample, uint32_t now_us,
                          uint32_t max_age_us);

#endif /* MOTION_SAMPLER_H */

/*** end of file ***/
