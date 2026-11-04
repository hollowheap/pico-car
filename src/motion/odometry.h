/** @file odometry.h
 * @brief Task-context, forward-normalised wheel speed and displacement.
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "quadrature.h"

typedef struct
{
    uint32_t wheel_travel_um;
    uint32_t edges_per_revolution;
    int8_t   forward_sign; /* +1 or -1 relative to quadrature positive. */
} odometry_scale_t;

typedef struct
{
    odometry_scale_t wheel[ENCODER_WHEELS];
    uint32_t       max_interval_us; /* Nonzero and <= INT32_MAX. */
} odometry_config_t;

typedef struct
{
    uint32_t              timestamp_us;
    quadrature_counts_t wheel[ENCODER_WHEELS];
} odometry_input_t;

typedef struct
{
    int64_t net_edges;
    int64_t
        displacement_um;    /* Signed displacement, not absolute path length. */
    int32_t  velocity_um_s; /* Signed interval average, positive car-forward. */
    uint32_t invalid_delta;
    bool     speed_valid;
    bool     odometry_valid;
} odometry_wheel_sample_t;

typedef struct
{
    uint32_t              timestamp_us;
    uint32_t              elapsed_us;
    odometry_wheel_sample_t wheel[ENCODER_WHEELS];
} odometry_sample_t;

typedef struct
{
    odometry_config_t config;
    odometry_input_t  previous;
    int64_t         net_edges[ENCODER_WHEELS];
    bool            odometry_valid[ENCODER_WHEELS];
    bool            initialised;
} odometry_t;

/* Single task owns this state. Supply a coherent capture snapshot. Re-init
 * explicitly after a capture reset or a calibration change; displacement
 * then starts at zero. Failure leaves state unchanged. */
bool odometry_init (odometry_t *      p_sampler,
                          const odometry_config_t * p_config,
                          const odometry_input_t *  p_baseline);

/* Failure (null/uninitialised input, zero dt or arithmetic overflow) changes
 * neither state nor output. Success may publish invalid quality flags:
 * capture invalids invalidate that interval's speed and latch odometry bad;
 * excessive dt invalidates both speeds and latches both odometries bad.
 * Later clean intervals can restore speed validity, not lost odometry.
 * Observed net counts remain available, without guessed missing edges.
 * Conversions truncate toward zero; total edges preserve fractional travel.
 * Speed/displacement magnitudes must fit INT32_MAX/INT64_MAX respectively.
 * Each event counter must increase by < 2^32 per interval; timestamps must
 * be sampled within one 2^32-us cycle. Resets/multiple wraps are not inferred.
 * A valid zero speed does not prove physical rest or encoder health.
 * Any shared publication needs caller-supplied synchronisation. */
bool odometry_update (odometry_t *     p_sampler,
                            const odometry_input_t * p_input,
                            odometry_sample_t *      p_output);

/* Caller checks freshness before consuming cached data. A fresh timestamp
 * does not override per-wheel quality or provisional-scale flags. */
bool odometry_fresh (const odometry_sample_t * p_sample, uint32_t now_us,
                          uint32_t max_age_us);

#endif /* ODOMETRY_H */

/*** end of file ***/
