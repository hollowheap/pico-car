/** @file odometry.c
 * @brief Checked signed odometry with measured sample intervals.
 */
#include <stddef.h>
#include "encoder_math.h"
#include "odometry.h"

static uint64_t magnitude (int64_t value)
{
    return value < 0 ? (uint64_t)(-(value + 1)) + UINT64_C(1) : (uint64_t)value;
}

static bool displacement (int64_t edges, const motion_scale_t * p_scale,
                          int64_t * p_distance)
{
    uint64_t count = magnitude(edges);
    uint64_t whole = count / p_scale->edges_per_revolution;
    uint64_t fraction =
        ((count % p_scale->edges_per_revolution) * p_scale->wheel_travel_um) /
        p_scale->edges_per_revolution;
    uint64_t distance = UINT64_C(0);

    if (whole > ((uint64_t)INT64_MAX / p_scale->wheel_travel_um))
    {
        return false;
    }
    distance = whole * p_scale->wheel_travel_um;
    if (fraction > ((uint64_t)INT64_MAX - distance))
    {
        return false;
    }
    distance += fraction;
    *p_distance = edges < 0 ? -(int64_t)distance : (int64_t)distance;
    return true;
}

bool odometry_init (odometry_t *      p_sampler,
                          const motion_config_t * p_config,
                          const motion_input_t *  p_baseline)
{
    odometry_t initial = {0};
    unsigned int     wheel = 0U;

    if ((p_sampler == NULL) || (p_config == NULL) || (p_baseline == NULL) ||
        (p_config->max_interval_us == 0U) ||
        (p_config->max_interval_us > (uint32_t)INT32_MAX))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        const motion_scale_t * p_scale = &p_config->wheel[wheel];

        if ((p_scale->wheel_travel_um == 0U) ||
            (p_scale->edges_per_revolution == 0U) ||
            ((p_scale->forward_sign != 1) && (p_scale->forward_sign != -1)))
        {
            return false;
        }
    }
    initial.config      = *p_config;
    initial.previous    = *p_baseline;
    initial.initialised = true;
    *p_sampler          = initial;
    return true;
}

bool odometry_rebase (odometry_t * p_sampler, const motion_input_t * p_baseline)
{
    if ((p_sampler == NULL) || (p_baseline == NULL) || !p_sampler->initialised)
    {
        return false;
    }
    p_sampler->previous = *p_baseline;
    p_sampler->stationary_span_us = 0U;
    for (uint32_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        p_sampler->odometry_span_us[wheel] = 0U;
    }
    return true;
}

/* Valid intervals and existing spans each fit in INT32_MAX. */
static uint32_t extend_span (uint32_t previous_us, uint32_t interval_us)
{
    uint32_t const span_us = previous_us + interval_us;

    return (span_us > (uint32_t)INT32_MAX) ? (uint32_t)INT32_MAX : span_us;
}

bool odometry_update (odometry_t *     p_sampler,
                            const motion_input_t * p_input,
                            motion_sample_t *      p_output)
{
    odometry_t next;
    motion_sample_t  output = {0};
    bool             b_stationary = true;
    unsigned int     wheel = 0U;

    if ((p_sampler == NULL) || (p_input == NULL) || (p_output == NULL) ||
        !p_sampler->initialised)
    {
        return false;
    }
    next                = *p_sampler;
    output.timestamp_us = p_input->timestamp_us;
    output.elapsed_us   = p_input->timestamp_us - next.previous.timestamp_us;
    if (output.elapsed_us == 0U)
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        const motion_scale_t *        p_scale  = &next.config.wheel[wheel];
        const quadrature_counts_t * p_before = &next.previous.wheel[wheel];
        const quadrature_counts_t * p_after  = &p_input->wheel[wheel];
        motion_wheel_sample_t *       p_result = &output.wheel[wheel];
        uint32_t                      positive =
            encoder_math_count_delta(p_before->positive, p_after->positive);
        uint32_t negative =
            encoder_math_count_delta(p_before->negative, p_after->negative);
        int64_t delta =
            ((int64_t)positive - (int64_t)negative) * p_scale->forward_sign;
        uint32_t speed = 0U;

        if (((delta > 0) && (next.net_edges[wheel] > INT64_MAX - delta)) ||
            ((delta < 0) && (next.net_edges[wheel] < INT64_MIN - delta)))
        {
            return false;
        }
        next.net_edges[wheel] += delta;
        p_result->net_edges = next.net_edges[wheel];
        if (!displacement(p_result->net_edges, p_scale,
                          &p_result->displacement_um) ||
            !encoder_math_speed_um_s((uint32_t)magnitude(delta),
                                     output.elapsed_us,
                                     p_scale->wheel_travel_um,
                                     p_scale->edges_per_revolution, &speed) ||
            (speed > (uint32_t)INT32_MAX))
        {
            return false;
        }
        p_result->velocity_um_s = delta < 0 ? -(int32_t)speed : (int32_t)speed;
        p_result->invalid_delta =
            encoder_math_count_delta(p_before->invalid, p_after->invalid);
        p_result->speed_valid =
            (p_result->invalid_delta == 0U) &&
            (output.elapsed_us <= next.config.max_interval_us);
        p_result->odometry_valid = p_result->speed_valid;
        if (p_result->speed_valid)
        {
            next.odometry_span_us[wheel] = extend_span(next.odometry_span_us[wheel], output.elapsed_us);
        }
        else
        {
            next.invalid_samples[wheel]++;
            next.odometry_span_us[wheel] = 0U;
        }
        p_result->invalid_samples = next.invalid_samples[wheel];
        p_result->odometry_span_us = next.odometry_span_us[wheel];
        /* A reversal can have zero net counts while both raw counters moved. */
        b_stationary = b_stationary && p_result->speed_valid && (0U == positive) && (0U == negative);
    }
    next.stationary_span_us = b_stationary ? extend_span(next.stationary_span_us, output.elapsed_us) : 0U;
    output.stationary_span_us = next.stationary_span_us;
    next.previous = *p_input;
    *p_sampler    = next;
    *p_output     = output;
    return true;
}

bool motion_sample_fresh (const motion_sample_t * p_sample, uint32_t now_us,
                          uint32_t max_age_us)
{
    return (p_sample != NULL) && (max_age_us <= (uint32_t)INT32_MAX) &&
           ((uint32_t)(now_us - p_sample->timestamp_us) <= max_age_us);
}

/*** end of file ***/
