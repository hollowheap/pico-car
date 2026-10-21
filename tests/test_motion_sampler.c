/** @file test_motion_sampler.c
 * @brief Direction, precision, quality, rollover and failure contracts.
 */
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "motion_sampler.h"

static unsigned int failures;

static void check (bool condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

int main (void)
{
    const motion_config_t config = {
        {{1000U, 10U, -1, false}, {1000U, 10U, 1, true}}, 200000U
    };
    motion_config_t changed;
    motion_sampler_t sampler = {0};
    motion_sampler_t saved;
    motion_input_t input = {0};
    motion_sample_t output = {0};
    motion_sample_t saved_output;
    unsigned int i;

    check(!motion_sampler_update(&sampler, &input, &output), "reject uninitialised");
    check(!motion_sampler_init(NULL, &config, &input), "reject null state");
    check(!motion_sampler_init(&sampler, NULL, &input), "reject null config");
    check(!motion_sampler_init(&sampler, &config, NULL), "reject null baseline");
    changed = config;
    changed.wheel[0].forward_sign = 0;
    check(!motion_sampler_init(&sampler, &changed, &input), "reject unknown sign");
    changed = config;
    changed.wheel[1].edges_per_revolution = 0U;
    check(!motion_sampler_init(&sampler, &changed, &input), "reject zero calibration");
    changed = config;
    changed.wheel[1].wheel_travel_um = 0U;
    check(!motion_sampler_init(&sampler, &changed, &input), "reject zero travel");
    changed = config;
    changed.max_interval_us = 0U;
    check(!motion_sampler_init(&sampler, &changed, &input), "reject zero gap bound");
    changed.max_interval_us = UINT32_MAX;
    check(!motion_sampler_init(&sampler, &changed, &input), "reject ambiguous bound");

    check(motion_sampler_init(&sampler, &config, &input), "initialise");
    saved = sampler;
    saved_output = output;
    check(!motion_sampler_update(&sampler, &input, &output), "reject zero dt");
    check(memcmp(&sampler, &saved, sizeof(saved)) == 0 &&
          memcmp(&output, &saved_output, sizeof(output)) == 0, "zero dt atomic");
    check(!motion_sampler_update(NULL, &input, &output), "reject null update state");
    check(!motion_sampler_update(&sampler, NULL, &output), "reject null input");
    check(!motion_sampler_update(&sampler, &input, NULL), "reject null output");

    input.timestamp_us = 100000U;
    input.wheel[0].negative = 10U;
    input.wheel[1].positive = 10U;
    check(motion_sampler_update(&sampler, &input, &output), "forward update");
    for (i = 0U; i < ENCODER_WHEELS; i++)
    {
        check(output.wheel[i].net_edges == 10 &&
              output.wheel[i].displacement_um == 1000 &&
              output.wheel[i].velocity_um_s == 10000 &&
              output.wheel[i].speed_valid && output.wheel[i].odometry_valid,
              "both wiring signs normalise forward");
    }
    check(!output.wheel[0].scale_verified && output.wheel[1].scale_verified,
          "calibration quality remains independent of capture quality");
    check(motion_sample_fresh(&output, 150000U, 50000U), "fresh inclusive bound");
    check(!motion_sample_fresh(&output, 150001U, 50000U), "stale rejected");
    check(!motion_sample_fresh(NULL, 150000U, 50000U), "null freshness");
    check(!motion_sample_fresh(&output, 100000U, UINT32_MAX), "invalid age bound");

    input.timestamp_us = 200000U;
    input.wheel[0].positive = 15U;
    input.wheel[1].negative = 15U;
    check(motion_sampler_update(&sampler, &input, &output), "reverse update");
    check(output.wheel[0].net_edges == -5 &&
          output.wheel[1].displacement_um == -500 &&
          output.wheel[0].velocity_um_s == -15000 &&
          output.wheel[1].velocity_um_s == -15000, "signed reverse displacement");
    input.timestamp_us = 300000U;
    input.wheel[0].invalid = 1U;
    check(motion_sampler_update(&sampler, &input, &output), "capture loss update");
    check(!output.wheel[0].speed_valid && !output.wheel[0].odometry_valid &&
          output.wheel[0].invalid_delta == 1U && output.wheel[1].odometry_valid,
          "invalid wheel isolated and odometry latched");
    input.timestamp_us = 400000U;
    check(motion_sampler_update(&sampler, &input, &output), "clean interval after loss");
    check(output.wheel[0].speed_valid && !output.wheel[0].odometry_valid &&
          output.wheel[0].velocity_um_s == 0, "speed recovers without invented odometry");
    input.timestamp_us += 200001U;
    check(motion_sampler_update(&sampler, &input, &output), "long gap update");
    check(!output.wheel[0].speed_valid && !output.wheel[1].speed_valid &&
          !output.wheel[1].odometry_valid, "long gap invalidates both wheels");
    check(motion_sampler_init(&sampler, &config, &input), "explicit new segment");
    input.timestamp_us += 100000U;
    check(motion_sampler_update(&sampler, &input, &output) &&
          output.wheel[0].odometry_valid && output.wheel[0].displacement_um == 0,
          "reinitialisation clears displacement and loss latch");

    (void)memset(&input, 0, sizeof(input));
    input.timestamp_us = UINT32_MAX - 49U;
    input.wheel[0].negative = UINT32_MAX - 4U;
    input.wheel[0].invalid = UINT32_MAX;
    check(motion_sampler_init(&sampler, &config, &input), "wrap baseline");
    input.timestamp_us = 50U;
    input.wheel[0].negative = 5U;
    input.wheel[0].invalid = 0U;
    check(motion_sampler_update(&sampler, &input, &output), "simultaneous wraps");
    check(output.elapsed_us == 100U && output.wheel[0].net_edges == 10 &&
          output.wheel[0].velocity_um_s == 10000000 &&
          output.wheel[0].invalid_delta == 1U, "wrap arithmetic");
    output.timestamp_us = UINT32_MAX - 9U;
    check(motion_sample_fresh(&output, 10U, 20U), "freshness clock wrap");

    changed = config;
    changed.wheel[0].wheel_travel_um = 1U;
    changed.wheel[0].edges_per_revolution = 3U;
    (void)memset(&input, 0, sizeof(input));
    check(motion_sampler_init(&sampler, &changed, &input), "fractional scale");
    for (i = 1U; i <= 3U; i++)
    {
        input.timestamp_us = i * 1000U;
        input.wheel[0].negative = i;
        check(motion_sampler_update(&sampler, &input, &output), "fractional update");
    }
    check(output.wheel[0].displacement_um == 1, "fractional distance not lost per interval");
    input.timestamp_us += 1000U;
    input.wheel[0].positive = 5U;
    check(motion_sampler_update(&sampler, &input, &output) &&
          output.wheel[0].net_edges == -2 && output.wheel[0].displacement_um == 0,
          "negative fractional displacement truncates toward zero");

    changed = config;
    changed.wheel[1].wheel_travel_um = UINT32_MAX;
    changed.wheel[1].edges_per_revolution = 1U;
    (void)memset(&input, 0, sizeof(input));
    check(motion_sampler_init(&sampler, &changed, &input), "overflow scale");
    saved = sampler;
    saved_output = output;
    input.timestamp_us = 1U;
    input.wheel[1].positive = 1U;
    check(!motion_sampler_update(&sampler, &input, &output), "speed overflow rejected");
    check(memcmp(&sampler, &saved, sizeof(saved)) == 0 &&
          memcmp(&output, &saved_output, sizeof(output)) == 0,
          "second-wheel failure leaves entire state and output unchanged");
    changed.wheel[1].wheel_travel_um = (uint32_t)INT32_MAX + 1U;
    (void)memset(&input, 0, sizeof(input));
    check(motion_sampler_init(&sampler, &changed, &input), "signed speed bound baseline");
    input.timestamp_us = 1000000U;
    input.wheel[1].positive = 1U;
    check(!motion_sampler_update(&sampler, &input, &output),
          "unsigned speed fitting uint32 still rejects signed output overflow");

    changed = config;
    changed.wheel[0].wheel_travel_um = 1U;
    changed.wheel[0].edges_per_revolution = 1U;
    (void)memset(&input, 0, sizeof(input));
    check(motion_sampler_init(&sampler, &changed, &input), "count limit baseline");
    sampler.net_edges[0] = INT64_MAX;
    input.timestamp_us = 100000U;
    input.wheel[0].negative = 1U;
    check(!motion_sampler_update(&sampler, &input, &output), "signed edge overflow");
    sampler.net_edges[0] = INT64_MIN;
    input.wheel[0].negative = 0U;
    input.wheel[0].positive = 1U;
    check(!motion_sampler_update(&sampler, &input, &output), "signed edge underflow");
    sampler.net_edges[0] = 5000000000LL;
    input.wheel[0].positive = 0U;
    check(motion_sampler_update(&sampler, &input, &output) &&
          output.wheel[0].displacement_um == 5000000000LL,
          "displacement exceeds unsigned 32-bit micrometres safely");
    sampler.config.wheel[0].wheel_travel_um = 2U;
    sampler.net_edges[0] = INT64_MAX;
    input.timestamp_us += 100000U;
    check(!motion_sampler_update(&sampler, &input, &output), "distance overflow rejected");
    sampler.config.wheel[0].wheel_travel_um = 1U;
    sampler.config.wheel[0].edges_per_revolution = UINT32_MAX;
    sampler.net_edges[0] = INT64_MIN;
    check(motion_sampler_update(&sampler, &input, &output) &&
          output.wheel[0].displacement_um == -2147483648LL,
          "INT64_MIN edge magnitude converts safely with large divisor");

    (void)printf("motion sampler: %u failures\n", failures);
    return failures == 0U ? 0 : 1;
}

/*** end of file ***/
