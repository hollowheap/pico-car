/** @file test_pid.c
 * @brief Synthetic control arithmetic and feedback rejection, not tuning.
 */
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>
#include "pid.h"

int main (void)
{
    /* Synthetic gains/ranges for exact arithmetic only; not car settings. */
    pid_config_t config = {16384U, 65536U, 0U, 10000U, 100000U, 40000U, 400U};
    pid_state_t pid;
    pid_state_t saved;
    odometry_sample_t sample = {0};
    int16_t duty = 99;
    unsigned int tick;

    sample.timestamp_us = UINT32_MAX - 10000U;
    sample.elapsed_us = 20000U;
    sample.wheel[0].speed_valid = true;
    assert(pid_init(&pid, &config));
    assert(pid_update(&pid, 1000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 270); /* P=250, I=20. */
    sample.timestamp_us += 20000U;
    assert(pid_update(&pid, 1000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 290); /* Wrap accepted; actual dt retained. */
    saved = pid;
    assert(!pid_update(&pid, 1000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 0 && memcmp(&pid, &saved, sizeof(pid)) == 0);
    sample.timestamp_us += 20000U;
    for (tick = 0U; tick < 4U; tick++)
    {
        sample.wheel[0].speed_valid = tick != 0U;
        if (tick == 1U)
        {
            assert(pid_update(&pid, 1000, &sample, 0U,
                                    sample.timestamp_us, &duty));
            saved = pid;
            continue;
        }
        assert(!pid_update(&pid, tick == 2U ? 10001 : 1000, &sample,
            0U, sample.timestamp_us + (tick == 3U ? 40001U : 0U), &duty));
        assert(duty == 0 && memcmp(&pid, &saved, sizeof(pid)) == 0);
    }
    sample.timestamp_us += 20000U;
    assert(pid_update(&pid, 0, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 0 && pid.integral_q16 == 0 && !pid.has_previous);
    for (tick = 0U; tick < 40U; tick++)
    {
        sample.timestamp_us += 20000U;
        assert(pid_update(&pid, 10000, &sample, 0U, sample.timestamp_us, &duty));
        assert(duty == 400 && pid.integral_q16 == 0); /* No windup. */
    }
    pid_reset(&pid);
    sample.timestamp_us += 20000U;
    assert(pid_update(&pid, -1000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == -270);
    config.kp_q16 = 0U;
    config.ki_q16 = 0U;
    config.kd_q16 = 65536U;
    assert(pid_init(&pid, &config));
    assert(pid_update(&pid, 1000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 0); /* No derivative impulse at first sample. */
    sample.timestamp_us += 20000U;
    assert(pid_update(&pid, 2000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 0); /* Setpoint change alone has no D kick. */
    sample.timestamp_us += 20000U;
    sample.wheel[0].velocity_um_s = 5;
    assert(pid_update(&pid, 2000, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == -250);
    config.ki_q16 = UINT32_MAX;
    config.max_target_um_s = INT32_MAX;
    assert(pid_init(&pid, &config));
    saved = pid;
    sample.wheel[0].velocity_um_s = INT32_MIN;
    assert(!pid_update(&pid, INT32_MAX, &sample, 0U, sample.timestamp_us, &duty));
    assert(duty == 0 && memcmp(&pid, &saved, sizeof(pid)) == 0);
    assert(!pid_update(NULL, 0, &sample, 0U, 0U, &duty) && duty == 0);
    assert(!pid_init(NULL, &config));
    config.max_duty_permille = 1001U;
    assert(!pid_init(&pid, &config));
    return 0;
}
