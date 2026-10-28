/** @file test_speed_probe.c
 * @brief Bench cap/configuration, intentional start and terminal time bounds.
 */
#include <assert.h>
#include <stddef.h>
#include "speed_probe.h"

static uint32_t start (speed_probe_t * p_probe, uint32_t now)
{
    assert(speed_probe_update(p_probe, now, false, false) == SPEED_PROBE_WAIT);
    now += MOTOR_RELEASE_US;
    assert(speed_probe_update(p_probe, now, false, false) == SPEED_PROBE_WAIT);
    now++;
    assert(speed_probe_update(p_probe, now, true, false) == SPEED_PROBE_WAIT);
    now += MOTOR_PRESS_US;
    assert(speed_probe_update(p_probe, now, true, false) == SPEED_PROBE_RUN);
    return now;
}

int main (void)
{
    speed_probe_t probe;
    motion_control_config_t config = speed_probe_configuration();
    uint32_t now;
    unsigned int tick;
    unsigned int wheel;

    assert(motion_control_config_valid(&config));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        assert(config.speed[wheel].max_duty_permille <= 200U);
        assert(config.speed[wheel].kd_q16 == 0U);
        assert(config.speed[wheel].max_target_um_s >= (uint32_t)SPEED_PROBE_TARGET_UM_S);
    }
    speed_probe_init(NULL);
    assert(speed_probe_update(NULL, 0U, true, false) == SPEED_PROBE_FINISHED);
    speed_probe_init(&probe);
    assert(speed_probe_update(&probe, 0U, true, false) == SPEED_PROBE_WAIT);
    assert(speed_probe_update(&probe, 200000U, true, false) == SPEED_PROBE_WAIT);
    now = start(&probe, UINT32_MAX - 200000U);
    for (tick = 1U; tick < 150U; tick++)
    {
        now += 20000U;
        assert(speed_probe_update(&probe, now, true, false) == SPEED_PROBE_RUN);
    }
    now += 20000U;
    assert(speed_probe_update(&probe, now, true, false) == SPEED_PROBE_FINISHED);
    assert(!probe.interrupted && !probe.gap_fault);
    assert(speed_probe_update(&probe, now + 100000U, false, false) == SPEED_PROBE_FINISHED);
    assert(speed_probe_update(&probe, now + 200000U, true, false) == SPEED_PROBE_FINISHED);
    speed_probe_init(&probe);
    now = start(&probe, 0U);
    assert(speed_probe_update(&probe, now + SPEED_PROBE_MAX_GAP_US + 1U,
                              false, false) == SPEED_PROBE_FINISHED);
    assert(probe.gap_fault);
    speed_probe_init(&probe);
    assert(speed_probe_update(&probe, 0U, true, true) == SPEED_PROBE_FINISHED);
    assert(probe.interrupted);
    speed_probe_init(&probe);
    now = start(&probe, 0U);
    assert(speed_probe_update(&probe, now + 1U, true, true) == SPEED_PROBE_FINISHED);
    assert(probe.interrupted);
    return 0;
}
