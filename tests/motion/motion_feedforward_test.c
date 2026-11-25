/** @file motion_feedforward_test.c
 * @brief Host checks for the speed feed-forward and static-friction terms of
 *        pid_update, and for the motor configuration keeping its old duty.
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "motion/motor_setup.h"
#include "motion_service.h"
#include "pid.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define Q16_ONE             INT64_C(65536)
#define TICK_US             100000U
#define CRUISE_UM_S         150000
#define STATIC_KS_PERMILLE  80
#define FEEDFORWARD_KV_Q16  131
#define DUTY_LIMIT_PERMILLE 450

static controller_config_t g_config;

ER motion_service_init (motion_service_config_t const * p_config, ID task_id)
{
    assert(1 == task_id);
    g_config = p_config->control;
    return E_OK;
}

static motion_sample_t g_sample;

static motion_sample_t const * sample_at (uint32_t now_us, int32_t velocity_um_s)
{
    g_sample                      = (motion_sample_t){0};
    g_sample.timestamp_us         = now_us;
    g_sample.elapsed_us           = TICK_US;
    g_sample.wheel[0].speed_valid = true;
    g_sample.wheel[0].velocity_um_s = velocity_um_s;
    return &g_sample;
}

static pid_config_t make_config (uint32_t ki_q16, uint16_t max_duty_permille,
                                 uint16_t ks_permille, uint32_t kv_q16)
{
    pid_config_t const config = {
        .kp_q16             = 0U,
        .ki_q16             = ki_q16,
        .kd_q16             = 0U,
        .max_target_um_s    = 300000U,
        .max_interval_us    = TICK_US,
        .max_sample_age_us  = 600000U,
        .max_duty_permille  = max_duty_permille,
        .base_duty_permille = 0U,
        .ks_permille        = ks_permille,
        .kv_q16             = kv_q16,
    };
    return config;
}

static void assert_duty_near (int16_t duty, int64_t expected_q16)
{
    int64_t const error_q16 = ((int64_t)duty * Q16_ONE) - expected_q16;

    assert((error_q16 <= Q16_ONE) && (error_q16 >= -Q16_ONE));
}

static void check_step (int32_t direction)
{
    pid_config_t const config = make_config(0U, DUTY_LIMIT_PERMILLE, STATIC_KS_PERMILLE,
                                            FEEDFORWARD_KV_Q16);
    int32_t const target = direction * CRUISE_UM_S;
    int64_t const expected_q16 = (int64_t)direction *
        (((int64_t)STATIC_KS_PERMILLE * Q16_ONE) + ((int64_t)FEEDFORWARD_KV_Q16 * CRUISE_UM_S));
    pid_t pid;
    int16_t duty = 0;

    assert(pid_init(&pid, &config));
    assert(pid_update(&pid, 0, sample_at(TICK_US, 0), 0U, TICK_US, &duty));
    assert(0 == duty);
    assert(pid_update(&pid, target, sample_at(2U * TICK_US, target), 0U,
                      2U * TICK_US, &duty));
    assert_duty_near(duty, expected_q16);
}

static void check_zero_target (void)
{
    pid_config_t const config = make_config(0U, DUTY_LIMIT_PERMILLE, STATIC_KS_PERMILLE,
                                            FEEDFORWARD_KV_Q16);
    pid_t pid;
    int16_t duty = 1;

    assert(pid_init(&pid, &config));
    assert(pid_update(&pid, 0, sample_at(TICK_US, 0), 0U, TICK_US, &duty));
    assert(0 == duty);
    assert(pid_update(&pid, CRUISE_UM_S, sample_at(2U * TICK_US, CRUISE_UM_S), 0U,
                      2U * TICK_US, &duty));
    assert(duty > 0);
    assert(pid_update(&pid, 0, sample_at(3U * TICK_US, CRUISE_UM_S), 0U,
                      3U * TICK_US, &duty));
    assert(0 == duty);
}

static void check_default_motor_duty (void)
{
    static const int32_t speed[] = {0, 40000, 80000, 110000, 130000, 140000, 145000, 150000};
    static const int16_t duty_before_feedforward[] = {400, 417, 288, 190, 122, 90, 74, 55};
    size_t const count = sizeof(speed) / sizeof(speed[0]);
    pid_t pid;
    int16_t duty = 0;
    size_t i;

    assert(E_OK == motion_setup(1));
    assert((0 == g_config.speed[0].ks_permille) && (0 == g_config.speed[0].kv_q16));
    assert((0 == g_config.speed[1].ks_permille) && (0 == g_config.speed[1].kv_q16));
    assert(pid_init(&pid, &g_config.speed[0]));
    for (i = 0U; i < count; i++)
    {
        uint32_t const now_us = (uint32_t)((i + 1U) * TICK_US);

        assert(pid_update(&pid, CRUISE_UM_S, sample_at(now_us, speed[i]), 0U,
                          now_us, &duty));
        assert(duty == duty_before_feedforward[i]);
    }
}

static void check_saturation (void)
{
    pid_config_t const config = make_config(256U, DUTY_LIMIT_PERMILLE, 0U,
                                            FEEDFORWARD_KV_Q16);
    pid_t pid;
    int16_t duty = 0;
    uint32_t tick;

    assert(pid_init(&pid, &config));
    for (tick = 1U; tick <= 20U; tick++)
    {
        uint32_t const now_us = tick * TICK_US;

        assert(pid_update(&pid, 300000, sample_at(now_us, 0), 0U, now_us, &duty));
        assert(DUTY_LIMIT_PERMILLE == duty);
        assert(0 == pid.integral_q16);
    }
}

int main (void)
{
    check_step(1);
    check_step(-1);
    check_zero_target();
    check_default_motor_duty();
    check_saturation();
    return EXIT_SUCCESS;
}

/*** end of file ***/
