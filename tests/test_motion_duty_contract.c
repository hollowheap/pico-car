/** Host probe for applied PWM duty using the actual car setup. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion/motor_setup.h"
#include "motion_service.h"
#include "platform/log.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define TICK_US 10000U
#define TARGET_UM_S 150000
#define NOMINAL_EDGE_COUNT 18U
#define LOADED_EDGE_COUNT 4U
#define FORMER_FIXED_DUTY 400

typedef struct
{
    int16_t left;
    int16_t right;
} motor_duty_t;

static controller_config_t g_config;

ER motion_service_init(motion_service_config_t const * p_config, ID task_id)
{
    assert(1 == task_id);
    g_config = p_config->control;
    return E_OK;
}

void log_write(log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

static motion_apply_result_t record_pwm(void * p_context, int16_t left, int16_t right)
{
    motor_duty_t * p_duty = p_context;

    p_duty->left = left;
    p_duty->right = right;
    return MOTION_APPLIED;
}

static bool record_brake(void * p_context, bool b_latched)
{
    (void)b_latched;
    (void)record_pwm(p_context, 0, 0);
    return true;
}

static motor_duty_t run_wheels(uint32_t left_edges)
{
    motion_input_t sample = {0};
    motor_duty_t duty = {0};
    motion_output_t const out = {&duty, record_pwm, record_brake};
    controller_t control;

    assert(E_OK == motion_setup(1));
    assert(controller_init(&control, &g_config, &sample, &out));
    for (uint32_t tick = 1U; tick <= 40U; tick++)
    {
        motion_command_t const command =
            {tick, tick * TICK_US, 100000U, {TARGET_UM_S, TARGET_UM_S}};

        sample.timestamp_us = command.issued_us;
        sample.wheel[0].negative += left_edges;
        sample.wheel[1].positive += NOMINAL_EDGE_COUNT;
        controller_tick(&control, &sample, sample.timestamp_us, &command, false);
    }
    assert(MOTION_RUNNING == control.status.state);
    assert(control.status.sample.wheel[0].speed_valid);
    assert(control.status.sample.wheel[1].speed_valid);
    return duty;
}

int main(int argc, char * argv[])
{
    if (argc != 2)
    {
        (void)fprintf(stderr, "Usage: %s unloaded|loaded\n", argv[0]);
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "unloaded") == 0)
    {
        motor_duty_t const duty = run_wheels(NOMINAL_EDGE_COUNT);

        assert(duty.left == duty.right);
        assert(duty.left > 0 && duty.left < FORMER_FIXED_DUTY);
        (void)printf("Unloaded wheel duty: %d/%d permille\n", duty.left, duty.right);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "loaded") == 0)
    {
        motor_duty_t const duty = run_wheels(LOADED_EDGE_COUNT);

        assert(duty.right > 0 && duty.right < FORMER_FIXED_DUTY);
        if (duty.left <= FORMER_FIXED_DUTY)
        {
            (void)fprintf(stderr, "Loaded wheel duty remained at %d permille\n", duty.left);
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    }
    (void)fprintf(stderr, "Unknown motor duty case: %s\n", argv[1]);
    return EXIT_FAILURE;
}
