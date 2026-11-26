/** Host probe for applied PWM duty using the actual car setup. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion/motor_setup.h"
#include "motion_service.h"
#include "platform/log.h"
#include "ir/line/line_control.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define TICK_US 10000U
#define TARGET_UM_S 150000
#define OBSTACLE_TEST_TARGET_UM_S 50000
#define EXPECTED_LEG_DUTY_GAIN 2
#define EXPECTED_PIVOT_DUTY_GAIN 4
#define STALLED_TARGET_UM_S 99000
#define STALLED_TICKS 100U
#define NOMINAL_EDGE_COUNT 18U
#define LOADED_EDGE_COUNT 4U
#define PIVOT_EDGE_COUNT 22U
#define BASE_DUTY 600
#define DRIVE_DUTY_MAX 675
#define PIVOT_DUTY_MAX 1000

typedef struct
{
    int16_t left;
    int16_t right;
    bool    b_latched;
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
    motor_duty_t * p_duty = p_context;

    p_duty->b_latched = b_latched;
    (void)record_pwm(p_context, 0, 0);
    return true;
}

static void step_wheels(controller_t * p_control, motion_input_t * p_sample,
                        int32_t left_target, int32_t right_target,
                        uint32_t left_edges, uint32_t right_edges, bool b_obstacle_drive)
{
    p_sample->timestamp_us += TICK_US;
    motion_command_t const command = {p_sample->timestamp_us / TICK_US,
        p_sample->timestamp_us, 100000U, {left_target, right_target}, b_obstacle_drive};

    if (left_target > 0)
    {
        p_sample->wheel[0].negative += left_edges;
    }
    else
    {
        p_sample->wheel[0].positive += left_edges;
    }
    if (right_target > 0)
    {
        p_sample->wheel[1].positive += right_edges;
    }
    else
    {
        p_sample->wheel[1].negative += right_edges;
    }
    controller_tick(p_control, p_sample, p_sample->timestamp_us, &command, false);
}

static motor_duty_t run_wheels(uint32_t left_edges)
{
    motion_input_t sample = {0};
    motor_duty_t duty = {0};
    motion_output_t const out = {&duty, record_pwm, record_brake};
    controller_t control;

    assert(controller_init(&control, &g_config, &sample, &out));
    for (uint32_t tick = 1U; tick <= 40U; tick++)
    {
        step_wheels(&control, &sample, TARGET_UM_S, TARGET_UM_S,
                    left_edges, NOMINAL_EDGE_COUNT, false);
    }
    assert(MOTION_RUNNING == control.status.state);
    assert(control.status.sample.wheel[0].speed_valid);
    assert(control.status.sample.wheel[1].speed_valid);
    return duty;
}

static void check_pivot(int32_t direction)
{
    int32_t const left_target = direction * LINE_TURN_FAST_UM_S;
    motion_input_t sample = {0};
    motor_duty_t duty = {0};
    motion_output_t const out = {&duty, record_pwm, record_brake};
    controller_t control;

    assert(controller_init(&control, &g_config, &sample, &out));
    for (uint32_t tick = 0U; tick < 40U; tick++)
    {
        step_wheels(&control, &sample, left_target, -left_target,
                    LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
        assert(abs(duty.left) <= PIVOT_DUTY_MAX && abs(duty.right) < BASE_DUTY);
    }
    (void)printf("Pivot targets %d/%d um/s, loaded duty %d/%d permille\n",
                 left_target, -left_target, duty.left, duty.right);
    (void)fflush(stdout);
    assert(MOTION_RUNNING == control.status.state);
    assert(duty.left * direction > DRIVE_DUTY_MAX);
    assert(duty.right * direction < 0);

    step_wheels(&control, &sample, left_target, -left_target, 0U, PIVOT_EDGE_COUNT, false);
    assert(abs(duty.left) <= BASE_DUTY);
    assert(MOTION_RUNNING == control.status.state);

    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
    assert(duty.left * direction > BASE_DUTY);

    sample.wheel[0].invalid++;
    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
    assert(!control.status.sample.wheel[0].speed_valid);
    assert(abs(duty.left) <= BASE_DUTY);
    assert(MOTION_RUNNING == control.status.state);

    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
    assert(duty.left * direction > BASE_DUTY);

    /* A steering correction with a stationary inner wheel uses the driving limit. */
    step_wheels(&control, &sample, left_target, 0, LOADED_EDGE_COUNT, 0U, false);
    assert(duty.left * direction > BASE_DUTY && abs(duty.left) <= DRIVE_DUTY_MAX);
    assert(duty.right == 0);
    controller_tick(&control, NULL, sample.timestamp_us + TICK_US, NULL, true);
    assert(MOTION_STOPPED == control.status.state);
    assert(duty.b_latched && duty.left == 0 && duty.right == 0);
}

static void check_obstacle_duty(int32_t direction, bool b_pivot)
{
    motion_input_t ordinary_sample = {0};
    motion_input_t obstacle_sample = {0};
    motor_duty_t ordinary = {0};
    motor_duty_t obstacle = {0};
    motion_output_t const ordinary_out = {&ordinary, record_pwm, record_brake};
    motion_output_t const obstacle_out = {&obstacle, record_pwm, record_brake};
    controller_t ordinary_control;
    controller_t obstacle_control;
    int32_t const low_target = direction * OBSTACLE_TEST_TARGET_UM_S;
    int32_t const high_target = direction * LINE_TURN_FAST_UM_S;
    int32_t const right_dir = b_pivot ? -1 : 1;
    int32_t const gain = b_pivot ? EXPECTED_PIVOT_DUTY_GAIN : EXPECTED_LEG_DUTY_GAIN;

    assert(controller_init(&ordinary_control, &g_config, &ordinary_sample, &ordinary_out));
    assert(controller_init(&obstacle_control, &g_config, &obstacle_sample, &obstacle_out));
    step_wheels(&ordinary_control, &ordinary_sample, low_target, low_target * right_dir, 0U, 0U, false);
    step_wheels(&obstacle_control, &obstacle_sample, low_target, low_target * right_dir, 0U, 0U, true);
    assert(ordinary.left * direction > 0 && ordinary.right * direction * right_dir > 0);
    assert(obstacle.left == gain * ordinary.left);
    assert(obstacle.right == gain * ordinary.right);

    obstacle_sample.wheel[0].invalid++;
    step_wheels(&obstacle_control, &obstacle_sample, low_target, low_target * right_dir, 0U, 0U, true);
    assert(obstacle.left == gain * ordinary.left);

    /* Leaving the centered pivot removes its extra duty even on a bad sample. */
    if (b_pivot)
    {
        obstacle_sample.wheel[0].invalid++;
        step_wheels(&obstacle_control, &obstacle_sample, low_target, -low_target + direction, 0U, 0U, true);
        assert(obstacle.left == EXPECTED_LEG_DUTY_GAIN * ordinary.left);
    }
    obstacle_sample.wheel[0].invalid++;
    step_wheels(&obstacle_control, &obstacle_sample, low_target, low_target * right_dir, 0U, 0U, false);
    assert(obstacle.left == ordinary.left);

    step_wheels(&obstacle_control, &obstacle_sample, high_target, high_target * right_dir, 0U, 0U, true);
    assert(obstacle.left == direction * PIVOT_DUTY_MAX);
    assert(obstacle.right == direction * right_dir * PIVOT_DUTY_MAX);

    step_wheels(&obstacle_control, &obstacle_sample, 0, 0, 0U, 0U, true);
    assert(obstacle.left == 0 && obstacle.right == 0);
    step_wheels(&obstacle_control, &obstacle_sample, high_target, high_target * right_dir, 0U, 0U, true);
    controller_tick(&obstacle_control, NULL, obstacle_sample.timestamp_us + TICK_US, NULL, true);
    assert(obstacle.b_latched && obstacle.left == 0 && obstacle.right == 0);
}

static void check_stationary_limit(int32_t direction)
{
    motion_input_t sample = {0};
    motor_duty_t duty = {0};
    motion_output_t const out = {&duty, record_pwm, record_brake};
    controller_t control;
    int32_t const target = direction * STALLED_TARGET_UM_S;

    assert(controller_init(&control, &g_config, &sample, &out));
    for (uint32_t tick = 0U; tick < STALLED_TICKS; tick++)
    {
        step_wheels(&control, &sample, target, target, 0U, PIVOT_EDGE_COUNT, false);
        assert(MOTION_RUNNING == control.status.state);
        assert(control.status.sample.wheel[0].speed_valid);
        assert(abs(duty.left) <= BASE_DUTY);
    }
    (void)printf("Stationary wheel duty: %d permille\n", duty.left);
    (void)fflush(stdout);
    assert(duty.left == direction * BASE_DUTY);
}

int main(int argc, char * argv[])
{
    if (argc != 2)
    {
        (void)fprintf(stderr, "Usage: %s unloaded|loaded|pivot|stationary-limit|obstacle-duty\n", argv[0]);
        return EXIT_FAILURE;
    }

    assert(E_OK == motion_setup(1));
    assert(g_config.pivot_duty_permille == PIVOT_DUTY_MAX);
    for (unsigned int wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        assert(g_config.speed[wheel].base_duty_permille == BASE_DUTY);
        assert(g_config.speed[wheel].max_duty_permille == DRIVE_DUTY_MAX);
    }

    if (strcmp(argv[1], "unloaded") == 0)
    {
        motor_duty_t const duty = run_wheels(NOMINAL_EDGE_COUNT);

        assert(duty.left == duty.right);
        assert(duty.left > 0 && duty.left < BASE_DUTY);
        (void)printf("Unloaded wheel duty: %d/%d permille\n", duty.left, duty.right);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "loaded") == 0)
    {
        motor_duty_t const duty = run_wheels(LOADED_EDGE_COUNT);

        assert(duty.right > 0 && duty.right < BASE_DUTY);
        if (duty.left <= BASE_DUTY)
        {
            (void)fprintf(stderr, "Loaded wheel duty remained at %d permille\n", duty.left);
            return EXIT_FAILURE;
        }
        assert(duty.left <= DRIVE_DUTY_MAX);
        (void)printf("Loaded wheel duty: %d/%d permille\n", duty.left, duty.right);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "pivot") == 0)
    {
        check_pivot(1);
        check_pivot(-1);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "stationary-limit") == 0)
    {
        check_stationary_limit(1);
        check_stationary_limit(-1);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "obstacle-duty") == 0)
    {
        check_obstacle_duty(1, false);
        check_obstacle_duty(-1, false);
        check_obstacle_duty(1, true);
        check_obstacle_duty(-1, true);
        return EXIT_SUCCESS;
    }
    (void)fprintf(stderr, "Unknown motor duty case: %s\n", argv[1]);
    return EXIT_FAILURE;
}
