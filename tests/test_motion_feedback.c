/** Exercise the motion feedback facade with a known published sample. */
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_service.c"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static uint32_t g_test_clock_us;

uint32_t shim_reg_read(uint32_t address)
{
    assert(address == TIMER_TIMERAWL);
    return g_test_clock_us;
}

static void publish_sample(uint32_t timestamp)
{
    g_start_called = true;
    g_published = (motion_service_status_t){.ready = true};
    g_published.control.sample_available = true;
    g_published.control.sample.timestamp_us = timestamp;
    g_published.control.sample.wheel[ENCODER_LEFT] = (motion_wheel_sample_t){
        .velocity_um_s = -125000, .displacement_um = -10425,
        .speed_valid = true, .odometry_valid = true, .invalid_samples = 2U,
    };
    g_published.control.sample.wheel[ENCODER_RIGHT] = (motion_wheel_sample_t){
        .velocity_um_s = 17000, .displacement_um = 4567,
        .speed_valid = false, .odometry_valid = false, .invalid_samples = 9U,
    };
}

static void test_signed_snapshot(void)
{
    uint32_t const stamp = UINT32_MAX - 499U;
    motion_feedback_t value;

    publish_sample(stamp);
    g_test_clock_us = 500U;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.timestamp_us == stamp);
    assert(value.age_us == 1000U && value.fresh);
    assert(value.left_speed_um_s == -125000 && value.left_displacement_um == -10425);
    assert(value.right_speed_um_s == 17000 && value.right_displacement_um == 4567);
    assert(value.speed_valid[0] && value.odometry_valid[0] && value.invalid_samples[0] == 2U);
    assert(!value.speed_valid[1] && !value.odometry_valid[1] && value.invalid_samples[1] == 9U);

    value.left_speed_um_s = 0;
    value.left_displacement_um = 0;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.left_speed_um_s == -125000 && value.left_displacement_um == -10425);
}

static void test_age_boundary(void)
{
    motion_feedback_t value;

    publish_sample(10U);
    g_test_clock_us = 10U + MOTION_FEEDBACK_MAX_AGE_US;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.fresh && value.age_us == MOTION_FEEDBACK_MAX_AGE_US);
    g_test_clock_us++;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(!value.fresh && value.age_us == MOTION_FEEDBACK_MAX_AGE_US + 1U);
}

static void test_reject_delayed_feedback(void)
{
    motion_feedback_t value;

    publish_sample(10U);
    g_test_clock_us = 26010U;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.age_us == 26000U);
    assert(!value.fresh);
}

int main(int argc, char * argv[])
{
    if (argc == 2 && strcmp(argv[1], "signed-snapshot") == 0)
    {
        test_signed_snapshot();
    }
    else if (argc == 2 && strcmp(argv[1], "feedback-age-boundary") == 0)
    {
        test_age_boundary();
    }
    else if (argc == 2 && strcmp(argv[1], "reject-delayed-feedback") == 0)
    {
        test_reject_delayed_feedback();
    }
    else
    {
        (void)fprintf(stderr, "Usage: %s signed-snapshot|feedback-age-boundary|reject-delayed-feedback\n", argv[0]);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
