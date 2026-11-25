#include "track/start_check.h"

#define WAIT_US            (1000000U)
#define PROBE_US           (500000U)
#define PROBE_TRAVEL_UM    (35000)
#define MIN_TRAVEL_UM      (2000)
#define WHEEL_RISE_UM_S    (45000)
#define RESPONSE_GRACE_US  (40000U) /* Two IMU periods to see the end of the speed rise. */
#define BASELINE_SAMPLES   (3U)
#define BASELINE_SPREAD_MG (15)
#define RESPONSE_MG        (10)
#define MAX_RESPONSE_MG    (200)
#define RESPONSE_SAMPLES   (2U)
#define US_PER_MS          (1000U)
#define UM_PER_MM          (1000)
/* 1 mg for 102 ms changes speed by 1 mm/s. Allow drivetrain compliance
 * and sampling error by requiring about 40 percent of the wheel speed rise. */
#define IMPULSE_PER_MM_S   (40)

void start_check_begin (start_check_t * p_check, uint32_t now_us)
{
    *p_check = (start_check_t){ .since_us = now_us, .phase = START_CHECK_WAITING };
}

static bool sample_at_rest (start_check_t * p_check, uint32_t sample_us, int16_t forward_mg)
{
    if (p_check->baseline_samples != 0U)
    {
        uint32_t const elapsed = sample_us - p_check->sample_us;
        if (elapsed == 0U)
        {
            return false;
        }
        int32_t const delta = (int32_t)forward_mg - p_check->baseline_sum_mg / p_check->baseline_samples;
        if (elapsed > START_CHECK_IMU_AGE_US || delta > BASELINE_SPREAD_MG || delta < -BASELINE_SPREAD_MG)
        {
            p_check->baseline_samples = 0U;
            p_check->baseline_sum_mg = 0;
        }
    }
    p_check->sample_us = sample_us;
    p_check->baseline_sum_mg += forward_mg;
    p_check->baseline_samples++;
    return p_check->baseline_samples == BASELINE_SAMPLES;
}

static void check_response (start_check_t * p_check, motion_feedback_t const * p_wheels,
                            uint32_t now_us, uint32_t sample_us, int16_t forward_mg)
{
    uint32_t const elapsed = sample_us - p_check->sample_us;
    if (elapsed != 0U)
    {
        p_check->sample_us = sample_us;
        int32_t const delta = (int32_t)forward_mg - p_check->baseline_mg;
        int32_t const bounded = (delta > MAX_RESPONSE_MG) ? MAX_RESPONSE_MG :
                                ((delta < -MAX_RESPONSE_MG) ? -MAX_RESPONSE_MG : delta);
        p_check->impulse_mg_ms += bounded * (int32_t)(elapsed / US_PER_MS);
        if ((delta >= RESPONSE_MG) && (p_check->response_samples < RESPONSE_SAMPLES))
        {
            p_check->response_samples++;
        }
    }

    int32_t const left = p_wheels->left_speed_um_s;
    int32_t const right = p_wheels->right_speed_um_s;
    int32_t const speed_mm_s = (left / UM_PER_MM + right / UM_PER_MM) / (int32_t)MOTION_WHEELS;
    bool const rose = (left >= WHEEL_RISE_UM_S) && (right >= WHEEL_RISE_UM_S);
    if (rose && !p_check->b_wheels_rose)
    {
        p_check->b_wheels_rose = true;
        p_check->rise_us = now_us;
    }
    if (p_check->b_wheels_rose && now_us - p_check->rise_us > RESPONSE_GRACE_US)
    {
        p_check->phase = START_CHECK_FAILED;
    }
    else if (rose && p_check->response_samples >= RESPONSE_SAMPLES &&
             p_wheels->left_displacement_um - p_check->origin_um[0] >= MIN_TRAVEL_UM &&
             p_wheels->right_displacement_um - p_check->origin_um[1] >= MIN_TRAVEL_UM &&
             p_check->impulse_mg_ms >= speed_mm_s * IMPULSE_PER_MM_S)
    {
        p_check->phase = START_CHECK_PASSED;
    }
    else
    {
        /* The measured wheel rise has not been corroborated yet. */
    }
}

static bool travel_limit (int64_t wheel_um, int64_t origin_um)
{
    int64_t const delta = wheel_um - origin_um;
    return delta >= PROBE_TRAVEL_UM || delta < 0;
}

start_check_phase_t start_check_step (start_check_t * p_check, uint32_t now_us, bool b_ready,
                                      motion_feedback_t const * p_wheels, uint32_t sample_us,
                                      int16_t forward_mg)
{
    if (p_check->phase == START_CHECK_WAITING)
    {
        if (now_us - p_check->since_us >= WAIT_US)
        {
            p_check->phase = START_CHECK_FAILED;
        }
        else if (b_ready && p_wheels->left_speed_um_s == 0 && p_wheels->right_speed_um_s == 0)
        {
            if (sample_at_rest(p_check, sample_us, forward_mg))
            {
                p_check->phase = START_CHECK_PROBING;
                p_check->since_us = now_us;
                p_check->baseline_mg = (int16_t)(p_check->baseline_sum_mg / p_check->baseline_samples);
                p_check->origin_um[0] = p_wheels->left_displacement_um;
                p_check->origin_um[1] = p_wheels->right_displacement_um;
                p_check->invalid_samples[0] = p_wheels->invalid_samples[0];
                p_check->invalid_samples[1] = p_wheels->invalid_samples[1];
            }
        }
        else
        {
            p_check->baseline_samples = 0U;
            p_check->baseline_sum_mg = 0;
        }
    }
    else if (p_check->phase == START_CHECK_PROBING)
    {
        if (!b_ready || now_us - p_check->since_us >= PROBE_US ||
            sample_us - p_check->sample_us > START_CHECK_IMU_AGE_US ||
            p_wheels->invalid_samples[0] != p_check->invalid_samples[0] ||
            p_wheels->invalid_samples[1] != p_check->invalid_samples[1] ||
            travel_limit(p_wheels->left_displacement_um, p_check->origin_um[0]) ||
            travel_limit(p_wheels->right_displacement_um, p_check->origin_um[1]))
        {
            p_check->phase = START_CHECK_FAILED;
        }
        else
        {
            check_response(p_check, p_wheels, now_us, sample_us, forward_mg);
        }
    }
    else
    {
        /* A completed check is final for this START. */
    }
    return (start_check_phase_t)p_check->phase;
}
