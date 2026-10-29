/** @file test_motor_batch.c
 * @brief Verify once-only automatic sequence, stop/fault priority and wrap.
 */
#include <assert.h>
#include <stddef.h>
#include "motor_batch.h"

static uint32_t start (motor_batch_t * p_batch, uint32_t now)
{
    assert(motor_batch_update(p_batch, now, false, false) == MOTOR_BATCH_WAIT);
    now += MOTOR_RELEASE_US;
    assert(motor_batch_update(p_batch, now, false, false) == MOTOR_BATCH_WAIT);
    now++;
    assert(motor_batch_update(p_batch, now, true, false) == MOTOR_BATCH_WAIT);
    now += MOTOR_PRESS_US;
    assert(motor_batch_update(p_batch, now, true, false) == MOTOR_BATCH_RUN);
    return now;
}

int main (void)
{
    static const int16_t expected[6][2] = {
        {200, 0}, {0, 200}, {200, 200}, {-200, 0}, {0, -200}, {-200, -200}
    };
    motor_batch_t batch;
    uint32_t now;
    unsigned int step;
    int16_t left;
    int16_t right;

    motor_batch_init(NULL);
    assert(motor_batch_update(NULL, 0U, true, false) == MOTOR_BATCH_STOPPED);
    motor_batch_init(&batch);
    assert(motor_batch_update(&batch, 0U, true, false) == MOTOR_BATCH_WAIT);
    assert(motor_batch_update(&batch, UINT32_C(5000000), true, false) == MOTOR_BATCH_WAIT);
    now = start(&batch, UINT32_MAX - UINT32_C(100000));
    for (step = 0U; step < MOTOR_BATCH_STEPS; step++)
    {
        unsigned int tick;

        assert(batch.step == step);
        motor_batch_duties(&batch, &left, &right);
        assert(left == expected[step][0] && right == expected[step][1]);
        for (tick = 0U; tick < 15U; tick++)
        {
            now += UINT32_C(20000);
            (void)motor_batch_update(&batch, now, true, false);
        }
        assert(batch.phase == MOTOR_BATCH_PAUSE);
        motor_batch_duties(&batch, &left, &right);
        assert(left == 0 && right == 0);
        now += MOTOR_BATCH_PAUSE_US;
        (void)motor_batch_update(&batch, now, true, false);
    }
    assert(batch.phase == MOTOR_BATCH_DONE && !batch.gap_fault);
    for (step = 0U; step < 5U; step++)
    {
        now += UINT32_C(1000000);
        assert(motor_batch_update(&batch, now, false, false) == MOTOR_BATCH_DONE);
        assert(motor_batch_update(&batch, now, true, false) == MOTOR_BATCH_DONE);
    }
    for (step = 0U; step < 3U; step++)
    {
        motor_batch_init(&batch);
        now = step == 0U ? 0U : start(&batch, 0U);
        if (step == 2U)
        {
            for (unsigned int tick = 0U; tick < 15U; tick++)
            {
                now += UINT32_C(20000);
                (void)motor_batch_update(&batch, now, false, false);
            }
            assert(batch.phase == MOTOR_BATCH_PAUSE);
        }
        assert(motor_batch_update(&batch, now, true, true) == MOTOR_BATCH_STOPPED);
        assert(motor_batch_update(&batch, now + UINT32_C(1000000), true, false) == MOTOR_BATCH_STOPPED);
        motor_batch_duties(&batch, &left, &right);
        assert(left == 0 && right == 0);
    }
    motor_batch_init(&batch);
    now = start(&batch, 0U);
    assert(motor_batch_update(&batch, now + MOTOR_BATCH_MAX_GAP_US + 1U,
                             false, false) == MOTOR_BATCH_STOPPED);
    assert(batch.gap_fault);
    {
        quadrature_counts_t baseline[2] = {{0U, UINT32_MAX - 2U, 0U}, {0U, 0U, 0U}};
        quadrature_counts_t endpoint[2] = {{0U, 7U, 0U}, {9U, 0U, 0U}};

        assert(motor_batch_feedback(baseline, endpoint, 200, 200) == MOTOR_BATCH_FEEDBACK_OK);
        assert(motor_batch_feedback(baseline, endpoint, -200, 0) == MOTOR_BATCH_FEEDBACK_REVERSED);
        assert(motor_batch_feedback(baseline, baseline, 0, 200) == MOTOR_BATCH_FEEDBACK_MISSING);
        endpoint[1].invalid++;
        assert(motor_batch_feedback(baseline, endpoint, 200, 0) == MOTOR_BATCH_FEEDBACK_INVALID);
        assert(motor_batch_feedback(NULL, endpoint, 200, 200) == MOTOR_BATCH_FEEDBACK_INVALID);
    }
    return 0;
}
