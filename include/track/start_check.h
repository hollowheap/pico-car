#ifndef TRACK_START_CHECK_H
#define TRACK_START_CHECK_H

#include <stdbool.h>
#include <stdint.h>
#include "motion/motion.h"
#include "platform/static_assert.h"

/* Initial bench values. The car must have room to move this far before testing. */
#define START_CHECK_SPEED_UM_S (80000)
#define START_CHECK_IMU_AGE_US (60000U)

typedef enum
{
    START_CHECK_WAITING, START_CHECK_PROBING, START_CHECK_PASSED, START_CHECK_FAILED
} start_check_phase_t;

STATIC_ASSERT (sizeof(start_check_phase_t) <= 4U, "start_check_phase_t size");

typedef struct
{
    int64_t origin_um[MOTION_WHEELS];
    uint32_t since_us;
    uint32_t sample_us;
    uint32_t rise_us;
    int32_t impulse_mg_ms;
    int32_t baseline_sum_mg;
    int16_t baseline_mg;
    uint8_t baseline_samples;
    uint8_t response_samples;
    uint8_t phase;
    uint8_t invalid_samples[MOTION_WHEELS];
    bool b_wheels_rose;
} start_check_t;

STATIC_ASSERT (sizeof(start_check_t) == 48U, "start_check_t size");

void start_check_begin (start_check_t * p_check, uint32_t now_us);
start_check_phase_t start_check_step (start_check_t * p_check, uint32_t now_us, bool b_ready,
                                      motion_feedback_t const * p_wheels, uint32_t sample_us,
                                      int16_t forward_mg);

#endif
