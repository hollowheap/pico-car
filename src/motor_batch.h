/** @file motor_batch.h
 * @brief One-start wheel-up diagnostic sequence; no hardware dependencies.
 */
#ifndef MOTOR_BATCH_H
#define MOTOR_BATCH_H

#include "motor_pulse.h"
#include "encoder_quad.h"

#define MOTOR_BATCH_STEPS 6U
#define MOTOR_BATCH_PULSE_US UINT32_C(300000)
#define MOTOR_BATCH_PAUSE_US UINT32_C(400000)
#define MOTOR_BATCH_MAX_GAP_US UINT32_C(100000)
#define MOTOR_BATCH_DUTY 200

typedef enum
{
    MOTOR_BATCH_WAIT = 0,
    MOTOR_BATCH_RUN,
    MOTOR_BATCH_PAUSE,
    MOTOR_BATCH_DONE,
    MOTOR_BATCH_STOPPED
} motor_batch_phase_t;

typedef struct
{
    motor_pulse_t gate;
    motor_batch_phase_t phase;
    unsigned int step;
    uint32_t changed_us;
    uint32_t previous_us;
    uint32_t max_gap_us;
    bool gap_fault;
} motor_batch_t;

void motor_batch_init (motor_batch_t * p_batch);

/* Call at intervals shorter than MAX_GAP while running. Stop/fault has
 * priority, latches until init, and never advances to another pulse. A long
 * active gap is detected only when this function runs again: no hardware
 * watchdog or physical stopping guarantee. Clock/counter arithmetic assumes
 * updates within one 2^32-us cycle. Start must first be released, then pressed.
 * After that single start the six steps run automatically, once only. */
motor_batch_phase_t motor_batch_update (motor_batch_t * p_batch,
    uint32_t now_us, bool start_pressed, bool stop_or_fault);

/* Nonzero outputs only in RUN; invalid state yields zero outputs. */
void motor_batch_duties (const motor_batch_t * p_batch,
                         int16_t * p_left, int16_t * p_right);

typedef enum
{
    MOTOR_BATCH_FEEDBACK_OK = 0,
    MOTOR_BATCH_FEEDBACK_INVALID,
    MOTOR_BATCH_FEEDBACK_MISSING,
    MOTOR_BATCH_FEEDBACK_REVERSED
} motor_batch_feedback_t;

/* Endpoint check for verified signs (L negative/R positive means forward).
 * Requires nonzero correctly signed net edges on every commanded wheel and
 * no invalid transitions on either wheel. Zero feedback does not diagnose
 * wiring versus insufficient starting torque. No exact edge-rate guarantee. */
motor_batch_feedback_t motor_batch_feedback (
    const encoder_quad_counts_t start[ENCODER_WHEELS],
    const encoder_quad_counts_t end[ENCODER_WHEELS], int16_t left, int16_t right);

#endif /* MOTOR_BATCH_H */
