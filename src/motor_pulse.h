/** @file motor_pulse.h
 * @brief Button-triggered, bounded motor pulses for the wheel-up diagnostic.
 */
#ifndef MOTOR_PULSE_H
#define MOTOR_PULSE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PULSE_US UINT32_C(750000)
#define MOTOR_RELEASE_US UINT32_C(50000)
#define MOTOR_PRESS_US UINT32_C(20000)

typedef enum
{
    MOTOR_PULSE_OFF = 0,
    MOTOR_PULSE_M1A,
    MOTOR_PULSE_M1B,
    MOTOR_PULSE_M2A,
    MOTOR_PULSE_M2B
} motor_pulse_phase_t;

typedef struct
{
    uint32_t started_us;
    uint32_t released_us;
    uint32_t pressed_us;
    motor_pulse_phase_t active;
    unsigned int next;
    bool release_tracking;
    bool press_tracking;
    bool ready;
    bool stopped;
} motor_pulse_t;

void motor_pulse_init (motor_pulse_t * p_pulse);

/* Pure policy: no hardware or timing claims. Call repeatedly using the low
 * 32 bits of a microsecond clock. Stop has priority and latches until init.
 * Holding start at boot or after a pulse never starts another pulse.
 * After four separately triggered phases, the sequence remains finished. */
motor_pulse_phase_t motor_pulse_update (motor_pulse_t * p_pulse,
    uint32_t now_us, bool start_pressed, bool stop_pressed);

#endif /* MOTOR_PULSE_H */
