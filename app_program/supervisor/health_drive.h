/** @file health_drive.h
 * @brief Drive check and encoder probe of the supervisor. Pure C, so a host
 *        test and the car bench use it without the kernel.
 */

#ifndef SUPERVISOR_HEALTH_DRIVE_H
#define SUPERVISOR_HEALTH_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

/* Drive check. It reads the wheel targets, speeds and duty.
 *
 *   crossed   The wheel told to go faster does not turn faster. The encoder
 *             plugs are swapped, or a wheel turns the wrong way.
 *   free      Both wheels turn on target with much less duty than they needed
 *             on the ground. The wheels run free: the car is in the air.
 *   loaded    A wheel told to go forward runs well under its target with much
 *             more duty than the grounded baseline needs for that target. The
 *             car pushes against something it cannot see.
 */

/* Only wheels told to go at least this fast count (um/s). */
#define DRIVE_TARGET_MIN_UM_S (100000)
/* A wheel is on target when its speed is within this percent of the target. */
#define DRIVE_TRACK_PERCENT (10)
/* The two targets must differ this much for the crossed test (um/s). */
#define DRIVE_TARGET_GAP_UM_S (60000)
/* The two speeds must differ this much to count as a vote (um/s). */
#define DRIVE_SPEED_GAP_UM_S (20000)
/* The crossed test judges a window of this length, then starts again. */
#define DRIVE_WINDOW_MS (2000U)
/* Disagreeing votes needed in a window, and the ratio against agreeing votes. */
#define DRIVE_CROSS_MS (800U)
#define DRIVE_CROSS_RATIO (3U)
/* Grounded time needed to learn the duty per speed of each wheel. */
#define DRIVE_BASE_MS (2000U)
/* The baseline moves 1/16 of the way to each new sample. */
#define DRIVE_BASE_DIVISOR (16)
/* A wheel is free when its duty per speed is under 1/DRIVE_FREE_DIVISOR of the baseline. */
#define DRIVE_FREE_DIVISOR (2U)
/* Checks in a row before the cue sets (300 ms at 100 ms) and before it clears. */
#define DRIVE_FREE_CHECKS (3U)
#define DRIVE_CLEAR_CHECKS (2U)
/* Loaded: forward target at least this (um/s), speed under this percent of it,
   duty at least this many times the baseline duty for the target, for this many
   checks in a row (guesses until measured on the car). */
#define DRIVE_LOAD_TARGET_MIN_UM_S (50000)
#define DRIVE_LOAD_SPEED_PERCENT (50)
#define DRIVE_LOAD_DUTY_TIMES (2U)
#define DRIVE_LOAD_CHECKS (2U)

/* Both wheels, left then right. */
#define DRIVE_WHEELS (2U)

typedef struct
{
    int16_t duty_pm[DRIVE_WHEELS];     /* signed duty, permille                */
    int32_t speed_um_s[DRIVE_WHEELS];  /* signed, positive car-forward         */
    int32_t target_um_s[DRIVE_WHEELS]; /* what the follower asked for          */
    bool    b_judge;        /* running, no hold, feedback fresh, car on the floor */
} drive_in_t;

typedef struct
{
    uint16_t agree_ms;
    uint16_t disagree_ms;
    uint16_t window_ms;
    uint32_t base[DRIVE_WHEELS]; /* duty permille per m/s of a grounded wheel; 0 = none */
    uint32_t base_ms;
    uint8_t  free_run;
    uint8_t  busy_run;
    uint8_t  load_run;
    bool     b_crossed;
    bool     b_free;
    bool     b_loaded;
} drive_t;

void health_drive_init (drive_t * p_drive);

/* One check every dt_ms. Returns true once, when b_crossed turns on. */
bool health_drive_update (drive_t * p_drive, const drive_in_t * p_in, uint32_t dt_ms);

/* Encoder probe. The bench drives one wheel alone and reads how far each
 * encoder moved, in mm with the car-forward sign. The verdict says whether the
 * encoder of the driven wheel answers. */
typedef enum
{
    DRIVE_PROBE_OK = 0,   /* the driven wheel moved forward, the other did not   */
    DRIVE_PROBE_DEAD,     /* neither encoder moved                                */
    DRIVE_PROBE_REVERSED, /* the driven encoder counted backward                  */
    DRIVE_PROBE_CROSSED,  /* the other encoder moved instead                      */
    DRIVE_PROBE_UNCLEAR   /* both moved, or too little to tell                    */
} drive_probe_t;

/* Travel of the driven encoder that counts as moving (mm). */
#define DRIVE_PROBE_MOVED_MM (15)
/* A wheel moved under 1/DRIVE_PROBE_DEAD_DIV of that is not moving. The other
   encoder counts as still when it moved under 1/DRIVE_PROBE_RATIO of the driven one. */
#define DRIVE_PROBE_DEAD_DIV (3)
#define DRIVE_PROBE_RATIO    (4)

drive_probe_t health_drive_probe_verdict (int32_t driven_mm, int32_t other_mm);

#endif /* SUPERVISOR_HEALTH_DRIVE_H */

/*** end of file ***/
