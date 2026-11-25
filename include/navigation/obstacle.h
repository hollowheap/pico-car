/** @file obstacle.h
 *
 * @brief Obstacle public facade.
 *
 * This is the only obstacle header another subsystem may include. The
 * packed status word, the echo flags, the scan profile layout and the
 * statechart are private to src/navigation/obstacle/.
 *
 * Start-up order (on processor 1, which is RP2040 core 0):
 *   1. gpio_irq_init()            shared dispatcher
 *   2. motion_service_init()      by the composition root
 *   3. obstacle_line_init()       pins, servo, ISR, task, RESET and START
 *
 * The line follower then calls obstacle_line_step() every pass. Telemetry
 * reads obstacle_snapshot() whenever it needs the latest state; it does
 * not read the packed word.
 */

#ifndef NAVIGATION_OBSTACLE_H
#define NAVIGATION_OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "navigation/obstacle_cmd.h"
#include "platform/static_assert.h"

/* Slots in the scan profile that obstacle_copy_profile() fills. */
#define OBSTACLE_PROFILE_SLOTS (37U)

/* Echo classes: VALID, OPEN, TOO_CLOSE, NO_ECHO, GLITCH, STUCK (D97). */
#define OBSTACLE_ECHO_CLASSES (6U)

/* Index of each class in obstacle_echo_counts(). */
#define OBSTACLE_ECHO_IDX_VALID     (0U)
#define OBSTACLE_ECHO_IDX_OPEN      (1U)
#define OBSTACLE_ECHO_IDX_TOO_CLOSE (2U)
#define OBSTACLE_ECHO_IDX_NO_ECHO   (3U)
#define OBSTACLE_ECHO_IDX_GLITCH    (4U)
#define OBSTACLE_ECHO_IDX_STUCK     (5U)

/* Emergency stop supplied by the motion subsystem. */
typedef ER (*obstacle_stop_fn_t)(void);

/* Distance travelled supplied by the motion subsystem, in mm. */
typedef ER (*obstacle_distance_fn_t)(uint32_t * p_distance_mm);

/* Links handed to obstacle at start-up. Any member may be 0 or NULL.
 * h_nav_mbf 0 means "use obstacle's own 4-slot outbox". */
typedef struct
{
    ID                     h_nav_mbf;
    obstacle_stop_fn_t     p_stop;
    obstacle_distance_fn_t p_get_distance_mm;
} obstacle_links_t;

STATIC_ASSERT (sizeof(obstacle_links_t) == (3U * sizeof(void *)),
               "obstacle_links_t size");

/* Latest state in engineering units. The numeric codes match the wire
 * schema owned by protocol/ and the dashboard decoder; this header does
 * not fix their values. */
typedef struct
{
    uint16_t distance_mm;   /* Latest range: VALID, or 20 for TOO_CLOSE. */
    uint16_t faults;        /* Latched OBSTACLE_FAULT_* bits (9).       */
    int16_t  servo_deci;    /* Sonar servo angle, 0.1 degree.           */
    int16_t  body_deci;     /* Body offset asked for, 0.1 degree.       */
    uint8_t  state;         /* STOPPED, MONITOR, WAIT_STOP, ...         */
    uint8_t  cause;         /* Why the statechart is stopped, else 0.   */
    uint8_t  echo_class;    /* Latest echo: VALID, OPEN, TOO_CLOSE, ... */
    uint8_t  req_id;        /* Current request id.                      */
    uint8_t  seq;           /* Ping sequence, wraps at 32.              */
    uint8_t  dropped_events; /* Events lost to a full queue.            */
    uint8_t  flags;         /* OBSTACLE_FLAG_* warning flags.           */
    bool     ready;         /* Task is running.                         */
} obstacle_status_t;

STATIC_ASSERT (sizeof(obstacle_status_t) == 16U, "obstacle_status_t size");

/* The plan from the last finished scan. It is valid after the first scan. */
typedef struct
{
    uint16_t closest_mm;    /* Closest range in the obstacle region.    */
    uint16_t width_mm;      /* Reported obstacle width.                 */
    uint16_t need_mm;       /* Side room the chosen side needs.         */
    uint8_t  status;        /* OK, CLEAR, NO_SAFE_PATH, FAILED.         */
    uint8_t  side;          /* Pass the obstacle on this side.          */
    uint8_t  start_deg;     /* Out-pivot angle.                         */
    bool     b_wall;        /* The plan follows a wall.                 */
} obstacle_plan_summary_t;

STATIC_ASSERT (sizeof(obstacle_plan_summary_t) == 10U, "obstacle_plan_summary_t size");

/* Start obstacle. Call once from the line task, after gpio_irq_init() and
 * motion_service_init(). Uses the default links: motion-service stop and
 * odometer, obstacle outbox. The composition root creates the task and
 * calls obstacle_run() from it. */
ER obstacle_line_init (void);

/* Task body. Never returns. */
void obstacle_run (void);

/* One handshake pass. Call every line-task loop. Returns true when obstacle
 * owns the motors this pass, so the line follower must send no speed. */
bool obstacle_line_step (bool b_line_left, bool b_line_right);

/* Lower-level API used when the caller needs its own links. */
ER obstacle_init (obstacle_links_t const * p_links);
ER obstacle_post_cmd (uint8_t cmd);
ER obstacle_get_event (uint32_t * p_evt, TMO tmout);

/* Copy the latest status. Returns E_OBJ before obstacle_init(). */
ER obstacle_snapshot (obstacle_status_t * p_status);

uint16_t obstacle_get_faults (void);

/* Copy of the scan profile: OBSTACLE_SLOT_COUNT packed slots. Never waits.
 * Returns E_BUSY if obstacle was writing it, E_OBJ before init. */
ER obstacle_copy_profile (uint16_t * p_profile);

/* Copy the plan that goes with the last finished profile. Never waits.
 * Returns E_BUSY if obstacle was writing it, E_OBJ before init. */
ER obstacle_copy_plan (obstacle_plan_summary_t * p_plan);

/* Pings per echo class since boot: OBSTACLE_ECHO_CLASSES counts. The caller
 * subtracts two reads to get a rate. Does nothing before init. */
void obstacle_echo_counts (uint32_t * p_counts);

/* Event flag for NAV_EVT and NAV_STEER waits. E_OBJ before init. */
ID obstacle_flag_id (void);

#endif /* NAVIGATION_OBSTACLE_H */

/*** end of file ***/
