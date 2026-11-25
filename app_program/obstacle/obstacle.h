/** @file obstacle.h
 *
 * @brief Obstacle public facade.
 *
 * The scan task owns the ultrasonic trigger, echo capture and servo during
 * a run. Consumers decode published measurements using obstacle_ping.h.
 *
 * Start-up order (on processor 1, which is RP2040 core 0):
 *   1. drv_gpio_irq_init()            shared dispatcher
 *   2. motion_service_init()      by the composition root
 *   3. obstacle_line_init()       pins, servo, ISR and obstacle resources
 *
 * In the pilot build the line task reads the latest ping and submits the
 * selected maneuver's scan request. The legacy build runs obstacle_line_step().
 * Telemetry can read obstacle_snapshot() independently.
 */

#ifndef NAVIGATION_OBSTACLE_H
#define NAVIGATION_OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/typedef.h>

#include "geometry.h"
#include "obstacle_cmd.h"
#include "obstacle_flags.h"
#include "obstacle_ping.h"
#include "scan_limits.h"
#include "platform/static_assert.h"

/* Slots in the scan profile that obstacle_copy_profile() fills. */
#define OBSTACLE_PROFILE_SLOTS (37U)

/* Echo classes: VALID, OPEN, TOO_CLOSE, NO_ECHO, GLITCH, STUCK. */
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

/* Wheel travel since boot, forward and backward apart, in mm. Both counts
 * only grow and wrap at 2^32. */
typedef struct
{
    uint32_t fwd_mm;
    uint32_t back_mm;
} obstacle_travel_t;

STATIC_ASSERT(sizeof(obstacle_travel_t) == 8U, "obstacle_travel_t size");

/* Wheel travel supplied by the motion subsystem. E_OK only with fresh wheel
 * feedback; any other result leaves *p_travel unchanged. */
typedef ER (*obstacle_distance_fn_t)(obstacle_travel_t * p_travel);

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
    uint16_t faults;        /* Stop faults and active echo faults.     */
    int16_t  servo_deci;    /* Sonar servo angle, 0.1 degree.           */
    int16_t  body_deci;     /* Body offset asked for, 0.1 degree.       */
    uint8_t  state;         /* STOPPED, MONITOR, WAIT_STOP, ...         */
    uint8_t  cause;         /* Why the statechart is stopped, else 0.   */
    uint8_t  echo_class;    /* Latest echo: VALID, OPEN, TOO_CLOSE, ... */
    uint8_t  req_id;        /* Current request id.                      */
    uint8_t  seq;           /* Ping sequence, wraps at 32.              */
    uint8_t  dropped_events; /* Commands, events lost to a full queue.  */
    uint8_t  flags;         /* OBSTACLE_FLAG_* warning flags.           */
    bool     ready;         /* Task is running.                         */
    uint8_t  stop_reason;   /* Bypass give-up reason, else 0.           */
} obstacle_status_t;

STATIC_ASSERT (sizeof(obstacle_status_t) == 18U, "obstacle_status_t size");

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
    uint8_t  req_id;        /* Scan request this plan answers.          */
} obstacle_plan_summary_t;

STATIC_ASSERT (sizeof(obstacle_plan_summary_t) == 12U, "obstacle_plan_summary_t size");

/* Register the task stack. */
void obstacle_init (void);

/* Create and start the obstacle task. Returns false when it could not. */
bool obstacle_start (void);

/* Initialize obstacle while parked. Uses the motion-service links and the
 * obstacle outbox. The startup task calls this after drv_gpio_irq_init() and
 * motion_service_init(); the task created by obstacle_start() runs
 * obstacle_run(). */
ER obstacle_line_init (void);

/* Optionally observe the initial center and subsequent commanded sonar angles.
 * Bind once before the scanner task starts. The observer runs in that task
 * before settling and must return without waiting. There is no shaft readback. */
typedef void (*obstacle_sonar_aim_fn_t)(int16_t commanded_deci);
void obstacle_bind_sonar_aim (obstacle_sonar_aim_fn_t p_observe);

typedef struct
{
    obstacle_plan_summary_t plan;
    const uint16_t * p_completed_profile;
    uint16_t distance_mm;
    uint8_t  state;
    uint8_t  cause;
    uint8_t  echo_class;
    uint8_t  stop_reason;
} obstacle_legacy_transition_t;

STATIC_ASSERT(sizeof(obstacle_legacy_transition_t) == ((sizeof(const uint16_t *) == 4U) ? 24U : 32U),
              "obstacle_legacy_transition_t size");

/* Bind before starting the legacy scanner. The task calls the observer after
 * publishing a transition and before notifying navigation. The profile is
 * present only at scan completion. All pointers are borrowed for the call.
 * The observer must return without waiting. */
typedef void (*obstacle_legacy_transition_fn_t)(const obstacle_legacy_transition_t * p_transition);
void obstacle_bind_legacy_transition (obstacle_legacy_transition_fn_t p_observe);

/* Task body. Never returns. */
void obstacle_run (void);

/* Legacy build only. True means the obstacle navigator supplied this tick's
 * movement request. In the pilot build the line task calls pilot_step(). */
bool obstacle_line_step (bool b_line_left, bool b_line_right);

/* Lower-level API used when the caller needs its own links. */
ER obstacle_setup (obstacle_links_t const * p_links);
ER obstacle_post_cmd (uint8_t cmd);

/* Copy the latest status. Returns E_OBJ before obstacle_setup(). */
ER obstacle_snapshot (obstacle_status_t * p_status);

/* The OBSTACLE_FAULT_* bits for stop faults and active echo faults. */
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

/* Ask the sonar task for one ping at a servo angle. Never blocks; safe from
 * either core. The task clamps the angle to the servo limit, waits for the
 * servo to settle and publishes the ping word with the reply flag, the same
 * 6-bit sequence and the angle it used. A new request replaces one that has
 * not started. Returns E_OBJ before init. */
ER obstacle_ping_request (int16_t servo_deci, uint8_t seq);

/* Non-consuming latest ping, shared by readers. Bit 27 identifies a reply.
 * The servo field records the commanded bearing. There is no capture timestamp. */
uint32_t obstacle_ping_read (void);

/* Event flag for NAV_EVT and NAV_STEER waits. E_OBJ before init. */
ID obstacle_flag_id (void);

#endif /* NAVIGATION_OBSTACLE_H */

/*** end of file ***/
