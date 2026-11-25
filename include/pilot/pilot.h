/** @file pilot.h
 * @brief Pilot selects one movement request per tick. Motion owns the motor outputs.
 *
 * Pure C, no kernel, no driver and no floating point. The caller owns the
 * clock, gathers the latest measurements and sends the returned command to
 * the motion layer.
 *
 * Safety can preempt any movement. Obstacle avoidance supplies detour commands
 * and reports when the pilot should return control to the line follower.
 *
 * Units: speed in mm/s (positive = forward), turn rate in deci-degrees per
 * second (counter-clockwise positive), travel in um, time in ms.
 */

#ifndef PILOT_PILOT_H
#define PILOT_PILOT_H

#include <stdbool.h>
#include <stdint.h>

#include "pilot/drive.h"
#include "pilot/avoidance.h"
#include "platform/static_assert.h"
#include "track/line_control.h"

/** Wheel feedback (stamp_ms) older than this gives a stop with reason STALE
 *  (ms). */
#define PILOT_STALE_MS            (100U)
/** A free-running forward ping normally arrives every 60 ms. Allow up to
 *  250 ms between observed sequences before stopping with reason SONAR. */
#define PILOT_FORWARD_PING_STALE_MS (250U)
/** A longer dropout needs a confirmed recovery before the pilot drives
 *  again. The time threshold is a trial value (ms). */
#define PILOT_SONAR_RESUME_MS     (1000U)
/** A command never exceeds these. The turn limit is both wheels at 300 mm/s
 *  (MOTOR_MAX_TARGET_UM_S), opposite. */
#define PILOT_SPEED_MAX_MM_S      (300)
#define PILOT_TURN_MAX_DECI_S     (3300)

/* All of these are guesses until measured on the car. */
/** Maximum wait for progress on each wheel commanded above its deadband (ms). */
#define PILOT_STUCK_MS            (1000U)
/** A wheel that travels less than this over a window shows no travel (um). */
#define PILOT_MOVE_MIN_UM         (2000)
/** A wheel commanded slower than this (mm/s) may stand still: it sits in the
 *  motor deadband, so no travel there is not a dead encoder. The start speed
 *  of the drive layer is the least speed it gives a wheel from rest. This is
 *  a guess, to be tuned on the car. */
#define PILOT_WHEEL_MIN_MM_S      (DRIVE_START_MM_S)
/** A stall during a confirmed obstacle maneuver permits this much back-off
 *  (um, the farther wheel) at this speed (mm/s). */
#define PILOT_BACKOFF_UM          (40000)
#define PILOT_BACKOFF_MM_S        (60)
#define PILOT_BACKOFF_MAX_MS      (2000U)

/* A hug move that stalls is backed off by the stuck check before it times out. */
STATIC_ASSERT(HUG_MOVE_FLOOR_MS >= PILOT_STUCK_MS, "hug move floor below the stuck window");

/** Why the command is a stop or a back-off. ESTOP, ENCODER and HUG_TIMEOUT
 *  latch until pilot_init(), which runs at boot only, except that
 *  An accepted START or RESET clears ENCODER and HUG_TIMEOUT; the others end
 *  when the cause clears, except SONAR and STALE below. SONAR is raised while
 *  the sonar task reports a fault or the forward ping sequence stops changing.
 *  STALE is raised while the wheel feedback is older than PILOT_STALE_MS.
 *  Either one that lasts PILOT_SONAR_RESUME_MS latches as SONAR_LATCHED or
 *  STALE_LATCHED, with the cause of the moment the latch sets. The pilot clears
 *  a data latch after consecutive usable forward pings with fresh feedback
 *  while the wheels are released. START or RESET also clears it. A latched
 *  STALE shows as HELD while motion holds the wheels. HELD is raised while
 *  motion holds the wheels; the stuck check starts again when the hold ends.
 *  HUG_TIMEOUT: a pivot or a drive of the hug ran past HUG_MOVE_TIMES times its
 *  planned time, and the hug has ended. The wire carries the number, so a new
 *  reason goes at the end. */
typedef enum
{
    PILOT_REASON_NONE = 0,
    PILOT_REASON_STALE,
    PILOT_REASON_ESTOP,
    PILOT_REASON_LINE_LOST_STOP,
    PILOT_REASON_STUCK,
    PILOT_REASON_ENCODER,
    PILOT_REASON_SONAR,
    PILOT_REASON_HELD,
    PILOT_REASON_HUG_TIMEOUT,
    PILOT_REASON_SONAR_LATCHED,
    PILOT_REASON_STALE_LATCHED,
    PILOT_REASON_COURSE_CLEARANCE, /* obstacle recovery lacks verified clearance */
    PILOT_REASON_COURSE_RECOVERY,
    PILOT_REASON_REJOIN_SEARCH,
    PILOT_REASON_REJOIN_STOP
} pilot_reason_t;

STATIC_ASSERT(sizeof(pilot_reason_t) <= 4U, "pilot_reason_t size");

/** Behaviour that set the command, in priority order. */
typedef enum
{
    PILOT_BEHAVIOUR_NONE = 0,
    PILOT_BEHAVIOUR_SAFETY,
    PILOT_BEHAVIOUR_HUG,
    PILOT_BEHAVIOUR_FOLLOW_LINE,
    PILOT_BEHAVIOUR_REJOIN_SEARCH
} pilot_behaviour_t;

STATIC_ASSERT(sizeof(pilot_behaviour_t) <= 4U, "pilot_behaviour_t size");

/** Latest measurements. The wheel travel is taken at stamp_ms. The ping is the
 *  latest one the sonar task published, with no time stamp of its own: a reply
 *  to the request of the last command (b_ping_reply, ping_seq is the request
 *  sequence) or a free-running ping straight ahead (ping_seq counts them).
 *  ping_echo is a LOOK_ECHO_* class. */
typedef struct
{
    int64_t  left_um;       /* left wheel displacement, positive forward  */
    int64_t  right_um;      /* right wheel displacement, positive forward */
    uint32_t now_ms;        /* free-running clock (wraps at 2^32)         */
    uint32_t stamp_ms;      /* clock at the wheel feedback                */
    bool     b_line_left;   /* left sensor is over the line               */
    bool     b_line_right;  /* right sensor is over the line              */
    bool     b_ping_reply;  /* the ping answers a request                 */
    bool     b_sonar_fault; /* the sonar task reports a fault             */
    bool     b_held;        /* motion holds the wheels: no command runs   */
    bool     b_barcode;     /* a barcode edge was seen within 250 ms      */
    uint16_t ping_mm;       /* distance of the ping, 0 for OPEN           */
    int16_t  ping_servo_deci; /* servo angle of the ping, left positive   */
    uint8_t  ping_echo;     /* LOOK_ECHO_VALID, _OPEN or _TOO_CLOSE       */
    uint8_t  ping_seq;      /* 6 bits                                     */
    bool     b_bounded_course : 1; /* use the bounded course stop and sweep */
    bool     b_marker_progress : 1; /* verified barcode stream advanced this pass */
    bool     b_stationary : 1;  /* both wheels report fresh zero speed       */
    bool     b_commanded_turn : 1; /* the turn executor supplies nominal wheel targets */
    uint8_t  marker_prefix_mm; /* measured elements of the current tentative start */
    int16_t  wheel_radius_mm; /* measured forward arc, left positive, zero if unavailable or straight */
} pilot_meas_t;

STATIC_ASSERT(sizeof(pilot_meas_t) == 40U, "pilot_meas_t size");

/** The command of one tick. behaviour and reason are for the decision log.
 *  During a commanded turn, NONE allows the turn executor to run that tick.
 *  While b_ping_req is set the sonar task moves the servo to ping_servo_deci
 *  and answers once with sequence ping_seq; without a request it pings
 *  straight ahead. */
typedef struct
{
    int16_t speed_mm_s;
    int16_t turn_deci_s;
    uint8_t behaviour;      /* pilot_behaviour_t */
    uint8_t reason;         /* pilot_reason_t    */
    int16_t ping_servo_deci;
    uint8_t ping_seq;       /* 6 bits */
    uint8_t b_ping_req;
    uint16_t contact_mm;   /* measured forward range on entry to an obstacle hold or hug */
    bool     b_scan_pivot : 1;
    bool     b_stop_left : 1;   /* exact zero from the wheel-space follower */
    bool     b_stop_right : 1;
    bool     b_rejoined : 1; /* IR confirmed the exit tape on this tick */
    uint8_t  course_scan_samples; /* at completion: usable rays at this encoder anchor, plus one */
} pilot_cmd_t;

STATIC_ASSERT(sizeof(pilot_cmd_t) == 14U, "pilot_cmd_t size");

typedef struct
{
    line_follow_t follow;
    uint32_t      ref_left_um;   /* travel at the start of the window, low 32 bits */
    uint32_t      ref_right_um;
    uint32_t      since_ms;      /* start of the window                            */
    uint32_t      forward_ping_ms; /* grace start, last forward observation or requested look */
    uint32_t      slip_left_um;   /* travel at the first stable forward range */
    uint32_t      slip_right_um;
    uint8_t       b_estop;       /* latched until pilot_init()                     */
    uint8_t       b_encoder;     /* latched until pilot_init() or pilot_clear_latches() */
    /* Last command and the open or latched data stop. */
    uint8_t       flags;
    uint8_t       b_timeout;     /* latched after an expired maneuver */
    uint8_t       forward_ping_seq;
    uint16_t      slip_range_mm;
    bool          b_slip_forward : 1;
    bool          b_check_left : 1;  /* wheel showed no travel through a commanded window */
    bool          b_check_right : 1;
    uint8_t       data_recovery_pings;
    bool          b_forward_timer_started;
    bool          b_follow_forward; /* forward line command during the measured interval */
    int16_t       line_radius_mm; /* prior forward line-steering request, left positive */
    avoidance_t   avoidance;
} pilot_state_t;

STATIC_ASSERT(sizeof(pilot_state_t) == 496U, "pilot_state_t size");

/** Clear the state: no latch, follower at its start. The line task calls it
 *  once, at boot. */
void pilot_init(pilot_state_t * p_state);

/**
 * One pilot tick.
 *
 * @param[in,out] p_state State.
 * @param[in]     p_meas  Latest measurements.
 * @param[out]    p_cmd   The command, within the speed and turn limits.
 */
void pilot_step(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                pilot_cmd_t * p_cmd);

/* Experimental verified contour input. The local caller binds a host-verified
 * route to the stopped axle pose which defines its origin and heading. */
#define PILOT_CONTOUR_STEPS (7U)

typedef struct
{
    uint16_t length_mm; /* zero only for a verified on-the-spot pivot */
    int16_t turn_deci;  /* signed total change of heading, left positive */
} pilot_contour_step_t;

typedef struct
{
    uint32_t id;             /* nonzero evidence generation */
    uint32_t issued_ms;
    uint32_t left_um;        /* low bits of the wheel counts at the anchor */
    uint32_t right_um;
    pilot_contour_step_t steps[PILOT_CONTOUR_STEPS];
    uint8_t count;
} pilot_contour_plan_t;

typedef enum
{
    PILOT_CONTOUR_IDLE = 0,
    PILOT_CONTOUR_WAIT,
    PILOT_CONTOUR_MOVE,
    PILOT_CONTOUR_LINE,
    PILOT_CONTOUR_DONE,
    PILOT_CONTOUR_STOPPED
} pilot_contour_status_t;

/* Caller-owned state, zeroed before first use. Production line-task state
 * need not allocate a contour while it has no verified geometry provider. */
typedef struct
{
    pilot_contour_plan_t plan;
    uint32_t last_left_um;
    uint32_t last_right_um;
    uint32_t step_ms;
    int32_t x_um;
    int32_t y_um;
    int32_t diff_um;
    int32_t progress_um;
    int32_t route_x_mm;
    int32_t route_y_mm;
    int16_t route_head_deci;
    uint8_t index;
    uint8_t status;
    bool b_terminal_stationary;
} pilot_contour_t;

STATIC_ASSERT(sizeof(pilot_contour_step_t) == 4U, "contour step size");
STATIC_ASSERT(sizeof(pilot_contour_plan_t) == 48U, "contour plan size");
STATIC_ASSERT(sizeof(pilot_contour_t) == 92U, "contour state size");

/* Local prototype seam. An ineligible offer leaves ordinary pilot behavior in
 * control. After takeover, keep the same state object until it finishes. A
 * cancelled or unsafe route holds the wheels until the caller explicitly
 * reinitializes at a surveyed pose. The installed bounded course never takes
 * a contour from this seam. */
void pilot_step_with_contour(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                             pilot_contour_t * p_contour,
                             pilot_contour_plan_t const * p_offer, pilot_cmd_t * p_cmd);

/** Operator emergency stop. The stop holds until pilot_init(): the motion
 *  service latches the emergency stop until reboot as well. */
void pilot_estop(pilot_state_t * p_state);

/** RESET: clear the ENCODER, HUG_TIMEOUT, SONAR and STALE latches, restart the
 *  stuck window and start the follower afresh, keeping the side of the line it
 *  recorded.
 *  The emergency stop stays. RESET does not start the car: the supervisor
 *  holds the wheels (PILOT_REASON_HELD) until START. */
void pilot_clear_latches(pilot_state_t * p_state);

/** Release the sonar and stale-data latches after fresh feedback recovers. */
void pilot_release_sonar(pilot_state_t * p_state);

/** Clear recoverable pilot latches on operator START. A stall hold may resume the original route once
 * new forward clearance is confirmed with the wheels stationary. */
void pilot_start_run(pilot_state_t * p_state);
/* Discard obstacle geometry and route motion when the selected floor changes. */
void pilot_change_floor(pilot_state_t * p_state, bool b_continuous_floor);
/* Replan at the recovered stationary pose. An interrupted hug or a near
 * front observation needs a new survey before the wheels move. */
void pilot_recover_pose(pilot_state_t * p_state, bool clear, uint32_t now_ms);

/** True when a supervisor count (RESET or START) differs from the value last
 *  seen, which it then records. Each count keeps its own last-seen value. */
bool pilot_count_moved(uint8_t * p_seen, uint8_t count);

/**
 * Wheel targets for one command, as the line task sends them.
 *
 * @param[in,out] p_drive      Drive state.
 * @param[in]     p_cmd        The command of this pass.
 * @param[in]     dt_ms        Time since the last pass.
 * @param[out]    p_left_um_s  Left wheel target in um/s.
 * @param[out]    p_right_um_s Right wheel target in um/s.
 */
void pilot_drive(drive_state_t * p_drive, pilot_cmd_t const * p_cmd, uint32_t dt_ms,
                 int32_t * p_left_um_s, int32_t * p_right_um_s);

#endif /* PILOT_PILOT_H */

/*** end of file ***/
