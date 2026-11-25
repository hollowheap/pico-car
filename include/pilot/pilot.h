/** @file pilot.h
 * @brief Pilot: the one owner of the wheels. One motion command per tick.
 *
 * Pure C, no kernel, no driver and no floating point. The caller owns the
 * clock, gathers the latest measurements and sends the returned command to
 * the motion layer.
 *
 * Each tick the behaviours run in priority order and the first one that asks
 * for control sets the command. The choice is made again every tick; the
 * behaviours do not hand anything over to each other.
 *
 * Units: speed in mm/s (positive = forward), turn rate in deci-degrees per
 * second (counter-clockwise positive), travel in um, time in ms.
 */

#ifndef PILOT_PILOT_H
#define PILOT_PILOT_H

#include <stdbool.h>
#include <stdint.h>

#include "pilot/drive.h"
#include "pilot/hug.h"
#include "pilot/look.h"
#include "platform/static_assert.h"
#include "track/line_control.h"

/** Wheel feedback (stamp_ms) older than this gives a stop with reason STALE
 *  (ms). The ping has no time stamp, so the sonar is not checked for age. */
#define PILOT_STALE_MS            (100U)
/** A sonar fault or stale wheel feedback that ends within this time lets the
 *  pilot drive on by itself; one that lasts longer latches the stop until
 *  pilot_release_sonar() or pilot_clear_latches() (ms). [Default] a guess, to
 *  be measured on the car. */
#define PILOT_SONAR_RESUME_MS     (1000U)
/** A command never exceeds these. The turn limit is both wheels at 300 mm/s
 *  (MOTOR_MAX_TARGET_UM_S), opposite. */
#define PILOT_SPEED_MAX_MM_S      (300)
#define PILOT_TURN_MAX_DECI_S     (3300)

/* All of these are guesses until measured on the car. */
/** Both wheels without travel for this long while the pilot commands motion
 *  count as stuck (ms). */
#define PILOT_STUCK_MS            (1000U)
/** A wheel that travels less than this over a window shows no travel (um). */
#define PILOT_MOVE_MIN_UM         (2000)
/** A wheel commanded slower than this (mm/s) may stand still: it sits in the
 *  motor deadband, so no travel there is not a dead encoder. The start speed
 *  of the drive layer is the least speed it gives a wheel from rest. This is
 *  a guess, to be tuned on the car. */
#define PILOT_WHEEL_MIN_MM_S      (DRIVE_START_MM_S)
/** A stuck car backs off this far (um, the farther wheel) at this speed
 *  (mm/s), and gives up the back-off after PILOT_BACKOFF_MAX_MS. */
#define PILOT_BACKOFF_UM          (40000)
#define PILOT_BACKOFF_MM_S        (60)
#define PILOT_BACKOFF_MAX_MS      (2000U)

/* A hug move that stalls is backed off by the stuck check before it times out. */
STATIC_ASSERT(HUG_MOVE_FLOOR_MS >= PILOT_STUCK_MS, "hug move floor below the stuck window");

/** Why the command is a stop or a back-off. ESTOP, ENCODER and HUG_TIMEOUT
 *  latch until pilot_init(), which runs at boot only, except that
 *  pilot_clear_latches() (RESET) clears ENCODER and HUG_TIMEOUT; the others end
 *  when the cause clears, except SONAR and STALE below. SONAR is raised while
 *  the sonar task reports a fault. STALE is raised while the wheel feedback is
 *  older than PILOT_STALE_MS; it says nothing of the sonar. Either one that
 *  lasts PILOT_SONAR_RESUME_MS latches: the stop then reads SONAR_LATCHED or
 *  STALE_LATCHED, with the cause of the moment the latch sets, until
 *  pilot_release_sonar() (START) or pilot_clear_latches() (RESET). A latched
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
    PILOT_REASON_STALE_LATCHED
} pilot_reason_t;

STATIC_ASSERT(sizeof(pilot_reason_t) <= 4U, "pilot_reason_t size");

/** Behaviour that set the command, in priority order. */
typedef enum
{
    PILOT_BEHAVIOUR_NONE = 0,
    PILOT_BEHAVIOUR_SAFETY,
    PILOT_BEHAVIOUR_HUG,
    PILOT_BEHAVIOUR_FOLLOW_LINE
} pilot_behaviour_t;

STATIC_ASSERT(sizeof(pilot_behaviour_t) <= 4U, "pilot_behaviour_t size");

/** Phase of the stuck check. A try drives and watches the wheel travel; a
 *  stuck try backs off, and the try after a back-off is a retry. A back-off
 *  after a try in which one wheel alone showed no travel ends in ENCODER when
 *  that wheel shows none in the back-off either. */
typedef enum
{
    PILOT_PHASE_TRY = 0,
    PILOT_PHASE_RETRY,
    PILOT_PHASE_BACKOFF,
    PILOT_PHASE_BACKOFF_ONE_WHEEL
} pilot_phase_t;

STATIC_ASSERT(sizeof(pilot_phase_t) <= 4U, "pilot_phase_t size");

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
} pilot_meas_t;

STATIC_ASSERT(sizeof(pilot_meas_t) == 40U, "pilot_meas_t size");

/** The command of one tick. behaviour and reason are for the decision log.
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
} pilot_cmd_t;

STATIC_ASSERT(sizeof(pilot_cmd_t) == 10U, "pilot_cmd_t size");

typedef struct
{
    line_follow_t follow;
    uint32_t      ref_left_um;   /* travel at the start of the window, low 32 bits */
    uint32_t      ref_right_um;
    uint32_t      since_ms;      /* start of the window                            */
    uint8_t       phase;         /* pilot_phase_t                                  */
    uint8_t       b_estop;       /* latched until pilot_init()                     */
    uint8_t       b_encoder;     /* latched until pilot_init() or pilot_clear_latches() */
    /* The last command: moving, slow wheels, reverse. Also the sonar run, its
     * latch and the cause of the latch. */
    uint8_t       flags;
    hug_t         hug;
    look_t        look;          /* the one look, used by the hug */
    uint8_t       b_timeout;     /* as b_encoder; in the padding                 */
} pilot_state_t;

STATIC_ASSERT(sizeof(pilot_state_t) == 384U, "pilot_state_t size");

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

/** Operator emergency stop. The stop holds until pilot_init(): the motion
 *  service latches the emergency stop until reboot as well. */
void pilot_estop(pilot_state_t * p_state);

/** RESET: clear the ENCODER, HUG_TIMEOUT, SONAR and STALE latches, restart the
 *  stuck window and start the follower afresh, keeping the side of the line it
 *  recorded.
 *  The emergency stop stays. RESET does not start the car: the supervisor
 *  holds the wheels (PILOT_REASON_HELD) until START. */
void pilot_clear_latches(pilot_state_t * p_state);

/** START: release the SONAR and STALE latches and end the open run, so that
 *  the next fault or stale feedback gets a full window. ENCODER, HUG_TIMEOUT
 *  and the emergency stop stay: only RESET clears the first two. */
void pilot_release_sonar(pilot_state_t * p_state);

/** True when a supervisor count (RESET or START) differs from the value last
 *  seen, which it then records. Each count keeps its own last-seen value. */
bool pilot_count_moved(uint8_t * p_seen, uint8_t count);

/**
 * Wheel targets for one command, as the line task sends them. The car and the
 * host sim both call it, so the sim drives through the same limits.
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
