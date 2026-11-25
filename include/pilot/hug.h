/** @file hug.h
 * @brief Hug: go round an obstacle on the line, keeping it at one side.
 *
 * Pure C99, integer only, no kernel and no driver calls. The pilot calls
 * hug_step() every tick. Two hits in the last three free-running forward
 * pings normally start a survey. On an established tape bend, the follower
 * can briefly recheck a return outside its measured turning path. A persistent
 * return starts the survey. A sonar-confirmed contact runs the Buddy 5 signed
 * approach, outward and parallel legs, return scans and optical rejoin in
 * bypass.c. A wheel-only stall without a usable sonar range uses the geometric
 * face recovery below.
 *
 * The hug keeps its own dead-reckoned pose in the look frame of look.h: the
 * origin is the axle at the hit and x runs along the heading of the hit. The
 * car turns in place only (speed 0, turn rate only), and the sonar looks
 * while the car stands still. Once the body clears a measured endpoint, the
 * axle stops abreast of it to look for the adjacent face. An inconclusive
 * clearance survey can repeat once there before the pilot holds the wheels.
 *
 * A wheel stall in a leg is the pilot's: its safety back-off backs the car up
 * (pilot.c), and when that ends the hug gets b_stall and starts a look from the
 * stalled pose. The hug itself does not back up for a stall.
 *
 * Trigonometry comes from src/navigation/obstacle/geometry.c, which must be
 * linked next to hug.c and look.c (include path src/navigation/obstacle).
 *
 * Every tunable below is a guess until it is measured on the car.
 */

#ifndef PILOT_HUG_H
#define PILOT_HUG_H

#include <stdbool.h>
#include <stdint.h>

#include "navigation/obstacle_ping.h"
#include "pilot/look.h"
#include "pilot/bypass.h"
#include "platform/chassis.h"
#include "platform/static_assert.h"

/** Trigger: this many of the last three forward pings are hits at or below
 *  the standoff (mm from the sonar). */
#define HUG_TRIGGER_HITS      (2U)
#define HUG_STANDOFF_MM       (250)

/** A free-running forward ping at or below this during a leg means a face
 *  ahead (mm from the sonar). */
#define HUG_GUARD_MM          (300)
#define HUG_FORWARD_RESERVE_MM (120)

/** Look: rays this far apart (deci-degrees) within HUG_NEAR_ZONE_DECI of the
 *  face normal. A face farther than HUG_FAR_MM from the sonar gets the finer
 *  looks below. */
#define HUG_NEAR_STEP_DECI    (50)
#define HUG_NEAR_ZONE_DECI    (350)
#define HUG_FAR_MM            (350)

/** The first look is coarser than the later ones: rays at these offsets
 *  (deci-degrees from the heading of the hit), in this order. Rays within
 *  HUG_CONTACT_NEAR_DECI are the face; the ones beyond it only measure the room
 *  beside it, from the two body steps at +-750. Not measured on the car. */
#define HUG_CONTACT_RAYS \
    -800, -650, -350, -300, -250, -200, -150, -100, -50, 0, 50, 100, 150, \
    200, 250, 300, 350, 650, 800
#define HUG_CONTACT_NEAR_DECI (350)

/** Once both edges are in trust each is looked at again: rays this far apart
 *  within this of the window centre (the FAR pair for a face farther than
 *  HUG_FAR_MM from the sonar, with as many rays), which lies this far outside
 *  the edge (the beam half-angle), so the window holds the last ray on the
 *  face and the first one past it. A finer edge that is farther than HUG_REFINE_TOL_MM from
 *  the first one is not believed. */
#define HUG_REFINE_STEP_DECI  (18)
#define HUG_REFINE_ZONE_DECI  (90)
#define HUG_REFINE_FAR_STEP_DECI (12)
#define HUG_REFINE_FAR_ZONE_DECI (60)
#define HUG_REFINE_OUT_DECI   LOOK_BEAM_DECI
#define HUG_REFINE_TOL_MM     (45)

/** Body step and servo reach for the contact and inner looks. */
#define HUG_PIVOT_STEP_DECI   (250)
#define HUG_SERVO_MAX_DECI    (HUG_PIVOT_STEP_DECI / 2)
STATIC_ASSERT(HUG_SERVO_MAX_DECI <= OBSTACLE_SERVO_LIMIT_DECI, "hug look fits servo travel");
STATIC_ASSERT((HUG_REFINE_ZONE_DECI <= HUG_SERVO_MAX_DECI) &&
              (HUG_REFINE_FAR_ZONE_DECI <= HUG_SERVO_MAX_DECI), "refinement fits hug servo travel");

/** Car body: radius about the axle. The car keeps HUG_BODY_GAP_MM between its
 *  body and the face it goes along, so the axle runs HUG_AXLE_OFF_MM off it.
 *  The first approach goes up to that, and every leg after a look leans toward
 *  or away from the face (at most HUG_TRIM_MAX_DECI) to bring it back there.
 *  The ordinary wrap ends a leg this far past a corner. Two close front
 *  endpoints permit a tighter straight diagonal between pivot-safe poses.
 *  An approach shorter than HUG_APPROACH_MIN_MM is not driven. */
#define HUG_BODY_MM           CHASSIS_BODY_RADIUS_MM
#define HUG_SWEEP_RADIUS_MM   (169)
/* Buddy 5's working margin outside our full axle-centred turning envelope. */
#define HUG_BODY_GAP_MM       (50)
#define HUG_AXLE_OFF_MM       (HUG_SWEEP_RADIUS_MM + HUG_BODY_GAP_MM)
#define HUG_TRIM_MAX_DECI     (300)
#define HUG_APPROACH_MIN_MM   (20)
#define HUG_TURN_FWD_MM       (100)

/** Fallback hop angle for a narrow or assumed face without a measured edge. */
#define HUG_HOP_ANGLE_DECI    (250)
#define HUG_LEG_MIN_MM        (60)
#define HUG_LEG_MAX_MM        (1500)

/** The approach zone: the line the car came on, in the frame of the hug (origin
 *  at the hit, x along its heading). The axle is in it up to HUG_EXIT_PAST_MM
 *  along x and HUG_LINE_WINDOW_MM across. Elsewhere an IR contact can start
 *  optical alignment when its projected position clears the neighboring tape. */
#define HUG_LINE_WINDOW_MM    (250)
#define HUG_EXIT_PAST_MM      (150)
/* Lateral positions of neighboring parallel tape, relative to the entry.
 * Allow Buddy 5's tape-position tolerance on either side of each. */
#define HUG_PARALLEL_SPACING_MM (300)
#define HUG_TAPE_TOL_MM         (80)

/** Side choice: edges within this of each other are a tie, a gap beside an
 *  edge narrower than this is blocked, and an OPEN ray counts as this much
 *  open space. With no edge the side with more open space wins when it has
 *  HUG_OPEN_TIE_MM more, summed over the wide rays of the first look; else
 *  the car goes left. */
#define HUG_TIE_MM            (30)
#define HUG_GAP_MM            (340)
#define HUG_OPEN_MM           (1500)
#define HUG_OPEN_TIE_MM       (500)

/** A reading this much farther than the nearest one of a look is behind the
 *  face, not on it. */
#define HUG_BEHIND_MM         (150)

/** A look finds the face again when its normal is within this of the
 *  expected one and its distance within HUG_FACE_TOL_MM of the last; after a
 *  corner the next face may be up to HUG_FACE_TOL_MM beyond the gap. A face
 *  with fewer than HUG_FACE_RAYS_MIN rays is not found. One with fewer than
 *  HUG_NORMAL_RAYS_MIN rays is narrow: its edges are too poor for a width, a
 *  tie between them, or a trim of the leg. */
#define HUG_FACE_ANGLE_DECI   (150)
#define HUG_FACE_RAYS_MIN     (4U)
#define HUG_NORMAL_RAYS_MIN   (7U)
#define HUG_FACE_TOL_MM       (200)
#define HUG_INNER_ANGLE_DECI  (600)

/** Motion. A pivot is done within HUG_HEAD_TOL_DECI; the car then stands
 *  HUG_SETTLE_MS before it looks or drives. */
#define HUG_PIVOT_MIN_DECI_S  (500)
#define HUG_PIVOT_MAX_DECI_S  (1000)
#define HUG_PIVOT_GAIN        (4)
#define HUG_HEAD_TOL_DECI     (20)
#define HUG_SETTLE_MS         (200U)
#define HUG_LEG_MM_S          (100)
#define HUG_SHORT_MM_S        (80)
#define HUG_HOLD_GAIN         (3)
#define HUG_HOLD_MAX_DECI_S   (300)
#define HUG_REPLY_TIMEOUT_MS  (800U)
#define HUG_STATIONARY_LIMIT_MS (2000U)
/* Leave one reply window and time to start the next chassis movement. */
#define HUG_SCAN_BATCH_MS (1800U)

/** A pivot or a drive that runs longer than HUG_MOVE_TIMES times its planned
 *  time ends the hug, and the pilot stops with reason HUG_TIMEOUT. The plan
 *  is the angle at HUG_PIVOT_MIN_DECI_S, of which the body turns
 *  HUG_PIVOT_PCT, or the length at the drive speed, and never less than
 *  HUG_MOVE_FLOOR_MS: the stuck window of the pilot (PILOT_STUCK_MS), so a
 *  move that stalls is backed off by the pilot before it times out, and the
 *  start from rest fits. The clock of a move starts at its first tick, and
 *  again after a tick that another behaviour owns. */
#define HUG_MOVE_TIMES        (3U)
#define HUG_MOVE_FLOOR_MS     (1000U)

/** A hug that began with a stall keeps a note of it. A look from the backed-off
 *  pose that sees no face ends the hug (a release): the block went away. A
 *  second stall while the note stands is the same place: the hug backs off at
 *  once and goes round the face it assumes there. The note clears when the car
 *  has gone HUG_STALL_PAST_MM along the heading of the first hit, from the pose
 *  where the hug began (the pilot's back-off short of the stall), when
 *  HUG_NOTE_MS have passed since the hug ended, and when a hug ends at the
 *  tape. Wheel slip against the face counts as travel and clears it, so the
 *  releases are counted as well: from HUG_RELEASES_MAX of them a stall with no
 *  face seen takes the assumed face, note or not. The count clears with the
 *  note's time, at the tape, and when the sonar starts a hug. All three are
 *  guesses, not measured. */
#define HUG_STALL_PAST_MM     (150)
#define HUG_NOTE_MS           (15000U)
#define HUG_RELEASES_MAX      (2U)

/** This many requests in a row without a reply (a guess) make the sonar dead:
 *  the hug stands with b_stop set, keeps asking, and goes on at the next reply. */
#define HUG_LOST_REPLIES      (2U)

/** Share of a pivot the wheel travel shows up as turning of the body; the
 *  wheels slip. Used for the heading of the pose while the physics pivot
 *  witness reports none, and for the planned time of a pivot. */
#define HUG_PIVOT_PCT         (80)

/** The two sides of a face, as LOOK_SIDE_LEFT and LOOK_SIDE_RIGHT. */
#define HUG_SIDES             (2U)
#define HUG_PIVOT_WITNESS_MASK (127U)

/** What hug_step() needs from one pilot tick. */
typedef struct
{
    int64_t  left_um;
    int64_t  right_um;
    uint32_t now_ms;
    uint16_t ping_mm;
    uint16_t bend_radius_mm; /* wider of requested and observed same-side line arcs, zero otherwise */
    int16_t  ping_servo_deci;
    uint8_t  ping_echo;
    uint8_t  ping_seq;
    bool     b_ping_reply : 1;
    bool     b_line_left : 1;
    bool     b_line_right : 1;
    bool     b_stall : 1;  /* a back-off after a stall has just ended              */
    bool     b_other : 1;  /* another behaviour owns the wheels: track the pose only */
    bool     b_stationary : 1; /* current feedback confirms both wheels have settled */
    bool     b_contact_backoff : 1; /* localized recovery on the same approach */
    bool     b_relook : 1; /* a back-off after a sustained forward range plateau ended */
    uint8_t  pivot_scale_pct : 7; /* physics pivot witness in percent, 0 for none */
    bool     b_independent_hold : 1; /* drive authority or wheel feedback unavailable */
    int16_t  drift_corr_um;   /* incremental wheel-diff correction from gated mag */
} hug_in_t;

STATIC_ASSERT(sizeof(hug_in_t) == 32U, "hug_in_t size");

/** What the hug asks for: one motion command and one ping request. */
typedef struct
{
    int16_t speed_mm_s;
    int16_t turn_deci_s;
    int16_t servo_deci;
    uint8_t seq;       /* 6 bits */
    uint8_t b_req;
    uint8_t b_stop;    /* the sonar is dead: the pilot stands with reason SONAR */
    uint8_t b_timeout; /* a move ran out of time and the hug has ended          */
    bool b_scan_pivot : 1;
    bool b_rejoined : 1;   /* completed and settled on a measured optical crossing */
    bool b_no_exit : 1; /* remaining measured contour exhausted after local search */
    bool b_no_tape : 1;    /* aligned behind the measured box at the entry lateral coordinate */
} hug_out_t;

STATIC_ASSERT(sizeof(hug_out_t) == 12U, "hug_out_t size");

typedef struct
{
    bypass_t bypass;
    int32_t  x_um;          /* axle, look frame; kept idle with b_note   */
    int32_t  y_um;
    int32_t  diff_um;       /* right minus left travel, slip corrected           */
    uint32_t last_left_um;  /* low 32 bits of the last wheel travel              */
    uint32_t last_right_um;
    uint32_t contact_scan_ms; /* start time of the full survey used for a stationary recheck */
    uint32_t last_motion_ms; /* last measured wheel displacement while the hug owns the chassis */
    uint32_t t_ms;          /* request, scan, or movement timer as selected by state */
    union
    {
        struct { int32_t x_mm; int32_t y_mm; } drive;
        struct { uint32_t left_um; uint32_t right_um; } survey;
    } origin; /* Legacy drive/survey anchors or the measured bypass wheel anchors. */
    int16_t  side_end_x_mm; /* observed far vertex on the chosen side, positive when qualified */
    int16_t  side_end_y_mm;
    int32_t  face_line_mm;  /* full fit's projection on its normal, relative to the hit */
    union
    {
        uint32_t idle_tape_ms; /* most recent actual approach-tape sample while idle */
        struct { int32_t x_mm; int32_t y_mm; } point;
        struct { int16_t heading_deci; uint8_t reader; } crossing;
    } tape; /* Observed tape point, then optical capture during the exit pivot. */
    int16_t  goal_head;     /* pivot target, and the heading held in a drive     */
    int16_t  trav_head;     /* heading of the last leg                           */
    int16_t  center;        /* the scan is centred on this heading               */
    int16_t  face_p_mm;     /* axle to face line                                 */
    int16_t  goal_len_mm;   /* drive length, negative backward                   */
    int16_t  u_head;        /* heading along the face, to the chosen side        */
    int16_t  leg_mm;
    int16_t  app_mm;
    int16_t  keep_x_mm[HUG_SIDES];  /* edges of the near look, kept while the finer look runs */
    int16_t  keep_y_mm[HUG_SIDES];
    int16_t  keep_normal;
    int16_t  keep_dist_mm;
    int16_t  look_head;     /* the look after the leg is centred on this         */
    uint16_t open_mm[HUG_SIDES];    /* sum of the readings on each side, LOOK_SIDE_*     */
    uint16_t wall_mm[HUG_SIDES];    /* nearest side reading from the face axis, 0 none   */
    uint16_t width_mm;      /* front face from both edges, 0 unknown             */
    uint16_t breadth_mm;    /* the next face along the hug, 0 unknown            */
    uint8_t  state;
    uint8_t  sub;
    uint8_t  then;
    uint8_t  purpose;
    uint8_t  idx;           /* ray of the scan                                   */
    uint8_t  body;          /* body step of the ray within the current look      */
    uint8_t  hist;          /* last three forward pings, one bit each            */
    uint8_t  last_seq;      /* of the last forward ping seen                     */
    uint8_t  req_seq;
    int8_t   side;          /* +1 goes left, -1 goes right                       */
    uint8_t  lost;          /* requests in a row that got no reply               */
    uint8_t  recheck_hits;  /* fresh near returns, or a completed inconclusive survey */
    uint16_t flags;
    int16_t  seen_mm;       /* axle to face line as measured, for the frame      */
    bool b_note : 1;        /* last hug began with a stall; HUG_STALL_PAST_MM */
    bool b_approach_tape : 1;
    bool b_require_rear : 1;
    bool b_can_resume : 1;
    bool b_remaining : 1;
    bool b_oblique : 1; /* fitted successor joined the last edge at an oblique angle */
    bool b_survey_displaced : 1; /* another owner interrupted an off-line vertex survey */
    bool b_close_front : 1; /* guarded diagonal past two close front endpoints */
    int8_t scan_lead_deci; /* chassis offset from the remaining bearings of a slow sweep */
    uint8_t  n_rel;         /* releases since the last tape or HUG_NOTE_MS    */
    bool b_bend_check : 1;  /* a forward return outside a traced bend is being rechecked */
    bool b_probe_front : 1; /* stepping sideways to observe a sparsely sampled contact */
    bool b_probe_retried : 1; /* repeated the look from the displaced axle */
    bool b_left_scan_gap : 1;
    bool b_right_scan_gap : 1;
    bool b_pivot_reaimed : 1; /* one post-settlement correction before replanning */
    uint32_t bend_start_ms;
    uint32_t bend_outer_um; /* outer wheel at the first such return, low 32 bits */
} hug_t;

STATIC_ASSERT(sizeof(hug_t) == 200U, "hug_t size");

/** Idle, no memory of a face. */
void hug_init(hug_t * p_hug);

/** After a complete empty IR sweep, resume the remaining perimeter once if
 *  actual approach tape was recorded and the localized car is back in pose. */
bool hug_resume_remaining(hug_t * p_hug, int64_t left_um, int64_t right_um, uint32_t now_ms);

/** Invalidate any previous run's eligibility for the alternate exit. */
void hug_forget_approach(hug_t * p_hug);

/** True while the hug has taken the wheels. */
bool hug_active(hug_t const * p_hug);

/** True while the follower is returning to a locally noted stall where the
 *  hug found no face. */
bool hug_reapproaching(hug_t const * p_hug);

/** Body heading from the heading at the hit, deci-degrees, left positive. */
int16_t hug_heading_deci(hug_t const * p_hug);

/** Body turn of a pivot's wheel travel difference: the physics witness share
 *  when one is reported (1 to 100 percent), else HUG_PIVOT_PCT. */
int32_t hug_pivot_turn_um(int32_t diff_um, uint8_t witness_pct);

/**
 * One hug tick.
 *
 * @param[in,out] p_hug  State.
 * @param[in,out] p_look The look: written during a scan, read for its face.
 * @param[in]     p_in   The tick.
 * @param[out]    p_out  The command and the ping request; zero when the hug
 *                       is idle or the tick is not its own.
 * @return true when the hug has taken control and filled p_out.
 */
bool hug_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out);

#endif /* PILOT_HUG_H */
