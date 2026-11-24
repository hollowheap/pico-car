/** @file hug.c
 * @brief Hug: go round an obstacle on the line, keeping it at one side.
 *
 * A small state machine. Motions (pivot, drive) and scans chain by a "then"
 * code that says what to do when the motion has ended and the car stood
 * still for a moment. The geometry of a face comes from look.c; this file
 * decides the side, the length of the legs and when to give the wheels back.
 */

#include "pilot/hug.h"

#include <stddef.h>

#include "hug_internal.h"
#include "geometry.h"
#include "obstacle_config.h"
#include "platform/chassis.h"
#include "platform/units.h"
#include "track/line_control.h"

#define PERCENT         (100)
#define HALVES          (2)
#define EXIT_CLEAR      (0U)
#define EXIT_LEFT       (1U)
#define EXIT_RIGHT      (2U)
#define EXIT_WAIT_CLEAR (EXIT_LEFT | EXIT_RIGHT)
#define QUARTER_DECI    (900)
/* One perimeter plus the observation turn towards a face. */
#define REMAINING_LIMIT_DECI  (3600 + QUARTER_DECI)
#define ODD_SEQ         (0xFFU)
#define SEQ_MASK        (0x3FU)
#define HIST_BITS       (3U)
#define HIST_MASK       (0x07U)
#define HOP_LIMIT_MM    (400)
#define RAY_NEAR_N      (((2U * HUG_NEAR_ZONE_DECI) / HUG_NEAR_STEP_DECI) + 1U)
#define SIDE_BODY_DECI  (HUG_NEAR_ZONE_DECI - OBSTACLE_SERVO_LIMIT_DECI)
#define RAY_CONTACT_N   ((uint32_t)(sizeof(g_contact_deci) / sizeof(g_contact_deci[0])))
#define PROBE_RAYS      (3U)
#define PROBE_STEP_DECI (150)
#define CENTER_STEP_DECI (20)
#define CENTER_RAYS      (((2U * OBSTACLE_SERVO_LIMIT_DECI) / CENTER_STEP_DECI) + 1U)
#define HUG_LANE_BUFFER_MM (30)
#define HUG_FLANK_GAP_MM (40)
#define HUG_FLANK_DECI (250)
#define HUG_SCAN_CLEARANCE_MM (HUG_SWEEP_RADIUS_MM + (int32_t)OBSTACLE_PLAN_MARGIN_MM)
#define FACE_SPREAD_MM  (15)
/* Reuse a completed contact look only across a brief handback with both
 * wheels still within this distance of their surveyed position. */
#define RECHECK_WHEEL_UM   (10000U)
#define RECHECK_READY      (UINT8_MAX)
#define RECHECK_RAYS       (((2U * OBSTACLE_SERVO_LIMIT_DECI) / HUG_NEAR_STEP_DECI) + 1U)
#define RAY_REFINE_N    (((2U * HUG_REFINE_ZONE_DECI) / HUG_REFINE_STEP_DECI) + 1U)
#define RAY_REFINE_FAR_N (((2U * HUG_REFINE_FAR_ZONE_DECI) / HUG_REFINE_FAR_STEP_DECI) + 1U)
#define BODY_BIAS       (10)
#define REFINE_AIMS     (3U)
#define BEAM_PASSES     (3U)
#define ROUND_HALF      (HUG_PIVOT_STEP_DECI / HALVES)
#define MS_PER_S        (1000U)

/* States. */
#define ST_IDLE         (0U)
#define ST_SCAN         (1U)
#define ST_PIVOT        (2U)
#define ST_DRIVE        (3U)

/* Sub-states. A pivot or a drive turns or drives, then settles. A scan
 * pivots to its body step, settles, then asks for a ping. A move is set up in
 * START, and its clock starts at its first tick, in MOVE. */
#define SUB_MOVE        (0U)
#define SUB_SETTLE      (1U)
#define SUB_ASK         (2U)
#define SUB_START       (3U)

/* What follows a motion or a scan. The wire state reads THEN_BACK as the
 * back-off (hug_word.h). */
#define THEN_SCAN       (0U)
#define THEN_BACK       (1U)
#define THEN_APPROACH   (2U)
#define THEN_ALIGN      (3U)
#define THEN_LEG        (4U)
#define THEN_LOOK       (5U)
#define THEN_TURN       (6U)
#define THEN_EXIT_TURN  (7U)
#define THEN_RELEASE    (8U)
#define THEN_ADVANCE    (9U)
#define THEN_REAR_LATERAL (10U)
#define THEN_REAR_ALIGN   (11U)
#define THEN_REAR_READY   (12U)
#define THEN_FLANK_LOOK   (13U)

/* Why the car looks. */
#define PU_CONTACT      (0U)
#define PU_LOOK         (1U)
#define PU_INNER        (2U)
#define PU_REFINE       (3U)
#define PU_CORNER       (4U)
#define PU_CHECK        (5U)
#define PU_CENTER       (6U)
#define PU_CORNER_RETRY (7U)
#define PU_EDGE_LOOK    (8U)
#define PU_EDGE_RETRY   (9U)
#define PU_MID_LOOK     (10U)
#define PU_FACET_CLEAR    (11U)
#define PU_FACET_RETRY    (12U)

#define F_PAST          (0x01U) /* the leg ended the body and the gap past the corner     */
#define F_EXIT_TAPE     (0x02U) /* a sensor located tape outside the approach line */
#define F_EXIT          (0x04U) /* the tape ended the hug: the turn out stops at the tape */
#define F_TURNED        (0x08U) /* the last motion was the turn round a corner    */
#define F_REFINE        (0x10U) /* the finer looks of a face whose edges are known */
#define F_WIN2          (0x20U) /* the second finer look adds to the first        */
#define F_STALL         (0x40U) /* the hit or the leg ended in a wheel stall      */
#define F_ASSUME        (0x80U) /* the face in use is the one assumed at a stall  */
#define F_BACKED        (0x100U) /* the first contact has backed off to the standoff */
#define F_REPEAT        (0x200U) /* a stall at the place of the last one, which may be a wall */
#define F_RECHECK       (0x400U) /* stationary near-sector check against the last contact survey */
#define F_AIMED         (0x800U) /* the pivot before the look has stopped at its first body step and settled */
#define F_REAPPROACH    (0x1000U) /* the last contact look released after an unseen stall */
#define F_EDGE_LEG      (0x2000U) /* the current leg ends beyond a measured edge */
#define F_COARSE_NARROW (0x4000U) /* width confidence of the saved coarse fit */
#define F_MEASURED_FACE (0x8000U) /* a full fit supplies the face line */

#define LEFT            ((uint8_t)LOOK_SIDE_LEFT)
#define RIGHT           ((uint8_t)LOOK_SIDE_RIGHT)

/* The longer end of the chassis determines the swept diagonal. */
#define HUG_LONG_END_MM ((CHASSIS_FRONT_TIP_MM > CHASSIS_AXLE_TO_CASTER_MM) ? \
                          CHASSIS_FRONT_TIP_MM : CHASSIS_AXLE_TO_CASTER_MM)
#define HUG_SWEEP_SQ_MM (HUG_LONG_END_MM * HUG_LONG_END_MM + CHASSIS_LEFT_REACH_MM * CHASSIS_LEFT_REACH_MM)
#define BEND_FRONT_RESERVE_MM (HUG_LONG_END_MM - LOOK_SONAR_FWD_MM + HUG_FORWARD_RESERVE_MM)
#define BEND_OUTER_TRAVEL_UM ((uint32_t)(HUG_FORWARD_RESERVE_MM / HALVES) * UM_PER_MM)
STATIC_ASSERT(((HUG_SWEEP_RADIUS_MM - 1) * (HUG_SWEEP_RADIUS_MM - 1) < HUG_SWEEP_SQ_MM) &&
              (HUG_SWEEP_RADIUS_MM * HUG_SWEEP_RADIUS_MM >= HUG_SWEEP_SQ_MM),
              "turn radius includes sensors and caster");
STATIC_ASSERT(RAY_REFINE_N == RAY_REFINE_FAR_N, "both refine windows hold the same rays");
STATIC_ASSERT((2U * RAY_REFINE_N) <= LOOK_RAYS_MAX, "two refine windows fit the look");
STATIC_ASSERT(RAY_NEAR_N <= LOOK_RAYS_MAX, "a look fits the look");
STATIC_ASSERT(CENTER_RAYS <= LOOK_RAYS_MAX, "the central servo sweep fits the look");
STATIC_ASSERT((SIDE_BODY_DECI > 0) && (SIDE_BODY_DECI <= OBSTACLE_SERVO_LIMIT_DECI),
              "two side-look headings cover the sweep");
STATIC_ASSERT(PROBE_STEP_DECI <= OBSTACLE_SERVO_LIMIT_DECI, "probe rays fit servo travel");

static int16_t const g_contact_deci[] = {HUG_CONTACT_RAYS};

STATIC_ASSERT(RAY_CONTACT_N <= LOOK_RAYS_MAX, "the contact look fits the look");

typedef struct
{
    look_edge_t edge[HUG_SIDES];
    bool        have[HUG_SIDES];
    int32_t     lat_mm[HUG_SIDES]; /* across the heading the car looked along, from the car, absolute */
    bool        b_narrow;          /* too few rays for the normal and the edges of the face           */
} hug_edges_t;

/* Encoders only: no gyro; the uncalibrated compass by the motors is worse. */
int32_t hug_pose_heading_deci(hug_t const * p_hug)
{
    return geom_wrap_deci((int32_t)wheel_yaw_deci(p_hug->diff_um));
}

static int32_t pos_x_mm(hug_t const * p_hug)
{
    return p_hug->x_um / UM_PER_MM;
}

static int32_t pos_y_mm(hug_t const * p_hug)
{
    return p_hug->y_um / UM_PER_MM;
}

look_pose_t hug_axle_pose(hug_t const * p_hug)
{
    return (look_pose_t){pos_x_mm(p_hug), pos_y_mm(p_hug), hug_pose_heading_deci(p_hug)};
}

static bool flag_get(hug_t const * p_hug, uint32_t mask)
{
    return 0U != ((uint32_t)p_hug->flags & mask);
}

static void flag_set(hug_t * p_hug, uint32_t mask, bool b_on)
{
    uint32_t flags = p_hug->flags;

    flags = b_on ? (flags | mask) : (flags & ~mask);
    p_hug->flags = (uint16_t)flags;
}

static void zero_out(hug_out_t * p_out)
{
    p_out->speed_mm_s = 0;
    p_out->turn_deci_s = 0;
    p_out->servo_deci = 0;
    p_out->seq = 0U;
    p_out->b_req = 0U;
    p_out->b_stop = 0U;
    p_out->b_timeout = 0U;
    p_out->b_scan_pivot = false;
    p_out->b_no_tape = false;
    p_out->b_no_exit = false;
    p_out->b_rejoined = false;
}

/* Wheel travel to pose. Wheels that run opposite turn the body less than
 * their travel says. */
static void odo_update(hug_t * p_hug, hug_in_t const * p_in)
{
    int32_t const dl = (int32_t)((uint32_t)p_in->left_um - p_hug->last_left_um);
    int32_t const dr = (int32_t)((uint32_t)p_in->right_um - p_hug->last_right_um);
    int32_t const ds = (dl + dr) / HALVES;
    int32_t dd = dr - dl;
    int32_t const before = hug_pose_heading_deci(p_hug);
    int32_t mid = 0;

    if ((0 != dl) || (0 != dr) || p_in->b_other)
    {
        p_hug->last_motion_ms = p_in->now_ms;
    }
    p_hug->last_left_um = (uint32_t)p_in->left_um;
    p_hug->last_right_um = (uint32_t)p_in->right_um;
    if ((BYPASS_IDLE == p_hug->bypass.phase) && (((dl > 0) && (dr < 0)) || ((dl < 0) && (dr > 0))))
    {
        dd = (dd * HUG_PIVOT_PCT) / PERCENT;
    }
    p_hug->diff_um += dd;
    mid = before + (geom_wrap_deci(hug_pose_heading_deci(p_hug) - before) / HALVES);
    p_hug->x_um += (int32_t)(((int64_t)ds * geom_cos_q14(mid)) / GEOM_Q14_ONE);
    p_hug->y_um += (int32_t)(((int64_t)ds * geom_sin_q14(mid)) / GEOM_Q14_ONE);
}

/* After a stall hug the pose runs while the hug is idle: x is the travel along
 * the heading of the first hit, from the pose where the hug began, which is
 * the pilot's back-off short of the stall. The note clears at HUG_STALL_PAST_MM
 * and with time. Wheel slip against the face counts as travel. */
static void note_update(hug_t * p_hug, hug_in_t const * p_in)
{
    odo_update(p_hug, p_in);
    if (pos_x_mm(p_hug) >= HUG_STALL_PAST_MM)
    {
        p_hug->b_note = 0U;
    }
    if ((p_in->now_ms - p_hug->t_ms) > HUG_NOTE_MS)
    {
        p_hug->b_note = 0U;
        p_hug->n_rel = 0U;
    }
}

static bool forward_ping(hug_t * p_hug, hug_in_t const * p_in, bool * p_b_guard)
{
    bool const b_new = !p_in->b_ping_reply && (p_in->ping_seq != p_hug->last_seq);
    bool b_hit = false;

    *p_b_guard = false;
    if (b_new)
    {
        bool const b_valid = (LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > 0U);
        bool const b_too_close = (LOOK_ECHO_TOO_CLOSE == p_in->ping_echo);

        if (p_in->ping_seq != (((uint32_t)p_hug->last_seq + 1U) & SEQ_MASK))
        {
            p_hug->b_bend_check = false;
        }
        p_hug->last_seq = p_in->ping_seq;
        b_hit = b_too_close || (b_valid && (p_in->ping_mm <= (uint16_t)HUG_STANDOFF_MM));
        *p_b_guard = b_too_close || (b_valid && (p_in->ping_mm <= (uint16_t)HUG_GUARD_MM));
        p_hug->hist = (uint8_t)(((uint32_t)p_hug->hist << 1U) | (b_hit ? 1U : 0U));
        p_hug->hist = (uint8_t)((uint32_t)p_hug->hist & HIST_MASK);
    }
    return b_new;
}

static bool triggered(hug_t const * p_hug)
{
    uint32_t hits = 0U;

    for (uint32_t i = 0U; i < HIST_BITS; i++)
    {
        hits += ((uint32_t)p_hug->hist >> i) & 1U;
    }
    return hits >= HUG_TRIGGER_HITS;
}

/* Relative to the instantaneous turn centre, every body corner travels
 * within the circle through the outermost corner. Compare the nearest
 * possible echo in the beam to that circle, including tracking margin. */
static bool echo_outside_bend(uint16_t radius_mm, uint16_t range_mm, bool b_left_turn)
{
    int32_t const radius = radius_mm;
    int32_t const margin = (int32_t)OBSTACLE_PLAN_MARGIN_MM;
    int32_t const outer = radius + (int32_t)(b_left_turn ? CHASSIS_RIGHT_REACH_MM : CHASSIS_LEFT_REACH_MM) + margin;
    int32_t const end = HUG_LONG_END_MM + margin;
    int32_t const ahead = LOOK_SONAR_FWD_MM + geom_mul_q14((int32_t)range_mm - (int32_t)OBSTACLE_AGREE_MM,
                                                           geom_cos_q14(CHASSIS_SONAR_OUTER_HALF_DECI));
    int32_t const across = radius - geom_mul_q14((int32_t)range_mm + (int32_t)OBSTACLE_AGREE_MM,
                                                 geom_sin_q14(CHASSIS_SONAR_OUTER_HALF_DECI)) - 1;
    int32_t const nearest = (across > 0) ? across : 0;
    /* Difference of squares keeps even a nearly straight radius in int32. */
    return (ahead * ahead - end * end) > ((outer - nearest) * (outer + nearest));
}

static bool recheck_on_bend(hug_t * p_hug, hug_in_t const * p_in, bool b_forward)
{
    bool const b_center = !p_in->b_ping_reply && (0 == p_in->ping_servo_deci);
    bool const b_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
        ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > HUG_STANDOFF_MM));
    if (p_in->b_other || !b_center || (0U == p_in->bend_radius_mm))
    {
        p_hug->b_bend_check = false;
        return false;
    }
    if (p_hug->b_bend_check && b_clear)
    {
        /* The first clear ping can still have two old hits behind it. Two
         * consecutive clears end this observation window. */
        if (b_forward && ((p_hug->hist & (HIST_MASK >> 1U)) == 0U))
        {
            p_hug->b_bend_check = false;
        }
        return true;
    }
    if ((LOOK_ECHO_VALID != p_in->ping_echo) || (p_in->ping_mm <= BEND_FRONT_RESERVE_MM) ||
        (p_in->ping_mm > HUG_STANDOFF_MM) || !echo_outside_bend(p_in->bend_radius_mm, p_in->ping_mm, p_in->b_line_left))
    {
        p_hug->b_bend_check = false;
        return false;
    }
    uint32_t const outer = (uint32_t)(p_in->b_line_left ? p_in->right_um : p_in->left_um);
    if (!p_hug->b_bend_check && b_forward && (1U == p_hug->hist))
    {
        p_hug->bend_outer_um = outer;
        p_hug->bend_start_ms = p_in->now_ms;
        p_hug->b_bend_check = true;
    }
    /* A point return supplies no obstacle width. A surface that stays in
     * view as the wheels turn must get the ordinary stationary survey. */
    return p_hug->b_bend_check && (outer - p_hug->bend_outer_um < BEND_OUTER_TRAVEL_UM) &&
           (p_in->now_ms - p_hug->bend_start_ms < HUG_REPLY_TIMEOUT_MS);
}

void hug_init(hug_t * p_hug)
{
    uint8_t * const p_bytes = (uint8_t *)p_hug;

    for (size_t i = 0U; i < sizeof(*p_hug); i++)
    {
        p_bytes[i] = 0U;
    }
    p_hug->last_seq = (uint8_t)ODD_SEQ;
    p_hug->side = 1;
}

void hug_forget_approach(hug_t * p_hug)
{
    p_hug->b_bend_check = false;
    p_hug->b_approach_tape = false;
    p_hug->b_can_resume = false;
}

bool hug_active(hug_t const * p_hug)
{
    return ST_IDLE != p_hug->state;
}

bool hug_reapproaching(hug_t const * p_hug)
{
    return (ST_IDLE == p_hug->state) && (0U != p_hug->b_note) &&
           flag_get(p_hug, F_REAPPROACH);
}

int16_t hug_heading_deci(hug_t const * p_hug)
{
    /* Cast is safe: the heading is wrapped to -1800..1800. */
    return (ST_IDLE == p_hug->state) ? (int16_t)0 : (int16_t)hug_pose_heading_deci(p_hug);
}

/* ---- motions ---- */

static void start_pivot(hug_t * p_hug, int32_t head, uint32_t then)
{
    p_hug->b_pivot_reaimed = false;
    p_hug->state = (uint8_t)ST_PIVOT;
    p_hug->sub = (uint8_t)SUB_START;
    p_hug->goal_head = (int16_t)head;
    p_hug->then = (uint8_t)then;
}

static void start_drive(hug_t * p_hug, int32_t len_mm, int32_t head, uint32_t then)
{
    p_hug->state = (uint8_t)ST_DRIVE;
    p_hug->sub = (uint8_t)SUB_START;
    p_hug->goal_head = (int16_t)head;
    p_hug->goal_len_mm = (int16_t)len_mm;
    p_hug->origin.drive.x_mm = pos_x_mm(p_hug);
    p_hug->origin.drive.y_mm = pos_y_mm(p_hug);
    p_hug->then = (uint8_t)then;
}

/* The first contact nearer than the gap, or a stall: back straight off along
 * the heading of the contact to the standoff from the face, then look again.
 * The contact look has turned the car away from that heading, so it turns
 * back first (proceed starts the drive, which keeps its length here). The
 * range is from the sonar, as HUG_STANDOFF_MM is. */
static void start_back(hug_t * p_hug, int32_t range_mm)
{
    flag_set(p_hug, F_BACKED, true);
    p_hug->goal_len_mm = (int16_t)(range_mm - HUG_STANDOFF_MM);
    start_pivot(p_hug, p_hug->center, THEN_BACK);
}

/* The kind of a drive, from what follows it. A leg goes along a face and a
 * look follows it: the tape past the hit ends it, a face ahead under the
 * guard cuts it short, and it runs at HUG_LEG_MM_S. The other drives (the
 * back-off step, the approach and the run onto the tape) end at their length
 * only, at HUG_SHORT_MM_S. */
static bool drive_is_leg(hug_t const * p_hug)
{
    return THEN_LOOK == p_hug->then;
}

static void start_scan(hug_t * p_hug, look_t * p_look, uint32_t purpose, int32_t center)
{
    p_hug->state = (uint8_t)ST_SCAN;
    p_hug->purpose = (uint8_t)purpose;
    p_hug->center = (int16_t)center;
    p_hug->idx = 0U;
    p_hug->body = 0U;
    p_hug->scan_lead_deci = 0;
    if (PU_CONTACT == purpose)
    {
        p_hug->b_left_scan_gap = false;
        p_hug->b_right_scan_gap = false;
    }
    if ((PU_CORNER != purpose) && (PU_CORNER_RETRY != purpose))
    {
        p_hug->b_close_front = false;
    }
    p_hug->sub = (uint8_t)SUB_START;
    if (!flag_get(p_hug, F_WIN2 | F_RECHECK))
    {
        look_clear(p_look);
    }
    if ((PU_REFINE != purpose) && (PU_CENTER != purpose))
    {
        p_hug->open_mm[LEFT] = 0U;
        p_hug->open_mm[RIGHT] = 0U;
        p_hug->wall_mm[LEFT] = 0U;
        p_hug->wall_mm[RIGHT] = 0U;
    }
}

/* Turn in place toward the target. True when within the tolerance. */
static bool pivot_cmd(hug_t const * p_hug, int32_t target, hug_out_t * p_out)
{
    int32_t const err = geom_wrap_deci(target - hug_pose_heading_deci(p_hug));
    bool const b_done = geom_abs(err) <= HUG_HEAD_TOL_DECI;

    if (!b_done)
    {
        int32_t const rate = clamp_i32(geom_abs(err) * HUG_PIVOT_GAIN,
                                       HUG_PIVOT_MIN_DECI_S, HUG_PIVOT_MAX_DECI_S);

        p_out->turn_deci_s = (int16_t)((err > 0) ? rate : -rate);
    }
    return b_done;
}

static int32_t drive_remaining_mm(hug_t const * p_hug)
{
    int32_t const dx = pos_x_mm(p_hug) - p_hug->origin.drive.x_mm;
    int32_t const dy = pos_y_mm(p_hug) - p_hug->origin.drive.y_mm;
    int32_t const travelled = (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy)));
    int32_t const remaining = geom_abs(p_hug->goal_len_mm) - travelled;
    return (remaining > 0) ? remaining : 0;
}

static bool drive_cmd(hug_t const * p_hug, int32_t speed_mm_s, hug_out_t * p_out)
{
    bool const b_done = (0 == drive_remaining_mm(p_hug));

    if (!b_done)
    {
        int32_t const err = geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug));

        p_out->speed_mm_s = (int16_t)((p_hug->goal_len_mm < 0) ? -speed_mm_s : speed_mm_s);
        p_out->turn_deci_s = (int16_t)clamp_i32(err * HUG_HOLD_GAIN, -HUG_HOLD_MAX_DECI_S,
                                                HUG_HOLD_MAX_DECI_S);
    }
    return b_done;
}

/* ---- scan ---- */

static bool facet_clearance(hug_t const * p_hug)
{
    return (PU_FACET_CLEAR == p_hug->purpose) || (PU_FACET_RETRY == p_hug->purpose);
}

static bool corner_scan(hug_t const * p_hug)
{
    return (PU_CORNER == p_hug->purpose) || (PU_CORNER_RETRY == p_hug->purpose) || facet_clearance(p_hug);
}

static bool probe_scan(hug_t const * p_hug)
{
    return corner_scan(p_hug) || (PU_CHECK == p_hug->purpose);
}

static bool edge_sweep(hug_t const * p_hug)
{
    return (PU_EDGE_LOOK == p_hug->purpose) || (PU_EDGE_RETRY == p_hug->purpose) ||
           (PU_MID_LOOK == p_hug->purpose);
}

static bool side_sweep(hug_t const * p_hug)
{
    return (PU_LOOK == p_hug->purpose) || edge_sweep(p_hug);
}

static uint32_t ray_count(hug_t const * p_hug)
{
    uint32_t n = RAY_NEAR_N;

    if (PU_CONTACT == p_hug->purpose)
    {
        n = flag_get(p_hug, F_RECHECK) ? RECHECK_RAYS : RAY_CONTACT_N;
    }
    else if (PU_CENTER == p_hug->purpose)
    {
        n = CENTER_RAYS;
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        n = RAY_REFINE_N;
    }
    else if (probe_scan(p_hug))
    {
        n = PROBE_RAYS;
    }
    else
    {
        /* A small look. */
    }
    return n;
}

/* Offset of ray i from the centre of the scan, deci-degrees: ascending, but a
 * look that goes right sweeps down. */
static int32_t ray_offset(hug_t const * p_hug, uint32_t i)
{
    int32_t offset = 0;

    if (PU_CENTER == p_hug->purpose)
    {
        offset = ((int32_t)i * CENTER_STEP_DECI) - OBSTACLE_SERVO_LIMIT_DECI;
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        bool const b_far = (p_hug->keep_dist_mm - LOOK_SONAR_FWD_MM) > HUG_FAR_MM;
        int32_t const zone = b_far ? HUG_REFINE_FAR_ZONE_DECI : HUG_REFINE_ZONE_DECI;

        offset = ((int32_t)i * ((HALVES * zone) / ((int32_t)RAY_REFINE_N - 1))) - zone;
    }
    else if (PU_CHECK == p_hug->purpose)
    {
        /* The lead ray tests whether the measured face still extends ahead. */
        offset = p_hug->side * (1 - (int32_t)i) * PROBE_STEP_DECI;
    }
    else if (corner_scan(p_hug))
    {
        offset = (((int32_t)i - 1) * PROBE_STEP_DECI);
    }
    else if (PU_CONTACT != p_hug->purpose)
    {
        offset = ((int32_t)i * HUG_NEAR_STEP_DECI) - HUG_NEAR_ZONE_DECI;
        /* The sweep ends on the side the next leg goes to, which is the
         * shorter turn to it. */
        offset = (side_sweep(p_hug) && (p_hug->side < 0)) ? -offset : offset;
    }
    else
    {
        offset = flag_get(p_hug, F_RECHECK)
                     ? (((int32_t)i * HUG_NEAR_STEP_DECI) - OBSTACLE_SERVO_LIMIT_DECI)
                     : g_contact_deci[i];
    }
    return offset;
}

static uint8_t body_step_of(int32_t offset)
{
    int32_t const q = (offset + HUG_SERVO_MAX_DECI + (BODY_BIAS * HUG_PIVOT_STEP_DECI))
                      / HUG_PIVOT_STEP_DECI;

    return (uint8_t)q;
}

/* The body heading for a ray, stored as a step number plus a bias. */
static uint8_t body_step(hug_t const * p_hug, int32_t offset)
{
    if ((PU_REFINE == p_hug->purpose) || (PU_CENTER == p_hug->purpose) ||
        probe_scan(p_hug) || flag_get(p_hug, F_RECHECK))
    {
        /* The servo alone covers the window: every ray from one place. */
        return (uint8_t)BODY_BIAS;
    }
    if (side_sweep(p_hug))
    {
        /* Servo travel covers each half of the side sweep from one pose. */
        return (uint8_t)((offset > 0) ? (BODY_BIAS + 1) : (BODY_BIAS - 1));
    }
    return body_step_of(offset);
}

static int32_t body_deci(hug_t const * p_hug, uint8_t step)
{
    int32_t const step_deci = side_sweep(p_hug) ? SIDE_BODY_DECI : HUG_PIVOT_STEP_DECI;

    return ((int32_t)step - BODY_BIAS) * step_deci;
}

static int32_t scan_body_aim(hug_t const * p_hug, int32_t offset)
{
    return (0 != p_hug->scan_lead_deci) ? offset + p_hug->scan_lead_deci
                                        : body_deci(p_hug, body_step(p_hug, offset));
}

static void scan_record(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                        int32_t offset)
{
    /* A ping carries no time: the reply pairs with the pose when it arrives,
     * which is right because the look runs only while the car stands still. */
    look_pose_t const pose = hug_axle_pose(p_hug);
    bool const b_usable = look_add(p_look, &pose, p_in->ping_servo_deci, p_in->ping_echo,
                                   (int32_t)p_in->ping_mm);

    if (!b_usable && (PU_CONTACT == p_hug->purpose))
    {
        /* The center bearing belongs to both possible departure sectors. */
        p_hug->b_left_scan_gap = p_hug->b_left_scan_gap || (offset >= 0);
        p_hug->b_right_scan_gap = p_hug->b_right_scan_gap || (offset <= 0);
    }
    if (b_usable && (PU_CONTACT == p_hug->purpose) && (geom_abs(offset) > HUG_CONTACT_NEAR_DECI))
    {
        bool const b_point = look_echo_is_point(p_in->ping_echo);
        int32_t const reach = b_point ? clamp_i32((int32_t)p_in->ping_mm, 0, HUG_OPEN_MM)
                                     : HUG_OPEN_MM;
        uint8_t const side = (offset > 0) ? LEFT : RIGHT;

        p_hug->open_mm[side] = (uint16_t)((uint32_t)p_hug->open_mm[side] + (uint32_t)reach);
        if (b_point)
        {
            int32_t const lat = geom_abs(geom_mul_q14((int32_t)p_in->ping_mm,
                                                      geom_sin_q14(offset)));

            if ((0U == p_hug->wall_mm[side]) || (lat < (int32_t)p_hug->wall_mm[side]))
            {
                p_hug->wall_mm[side] = (uint16_t)lat;
            }
        }
    }
}

static void emit_request(hug_t const * p_hug, hug_out_t * p_out, int32_t servo)
{
    p_out->servo_deci = (int16_t)servo;
    p_out->seq = p_hug->req_seq;
    p_out->b_req = 1U;
}

/* After a scan the car has the look, and the next move is planned from it. */
static void after_scan(hug_t * p_hug, look_t * p_look, hug_out_t * p_out);
static void look_along_face(hug_t * p_hug, uint8_t purpose);
static void finish(hug_t * p_hug);
static void end_stationary_batch(hug_t * p_hug, look_t * p_look);

static bool prior_near_return(look_t const * p_look, int32_t bearing, int32_t range_mm)
{
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];

        if ((LOOK_ECHO_VALID == look_ray_echo(p_ray)) &&
            (geom_abs(look_ray_angle_deci(p_ray) - bearing) <= LOOK_BEAM_DECI) &&
            (geom_abs((int32_t)p_ray->range_mm - range_mm) <= LOOK_FACE_MATCH_MM))
        {
            return true;
        }
    }
    return false;
}

static bool recheck_ray(hug_t * p_hug, look_t const * p_look, hug_in_t const * p_in,
                        int32_t bearing)
{
    if (LOOK_ECHO_OPEN == p_in->ping_echo)
    {
        return true;
    }
    if ((LOOK_ECHO_VALID != p_in->ping_echo) || (0U == p_in->ping_mm))
    {
        return false;
    }
    if (p_in->ping_mm > HUG_GUARD_MM)
    {
        return true;
    }
    if (p_in->ping_mm <= HUG_STANDOFF_MM)
    {
        p_hug->recheck_hits++;
    }
    return (p_hug->recheck_hits < HUG_FACE_RAYS_MIN) &&
           prior_near_return(p_look, bearing, p_in->ping_mm);
}

static void restart_contact(hug_t * p_hug, look_t * p_look)
{
    flag_set(p_hug, F_RECHECK, false);
    p_hug->recheck_hits = 0U;
    start_scan(p_hug, p_look, PU_CONTACT, 0);
}

static void accept_scan_ray(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                            int32_t offset, bool b_recheck, hug_out_t * p_out)
{
    p_hug->lost = 0U;
    if (!b_recheck)
    {
        scan_record(p_hug, p_look, p_in, offset);
    }
    p_hug->idx++;
    p_hug->req_seq = (uint8_t)(((uint32_t)p_hug->req_seq + 1U) & SEQ_MASK);
    if (p_hug->idx >= ray_count(p_hug))
    {
        if (b_recheck)
        {
            start_pivot(p_hug, 0, THEN_RELEASE);
        }
        else
        {
            after_scan(p_hug, p_look, p_out);
        }
    }
    else if ((0 != p_hug->scan_lead_deci) || (body_step(p_hug, ray_offset(p_hug, p_hug->idx)) != p_hug->body))
    {
        p_hug->sub = (uint8_t)SUB_START;
    }
    else
    {
        p_hug->t_ms = p_in->now_ms;
        bool const b_evidence = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
            (look_echo_is_point(p_in->ping_echo) && (0U != p_in->ping_mm));
        if (b_evidence && (p_in->now_ms - p_hug->last_motion_ms >= HUG_SCAN_BATCH_MS))
        {
            end_stationary_batch(p_hug, p_look);
        }
    }
}

static void scan_reply(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       int32_t offset, hug_out_t * p_out)
{
    int32_t const servo_deci = offset - scan_body_aim(p_hug, offset);
    bool const b_recheck = flag_get(p_hug, F_RECHECK);
    bool const b_expired = (p_in->now_ms - p_hug->t_ms) > HUG_REPLY_TIMEOUT_MS;
    bool const b_short = (PU_CHECK == p_hug->purpose);
    bool const b_match = !b_expired && obstacle_ping_reply_matches(
        p_in->b_ping_reply, p_in->ping_seq, p_in->ping_servo_deci,
        p_hug->req_seq, servo_deci, OBSTACLE_PING_MAX_QUANTIZATION_DECI);
    bool const b_corner_unknown = (LOOK_ECHO_OPEN != p_in->ping_echo) &&
        (LOOK_ECHO_TOO_CLOSE != p_in->ping_echo) &&
        ((LOOK_ECHO_VALID != p_in->ping_echo) || (0U == p_in->ping_mm));

    if (corner_scan(p_hug) && (b_expired || (b_match && b_corner_unknown)))
    {
        if ((PU_CORNER_RETRY == p_hug->purpose) || (PU_FACET_RETRY == p_hug->purpose))
        {
            finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else
        {
            /* The axle is still at the edge. Keep its position and side,
             * but measure all three clearance rays together again. */
            start_scan(p_hug, p_look, (PU_FACET_CLEAR == p_hug->purpose) ? PU_FACET_RETRY : PU_CORNER_RETRY,
                       p_hug->center);
        }
    }
    else if (b_recheck && (b_expired ||
        (b_match && !recheck_ray(p_hug, p_look, p_in, offset))))
    {
        restart_contact(p_hug, p_look);
    }
    else if (b_short && b_match &&
        ((LOOK_ECHO_VALID != p_in->ping_echo) || (0U == p_in->ping_mm)))
    {
        look_along_face(p_hug, PU_LOOK);
    }
    else if (b_match)
    {
        accept_scan_ray(p_hug, p_look, p_in, offset, b_recheck, p_out);
    }
    else if (b_expired)
    {
        if (probe_scan(p_hug))
        {
            look_along_face(p_hug, PU_LOOK);
        }
        else
        {
            p_hug->lost = (uint8_t)((p_hug->lost < HUG_LOST_REPLIES) ? (p_hug->lost + 1U)
                                                                     : p_hug->lost);
            p_hug->req_seq = (uint8_t)(((uint32_t)p_hug->req_seq + 1U) & SEQ_MASK);
            p_hug->t_ms = p_in->now_ms;
        }
    }
    else
    {
        /* Waiting for the reply. */
    }
    if ((ST_SCAN == p_hug->state) && (SUB_ASK == p_hug->sub))
    {
        int32_t const next_offset = ray_offset(p_hug, p_hug->idx);
        emit_request(p_hug, p_out, next_offset - scan_body_aim(p_hug, next_offset));
        p_out->b_stop = (p_hug->lost >= HUG_LOST_REPLIES) ? 1U : 0U;
    }
}

static void scan_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                      hug_out_t * p_out)
{
    int32_t const offset = ray_offset(p_hug, p_hug->idx);
    uint8_t const want = body_step(p_hug, offset);

    if (SUB_MOVE == p_hug->sub)
    {
        bool const b_aimed = flag_get(p_hug, F_AIMED);

        flag_set(p_hug, F_AIMED, false);
        if (pivot_cmd(p_hug, p_hug->center + scan_body_aim(p_hug, offset), p_out))
        {
            p_hug->sub = (uint8_t)SUB_SETTLE;
            p_hug->t_ms = p_in->now_ms - (b_aimed ? HUG_SETTLE_MS : 0U);
            p_hug->body = want;
        }
        else
        {
            p_out->b_scan_pivot = true;
        }
    }
    else if (SUB_SETTLE == p_hug->sub)
    {
        if (p_in->b_stationary && ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS))
        {
            p_hug->sub = (uint8_t)SUB_ASK;
            p_hug->t_ms = p_in->now_ms;
            p_hug->req_seq = (uint8_t)(((uint32_t)p_hug->req_seq + 1U) & SEQ_MASK);
        }
    }
    else if (!p_in->b_stationary)
    {
        /* Invalidate the outstanding request. A short clearance scan must
         * collect all three bearings again from one stationary pose. */
        if (probe_scan(p_hug) || (PU_CENTER == p_hug->purpose))
        {
            start_scan(p_hug, p_look, p_hug->purpose, p_hug->center);
        }
        p_hug->sub = (uint8_t)SUB_START;
    }
    else
    {
        scan_reply(p_hug, p_look, p_in, offset, p_out);
    }
}

/* ---- planning ---- */

static uint32_t valid_rays(look_t const * p_look)
{
    uint32_t n = 0U;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        n += (LOOK_ECHO_OPEN != look_ray_echo(&p_look->rays[i])) ? 1U : 0U;
    }
    return n;
}

/* Lateral distances are measured across the heading the car looked along, not
 * across the fitted normal: the fit of a narrow face is a few degrees off. */

static void find_edges(look_t const * p_look, look_face_t const * p_face,
                       hug_t const * p_hug, hug_edges_t * p_edges)
{
    int32_t const left_deci = ((PU_CONTACT == p_hug->purpose) || (PU_REFINE == p_hug->purpose)
                               ? 0
                               : p_hug->center)
                              + QUARTER_DECI;

    p_edges->b_narrow = valid_rays(p_look) < HUG_NORMAL_RAYS_MIN;
    look_edges(p_look, p_face, p_edges->edge, p_edges->have);
    for (uint8_t s = 0U; s < HUG_SIDES; s++)
    {
        p_edges->lat_mm[s] = 0;
        if (p_edges->have[s])
        {
            p_edges->lat_mm[s] = geom_abs(geom_dot_mm(left_deci,
                                                 p_edges->edge[s].x_mm - pos_x_mm(p_hug),
                                                 p_edges->edge[s].y_mm - pos_y_mm(p_hug)));
        }
    }
}

static int32_t edges_apart_mm(hug_edges_t const * p_edges)
{
    int32_t const dx = p_edges->edge[LEFT].x_mm - p_edges->edge[RIGHT].x_mm;
    int32_t const dy = p_edges->edge[LEFT].y_mm - p_edges->edge[RIGHT].y_mm;

    return (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy)));
}

/* The nearer edge, else the more open side; a side whose gap beside the
 * edge is too narrow for the car is passed over. A tie goes left. */
static int8_t pick_side(hug_t const * p_hug, hug_edges_t const * p_edges)
{
    bool b_block[HUG_SIDES] = {false, false};
    int8_t side = 1;

    for (uint8_t s = 0U; s < HUG_SIDES; s++)
    {
        b_block[s] = p_edges->have[s] && (p_hug->wall_mm[s] > 0U)
                     && (((int32_t)p_hug->wall_mm[s] - p_edges->lat_mm[s]) < HUG_GAP_MM);
    }
    if (b_block[LEFT] != b_block[RIGHT])
    {
        side = b_block[LEFT] ? (int8_t)-1 : (int8_t)1;
    }
    else if (p_edges->have[LEFT] && p_edges->have[RIGHT])
    {
        /* Both edges of a narrow face are near the path; the nearer one wins. */
        int32_t const tie = p_edges->b_narrow ? 0 : HUG_TIE_MM;

        side = ((p_edges->lat_mm[RIGHT] + tie) < p_edges->lat_mm[LEFT]) ? (int8_t)-1 : (int8_t)1;
    }
    else if (p_edges->have[LEFT] != p_edges->have[RIGHT])
    {
        /* The other edge is farther than the look trusts, unless the face is
         * so narrow that the look lost it. */
        side = p_edges->have[LEFT] ? (int8_t)1 : (int8_t)-1;
    }
    else
    {
        side = ((uint32_t)p_hug->open_mm[RIGHT] > ((uint32_t)p_hug->open_mm[LEFT] + HUG_OPEN_TIE_MM))
                   ? (int8_t)-1 : (int8_t)1;
    }
    return side;
}

/* Distance from the saved vertex along an axis in the contact frame. */
static int32_t from_edge_mm(hug_t const * p_hug, int32_t axis)
{
    uint8_t const side = (p_hug->side > 0) ? LEFT : RIGHT;
    return geom_dot_mm(axis, pos_x_mm(p_hug) - p_hug->keep_x_mm[side],
                       pos_y_mm(p_hug) - p_hug->keep_y_mm[side]);
}

/* Where the lateral setup leg ends. The following straight line passes the
 * vertex at half the body width plus the working gap, measured perpendicular
 * to travel. The pivot here still has the full front standoff. */
/* Passing the obstacle on its right puts the left reader next to its face. */
static int32_t obstacle_side_reach_mm(hug_t const * p_hug)
{
    return (int32_t)((p_hug->side < 0) ? CHASSIS_LEFT_REACH_MM : CHASSIS_RIGHT_REACH_MM);
}

static int32_t flank_setup_mm(int32_t front_mm, int32_t side_reach_mm)
{
    int32_t const across = side_reach_mm + HUG_FLANK_GAP_MM
                          - geom_mul_q14(front_mm, geom_sin_q14(HUG_FLANK_DECI));
    int32_t const cosine = geom_cos_q14(HUG_FLANK_DECI);
    return (across * GEOM_Q14_ONE + cosine - 1) / cosine;
}

static int32_t approach_mm(hug_t const * p_hug, int32_t dist_mm)
{
    /* After a stall the measured approach must also clear the stall point. */
    int32_t const limit = flag_get(p_hug, F_STALL) ? HUG_STANDOFF_MM - HUG_BODY_GAP_MM : HOP_LIMIT_MM;
    int32_t const travel = clamp_i32(dist_mm - HUG_AXLE_OFF_MM, 0, limit);
    return (travel >= HUG_APPROACH_MIN_MM) ? travel : 0;
}

/* From the face and its edges: the heading along the face, the length of the
 * leg, and for the first leg the approach to the gap. A face assumed at a stall
 * is not measured, and a narrow face gives a poor distance, so their legs are
 * not trimmed. */
static void plan_leg(hug_t * p_hug, look_face_t const * p_face,
                     hug_edges_t const * p_edges, bool b_first, look_t const * p_look)
{
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const along = geom_wrap_deci(p_face->normal_deci + (p_hug->side * QUARTER_DECI));
    int32_t const sonar_mm = p_face->dist_mm - LOOK_SONAR_FWD_MM;
    int32_t const min_leg = (p_hug->b_close_front || (p_hug->side_end_x_mm < 0)) ? HUG_APPROACH_MIN_MM : HUG_LEG_MIN_MM;
    int32_t const app = b_first ? approach_mm(p_hug, p_face->dist_mm) : 0;
    int32_t leg = 0;
    int32_t trim = 0;
    bool const b_measured = (NULL != p_look) && !p_edges->b_narrow;
    /* Width and trim need a well-conditioned normal. A bounded front
     * with two hit-to-open edges can still be approached for a fresh
     * side survey, which must independently confirm its next face. */
    bool const b_bounded = b_first && p_edges->have[LEFT] && p_edges->have[RIGHT];

    flag_set(p_hug, F_ASSUME, false);
    flag_set(p_hug, F_PAST, p_edges->have[s]);
    flag_set(p_hug, F_EDGE_LEG, p_edges->have[s] && (!p_edges->b_narrow || b_bounded || p_hug->b_oblique));
    flag_set(p_hug, F_MEASURED_FACE, b_measured);
    if (b_measured && (PU_CHECK != p_hug->purpose))
    {
        p_hug->face_line_mm = p_face->dist_mm + geom_dot_mm(p_face->normal_deci,
                                                           pos_x_mm(p_hug), pos_y_mm(p_hug));
    }
    if (flag_get(p_hug, F_EDGE_LEG))
    {
        /* The refine buffers are free once the leg has been planned. */
        p_hug->keep_x_mm[s] = (int16_t)p_edges->edge[s].x_mm;
        p_hug->keep_y_mm[s] = (int16_t)p_edges->edge[s].y_mm;
    }
    if (p_edges->have[s])
    {
        leg = geom_dot_mm(along, p_edges->edge[s].x_mm - pos_x_mm(p_hug),
                     p_edges->edge[s].y_mm - pos_y_mm(p_hug));
        if (p_hug->b_close_front)
        {
            leg += flank_setup_mm(p_face->dist_mm - app, obstacle_side_reach_mm(p_hug));
        }
        else
        {
            leg += HUG_AXLE_OFF_MM
                   + ((p_hug->b_oblique || (b_bounded && p_edges->b_narrow)) ? HUG_APPROACH_MIN_MM : 0);
        }
        if (b_first && b_measured && !p_hug->b_close_front)
        {
            /* The guarded approach to the first side can run inward if
             * the front is oblique to the entry heading. Reserve that
             * lateral displacement before turning around its endpoint. */
            int32_t const inward = -p_hug->side *
                geom_mul_q14(p_face->dist_mm - app, geom_sin_q14(p_face->normal_deci));
            leg += (inward > 0) ? inward : 0;
        }
    }
    else if (b_measured)
    {
        look_pose_t const pose = hug_axle_pose(p_hug);
        look_side_t const side = (p_hug->side > 0) ? LOOK_SIDE_LEFT : LOOK_SIDE_RIGHT;
        look_side_t const other = (LOOK_SIDE_LEFT == side) ? LOOK_SIDE_RIGHT : LOOK_SIDE_LEFT;
        int32_t const reach = look_face_reach_mm(p_look, &pose, p_face, side);
        int32_t const behind = look_face_reach_mm(p_look, &pose, p_face, other);
        /* Buddy 5 advances past the last on-face return by a half lane.
         * Keep successive views overlapping with this narrower servo.
         * While seeking the third vertex, stop at the observed extent. */
        int32_t const lane = (int32_t)OBSTACLE_CAR_HALF_W_MM + HUG_LANE_BUFFER_MM;
        leg = reach + ((p_hug->side_end_x_mm < 0) ? 0 : clamp_i32(behind / HALVES, 0, lane));
    }
    else
    {
        leg = geom_mul_q14(sonar_mm - app, geom_tan_q14(HUG_HOP_ANGLE_DECI));
    }
    leg = clamp_i32(leg, min_leg, HUG_LEG_MAX_MM);
    if (b_measured && !p_hug->b_close_front)
    {
        trim = clamp_i32(geom_atan_deci(p_face->dist_mm - app - HUG_AXLE_OFF_MM, leg),
                         -HUG_TRIM_MAX_DECI, HUG_TRIM_MAX_DECI);
    }
    /* The leg is the way along the face; leaning makes the way longer. */
    leg = clamp_i32((leg * GEOM_Q14_ONE) / geom_cos_q14(trim), min_leg, HUG_LEG_MAX_MM);
    p_hug->u_head = (int16_t)geom_wrap_deci(along - (p_hug->side * trim));
    p_hug->look_head = (int16_t)p_face->normal_deci;
    p_hug->leg_mm = (int16_t)leg;
    p_hug->app_mm = (int16_t)app;
    p_hug->face_p_mm = (int16_t)(p_face->dist_mm - app);
    p_hug->seen_mm = (int16_t)p_face->dist_mm;
    if (app > 0)
    {
        start_pivot(p_hug, p_face->normal_deci, THEN_APPROACH);
    }
    else
    {
        start_pivot(p_hug, p_hug->u_head, THEN_LEG);
    }
}

static void release(hug_t * p_hug)
{
    flag_set(p_hug, F_REAPPROACH, flag_get(p_hug, F_STALL));
    start_pivot(p_hug, 0, THEN_RELEASE);
}

/* Give the wheels back. The sizes belong to the obstacle just left. */
static void finish(hug_t * p_hug)
{
    if (flag_get(p_hug, F_EXIT))
    {
        p_hug->b_note = 0U;
        p_hug->n_rel = 0U;
    }
    p_hug->state = (uint8_t)ST_IDLE;
    p_hug->hist = 0U;
    p_hug->width_mm = 0U;
    p_hug->breadth_mm = 0U;
    p_hug->seen_mm = 0;
}

static void keep_edges(hug_t * p_hug, hug_edges_t const * p_edges,
                       look_face_t const * p_face)
{
    for (uint8_t s = 0U; s < HUG_SIDES; s++)
    {
        p_hug->keep_x_mm[s] = p_edges->have[s] ? (int16_t)p_edges->edge[s].x_mm : 0;
        p_hug->keep_y_mm[s] = p_edges->have[s] ? (int16_t)p_edges->edge[s].y_mm : 0;
    }
    p_hug->keep_normal = (int16_t)p_face->normal_deci;
    p_hug->keep_dist_mm = (int16_t)p_face->dist_mm;
    flag_set(p_hug, F_COARSE_NARROW, p_edges->b_narrow);
}

static void restore_edges(hug_t const * p_hug, hug_edges_t * p_edges, look_face_t * p_face)
{
    int32_t const left_deci = QUARTER_DECI;

    p_edges->b_narrow = flag_get(p_hug, F_COARSE_NARROW);
    for (uint8_t s = 0U; s < HUG_SIDES; s++)
    {
        p_edges->edge[s].x_mm = p_hug->keep_x_mm[s];
        p_edges->edge[s].y_mm = p_hug->keep_y_mm[s];
        p_edges->have[s] = true;
        p_edges->lat_mm[s] = geom_abs(geom_dot_mm(left_deci, p_edges->edge[s].x_mm - pos_x_mm(p_hug),
                                             p_edges->edge[s].y_mm - pos_y_mm(p_hug)));
    }
    p_face->normal_deci = p_hug->keep_normal;
    p_face->dist_mm = p_hug->keep_dist_mm;
}

/* A finer look at one edge: the window of rays is centred a little outside
 * the edge, and the car stands still, so every ray starts from one place. */
static void start_refine(hug_t * p_hug, look_t * p_look, uint8_t side)
{
    int32_t const out = (LEFT == side) ? HUG_REFINE_OUT_DECI : -HUG_REFINE_OUT_DECI;
    int32_t center = 0;

    /* The sonar sits ahead of the axle, so turning the body moves it
     * sideways: aim from where it will be. */
    for (uint32_t i = 0U; i < REFINE_AIMS; i++)
    {
        int32_t const ox = pos_x_mm(p_hug) + geom_mul_q14(LOOK_SONAR_FWD_MM, geom_cos_q14(center));
        int32_t const oy = pos_y_mm(p_hug) + geom_mul_q14(LOOK_SONAR_FWD_MM, geom_sin_q14(center));

        center = geom_wrap_deci(geom_atan_deci(p_hug->keep_y_mm[side] - oy,
                                               p_hug->keep_x_mm[side] - ox)
                                + out);
    }
    flag_set(p_hug, F_WIN2, RIGHT == side);
    start_scan(p_hug, p_look, PU_REFINE, center);
}

static void decide_contact(hug_t * p_hug, look_t const * p_look, look_face_t const * p_face,
                           hug_edges_t const * p_edges)
{
    /* A narrow face gives edges too poor to report a width from. */
    int32_t const span = (p_edges->have[LEFT] && p_edges->have[RIGHT]) ? edges_apart_mm(p_edges) : 0;
    p_hug->b_close_front = (span > 0) && (span <= (int32_t)OBSTACLE_CAR_HALF_W_MM) &&
        (p_face->dist_mm >= HUG_SCAN_CLEARANCE_MM) &&
        !flag_get(p_hug, F_STALL) && (geom_abs(p_face->normal_deci) <= HUG_FACE_ANGLE_DECI) &&
        (geom_dot_mm(p_face->normal_deci + QUARTER_DECI, p_edges->edge[LEFT].x_mm - p_edges->edge[RIGHT].x_mm,
                     p_edges->edge[LEFT].y_mm - p_edges->edge[RIGHT].y_mm) > 0);
    p_hug->width_mm = p_edges->b_narrow ? 0U : (uint16_t)span;
    p_hug->side = pick_side(p_hug, p_edges);
    plan_leg(p_hug, p_face, p_edges, true, p_look);
}

/* The first look of a face. With both edges in trust each edge is looked at
 * once more with finer rays, which fixes it to a few mm. Without them the
 * width stays unknown and the car goes on from what it saw. */
static void after_contact(hug_t * p_hug, look_t * p_look, look_face_t const * p_face)
{
    hug_edges_t edges;

    find_edges(p_look, p_face, p_hug, &edges);
    if (edges.have[LEFT] && edges.have[RIGHT])
    {
        keep_edges(p_hug, &edges, p_face);
        flag_set(p_hug, F_REFINE, true);
        start_refine(p_hug, p_look, LEFT);
    }
    else if (edges.b_narrow && (edges.have[LEFT] || edges.have[RIGHT]))
    {
        /* With only one edge, compare the two halves of the beam from one
         * stationary, centred pose. Preserve the measured edge for the leg. */
        keep_edges(p_hug, &edges, p_face);
        p_hug->side = edges.have[LEFT] ? (int8_t)1 : (int8_t)-1;
        start_scan(p_hug, p_look, PU_CENTER, 0);
    }
    else
    {
        decide_contact(p_hug, p_look, p_face, &edges);
    }
}

static void after_center(hug_t * p_hug, look_t const * p_look, look_face_t const * p_fit, bool b_found)
{
    hug_edges_t edges;
    look_face_t face = {0, 0, 0, 0};
    uint8_t const missing = (p_hug->side > 0) ? RIGHT : LEFT;
    int32_t balance = 0;

    restore_edges(p_hug, &edges, &face);
    edges.have[missing] = false;
    p_hug->side = pick_side(p_hug, &edges);
    /* Only a complete symmetric sweep can compare the two sides. The wide
     * scan's side-wall clearance retains priority. */
    if ((CENTER_RAYS == p_look->count) && (valid_rays(p_look) >= HUG_FACE_RAYS_MIN) &&
        (0U == p_hug->wall_mm[LEFT]) && (0U == p_hug->wall_mm[RIGHT]))
    {
        for (uint8_t i = 0U; i < CENTER_RAYS / HALVES; i++)
        {
            /* Each pair is equidistant from the central ray. */
            uint8_t const left = (uint8_t)(CENTER_RAYS - 1U - i);
            balance += (int32_t)look_echo_is_point(look_ray_echo(&p_look->rays[left]))
                       - (int32_t)look_echo_is_point(look_ray_echo(&p_look->rays[i]));
        }
        if (b_found && (geom_abs(p_fit->dist_mm - face.dist_mm) <= HUG_REFINE_TOL_MM))
        {
            face = *p_fit;
        }
        if (0 != balance)
        {
            p_hug->side = (balance > 0) ? (int8_t)-1 : (int8_t)1;
        }
    }
    plan_leg(p_hug, &face, &edges, true, p_look);
}

/* The sonar reads the nearest point inside its beam, so a ray more than the
 * half-angle off the face normal has hit the face half a beam nearer the
 * normal, and a ray within it has hit the foot of the normal. Fit the face
 * with the rays moved there; the rays go back afterwards. */
static bool fit_beam(hug_t const * p_hug, look_t * p_look, look_face_t * p_face)
{
    look_pose_t const pose = hug_axle_pose(p_hug);
    int16_t moved[LOOK_RAYS_MAX];
    bool b_ok = false;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t * const p_ray = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(p_ray);
        int32_t const diff = geom_wrap_deci(angle - p_face->normal_deci);
        int32_t move = 0;

        if (LOOK_ECHO_OPEN != look_ray_echo(p_ray))
        {
            move = (geom_abs(diff) > LOOK_BEAM_DECI) ? ((diff > 0) ? -LOOK_BEAM_DECI : LOOK_BEAM_DECI)
                                                     : -diff;
        }
        moved[i] = (int16_t)move;
        look_ray_set_angle(p_ray, geom_wrap_deci(angle + move));
    }
    b_ok = look_face(p_look, &pose, p_face);
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t * const p_ray = &p_look->rays[i];

        look_ray_set_angle(p_ray, geom_wrap_deci(look_ray_angle_deci(p_ray) - moved[i]));
    }
    return b_ok;
}

static bool refit_beam(hug_t const * p_hug, look_t * p_look, look_face_t * p_face,
                       hug_edges_t * p_edges)
{
    bool b_ok = fit_beam(p_hug, p_look, p_face);

    if (b_ok)
    {
        find_edges(p_look, p_face, p_hug, p_edges);
        b_ok = p_edges->have[LEFT] && p_edges->have[RIGHT];
    }
    return b_ok;
}

static void after_refine(hug_t * p_hug, look_t * p_look)
{
    if (!flag_get(p_hug, F_WIN2))
    {
        start_refine(p_hug, p_look, RIGHT);
    }
    else
    {
        look_pose_t const pose = hug_axle_pose(p_hug);
        look_face_t face = {0, 0, 0, 0};
        hug_edges_t edges;
        bool b_fine = look_face(p_look, &pose, &face);

        flag_set(p_hug, F_REFINE | F_WIN2, false);
        if (b_fine)
        {
            find_edges(p_look, &face, p_hug, &edges);
            b_fine = edges.have[LEFT] && edges.have[RIGHT];
        }

        if (b_fine && ((p_hug->keep_dist_mm - LOOK_SONAR_FWD_MM) > HUG_FAR_MM))
        {
            /* Far off the beam sags the points by a few percent. A near face is
             * narrow in rays and the correction would only thin them out. */
            b_fine = refit_beam(p_hug, p_look, &face, &edges);
        }
        for (uint8_t s = 0U; b_fine && (s < HUG_SIDES); s++)
        {
            int32_t const dx = edges.edge[s].x_mm - p_hug->keep_x_mm[s];
            int32_t const dy = edges.edge[s].y_mm - p_hug->keep_y_mm[s];

            b_fine = (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy))) <= HUG_REFINE_TOL_MM;
        }
        if (!b_fine)
        {
            restore_edges(p_hug, &edges, &face);
        }
        decide_contact(p_hug, p_look, &face, &edges);
    }
}

/* Keep the rays already measured. Optional close-ups can use their coarse
 * edges immediately. On a full sweep, aim the chassis at each remaining ray
 * and use the actual sonar pose when its reply arrives. */
static void end_stationary_batch(hug_t * p_hug, look_t * p_look)
{
    if (flag_get(p_hug, F_RECHECK))
    {
        restart_contact(p_hug, p_look);
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        look_face_t face = {0};
        hug_edges_t edges;
        restore_edges(p_hug, &edges, &face);
        flag_set(p_hug, F_REFINE | F_WIN2, false);
        decide_contact(p_hug, p_look, &face, &edges);
    }
    else if (PU_CENTER == p_hug->purpose)
    {
        /* The saved single edge still determines a candidate side. */
        after_center(p_hug, p_look, NULL, false);
    }
    else if (PU_CHECK == p_hug->purpose)
    {
        look_along_face(p_hug, PU_LOOK);
    }
    else if (!corner_scan(p_hug))
    {
        int32_t const offset = ray_offset(p_hug, p_hug->idx);
        int32_t const delta = geom_wrap_deci(p_hug->center + offset - hug_pose_heading_deci(p_hug));
        p_hug->scan_lead_deci = (int8_t)((delta >= 0) ? HUG_NEAR_STEP_DECI : -HUG_NEAR_STEP_DECI);
        p_hug->sub = (uint8_t)SUB_START;
        flag_set(p_hug, F_AIMED, false);
    }
    else
    {
        /* All three corner-clearance rays must share the stationary pose. */
    }
}

/* The heading along the face the last look was square to. */
static int32_t along_head(hug_t const * p_hug)
{
    return geom_wrap_deci(p_hug->look_head + (p_hug->side * QUARTER_DECI));
}

/* The lead-side probe must still see the part of the face measured before
 * the leg, even at the allowed aim error toward travel. */
static bool side_check_supported(hug_t const * p_hug, look_t const * p_look)
{
    int32_t const normal = p_hug->look_head;
    look_pose_t const pose = hug_axle_pose(p_hug);
    int32_t const expected = p_hug->face_line_mm - geom_dot_mm(normal, pose.x_mm, pose.y_mm);
    look_face_t const face = {0, 0, normal, expected};
    look_side_t const side = (p_hug->side > 0) ? LOOK_SIDE_LEFT : LOOK_SIDE_RIGHT;
    int32_t const sonar_normal = geom_mul_q14(LOOK_SONAR_FWD_MM, geom_cos_q14(HUG_HEAD_TOL_DECI));
    int32_t const sonar_lead = geom_mul_q14(LOOK_SONAR_FWD_MM, geom_sin_q14(HUG_HEAD_TOL_DECI));
    int32_t const lead_mm = sonar_lead + geom_mul_q14(expected - sonar_normal,
        geom_tan_q14(PROBE_STEP_DECI + HUG_HEAD_TOL_DECI - LOOK_BEAM_DECI));

    int32_t const reach = look_face_reach_mm(p_look, &pose, &face, side);
    return (reach > 0) && (reach >= lead_mm);
}

static void look_along_face(hug_t * p_hug, uint8_t purpose)
{
    p_hug->b_close_front = false;
    p_hug->purpose = purpose;
    p_hug->center = p_hug->look_head;
    if (PU_MID_LOOK == purpose)
    {
        p_hug->center = (int16_t)geom_wrap_deci(p_hug->look_head + p_hug->side * (QUARTER_DECI / HALVES));
    }
    p_hug->idx = 0U;
    flag_set(p_hug, F_AIMED, true);
    start_pivot(p_hug, p_hug->center + body_deci(p_hug, body_step(p_hug, ray_offset(p_hug, 0U))),
                THEN_SCAN);
}

bool hug_resume_remaining(hug_t * p_hug, int64_t left_um, int64_t right_um, uint32_t now_ms)
{
    bool const b_offered = p_hug->b_can_resume && !hug_active(p_hug);
    p_hug->b_can_resume = false;
    if (b_offered)
    {
        /* A local sweep pivots in place and returns to its starting heading.
         * A translated car no longer has this surveyed return pose. */
        int32_t const dl = (int32_t)((uint32_t)left_um - p_hug->last_left_um);
        int32_t const dr = (int32_t)((uint32_t)right_um - p_hug->last_right_um);
        int64_t const translation = (int64_t)dl + dr;
        int64_t const yaw = ((int64_t)dr - dl) * DECI_PER_RAD;
        int64_t const yaw_limit = (int64_t)HUG_HEAD_TOL_DECI * CHASSIS_WHEEL_TRACK_MM * UM_PER_MM;
        if ((translation < -(LINE_WIDTH_MM * UM_PER_MM)) || (translation > (LINE_WIDTH_MM * UM_PER_MM)) ||
            (yaw < -yaw_limit) || (yaw > yaw_limit))
        {
            return false;
        }
        p_hug->last_left_um = (uint32_t)left_um;
        p_hug->last_right_um = (uint32_t)right_um;
        p_hug->diff_um = 0; /* restored entry heading */
        p_hug->b_remaining = true;
        p_hug->last_motion_ms = now_ms;
        flag_set(p_hug, F_EXIT | F_EXIT_TAPE | F_EDGE_LEG, false);
        look_along_face(p_hug, PU_LOOK);
    }
    return b_offered;
}

static void set_next_face(hug_t * p_hug)
{
    flag_set(p_hug, F_PAST | F_MEASURED_FACE, false);
    flag_set(p_hug, F_TURNED, true);
    p_hug->trav_head = p_hug->center;
    p_hug->look_head = (int16_t)geom_wrap_deci(p_hug->center - (p_hug->side * QUARTER_DECI));
}

static int32_t flank_drive_mm(hug_t const * p_hug)
{
    int32_t const outward = p_hug->side * QUARTER_DECI;
    int32_t const front = -from_edge_mm(p_hug, p_hug->look_head);
    int32_t const nearest = from_edge_mm(p_hug, p_hug->center + outward);
    if ((front < HUG_SCAN_CLEARANCE_MM) ||
        (nearest < obstacle_side_reach_mm(p_hug) + (int32_t)OBSTACLE_PLAN_MARGIN_MM))
    {
        return 0;
    }
    int32_t const gain = HUG_SCAN_CLEARANCE_MM - from_edge_mm(p_hug, p_hug->look_head + outward);
    /* Cover the shallowest allowed heading. Odometry ends the leg as soon
     * as the chassis can turn at its new observation station. */
    int32_t const sine = geom_sin_q14(HUG_FLANK_DECI - HUG_HEAD_TOL_DECI);
    return (gain * GEOM_Q14_ONE + sine - 1) / sine;
}

static void after_corner(hug_t * p_hug, look_t const * p_look, hug_out_t * p_out)
{
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    /* Stop abreast of the measured endpoint to look sideways. A fixed
     * advance past its plane can skip the whole next face of a small box. */
    bool const b_mid = facet_clearance(p_hug);
    int32_t leg_mm = b_mid ?
        geom_dot_mm(p_hug->center, p_hug->origin.drive.x_mm - pos_x_mm(p_hug),
                    p_hug->origin.drive.y_mm - pos_y_mm(p_hug)) / HALVES :
        geom_dot_mm(p_hug->center, p_hug->keep_x_mm[s] - pos_x_mm(p_hug),
                    p_hug->keep_y_mm[s] - pos_y_mm(p_hug));
    if (p_hug->b_close_front)
    {
        leg_mm = flank_drive_mm(p_hug);
    }
    int32_t const min_mm = leg_mm + HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM;
    bool b_clear = (PROBE_RAYS == p_look->count) && (leg_mm >= HUG_LEG_MIN_MM) && (leg_mm <= HUG_LEG_MAX_MM);

    for (uint8_t i = 0U; b_clear && (i < p_look->count); i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(p_ray) - p_hug->center;
        int32_t const along_mm = geom_mul_q14(p_ray->range_mm, geom_cos_q14(angle));

        b_clear = (LOOK_ECHO_OPEN == look_ray_echo(p_ray)) ||
                  ((LOOK_ECHO_VALID == look_ray_echo(p_ray)) && (along_mm >= min_mm));
    }
    if (b_clear && p_hug->b_close_front)
    {
        flag_set(p_hug, F_EXIT_TAPE, false);
        start_drive(p_hug, leg_mm, p_hug->center, THEN_FLANK_LOOK);
    }
    else if (b_clear)
    {
        if (!b_mid)
        {
            set_next_face(p_hug);
        }
        start_drive(p_hug, (!b_mid && p_hug->b_oblique) ? leg_mm / HALVES : leg_mm, p_hug->center, THEN_LOOK);
    }
    else if (b_mid)
    {
        finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else
    {
        look_along_face(p_hug, PU_LOOK);
    }
}

/* Confirm the existing plane at separated bearings. A short check cannot
 * fit a new normal or infer a new edge, and it never advances the plane. */
static void after_side_check(hug_t * p_hug, look_t const * p_look)
{
    int32_t const normal = p_hug->look_head;
    int32_t const axle = geom_dot_mm(normal, pos_x_mm(p_hug), pos_y_mm(p_hug));
    int32_t const expected_mm = p_hug->face_line_mm - axle;
    int32_t near_mm = INT32_MAX;
    int32_t far_mm  = INT32_MIN;
    bool b_confirmed = (PROBE_RAYS == p_look->count);

    for (uint8_t i = 0U; b_confirmed && (i < p_look->count); i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(p_ray) - normal;
        int32_t const normal_mm = geom_dot_mm(normal, p_ray->x_mm, p_ray->y_mm) - axle
                            + geom_mul_q14(p_ray->range_mm, geom_cos_q14(angle));

        b_confirmed = (LOOK_ECHO_VALID == look_ray_echo(p_ray)) &&
                      (geom_abs(normal_mm - expected_mm) <= LOOK_FACE_MATCH_MM);
        near_mm = (normal_mm < near_mm) ? normal_mm : near_mm;
        far_mm = (normal_mm > far_mm) ? normal_mm : far_mm;
    }
    if (b_confirmed && (near_mm >= (HUG_AXLE_OFF_MM - LOOK_FACE_MATCH_MM)) &&
        ((far_mm - near_mm) <= FACE_SPREAD_MM))
    {
        look_face_t const face = {0, 0, normal, near_mm};
        hug_edges_t const edges = {0};

        plan_leg(p_hug, &face, &edges, false, p_look);
    }
    else
    {
        look_along_face(p_hug, PU_LOOK);
    }
}

/* A quadrilateral's measured side endpoint must lie on the observed rear
 * plane. The whole swept body must clear that plane at both ends of the
 * straight traverse. This also bounds clearance during the two pivots. */
static bool begin_rear_return(hug_t * p_hug, look_face_t const * p_face)
{
    int32_t const normal = p_face->normal_deci;
    int32_t const x = pos_x_mm(p_hug);
    int32_t const y = pos_y_mm(p_hug);
    int32_t const radius = HUG_SWEEP_RADIUS_MM;
    uint8_t const side = (p_hug->side > 0) ? LEFT : RIGHT;
    bool const b_joined = p_hug->b_oblique && (PU_MID_LOOK == p_hug->purpose) && flag_get(p_hug, F_TURNED);
    /* A chamfer links the side endpoint to a second measured endpoint on
     * the rear plane. Use that connected endpoint for the plane check. */
    int32_t const end_x = b_joined ? p_hug->keep_x_mm[side] : p_hug->side_end_x_mm;
    int32_t const end_y = b_joined ? p_hug->keep_y_mm[side] : p_hug->side_end_y_mm;
    int32_t const end_plane = geom_dot_mm(normal, end_x - x, end_y - y);
    int32_t const target_gap = p_face->dist_mm + geom_mul_q14(y, geom_sin_q14(normal));
    bool const b_verified = !p_hug->b_remaining && (p_hug->side_end_x_mm > 0) && !flag_get(p_hug, F_EXIT_TAPE) &&
        (geom_abs(geom_wrap_deci(normal - HALVES * QUARTER_DECI)) <= HUG_FACE_ANGLE_DECI) &&
        (geom_abs(end_plane - p_face->dist_mm) <= HUG_REFINE_TOL_MM) &&
        (p_face->dist_mm >= (radius + HUG_REFINE_TOL_MM)) && (target_gap >= (radius + HUG_REFINE_TOL_MM)) &&
        (geom_abs(y) <= HUG_LEG_MAX_MM);
    if (b_verified)
    {
        p_hug->look_head = (int16_t)normal;
        p_hug->face_p_mm = (int16_t)target_gap;
        if (geom_abs(y) <= (LINE_WIDTH_MM / HALVES))
        {
            start_pivot(p_hug, 0, THEN_REAR_READY);
        }
        else
        {
            start_pivot(p_hug, (y > 0) ? -QUARTER_DECI : QUARTER_DECI, THEN_REAR_LATERAL);
        }
    }
    return b_verified;
}

static void follow_measured_face(hug_t * p_hug, look_t const * p_look, look_face_t const * p_face)
{
    hug_edges_t edges;
    find_edges(p_look, p_face, p_hug, &edges);
    /* Only a measured face running along the approach can supply the
     * rearward endpoint. The other front corner says nothing about it. */
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const side_normal = -p_hug->side * QUARTER_DECI;
    bool const b_same_side = (p_hug->side_end_x_mm < 0) && !flag_get(p_hug, F_TURNED) &&
        (geom_abs(geom_wrap_deci(p_hug->center - side_normal)) <= HUG_FACE_ANGLE_DECI);
    bool const b_side = b_same_side || (geom_abs(geom_wrap_deci(p_face->normal_deci - side_normal)) <= HUG_FACE_ANGLE_DECI);
    if (b_side && (0 >= p_hug->side_end_x_mm))
    {
        if (edges.have[s] && (edges.edge[s].x_mm > 0))
        {
            p_hug->side_end_x_mm = (int16_t)edges.edge[s].x_mm;
            p_hug->side_end_y_mm = (int16_t)edges.edge[s].y_mm;
        }
        else
        {
            p_hug->side_end_x_mm = -1;
        }
    }
    if (flag_get(p_hug, F_TURNED) && edges.have[LEFT] && edges.have[RIGHT] && !edges.b_narrow)
    {
        p_hug->breadth_mm = (uint16_t)edges_apart_mm(&edges);
    }
    flag_set(p_hug, F_TURNED, false);
    plan_leg(p_hug, p_face, &edges, false, p_look);
}

static void after_look(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                       bool b_found, hug_out_t * p_out)
{
    /* The look after an assumed face is the first one of the real face, which
     * may be turned as far as an INNER look takes it. */
    bool const b_near = b_found && (valid_rays(p_look) >= HUG_FACE_RAYS_MIN)
                        && (geom_abs(geom_wrap_deci(p_face->normal_deci - p_hug->center))
                            <= ((p_hug->b_probe_front || flag_get(p_hug, F_ASSUME)) ?
                                HUG_INNER_ANGLE_DECI : HUG_FACE_ANGLE_DECI))
                        && (flag_get(p_hug, F_TURNED)
                            ? (p_face->dist_mm <= (HUG_AXLE_OFF_MM + HUG_FACE_TOL_MM))
                            : (geom_abs(p_face->dist_mm - p_hug->face_p_mm) <= HUG_FACE_TOL_MM));

    if (p_hug->b_probe_front && !b_near)
    {
        /* Repeat the look at this station before spending any more
         * lateral clearance toward the neighboring line. */
        if (p_hug->b_probe_retried)
        {
            finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else
        {
            p_hug->b_probe_retried = true;
            look_along_face(p_hug, PU_LOOK);
        }
    }
    else if (b_near)
    {
        p_hug->b_probe_front = false;
        if (p_hug->b_remaining && (geom_abs(p_face->normal_deci) <= HUG_FACE_ANGLE_DECI) &&
            (pos_x_mm(p_hug) < p_hug->side_end_x_mm))
        {
            /* The outstanding arc has led back to the entry face. */
            finish(p_hug);
            p_out->b_no_exit = true;
            return;
        }
        if (begin_rear_return(p_hug, p_face))
        {
            return;
        }
        if (p_hug->b_oblique && !p_hug->b_remaining &&
            ((geom_abs(geom_wrap_deci(p_face->normal_deci - HALVES * QUARTER_DECI)) <= HUG_FACE_ANGLE_DECI) ||
             (p_hug->side * p_face->normal_deci > QUARTER_DECI)))
        {
            /* Reaching the rear without a qualified centered traverse cannot
             * justify another lap around the same obstacle. */
            finish(p_hug);
            p_out->b_timeout = 1U;
            return;
        }
        follow_measured_face(p_hug, p_look, p_face);
    }
    else if (p_hug->b_oblique)
    {
        finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else if (p_hug->side_end_x_mm < 0)
    {
        if (PU_EDGE_RETRY == p_hug->purpose)
        {
            finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else
        {
            look_along_face(p_hug, PU_EDGE_RETRY);
        }
    }
    else
    {
        if (flag_get(p_hug, F_TURNED))
        {
            /* The next face is short and behind the car: go on until the
             * body clears its end, then look again. */
            flag_set(p_hug, F_TURNED, false);
            p_hug->face_p_mm = (int16_t)HUG_AXLE_OFF_MM;
            p_hug->leg_mm = (int16_t)HUG_AXLE_OFF_MM;
            start_pivot(p_hug, p_hug->trav_head, THEN_ADVANCE);
        }
        else if (!flag_get(p_hug, F_PAST))
        {
            /* The face is gone, but a look beside a corner can lose it with the
             * corner just behind the car: go on a body and a gap along the face
             * before the turn, then look again. */
            flag_set(p_hug, F_PAST, true);
            p_hug->leg_mm = (int16_t)HUG_AXLE_OFF_MM;
            start_pivot(p_hug, along_head(p_hug), THEN_ADVANCE);
        }
        else
        {
            /* The face is gone: the corner is behind the car. The next face
             * lies across the old heading of the leg, on the side of the face. */
            set_next_face(p_hug);
            start_pivot(p_hug, p_hug->center, THEN_TURN);
        }
    }
}

/* A wheel stall is a face across the heading at the body radius, which the
 * sonar did not see. It has no edges, so it gives no width or breadth and the
 * leg is the short one of a face with no edge. The next look sees the real
 * faces at a better angle. The face is measured from the stall pose, where
 * the car stands. */
static void plan_assumed(hug_t * p_hug, bool b_first)
{
    int32_t const head = p_hug->center; /* the stall heading */
    /* The first stall backed off by the standoff before this look. */
    int32_t const ahead_mm = HUG_BODY_MM + (b_first ? HUG_STANDOFF_MM : 0);
    look_face_t face = {0, 0, 0, 0};
    hug_edges_t edges = {0};

    face.normal_deci = head;
    face.x_mm = pos_x_mm(p_hug) + geom_mul_q14(ahead_mm, geom_cos_q14(head));
    face.y_mm = pos_y_mm(p_hug) + geom_mul_q14(ahead_mm, geom_sin_q14(head));
    face.dist_mm = geom_dot_mm(head, face.x_mm - pos_x_mm(p_hug), face.y_mm - pos_y_mm(p_hug));
    if (b_first)
    {
        p_hug->width_mm = 0U;
        p_hug->side = pick_side(p_hug, &edges);
    }
    else
    {
        /* The way round on this side is blocked by a face the sonar does not
         * see, so the car goes round the other way. */
        p_hug->side = (int8_t)(-p_hug->side);
    }
    plan_leg(p_hug, &face, &edges, b_first, NULL);
    /* The look after the leg comes as after a turn: if the sonar does not see
     * the face either, the car goes on a body length before it turns back. */
    flag_set(p_hug, F_ASSUME | F_TURNED, true);
}

static void after_inner(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                        bool b_found)
{
    if (b_found
        && (geom_abs(geom_wrap_deci(p_face->normal_deci - p_hug->center)) <= HUG_INNER_ANGLE_DECI))
    {
        hug_edges_t edges;

        find_edges(p_look, p_face, p_hug, &edges);
        plan_leg(p_hug, p_face, &edges, false, p_look);
    }
    else if (flag_get(p_hug, F_STALL))
    {
        plan_assumed(p_hug, false);
    }
    else
    {
        p_hug->leg_mm = (int16_t)HUG_LEG_MIN_MM;
        start_pivot(p_hug, p_hug->trav_head, THEN_ADVANCE);
    }
}

/* Readings that are farther than the nearest one by more than the depth of
 * a face are what lies behind it, not the face: they count as OPEN. */
static void prune_far(look_t * p_look, bool b_check_cluster)
{
    uint16_t near_mm = UINT16_MAX;
    uint8_t near_count = 0U;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if ((LOOK_ECHO_OPEN != look_ray_echo(&p_look->rays[i])) && (p_look->rays[i].range_mm < near_mm))
        {
            near_mm = p_look->rays[i].range_mm;
        }
    }
    /* A narrow front can expose a deeper corner to one ray. Fit the nearby
     * surface when enough returns agree before discarding deeper points. */
    for (uint8_t i = 0U; b_check_cluster && (i < p_look->count); i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        if (look_echo_is_point(look_ray_echo(p_ray)) && (p_ray->range_mm <= ((int32_t)near_mm + LOOK_FACE_MATCH_MM)))
        {
            near_count++;
        }
    }
    int32_t const depth_mm = (near_count >= HUG_FACE_RAYS_MIN) ? (HALVES * LOOK_FACE_MATCH_MM) : HUG_BEHIND_MM;
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if ((LOOK_ECHO_OPEN != look_ray_echo(&p_look->rays[i]))
            && ((int32_t)p_look->rays[i].range_mm > ((int32_t)near_mm + depth_mm)))
        {
            look_ray_mark_open(&p_look->rays[i]);
        }
    }
}

/* The wide rays of the contact look are for the room beside the face. What
 * they read (a wall) is not the face, and it would pull the fit off it. */
static void drop_wide(look_t * p_look)
{
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if (geom_abs(look_ray_angle_deci(&p_look->rays[i])) > HUG_CONTACT_NEAR_DECI)
        {
            look_ray_mark_open(&p_look->rays[i]);
        }
    }
}

/* Face fitting drops distant echoes. A fresh return in the forward guard
 * still has to match the actual range recorded there before fitting. */
static void save_guard_ranges(look_t const * p_look, uint16_t ranges[RAY_CONTACT_N])
{
    for (uint8_t i = 0U; i < RAY_CONTACT_N; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];

        ranges[i] = ((LOOK_ECHO_VALID == look_ray_echo(p_ray)) && (p_ray->range_mm <= HUG_GUARD_MM))
                        ? p_ray->range_mm : 0U;
    }
}

static void restore_guard_ranges(look_t * p_look, uint16_t const ranges[RAY_CONTACT_N])
{
    for (uint8_t i = 0U; i < RAY_CONTACT_N; i++)
    {
        if (ranges[i] > 0U)
        {
            look_ray_t * const p_ray = &p_look->rays[i];

            p_ray->angle_echo = (uint16_t)((p_ray->angle_echo & LOOK_RAY_ANGLE_MASK) |
                                           (LOOK_ECHO_VALID << LOOK_RAY_ECHO_SHIFT));
            p_ray->range_mm = ranges[i];
        }
    }
}

/* A successor must meet the actual vertex, including when the two normals
 * are oblique. A parallel side offset from that point can hide a chamfer. */
static void after_edge_sweep(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                             bool b_found, hug_out_t * p_out)
{
    uint8_t const side = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const expected = geom_dot_mm(p_face->normal_deci, p_hug->keep_x_mm[side] - pos_x_mm(p_hug),
                                         p_hug->keep_y_mm[side] - pos_y_mm(p_hug));
    int32_t const angle = p_hug->side * geom_wrap_deci(p_face->normal_deci - p_hug->look_head);
    bool const b_mid = (PU_MID_LOOK == p_hug->purpose);
    bool const b_face = b_found && (valid_rays(p_look) >= HUG_FACE_RAYS_MIN);
    int32_t const min_angle = b_mid ? HUG_FACE_ANGLE_DECI + 1 : -HUG_FACE_ANGLE_DECI;
    int32_t const max_angle = (b_mid || p_hug->b_oblique) ? HUG_INNER_ANGLE_DECI : HUG_FACE_ANGLE_DECI;
    if (b_face && (angle >= min_angle) && (angle <= max_angle) &&
        (geom_abs(p_face->dist_mm - expected) <= HUG_REFINE_TOL_MM))
    {
        p_hug->b_oblique = p_hug->b_oblique || b_mid;
        if (p_hug->b_oblique)
        {
            p_hug->center = (int16_t)p_face->normal_deci;
        }
        after_look(p_hug, p_look, p_face, true, p_out);
    }
    else if (!b_mid && b_face && flag_get(p_hug, F_TURNED) &&
             (geom_abs(angle) <= HUG_FACE_ANGLE_DECI) && (p_face->dist_mm < expected - HUG_REFINE_TOL_MM))
    {
        /* The side is real, but misses the last endpoint. Check the return
         * corridor before taking a different view from its midpoint. */
        start_scan(p_hug, p_look, PU_FACET_CLEAR, geom_wrap_deci(p_hug->trav_head + HALVES * QUARTER_DECI));
    }
    else if (PU_EDGE_LOOK == p_hug->purpose)
    {
        look_along_face(p_hug, PU_EDGE_RETRY);
    }
    else
    {
        finish(p_hug);
        p_out->b_timeout = 1U;
    }
}

static void after_scan(hug_t * p_hug, look_t * p_look, hug_out_t * p_out)
{
    uint16_t guard_ranges[RAY_CONTACT_N];
    look_pose_t const pose = hug_axle_pose(p_hug);

    if (corner_scan(p_hug))
    {
        after_corner(p_hug, p_look, p_out);
        return;
    }
    if (PU_CHECK == p_hug->purpose)
    {
        after_side_check(p_hug, p_look);
        return;
    }
    if (PU_CONTACT == p_hug->purpose)
    {
        if (p_look->count == RAY_CONTACT_N)
        {
            save_guard_ranges(p_look, guard_ranges);
        }
        drop_wide(p_look);
    }
    prune_far(p_look, ((PU_CONTACT == p_hug->purpose) && (valid_rays(p_look) < HUG_NORMAL_RAYS_MIN)) ||
              (PU_REFINE == p_hug->purpose) || (PU_CENTER == p_hug->purpose) || edge_sweep(p_hug));
    look_face_t face = {0, 0, 0, 0};
    bool const b_found = look_face(p_look, &pose, &face);

    /* A face turned from the heading reads as an arc: rays within the beam of
     * the normal all read its foot. The first fit is a few degrees off, so the
     * rays are moved to the face again with each better normal. */
    for (uint32_t i = 0U; b_found && (PU_REFINE != p_hug->purpose) && (i < BEAM_PASSES); i++)
    {
        look_face_t next = face;

        if (fit_beam(p_hug, p_look, &next))
        {
            face = next;
        }
    }
    if (PU_CENTER == p_hug->purpose)
    {
        after_center(p_hug, p_look, &face, b_found);
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        after_refine(p_hug, p_look);
    }
    else if (PU_CONTACT == p_hug->purpose)
    {
        bool const b_seen = b_found && (valid_rays(p_look) >= HUG_FACE_RAYS_MIN);

        if (!flag_get(p_hug, F_BACKED) && flag_get(p_hug, F_STALL))
        {
            /* A stall is a face at range 0. */
            start_back(p_hug, 0);
        }
        else if (!flag_get(p_hug, F_BACKED) && b_seen && (face.dist_mm < HUG_AXLE_OFF_MM))
        {
            start_back(p_hug, face.dist_mm - LOOK_SONAR_FWD_MM);
        }
        else if (b_seen)
        {
            after_contact(p_hug, p_look, &face);
        }
        else if (flag_get(p_hug, F_STALL) && flag_get(p_hug, F_REPEAT))
        {
            plan_assumed(p_hug, true);
        }
        else
        {
            /* The hit is gone, or the stall was a block that went away: nothing
             * to go round. */
            if (flag_get(p_hug, F_STALL) && (p_hug->n_rel < UINT8_MAX))
            {
                p_hug->n_rel++;
            }
            if ((p_look->count == RAY_CONTACT_N) && (valid_rays(p_look) < HUG_FACE_RAYS_MIN))
            {
                restore_guard_ranges(p_look, guard_ranges);
                if (!flag_get(p_hug, F_STALL | F_BACKED))
                {
                    p_hug->recheck_hits = RECHECK_READY;
                }
            }
            release(p_hug);
        }
    }
    else if (PU_LOOK == p_hug->purpose)
    {
        after_look(p_hug, p_look, &face, b_found, p_out);
    }
    else if (edge_sweep(p_hug))
    {
        after_edge_sweep(p_hug, p_look, &face, b_found, p_out);
    }
    else
    {
        after_inner(p_hug, p_look, &face, b_found);
    }
}

/* ---- chaining ---- */

static void proceed(hug_t * p_hug, look_t * p_look)
{
    switch (p_hug->then)
    {
    case THEN_BACK:
        if (ST_PIVOT == p_hug->state)
        {
            start_drive(p_hug, p_hug->goal_len_mm, p_hug->goal_head, THEN_BACK);
        }
        else
        {
            start_scan(p_hug, p_look, p_hug->purpose, p_hug->center);
        }
        break;
    case THEN_SCAN:
        start_scan(p_hug, p_look, p_hug->purpose, p_hug->center);
        break;
    case THEN_APPROACH:
        start_drive(p_hug, p_hug->app_mm, p_hug->goal_head, THEN_ALIGN);
        break;
    case THEN_ALIGN:
        start_pivot(p_hug, p_hug->u_head, THEN_LEG);
        break;
    case THEN_LEG:
        p_hug->trav_head = p_hug->u_head;
        start_drive(p_hug, p_hug->leg_mm, p_hug->u_head, THEN_LOOK);
        break;
    case THEN_LOOK:
        if (p_hug->b_close_front && flag_get(p_hug, F_EDGE_LEG))
        {
            start_scan(p_hug, p_look, PU_CORNER,
                       geom_wrap_deci(p_hug->look_head + p_hug->side * HUG_FLANK_DECI));
        }
        else if (corner_scan(p_hug) && flag_get(p_hug, F_TURNED))
        {
            look_along_face(p_hug, (facet_clearance(p_hug) || p_hug->b_oblique) ? PU_MID_LOOK : PU_EDGE_LOOK);
        }
        else if (flag_get(p_hug, F_EDGE_LEG) && !flag_get(p_hug, F_TURNED) &&
                 (from_edge_mm(p_hug, along_head(p_hug)) >= HUG_AXLE_OFF_MM))
        {
            start_scan(p_hug, p_look, PU_CORNER, p_hug->look_head);
        }
        else if (flag_get(p_hug, F_MEASURED_FACE) && !flag_get(p_hug, F_PAST | F_TURNED)
                 && side_check_supported(p_hug, p_look))
        {
            start_scan(p_hug, p_look, PU_CHECK, p_hug->look_head);
        }
        else
        {
            look_along_face(p_hug, PU_LOOK);
        }
        flag_set(p_hug, F_EDGE_LEG, false);
        break;
    case THEN_FLANK_LOOK:
        p_hug->center = p_hug->look_head;
        set_next_face(p_hug);
        look_along_face(p_hug, PU_EDGE_LOOK);
        break;
    case THEN_TURN:
        start_drive(p_hug, p_hug->face_p_mm + HUG_TURN_FWD_MM, p_hug->goal_head, THEN_LOOK);
        break;
    case THEN_ADVANCE:
        start_drive(p_hug, p_hug->leg_mm, p_hug->goal_head, THEN_LOOK);
        break;
    case THEN_REAR_ALIGN:
        start_pivot(p_hug, 0, THEN_REAR_READY);
        break;
    case THEN_EXIT_TURN:
        /* Away from the obstacle, which is on the other side than the way round. */
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
        start_pivot(p_hug, geom_wrap_deci(p_hug->goal_head + (p_hug->side * LINE_SEARCH_SWEEP_DECI)),
                    THEN_RELEASE);
        break;
    default:
        finish(p_hug);
        break;
    }
}

/* Both wheel deltas are modular low-word readings. Bound them before
 * converting to signed travel or subtracting them for the heading. */
static bool near_survey_pose(hug_t const * p_hug, hug_in_t const * p_in)
{
    uint32_t const dl = (uint32_t)p_in->left_um - p_hug->origin.survey.left_um;
    uint32_t const dr = (uint32_t)p_in->right_um - p_hug->origin.survey.right_um;
    bool const b_near = ((dl + RECHECK_WHEEL_UM) <= (2U * RECHECK_WHEEL_UM)) &&
                        ((dr + RECHECK_WHEEL_UM) <= (2U * RECHECK_WHEEL_UM));

    return b_near && (geom_abs((int32_t)dr - (int32_t)dl) * DECI_PER_RAD <=
                      (HUG_HEAD_TOL_DECI * CHASSIS_WHEEL_TRACK_MM * UM_PER_MM));
}

static bool can_recheck(hug_t const * p_hug, look_t const * p_look, hug_in_t const * p_in)
{
    return (RECHECK_READY == p_hug->recheck_hits) && !p_in->b_stall && !p_in->b_relook &&
           !p_in->b_ping_reply && (0 == p_in->ping_servo_deci) &&
           (LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > 0U) &&
           (p_in->ping_mm <= HUG_STANDOFF_MM) &&
           ((p_in->now_ms - p_hug->t_ms) <= HUG_REPLY_TIMEOUT_MS) &&
           ((p_in->now_ms - p_hug->contact_scan_ms) <= HUG_NOTE_MS) &&
           near_survey_pose(p_hug, p_in) && prior_near_return(p_look, 0, p_in->ping_mm);
}

static void begin(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in)
{
    bool const b_recheck = can_recheck(p_hug, p_look, p_in);

    p_hug->b_bend_check = false;
    p_hug->recheck_hits = 0U;
    p_hug->last_motion_ms = p_in->now_ms;
    p_hug->x_um = 0;
    p_hug->b_require_rear = !p_hug->b_approach_tape;
    p_hug->b_can_resume = false;
    p_hug->b_remaining = false;
    p_hug->b_oblique = false;
    p_hug->b_close_front = false;
    p_hug->b_survey_displaced = false;
    p_hug->b_probe_front = false;
    p_hug->side_end_x_mm = 0;
    p_hug->side_end_y_mm = 0;
    p_hug->y_um = 0;
    p_hug->diff_um = 0;
    p_hug->last_left_um = (uint32_t)p_in->left_um;
    p_hug->last_right_um = (uint32_t)p_in->right_um;
    p_hug->width_mm = 0U;
    p_hug->breadth_mm = 0U;
    p_hug->seen_mm = 0;
    p_hug->flags = 0U;
    flag_set(p_hug, F_STALL, p_in->b_stall);
    flag_set(p_hug, F_REPEAT,
             p_in->b_stall &&
                 ((0U != p_hug->b_note) || (p_hug->n_rel >= HUG_RELEASES_MAX)));
    p_hug->b_note = p_in->b_stall ? 1U : 0U;
    p_hug->n_rel = p_in->b_stall ? p_hug->n_rel : 0U;
    p_hug->side = 1;
    p_hug->trav_head = 0;
    p_hug->look_head = 0;
    p_hug->lost = 0U;
    p_hug->purpose = (uint8_t)PU_CONTACT;
    p_hug->center = 0;
    flag_set(p_hug, F_RECHECK, b_recheck);
    if (!b_recheck)
    {
        p_hug->contact_scan_ms = p_in->now_ms;
        p_hug->origin.survey.left_um = (uint32_t)p_in->left_um;
        p_hug->origin.survey.right_um = (uint32_t)p_in->right_um;
        look_clear(p_look);
    }
    if (flag_get(p_hug, F_REPEAT))
    {
        /* The look at the stall pose would not change the back-off. */
        start_back(p_hug, 0);
    }
    else
    {
        start_pivot(p_hug, 0, THEN_SCAN);
    }
}

/* Tape under a sensor outside the line the car came on. The line may run at
 * any angle from here: it can turn under the obstacle. */
static void observe_exit_tape(hug_t * p_hug, hug_in_t const * p_in)
{
    bool const b_approach = (pos_x_mm(p_hug) <= HUG_EXIT_PAST_MM)
                            && (geom_abs(pos_y_mm(p_hug)) <= HUG_LINE_WINDOW_MM);

    if (b_approach || !(p_in->b_line_left || p_in->b_line_right))
    {
        return;
    }
    if (p_hug->b_require_rear && ((p_hug->side_end_x_mm <= 0) ||
        (pos_x_mm(p_hug) - p_hug->side_end_x_mm <= HUG_SWEEP_RADIUS_MM)))
    {
        return;
    }
    int32_t const head = hug_pose_heading_deci(p_hug);
    int32_t const left_mm = ((int32_t)p_in->b_line_left - (int32_t)p_in->b_line_right)
                            * (LINE_SENSOR_SPACING_MM / HALVES);
    int32_t const cos_head = geom_cos_q14(head);
    int32_t const sin_head = geom_sin_q14(head);
    int32_t const y_mm = pos_y_mm(p_hug) + geom_mul_q14(LINE_SENSOR_LEAD_MM, sin_head)
                         + geom_mul_q14(left_mm, cos_head);
    /* Two dark readers farther apart laterally than one stripe can be a
     * bent exit here. The normal optical alignment still has to confirm it. */
    bool const b_transverse = p_in->b_line_left && p_in->b_line_right &&
        (geom_abs(geom_mul_q14(LINE_SENSOR_SPACING_MM, cos_head)) > LINE_WIDTH_MM);
    if (!b_transverse && (geom_abs(geom_abs(y_mm) - HUG_PARALLEL_SPACING_MM) <= HUG_TAPE_TOL_MM))
    {
        return;
    }
    p_hug->tape.point.y_mm = y_mm;
    p_hug->tape.point.x_mm = pos_x_mm(p_hug) + geom_mul_q14(LINE_SENSOR_LEAD_MM, cos_head)
                             - geom_mul_q14(left_mm, sin_head);
    flag_set(p_hug, F_EXIT_TAPE, true);
}

/* Only shorten the forward guard for a measured, aligned straight leg.
 * Reserve the complete turning envelope, the working gap, and one settling
 * interval at leg speed beyond its endpoint. */
static bool side_leg_blocked(hug_in_t const * p_in, int32_t remaining_mm)
{
    int32_t const coast = (int32_t)HUG_SETTLE_MS * HUG_LEG_MM_S / (int32_t)MS_PER_S;
    /* Keep the head-on stopping reserve independent of the side working gap. */
    int32_t const clearance = remaining_mm + HUG_SWEEP_RADIUS_MM - LOOK_SONAR_FWD_MM
                              + HUG_FORWARD_RESERVE_MM + coast;
    return (LOOK_ECHO_VALID != p_in->ping_echo) || (0 != p_in->ping_servo_deci) ||
           ((int32_t)p_in->ping_mm <= clearance);
}

static void drive_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       bool b_guard, hug_out_t * p_out)
{
    if (THEN_FLANK_LOOK == p_hug->then)
    {
        int32_t const side = from_edge_mm(p_hug, along_head(p_hug));
        bool const b_pivot_clear = side >= HUG_SCAN_CLEARANCE_MM;
        if (p_in->b_stall || b_guard || (!b_pivot_clear && (0 == drive_remaining_mm(p_hug))))
        {
            /* A turn to investigate here could swing the nose into the post. */
            finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else if (b_pivot_clear && p_in->b_stationary && (SUB_SETTLE == p_hug->sub) &&
                 (p_in->now_ms - p_hug->t_ms >= HUG_SETTLE_MS))
        {
            proceed(p_hug, p_look);
        }
        else if (b_pivot_clear)
        {
            if (SUB_SETTLE != p_hug->sub)
            {
                p_hug->sub = (uint8_t)SUB_SETTLE;
                p_hug->t_ms = p_in->now_ms;
            }
        }
        else
        {
            p_hug->sub = (uint8_t)SUB_MOVE;
            (void)drive_cmd(p_hug, HUG_SHORT_MM_S, p_out);
        }
        return;
    }
    bool const b_leg = drive_is_leg(p_hug);
    bool const b_blocked_leg = b_leg && b_guard &&
        (!flag_get(p_hug, F_MEASURED_FACE) ||
         (geom_abs(geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug))) > HUG_HEAD_TOL_DECI) ||
         side_leg_blocked(p_in, drive_remaining_mm(p_hug)));
    if (p_hug->b_probe_front && (p_in->b_stall || b_blocked_leg))
    {
        finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else if (p_in->b_stall || b_blocked_leg || (b_guard &&
        ((THEN_EXIT_TURN == p_hug->then) || (THEN_REAR_ALIGN == p_hug->then))))
    {
        flag_set(p_hug, F_STALL, p_in->b_stall);
        flag_set(p_hug, F_EDGE_LEG | F_EXIT | F_EXIT_TAPE, false);
        start_scan(p_hug, p_look, PU_INNER, hug_pose_heading_deci(p_hug));
    }
    else if (SUB_SETTLE == p_hug->sub)
    {
        if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
        {
            proceed(p_hug, p_look);
        }
    }
    else if ((THEN_EXIT_TURN != p_hug->then) && flag_get(p_hug, F_EXIT_TAPE))
    {
        int32_t const head = hug_pose_heading_deci(p_hug);
        /* A sensor can find the tape while the chassis is still turning.
         * The crossing distance belongs to that measured point. */
        int32_t const advance = geom_dot_mm(head, p_hug->tape.point.x_mm - pos_x_mm(p_hug),
                                           p_hug->tape.point.y_mm - pos_y_mm(p_hug));
        flag_set(p_hug, F_EXIT, true);
        flag_set(p_hug, F_EXIT_TAPE, false);
        start_drive(p_hug, clamp_i32(advance, 0, LINE_SENSOR_LEAD_MM), head, THEN_EXIT_TURN);
    }
    else if (drive_cmd(p_hug, drive_is_leg(p_hug) ? HUG_LEG_MM_S : HUG_SHORT_MM_S, p_out))
    {
        p_hug->sub = (uint8_t)SUB_SETTLE;
        p_hug->t_ms = p_in->now_ms;
    }
    else
    {
        /* Driving. */
    }
}

/* A single detector must sweep across a measurable part of the stripe.
 * Both dark can be a broad patch. Both clear arms the next crossing. */
static bool exit_crossed(hug_t * p_hug, hug_in_t const * p_in)
{
    uint8_t const reader = (uint8_t)((p_in->b_line_left ? EXIT_LEFT : 0U) |
                                     (p_in->b_line_right ? EXIT_RIGHT : 0U));
    uint8_t const leading = (p_hug->side > 0) ? (uint8_t)EXIT_LEFT : (uint8_t)EXIT_RIGHT;
    int32_t const head = hug_pose_heading_deci(p_hug);
    if ((EXIT_CLEAR == reader) && (leading == p_hug->tape.crossing.reader))
    {
        int32_t const sweep = p_hug->side * geom_wrap_deci(head - p_hug->tape.crossing.heading_deci);
        int32_t const arc = sweep * LINE_SENSOR_LEAD_MM;
        p_hug->tape.crossing.reader = (uint8_t)EXIT_CLEAR;
        return (arc * HALVES >= LINE_WIDTH_MM * DECI_PER_RAD) &&
               (arc <= HALVES * LINE_WIDTH_MM * DECI_PER_RAD);
    }
    if (EXIT_CLEAR == reader)
    {
        p_hug->tape.crossing.reader = reader;
    }
    else if (leading != reader)
    {
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
    }
    else if ((reader != p_hug->tape.crossing.reader) && (EXIT_WAIT_CLEAR != p_hug->tape.crossing.reader))
    {
        p_hug->tape.crossing.reader = reader;
        p_hug->tape.crossing.heading_deci = (int16_t)head;
    }
    return false;
}

static bool exit_settled(hug_t const * p_hug)
{
    int32_t const dx = pos_x_mm(p_hug) - p_hug->origin.drive.x_mm;
    int32_t const dy = pos_y_mm(p_hug) - p_hug->origin.drive.y_mm;
    int32_t const angle = geom_abs(geom_wrap_deci(hug_pose_heading_deci(p_hug) - p_hug->tape.crossing.heading_deci));
    return (geom_abs(dx) + geom_abs(dy) <= LINE_WIDTH_MM / HALVES) &&
           (angle * LINE_SENSOR_LEAD_MM <= LINE_WIDTH_MM * DECI_PER_RAD);
}

/* Check fresh centered sonar after the wheels settle. A blocked or missing
 * result cannot transfer the wheels to the line search. */
static void rear_settled(hug_t * p_hug, hug_in_t const * p_in, bool b_forward, hug_out_t * p_out)
{
    bool const b_center = THEN_REAR_LATERAL == p_hug->then;
    int32_t const y = geom_abs(pos_y_mm(p_hug));
    int32_t const min_range = b_center ? y + LINE_SENSOR_LEAD_MM - LOOK_SONAR_FWD_MM : HUG_GUARD_MM;
    bool const b_ping = b_forward && (0 == p_in->ping_servo_deci);
    bool const b_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
                         ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > min_range));
    bool const b_blocked = (LOOK_ECHO_TOO_CLOSE == p_in->ping_echo) ||
        ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > 0U) && !b_clear);
    if ((p_in->now_ms - p_hug->t_ms >= HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS) || (b_ping && b_blocked))
    {
        finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else if (b_ping && b_clear && p_in->b_stationary)
    {
        if (geom_abs(geom_wrap_deci(hug_pose_heading_deci(p_hug) - p_hug->goal_head)) > HUG_HEAD_TOL_DECI)
        {
            p_hug->sub = (uint8_t)SUB_START;
        }
        else if (b_center)
        {
            start_drive(p_hug, y, p_hug->goal_head, THEN_REAR_ALIGN);
        }
        else
        {
            finish(p_hug);
            p_out->b_no_tape = y <= (LINE_WIDTH_MM / HALVES);
            p_out->b_timeout = (uint8_t)!p_out->b_no_tape;
            p_hug->b_can_resume = p_out->b_no_tape && p_hug->b_approach_tape;
            p_hug->b_approach_tape = false;
        }
    }
    else
    {
        /* Stand still until a centered reading arrives. */
    }
}

static void settle_optical_exit(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    if (!exit_settled(p_hug) || (p_in->b_line_left && p_in->b_line_right))
    {
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
        p_hug->sub = (uint8_t)SUB_START;
    }
    else
    {
        /* The leading detector crossed the stripe and the axle settled
         * within its measured footprint. Emit the transfer once. */
        finish(p_hug);
        p_out->b_rejoined = true;
    }
}

/* A persistent small target may never supply a face normal from here.
 * Check both wide rays on the chosen side and measure one short step.
 * The normal leg admission checks the centered beam again after the pivot. */
static bool probe_sparse_front(hug_t * p_hug, look_t const * p_look, int32_t range_mm)
{
    if ((range_mm <= 0) || flag_get(p_hug, F_RECHECK | F_STALL) ||
        !prior_near_return(p_look, 0, range_mm))
    {
        return false;
    }
    int32_t const room_mm = HUG_SCAN_CLEARANCE_MM + HUG_LEG_MIN_MM;
    bool const b_left = !p_hug->b_left_scan_gap && (0U != p_hug->open_mm[LEFT]) &&
        ((0U == p_hug->wall_mm[LEFT]) || (p_hug->wall_mm[LEFT] > room_mm));
    bool const b_right = !p_hug->b_right_scan_gap && (0U != p_hug->open_mm[RIGHT]) &&
        ((0U == p_hug->wall_mm[RIGHT]) || (p_hug->wall_mm[RIGHT] > room_mm));
    if (!b_left && !b_right)
    {
        return false;
    }
    if (range_mm + LOOK_SONAR_FWD_MM < HUG_AXLE_OFF_MM)
    {
        if (flag_get(p_hug, F_BACKED))
        {
            return false;
        }
        start_back(p_hug, range_mm);
    }
    else
    {
        bool const b_prefer_left = (p_hug->open_mm[LEFT] >= p_hug->open_mm[RIGHT]);
        p_hug->side = (b_left && (!b_right || b_prefer_left)) ? (int8_t)1 : (int8_t)-1;
        p_hug->b_probe_front = true;
        p_hug->b_probe_retried = false;
        p_hug->face_p_mm = (int16_t)(LOOK_SONAR_FWD_MM + range_mm);
        p_hug->look_head = (int16_t)geom_atan_deci(-p_hug->side * HUG_LEG_MIN_MM, p_hug->face_p_mm);
        p_hug->leg_mm = HUG_LEG_MIN_MM;
        p_hug->u_head = (int16_t)(p_hug->side * QUARTER_DECI);
        start_pivot(p_hug, p_hug->u_head, THEN_LEG);
    }
    return true;
}

static void settle_contact(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, bool b_forward)
{
    if ((p_in->now_ms - p_hug->t_ms) >= (HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS))
    {
        restart_contact(p_hug, p_look);
    }
    else if (b_forward && p_in->b_stationary && (0 == p_in->ping_servo_deci) &&
             (geom_abs(hug_pose_heading_deci(p_hug)) <= HUG_HEAD_TOL_DECI))
    {
        if ((LOOK_ECHO_OPEN == p_in->ping_echo) ||
            ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > HUG_GUARD_MM)))
        {
            proceed(p_hug, p_look);
        }
        else if ((LOOK_ECHO_TOO_CLOSE == p_in->ping_echo) ||
                 ((LOOK_ECHO_VALID == p_in->ping_echo) && (0U != p_in->ping_mm) &&
                  !probe_sparse_front(p_hug, p_look, p_in->ping_mm)))
        {
            restart_contact(p_hug, p_look);
        }
        else
        {
            /* An unresolved echo can still be replaced inside this reply window. */
        }
    }
    else
    {
        /* Only the settled beam on the approach heading can release this contact. */
    }
}

/* A side-looking ping from the survey cannot clear the forward leg. */
static void settle_leg(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       bool b_forward, bool b_guard)
{
    int32_t const error = geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug));
    bool const b_aligned = (geom_abs(error) <= HUG_HEAD_TOL_DECI);
    bool const b_ready = b_aligned && b_forward && p_in->b_stationary && (0 == p_in->ping_servo_deci);
    bool const b_blocked = b_ready && b_guard &&
        (!flag_get(p_hug, F_MEASURED_FACE) || side_leg_blocked(p_in, p_hug->leg_mm));
    if (b_blocked || ((p_in->now_ms - p_hug->t_ms) >= (HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS)))
    {
        if (p_hug->b_probe_front)
        {
            p_hug->b_probe_front = false;
            restart_contact(p_hug, p_look);
        }
        else if (b_blocked)
        {
            start_scan(p_hug, p_look, PU_INNER, hug_pose_heading_deci(p_hug));
        }
        else
        {
            look_along_face(p_hug, PU_LOOK);
        }
    }
    else if (b_ready && ((LOOK_ECHO_OPEN == p_in->ping_echo) ||
             ((LOOK_ECHO_VALID == p_in->ping_echo) && (0U != p_in->ping_mm))))
    {
        proceed(p_hug, p_look);
    }
    else
    {
        /* Still settling or waiting for an observation in the direction of travel. */
    }
}

static void pivot_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       bool b_forward, bool b_guard, hug_out_t * p_out)
{
    if (SUB_MOVE == p_hug->sub)
    {
        bool b_done = pivot_cmd(p_hug, p_hug->goal_head, p_out);

        if (flag_get(p_hug, F_EXIT))
        {
            /* Keep the braking travel inside the tape that the optical sweep measures. */
            p_out->turn_deci_s = (int16_t)clamp_i32(p_out->turn_deci_s, -HUG_PIVOT_MIN_DECI_S,
                                                   HUG_PIVOT_MIN_DECI_S);
        }
        /* The line may cross the way out at any angle, and the follower needs
         * a sensor on it: the turn stops when one sees it. */
        if (flag_get(p_hug, F_EXIT) && exit_crossed(p_hug, p_in))
        {
            p_hug->tape.crossing.heading_deci = (int16_t)hug_pose_heading_deci(p_hug);
            p_hug->origin.drive.x_mm = pos_x_mm(p_hug);
            p_hug->origin.drive.y_mm = pos_y_mm(p_hug);
            p_out->turn_deci_s = 0;
            b_done = true;
        }
        else if (b_done && flag_get(p_hug, F_EXIT))
        {
            finish(p_hug);
            p_out->b_timeout = 1U;
            return;
        }
        if (b_done)
        {
            p_hug->sub = (uint8_t)SUB_SETTLE;
            p_hug->t_ms = p_in->now_ms;
        }
        else
        {
            p_out->b_scan_pivot = THEN_SCAN == p_hug->then;
        }
    }
    else if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
    {
        if (flag_get(p_hug, F_EXIT))
        {
            settle_optical_exit(p_hug, p_in, p_out);
        }
        else if ((THEN_REAR_LATERAL == p_hug->then) || (THEN_REAR_READY == p_hug->then))
        {
            rear_settled(p_hug, p_in, b_forward, p_out);
        }
        else if (((THEN_RELEASE == p_hug->then) || (THEN_LEG == p_hug->then)) &&
                 p_in->b_stationary && !p_hug->b_pivot_reaimed &&
                 (p_in->now_ms - p_hug->t_ms < HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS) &&
                 (geom_abs(geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug))) > HUG_HEAD_TOL_DECI))
        {
            p_hug->b_pivot_reaimed = true;
            p_hug->sub = (uint8_t)SUB_START;
        }
        else if (THEN_RELEASE == p_hug->then)
        {
            settle_contact(p_hug, p_look, p_in, b_forward);
        }
        else if (THEN_LEG != p_hug->then)
        {
            proceed(p_hug, p_look);
        }
        else
        {
            settle_leg(p_hug, p_look, p_in, b_forward, b_guard);
        }
    }
    else
    {
        /* Standing still. */
    }
}

/* The time the move that starts now may take: HUG_MOVE_TIMES times its plan
 * (hug.h). */
static uint32_t move_limit_ms(hug_t const * p_hug)
{
    uint32_t plan_ms = 0U;

    if (ST_DRIVE == p_hug->state)
    {
        int32_t const speed = drive_is_leg(p_hug) ? HUG_LEG_MM_S : HUG_SHORT_MM_S;

        plan_ms = ((uint32_t)geom_abs(p_hug->goal_len_mm) * MS_PER_S) / (uint32_t)speed;
    }
    else
    {
        int32_t const target = (ST_SCAN == p_hug->state)
                                   ? (p_hug->center
                                      + scan_body_aim(p_hug, ray_offset(p_hug, p_hug->idx)))
                                   : p_hug->goal_head;
        uint32_t const angle = (uint32_t)geom_abs(geom_wrap_deci(target - hug_pose_heading_deci(p_hug)));

        plan_ms = (angle * MS_PER_S * (uint32_t)PERCENT)
                  / ((uint32_t)HUG_PIVOT_MIN_DECI_S * (uint32_t)HUG_PIVOT_PCT);
    }
    return HUG_MOVE_TIMES * ((plan_ms < HUG_MOVE_FLOOR_MS) ? HUG_MOVE_FLOOR_MS : plan_ms);
}

/* A sweep or cached sighting belongs to the motion owner that observed it. */
static void interrupt_observations(hug_t * p_hug, look_t * p_look, bool b_contact_backoff)
{
    if (p_hug->b_probe_front && (ST_IDLE != p_hug->state))
    {
        p_hug->b_survey_displaced = true;
    }
    if (!b_contact_backoff)
    {
        if (edge_sweep(p_hug) && ((ST_SCAN == p_hug->state) ||
            ((ST_PIVOT == p_hug->state) && (THEN_SCAN == p_hug->then))))
        {
            /* A wheel takeover can displace the chassis even when its
             * encoder position is unchanged. Discard both partial views. */
            look_clear(p_look);
            p_hug->b_survey_displaced = true;
        }
        hug_forget_approach(p_hug);
    }
    p_hug->b_can_resume = false;
    p_hug->side_end_x_mm = 0;
    p_hug->side_end_y_mm = 0;
    if ((ST_DRIVE == p_hug->state) && (THEN_FLANK_LOOK == p_hug->then))
    {
        p_hug->b_survey_displaced = true;
    }
    else if ((ST_IDLE != p_hug->state) && (p_hug->then >= THEN_REAR_LATERAL))
    {
        look_along_face(p_hug, PU_LOOK);
    }
    p_hug->recheck_hits = 0U;
    flag_set(p_hug, F_EXIT_TAPE, false);
    if ((ST_PIVOT == p_hug->state) && flag_get(p_hug, F_EXIT))
    {
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
        p_hug->sub = (uint8_t)SUB_START;
    }
    if (((ST_SCAN == p_hug->state) && ((PU_CONTACT == p_hug->purpose) || (PU_CENTER == p_hug->purpose))) ||
        ((ST_IDLE != p_hug->state) && flag_get(p_hug, F_RECHECK)))
    {
        restart_contact(p_hug, p_look);
    }
    if (ST_IDLE != p_hug->state)
    {
        bool const b_probe_leg = ((PU_CHECK == p_hug->purpose) && (ST_PIVOT == p_hug->state) &&
                                  (THEN_LEG == p_hug->then)) ||
            (probe_scan(p_hug) && (ST_DRIVE == p_hug->state) && (THEN_LOOK == p_hug->then));

        flag_set(p_hug, F_EDGE_LEG | F_MEASURED_FACE, false);
        if (((ST_SCAN == p_hug->state) && probe_scan(p_hug)) || b_probe_leg)
        {
            look_along_face(p_hug, PU_LOOK);
        }
        else if (SUB_MOVE == p_hug->sub)
        {
            /* The clock of the move starts again when the wheels come back. */
            p_hug->sub = (uint8_t)SUB_START;
        }
    }
}

static void observe_approach_tape(hug_t * p_hug, hug_in_t const * p_in)
{
    if (!p_in->b_other && (p_in->b_line_left || p_in->b_line_right))
    {
        p_hug->b_approach_tape = true;
        p_hug->tape.idle_tape_ms = p_in->now_ms;
    }
    else if (p_hug->b_approach_tape && (p_in->now_ms - p_hug->tape.idle_tape_ms > HUG_NOTE_MS))
    {
        p_hug->b_approach_tape = false;
    }
}

void hug_begin_bypass(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, uint16_t range_mm)
{
    begin(p_hug, p_look, p_in);
    look_clear(p_look);
    bypass_begin(p_hug, p_in, range_mm);
}

static void observe_idle(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, bool b_forward)
{
    bool const b_rechecking = recheck_on_bend(p_hug, p_in, b_forward);
    observe_approach_tape(p_hug, p_in);
    /* A stall with the follower driving is a contact: the sonar did not see
     * the face, so look from the backed-off pose. */
    if (!p_in->b_other && ((!b_rechecking && triggered(p_hug)) || p_in->b_stall || p_in->b_relook))
    {
        uint16_t range = (!p_in->b_ping_reply && (0 == p_in->ping_servo_deci) &&
            look_echo_is_point(p_in->ping_echo) && (p_in->ping_mm <= HUG_STANDOFF_MM))
            ? p_in->ping_mm : 0U;
        if (!p_in->b_ping_reply && (p_in->ping_servo_deci == 0) && (p_in->ping_echo == LOOK_ECHO_TOO_CLOSE))
        {
            range = OBSTACLE_PING_TOO_CLOSE_MM;
        }
        if ((range == 0U) && !p_in->b_stall && !p_in->b_relook)
        {
            return;
        }
        if (p_in->b_relook)
        {
            /* A post-recovery contact may be on a side ray while the
             * latest centre ping is open. Save it before begin resets the look. */
            for (uint8_t i = 0U; i < p_look->count; i++)
            {
                look_ray_t const * const p_ray = &p_look->rays[i];
                if (look_echo_is_point(look_ray_echo(p_ray)) && p_ray->range_mm <= HUG_STANDOFF_MM &&
                    (range == 0U || p_ray->range_mm < range))
                {
                    range = p_ray->range_mm;
                }
            }
        }
        if (range != 0U)
        {
            hug_begin_bypass(p_hug, p_look, p_in, range);
        }
        else
        {
            begin(p_hug, p_look, p_in);
        }
    }
}

bool hug_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bool const b_was_active = (ST_IDLE != p_hug->state);
    bool b_guard = false;
    bool b_forward = false;
    bool b_takes = false;

    zero_out(p_out);
    if (ST_IDLE != p_hug->state)
    {
        odo_update(p_hug, p_in);
        if ((ST_SCAN == p_hug->state) && (p_hug->lost >= HUG_LOST_REPLIES))
        {
            /* The sonar-fault stop lasts until replies recover. */
            p_hug->last_motion_ms = p_in->now_ms;
        }
    }
    else if ((0U != p_hug->b_note) || (0U != p_hug->n_rel))
    {
        note_update(p_hug, p_in);
    }
    else
    {
        /* Nothing to track while idle. */
    }
    if (p_hug->bypass.phase != BYPASS_IDLE)
    {
        bypass_step(p_hug, p_look, p_in, p_out);
        return !p_in->b_other && (ST_IDLE != p_hug->state);
    }
    if (p_in->b_other)
    {
        interrupt_observations(p_hug, p_look, p_in->b_contact_backoff);
    }
    else if ((ST_IDLE != p_hug->state) && flag_get(p_hug, F_RECHECK) &&
        (((p_in->now_ms - p_hug->contact_scan_ms) > HUG_NOTE_MS) ||
         !near_survey_pose(p_hug, p_in)))
    {
        restart_contact(p_hug, p_look);
    }
    b_forward = forward_ping(p_hug, p_in, &b_guard);
    if (ST_IDLE == p_hug->state)
    {
        observe_idle(p_hug, p_look, p_in, b_forward);
    }
    if (p_hug->bypass.phase != BYPASS_IDLE)
    {
        bypass_step(p_hug, p_look, p_in, p_out);
        return true;
    }
    if (!p_in->b_other && (ST_IDLE != p_hug->state))
    {
        b_takes = true;
        if (!flag_get(p_hug, F_EXIT) && (THEN_FLANK_LOOK != p_hug->then) &&
            ((ST_DRIVE != p_hug->state) || (SUB_SETTLE != p_hug->sub)))
        {
            observe_exit_tape(p_hug, p_in);
        }
        if (SUB_START == p_hug->sub)
        {
            p_hug->t_ms = p_in->now_ms + move_limit_ms(p_hug);
            p_hug->sub = (uint8_t)SUB_MOVE;
        }
        /* The difference is right across the wrap of the clock. */
        /* An incomplete contour cannot keep circulating if a surface is
         * lost. This odometric bound is a stop, never an inferred exit. */
        if (p_hug->b_remaining && (geom_abs(p_hug->diff_um) / UM_PER_MM >
            ((REMAINING_LIMIT_DECI * CHASSIS_WHEEL_TRACK_MM) / DECI_PER_RAD)))
        {
            finish(p_hug);
            p_out->b_no_exit = true;
        }
        else if (p_hug->b_survey_displaced ||
                 ((p_in->now_ms - p_hug->last_motion_ms) >= HUG_STATIONARY_LIMIT_MS) ||
                 ((SUB_MOVE == p_hug->sub) && ((int32_t)(p_in->now_ms - p_hug->t_ms) > 0)))
        {
            /* A displaced survey or an expired movement ends under a hold. */
            finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else if (ST_SCAN == p_hug->state)
        {
            scan_tick(p_hug, p_look, p_in, p_out);
        }
        else if (ST_PIVOT == p_hug->state)
        {
            pivot_tick(p_hug, p_look, p_in, b_forward, b_guard, p_out);
        }
        else
        {
            drive_tick(p_hug, p_look, p_in, b_guard, p_out);
        }
        b_takes = (ST_IDLE != p_hug->state);
    }
    if (b_was_active && (ST_IDLE == p_hug->state))
    {
        /* The note and the releases run from here. */
        p_hug->t_ms = p_in->now_ms;
    }
    return b_takes;
}

/*** end of file ***/
