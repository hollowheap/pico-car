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

#include "geometry.h"
#include "platform/chassis.h"
#include "platform/units.h"
#include "track/line_control.h"

#define PERCENT         (100)
#define HALVES          (2)
#define QUARTER_DECI    (900)
#define ODD_SEQ         (0xFFU)
#define SEQ_MASK        (0x3FU)
#define HIST_BITS       (3U)
#define HIST_MASK       (0x07U)
#define HOP_LIMIT_MM    (400)
#define RAY_NEAR_N      (((2U * HUG_NEAR_ZONE_DECI) / HUG_NEAR_STEP_DECI) + 1U)
#define RAY_CONTACT_N   ((uint32_t)(sizeof(g_contact_deci) / sizeof(g_contact_deci[0])))
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

/* Why the car looks. */
#define PU_CONTACT      (0U)
#define PU_LOOK         (1U)
#define PU_INNER        (2U)
#define PU_REFINE       (3U)

#define F_PAST          (0x01U) /* the leg ended the body and the gap past the corner     */
#define F_SWEPT         (0x02U) /* a sensor crossed tape outside the approach line in a pivot */
#define F_EXIT          (0x04U) /* the tape ended the hug: the turn out stops at the tape */
#define F_TURNED        (0x08U) /* the last motion was the turn round a corner    */
#define F_REFINE        (0x10U) /* the finer looks of a face whose edges are known */
#define F_WIN2          (0x20U) /* the second finer look adds to the first        */
#define F_STALL         (0x40U) /* the hit or the leg ended in a wheel stall      */
#define F_ASSUME        (0x80U) /* the face in use is the one assumed at a stall  */
#define F_BACKED        (0x100U) /* the first contact has backed off to the standoff */
#define F_REPEAT        (0x200U) /* a stall at the place of the last one, which may be a wall */
#define F_AIMED         (0x800U) /* the pivot before the look has stopped at its first body step and settled */

#define LEFT            ((uint8_t)LOOK_SIDE_LEFT)
#define RIGHT           ((uint8_t)LOOK_SIDE_RIGHT)

STATIC_ASSERT(RAY_REFINE_N == RAY_REFINE_FAR_N, "both refine windows hold the same rays");
STATIC_ASSERT((2U * RAY_REFINE_N) <= LOOK_RAYS_MAX, "two refine windows fit the look");
STATIC_ASSERT(RAY_NEAR_N <= LOOK_RAYS_MAX, "a look fits the look");

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
static int32_t head_deci(hug_t const * p_hug)
{
    int64_t const turn = ((int64_t)p_hug->diff_um * DECI_PER_RAD)
                         / ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM);

    return geom_wrap_deci((int32_t)turn);
}

static int32_t pos_x_mm(hug_t const * p_hug)
{
    return p_hug->x_um / UM_PER_MM;
}

static int32_t pos_y_mm(hug_t const * p_hug)
{
    return p_hug->y_um / UM_PER_MM;
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
}

/* Wheel travel to pose. Wheels that run opposite turn the body less than
 * their travel says. */
static void odo_update(hug_t * p_hug, hug_in_t const * p_in)
{
    int32_t const dl = (int32_t)((uint32_t)p_in->left_um - p_hug->last_left_um);
    int32_t const dr = (int32_t)((uint32_t)p_in->right_um - p_hug->last_right_um);
    int32_t const ds = (dl + dr) / HALVES;
    int32_t dd = dr - dl;
    int32_t const before = head_deci(p_hug);
    int32_t mid = 0;

    p_hug->last_left_um = (uint32_t)p_in->left_um;
    p_hug->last_right_um = (uint32_t)p_in->right_um;
    if (((dl > 0) && (dr < 0)) || ((dl < 0) && (dr > 0)))
    {
        dd = (dd * HUG_PIVOT_PCT) / PERCENT;
    }
    p_hug->diff_um += dd;
    mid = before + (geom_wrap_deci(head_deci(p_hug) - before) / HALVES);
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

static void forward_ping(hug_t * p_hug, hug_in_t const * p_in, bool * p_b_guard)
{
    bool b_hit = false;

    *p_b_guard = false;
    if ((!p_in->b_ping_reply) && (p_in->ping_seq != p_hug->last_seq))
    {
        bool const b_valid = look_echo_is_point(p_in->ping_echo) && (p_in->ping_mm > 0U);

        p_hug->last_seq = p_in->ping_seq;
        b_hit = b_valid && (p_in->ping_mm <= (uint16_t)HUG_STANDOFF_MM);
        *p_b_guard = b_valid && (p_in->ping_mm <= (uint16_t)HUG_GUARD_MM);
        p_hug->hist = (uint8_t)(((uint32_t)p_hug->hist << 1U) | (b_hit ? 1U : 0U));
        p_hug->hist = (uint8_t)((uint32_t)p_hug->hist & HIST_MASK);
    }
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

bool hug_active(hug_t const * p_hug)
{
    return ST_IDLE != p_hug->state;
}

int16_t hug_heading_deci(hug_t const * p_hug)
{
    /* Cast is safe: the heading is wrapped to -1800..1800. */
    return (ST_IDLE == p_hug->state) ? (int16_t)0 : (int16_t)head_deci(p_hug);
}

/* ---- motions ---- */

static void start_pivot(hug_t * p_hug, int32_t head, uint32_t then)
{
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
    p_hug->x0_mm = pos_x_mm(p_hug);
    p_hug->y0_mm = pos_y_mm(p_hug);
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
    p_hug->sub = (uint8_t)SUB_START;
    if (!flag_get(p_hug, F_WIN2))
    {
        look_clear(p_look);
    }
    if (PU_REFINE != purpose)
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
    int32_t const err = geom_wrap_deci(target - head_deci(p_hug));
    bool const b_done = geom_abs(err) <= HUG_HEAD_TOL_DECI;

    if (!b_done)
    {
        int32_t const rate = clamp_i32(geom_abs(err) * HUG_PIVOT_GAIN,
                                       HUG_PIVOT_MIN_DECI_S, HUG_PIVOT_MAX_DECI_S);

        p_out->turn_deci_s = (int16_t)((err > 0) ? rate : -rate);
    }
    return b_done;
}

static bool drive_cmd(hug_t const * p_hug, int32_t speed_mm_s, hug_out_t * p_out)
{
    int32_t const dx = pos_x_mm(p_hug) - p_hug->x0_mm;
    int32_t const dy = pos_y_mm(p_hug) - p_hug->y0_mm;
    int32_t const len = geom_abs(p_hug->goal_len_mm);
    bool const b_done = (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy))) >= len;

    if (!b_done)
    {
        int32_t const err = geom_wrap_deci(p_hug->goal_head - head_deci(p_hug));

        p_out->speed_mm_s = (int16_t)((p_hug->goal_len_mm < 0) ? -speed_mm_s : speed_mm_s);
        p_out->turn_deci_s = (int16_t)clamp_i32(err * HUG_HOLD_GAIN, -HUG_HOLD_MAX_DECI_S,
                                                HUG_HOLD_MAX_DECI_S);
    }
    return b_done;
}

/* ---- scan ---- */

static uint32_t ray_count(hug_t const * p_hug)
{
    uint32_t n = RAY_NEAR_N;

    if (PU_CONTACT == p_hug->purpose)
    {
        n = RAY_CONTACT_N;
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        n = RAY_REFINE_N;
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

    if (PU_REFINE == p_hug->purpose)
    {
        bool const b_far = (p_hug->keep_dist_mm - LOOK_SONAR_FWD_MM) > HUG_FAR_MM;
        int32_t const zone = b_far ? HUG_REFINE_FAR_ZONE_DECI : HUG_REFINE_ZONE_DECI;

        offset = ((int32_t)i * ((HALVES * zone) / ((int32_t)RAY_REFINE_N - 1))) - zone;
    }
    else if (PU_CONTACT != p_hug->purpose)
    {
        offset = ((int32_t)i * HUG_NEAR_STEP_DECI) - HUG_NEAR_ZONE_DECI;
        /* The sweep ends on the side the next leg goes to, which is the
         * shorter turn to it. */
        offset = ((PU_LOOK == p_hug->purpose) && (p_hug->side < 0)) ? -offset : offset;
    }
    else
    {
        offset = g_contact_deci[i];
    }
    return offset;
}

static uint8_t body_step_of(int32_t offset)
{
    int32_t const q = (offset + HUG_SERVO_MAX_DECI + (BODY_BIAS * HUG_PIVOT_STEP_DECI))
                      / HUG_PIVOT_STEP_DECI;

    return (uint8_t)q;
}

/* Body step of ray offset: the multiple of HUG_PIVOT_STEP_DECI that leaves the
 * servo within its reach. Returned as a step number plus a bias. */
static uint8_t body_step(hug_t const * p_hug, int32_t offset)
{
    if (PU_REFINE == p_hug->purpose)
    {
        /* The servo alone covers the window: every ray from one place. */
        return (uint8_t)BODY_BIAS;
    }
    return body_step_of(offset);
}

static int32_t body_deci(uint8_t step)
{
    return ((int32_t)step - BODY_BIAS) * HUG_PIVOT_STEP_DECI;
}

static void scan_record(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                        int32_t offset)
{
    /* A ping carries no time: the reply pairs with the pose when it arrives,
     * which is right because the look runs only while the car stands still. */
    look_pose_t const pose = {pos_x_mm(p_hug), pos_y_mm(p_hug), head_deci(p_hug)};
    bool const b_point = look_echo_is_point(p_in->ping_echo);
    int32_t const reach = b_point ? clamp_i32((int32_t)p_in->ping_mm, 0, HUG_OPEN_MM)
                                  : HUG_OPEN_MM;
    uint8_t const side = (offset > 0) ? LEFT : RIGHT;

    (void)look_add(p_look, &pose, p_in->ping_servo_deci, p_in->ping_echo,
                   (int32_t)p_in->ping_mm);
    if ((PU_CONTACT == p_hug->purpose) && (geom_abs(offset) > HUG_CONTACT_NEAR_DECI))
    {
        p_hug->open_mm[side] = (uint16_t)((uint32_t)p_hug->open_mm[side] + (uint32_t)reach);
        if (b_point && (p_in->ping_mm > 0U))
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
static void after_scan(hug_t * p_hug, look_t * p_look);

static void scan_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                      hug_out_t * p_out)
{
    int32_t const offset = ray_offset(p_hug, p_hug->idx);
    uint8_t const want = body_step(p_hug, offset);

    if (SUB_MOVE == p_hug->sub)
    {
        bool const b_aimed = flag_get(p_hug, F_AIMED);

        flag_set(p_hug, F_AIMED, false);
        if (pivot_cmd(p_hug, p_hug->center + body_deci(want), p_out))
        {
            p_hug->sub = (uint8_t)SUB_SETTLE;
            p_hug->t_ms = p_in->now_ms - (b_aimed ? HUG_SETTLE_MS : 0U);
            p_hug->body = want;
        }
    }
    else if (SUB_SETTLE == p_hug->sub)
    {
        if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
        {
            p_hug->sub = (uint8_t)SUB_ASK;
            p_hug->t_ms = p_in->now_ms;
            p_hug->req_seq = (uint8_t)(((uint32_t)p_hug->req_seq + 1U) & SEQ_MASK);
        }
    }
    else
    {
        if (p_in->b_ping_reply && (p_in->ping_seq == p_hug->req_seq))
        {
            p_hug->lost = 0U;
            scan_record(p_hug, p_look, p_in, offset);
            p_hug->idx++;
            p_hug->req_seq = (uint8_t)(((uint32_t)p_hug->req_seq + 1U) & SEQ_MASK);
            if (p_hug->idx >= ray_count(p_hug))
            {
                after_scan(p_hug, p_look);
            }
            else if (body_step(p_hug, ray_offset(p_hug, p_hug->idx)) != p_hug->body)
            {
                p_hug->sub = (uint8_t)SUB_START;
            }
            else
            {
                p_hug->t_ms = p_in->now_ms;
            }
        }
        else if ((p_in->now_ms - p_hug->t_ms) > HUG_REPLY_TIMEOUT_MS)
        {
            p_hug->lost = (uint8_t)((p_hug->lost < HUG_LOST_REPLIES) ? (p_hug->lost + 1U)
                                                                     : p_hug->lost);
            p_hug->req_seq = (uint8_t)(((uint32_t)p_hug->req_seq + 1U) & SEQ_MASK);
            p_hug->t_ms = p_in->now_ms;
        }
        else
        {
            /* Waiting for the reply. */
        }
        if ((ST_SCAN == p_hug->state) && (SUB_ASK == p_hug->sub))
        {
            emit_request(p_hug, p_out, offset - body_deci(p_hug->body));
            p_out->b_stop = (p_hug->lost >= HUG_LOST_REPLIES) ? 1U : 0U;
        }
    }
}

/* ---- planning ---- */

static uint32_t valid_rays(look_t const * p_look)
{
    uint32_t n = 0U;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        n += (LOOK_ECHO_OPEN != p_look->rays[i].echo) ? 1U : 0U;
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
        side = (p_edges->b_narrow || p_edges->have[LEFT]) ? (int8_t)1 : (int8_t)-1;
    }
    else
    {
        side = ((uint32_t)p_hug->open_mm[RIGHT] > ((uint32_t)p_hug->open_mm[LEFT] + HUG_OPEN_TIE_MM))
                   ? (int8_t)-1 : (int8_t)1;
    }
    return side;
}

/* The angle a leg leans toward the face (away when negative) to end with the
 * axle HUG_AXLE_OFF_MM off it: the error over the length of the leg. */
static int32_t trim_deci(int32_t dist_mm, int32_t leg_mm)
{
    int32_t const err = dist_mm - HUG_AXLE_OFF_MM;

    return clamp_i32(geom_atan_deci(err, leg_mm), -HUG_TRIM_MAX_DECI, HUG_TRIM_MAX_DECI);
}

/* From the face and its edges: the heading along the face, the length of the
 * leg, and for the first leg the approach to the gap. A face assumed at a stall
 * is not measured, and a narrow face gives a poor distance, so their legs are
 * not trimmed. */
static void plan_leg(hug_t * p_hug, look_face_t const * p_face,
                     hug_edges_t const * p_edges, bool b_first, bool b_trim)
{
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const along = geom_wrap_deci(p_face->normal_deci + (p_hug->side * QUARTER_DECI));
    int32_t const sonar_mm = p_face->dist_mm - LOOK_SONAR_FWD_MM;
    int32_t app = 0;
    int32_t leg = 0;
    int32_t trim = 0;

    flag_set(p_hug, F_ASSUME, false);
    flag_set(p_hug, F_PAST, p_edges->have[s]);
    if (b_first)
    {
        app = clamp_i32(p_face->dist_mm - HUG_AXLE_OFF_MM, 0, HOP_LIMIT_MM);
        /* A face found after a stall lies no nearer than the stall was. */
        if (flag_get(p_hug, F_STALL))
        {
            app = clamp_i32(app, 0, HUG_STANDOFF_MM - HUG_BODY_GAP_MM);
        }
        app = (app < HUG_APPROACH_MIN_MM) ? 0 : app;
    }
    if (p_edges->have[s])
    {
        leg = geom_dot_mm(along, p_edges->edge[s].x_mm - pos_x_mm(p_hug),
                     p_edges->edge[s].y_mm - pos_y_mm(p_hug))
              + HUG_BODY_MM + HUG_BODY_GAP_MM;
    }
    else
    {
        leg = geom_mul_q14(sonar_mm - app, geom_tan_q14(HUG_HOP_ANGLE_DECI));
    }
    leg = clamp_i32(leg, HUG_LEG_MIN_MM, HUG_LEG_MAX_MM);
    if (b_trim && !p_edges->b_narrow)
    {
        trim = trim_deci(p_face->dist_mm - app, leg);
    }
    /* The leg is the way along the face; leaning makes the way longer. */
    leg = clamp_i32((leg * GEOM_Q14_ONE) / geom_cos_q14(trim), HUG_LEG_MIN_MM, HUG_LEG_MAX_MM);
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
        p_hug->keep_x_mm[s] = (int16_t)p_edges->edge[s].x_mm;
        p_hug->keep_y_mm[s] = (int16_t)p_edges->edge[s].y_mm;
    }
    p_hug->keep_normal = (int16_t)p_face->normal_deci;
    p_hug->keep_dist_mm = (int16_t)p_face->dist_mm;
}

static void restore_edges(hug_t const * p_hug, hug_edges_t * p_edges, look_face_t * p_face)
{
    int32_t const left_deci = QUARTER_DECI;

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

static void decide_contact(hug_t * p_hug, look_face_t const * p_face,
                           hug_edges_t const * p_edges)
{
    /* A narrow face gives edges too poor to report a width from. */
    bool const b_both = p_edges->have[LEFT] && p_edges->have[RIGHT] && !p_edges->b_narrow;

    p_hug->width_mm = b_both ? (uint16_t)edges_apart_mm(p_edges) : 0U;
    p_hug->side = pick_side(p_hug, p_edges);
    plan_leg(p_hug, p_face, p_edges, true, true);
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
    else
    {
        decide_contact(p_hug, p_face, &edges);
    }
}

/* The sonar reads the nearest point inside its beam, so a ray more than the
 * half-angle off the face normal has hit the face half a beam nearer the
 * normal, and a ray within it has hit the foot of the normal. Fit the face
 * with the rays moved there; the rays go back afterwards. */
static bool fit_beam(hug_t const * p_hug, look_t * p_look, look_face_t * p_face)
{
    look_pose_t const pose = {pos_x_mm(p_hug), pos_y_mm(p_hug), head_deci(p_hug)};
    int16_t moved[LOOK_RAYS_MAX];
    bool b_ok = false;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t * const p_ray = &p_look->rays[i];
        int32_t const diff = geom_wrap_deci(p_ray->angle_deci - p_face->normal_deci);
        int32_t move = 0;

        if (LOOK_ECHO_OPEN != p_ray->echo)
        {
            move = (geom_abs(diff) > LOOK_BEAM_DECI) ? ((diff > 0) ? -LOOK_BEAM_DECI : LOOK_BEAM_DECI)
                                                     : -diff;
        }
        moved[i] = (int16_t)move;
        p_ray->angle_deci = (int16_t)geom_wrap_deci(p_ray->angle_deci + move);
    }
    b_ok = look_face(p_look, &pose, p_face);
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        p_look->rays[i].angle_deci = (int16_t)geom_wrap_deci(p_look->rays[i].angle_deci - moved[i]);
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
        look_pose_t const pose = {pos_x_mm(p_hug), pos_y_mm(p_hug), head_deci(p_hug)};
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
        decide_contact(p_hug, &face, &edges);
    }
}

/* The heading along the face the last look was square to. */
static int32_t along_head(hug_t const * p_hug)
{
    return geom_wrap_deci(p_hug->look_head + (p_hug->side * QUARTER_DECI));
}

static void after_look(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                       bool b_found)
{
    /* The look after an assumed face is the first one of the real face, which
     * may be turned as far as an INNER look takes it. */
    bool const b_near = b_found && (valid_rays(p_look) >= HUG_FACE_RAYS_MIN)
                        && (geom_abs(geom_wrap_deci(p_face->normal_deci - p_hug->center))
                            <= (flag_get(p_hug, F_ASSUME) ? HUG_INNER_ANGLE_DECI : HUG_FACE_ANGLE_DECI))
                        && (flag_get(p_hug, F_TURNED)
                            ? (p_face->dist_mm <= (HUG_AXLE_OFF_MM + HUG_FACE_TOL_MM))
                            : (geom_abs(p_face->dist_mm - p_hug->face_p_mm) <= HUG_FACE_TOL_MM));

    if (b_near)
    {
        hug_edges_t edges;

        find_edges(p_look, p_face, p_hug, &edges);
        if (flag_get(p_hug, F_TURNED) && edges.have[LEFT] && edges.have[RIGHT] && !edges.b_narrow)
        {
            p_hug->breadth_mm = (uint16_t)edges_apart_mm(&edges);
        }
        flag_set(p_hug, F_TURNED, false);
        plan_leg(p_hug, p_face, &edges, false, true);
    }
    else
    {
        if (flag_get(p_hug, F_TURNED))
        {
            /* The next face is short and behind the car: go on until the
             * body clears its end, then look again. */
            flag_set(p_hug, F_TURNED, false);
            p_hug->face_p_mm = (int16_t)(HUG_BODY_MM + HUG_BODY_GAP_MM);
            p_hug->leg_mm = (int16_t)(HUG_BODY_MM + HUG_BODY_GAP_MM);
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
            flag_set(p_hug, F_PAST, false);
            p_hug->trav_head = p_hug->center;
            p_hug->look_head = (int16_t)geom_wrap_deci(p_hug->center - (p_hug->side * QUARTER_DECI));
            flag_set(p_hug, F_TURNED, true);
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
    plan_leg(p_hug, &face, &edges, b_first, false);
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
        plan_leg(p_hug, p_face, &edges, false, true);
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
static void prune_far(look_t * p_look)
{
    uint16_t near_mm = UINT16_MAX;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if ((LOOK_ECHO_OPEN != p_look->rays[i].echo) && (p_look->rays[i].range_mm < near_mm))
        {
            near_mm = p_look->rays[i].range_mm;
        }
    }
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if ((LOOK_ECHO_OPEN != p_look->rays[i].echo)
            && ((int32_t)p_look->rays[i].range_mm > ((int32_t)near_mm + HUG_BEHIND_MM)))
        {
            p_look->rays[i].echo = (uint8_t)LOOK_ECHO_OPEN;
            p_look->rays[i].range_mm = 0U;
        }
    }
}

/* The wide rays of the contact look are for the room beside the face. What
 * they read (a wall) is not the face, and it would pull the fit off it. */
static void drop_wide(look_t * p_look)
{
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if (geom_abs(p_look->rays[i].angle_deci) > HUG_CONTACT_NEAR_DECI)
        {
            p_look->rays[i].echo = (uint8_t)LOOK_ECHO_OPEN;
            p_look->rays[i].range_mm = 0U;
        }
    }
}

static void after_scan(hug_t * p_hug, look_t * p_look)
{
    look_pose_t const pose = {pos_x_mm(p_hug), pos_y_mm(p_hug), head_deci(p_hug)};

    if (PU_CONTACT == p_hug->purpose)
    {
        drop_wide(p_look);
    }
    prune_far(p_look);
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
    if (PU_REFINE == p_hug->purpose)
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
            release(p_hug);
        }
    }
    else if (PU_LOOK == p_hug->purpose)
    {
        after_look(p_hug, p_look, &face, b_found);
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
        p_hug->purpose = (uint8_t)PU_LOOK;
        p_hug->center = p_hug->look_head;
        p_hug->idx = 0U;
        flag_set(p_hug, F_AIMED, true);
        start_pivot(p_hug, p_hug->center + body_deci(body_step(p_hug, ray_offset(p_hug, 0U))),
                    THEN_SCAN);
        break;
    case THEN_TURN:
        start_drive(p_hug, p_hug->face_p_mm + HUG_TURN_FWD_MM, p_hug->goal_head, THEN_LOOK);
        break;
    case THEN_ADVANCE:
        start_drive(p_hug, p_hug->leg_mm, p_hug->goal_head, THEN_LOOK);
        break;
    case THEN_EXIT_TURN:
        /* Away from the obstacle, which is on the other side than the way round. */
        start_pivot(p_hug, geom_wrap_deci(p_hug->goal_head + (p_hug->side * QUARTER_DECI)),
                    THEN_RELEASE);
        break;
    default:
        finish(p_hug);
        break;
    }
}

static void begin(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in)
{
    p_hug->x_um = 0;
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
    look_clear(p_look);
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
static bool tape_found(hug_t const * p_hug, hug_in_t const * p_in)
{
    bool const b_approach = (pos_x_mm(p_hug) <= HUG_EXIT_PAST_MM)
                            && (geom_abs(pos_y_mm(p_hug)) <= HUG_LINE_WINDOW_MM);

    return (p_in->b_line_left || p_in->b_line_right) && !b_approach;
}

static void drive_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       bool b_guard, hug_out_t * p_out)
{
    if (SUB_SETTLE == p_hug->sub)
    {
        if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
        {
            proceed(p_hug, p_look);
        }
    }
    else if ((THEN_EXIT_TURN != p_hug->then) && (flag_get(p_hug, F_SWEPT) || tape_found(p_hug, p_in)))
    {
        flag_set(p_hug, F_EXIT, true);
        flag_set(p_hug, F_SWEPT, false);
        start_drive(p_hug, LINE_SENSOR_LEAD_MM, head_deci(p_hug),
                    THEN_EXIT_TURN);
    }
    else if (p_in->b_stall || (drive_is_leg(p_hug) && b_guard))
    {
        flag_set(p_hug, F_STALL, p_in->b_stall);
        start_scan(p_hug, p_look, PU_INNER, head_deci(p_hug));
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

static void pivot_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       hug_out_t * p_out)
{
    if (SUB_MOVE == p_hug->sub)
    {
        bool b_done = pivot_cmd(p_hug, p_hug->goal_head, p_out);

        /* The line may cross the way out at any angle, and the follower needs
         * a sensor on it: the turn stops when one sees it. */
        if (!b_done && flag_get(p_hug, F_EXIT) && (p_in->b_line_left || p_in->b_line_right))
        {
            p_out->turn_deci_s = 0;
            b_done = true;
        }
        if (b_done)
        {
            p_hug->sub = (uint8_t)SUB_SETTLE;
            p_hug->t_ms = p_in->now_ms;
        }
    }
    else if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
    {
        proceed(p_hug, p_look);
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
                                      + body_deci(body_step(p_hug, ray_offset(p_hug, p_hug->idx))))
                                   : p_hug->goal_head;
        uint32_t const angle = (uint32_t)geom_abs(geom_wrap_deci(target - head_deci(p_hug)));

        plan_ms = (angle * MS_PER_S * (uint32_t)PERCENT)
                  / ((uint32_t)HUG_PIVOT_MIN_DECI_S * (uint32_t)HUG_PIVOT_PCT);
    }
    return HUG_MOVE_TIMES * ((plan_ms < HUG_MOVE_FLOOR_MS) ? HUG_MOVE_FLOOR_MS : plan_ms);
}

bool hug_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bool const b_was_active = (ST_IDLE != p_hug->state);
    bool b_guard = false;
    bool b_takes = false;

    zero_out(p_out);
    if (ST_IDLE != p_hug->state)
    {
        odo_update(p_hug, p_in);
    }
    else if ((0U != p_hug->b_note) || (0U != p_hug->n_rel))
    {
        note_update(p_hug, p_in);
    }
    else
    {
        /* Nothing to track while idle. */
    }
    forward_ping(p_hug, p_in, &b_guard);
    if (ST_IDLE == p_hug->state)
    {
        /* A stall with the follower driving is a contact: the sonar did not see
         * the face, so look from the backed-off pose. */
        if (!p_in->b_other && (triggered(p_hug) || p_in->b_stall))
        {
            begin(p_hug, p_look, p_in);
        }
    }
    if ((ST_IDLE != p_hug->state) && p_in->b_other)
    {
        if (SUB_MOVE == p_hug->sub)
        {
            /* The clock of the move starts again when the wheels come back. */
            p_hug->sub = (uint8_t)SUB_START;
        }
    }
    else if (ST_IDLE != p_hug->state)
    {
        b_takes = true;
        if (SUB_START == p_hug->sub)
        {
            p_hug->t_ms = p_in->now_ms + move_limit_ms(p_hug);
            p_hug->sub = (uint8_t)SUB_MOVE;
        }
        /* The difference is right across the wrap of the clock. */
        if ((SUB_MOVE == p_hug->sub) && ((int32_t)(p_in->now_ms - p_hug->t_ms) > 0))
        {
            /* The move makes too little headway: the hug ends, the pilot stops. */
            finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else if (ST_SCAN == p_hug->state)
        {
            flag_set(p_hug, F_SWEPT, flag_get(p_hug, F_SWEPT) || tape_found(p_hug, p_in));
            scan_tick(p_hug, p_look, p_in, p_out);
        }
        else if (ST_PIVOT == p_hug->state)
        {
            flag_set(p_hug, F_SWEPT, flag_get(p_hug, F_SWEPT) || tape_found(p_hug, p_in));
            pivot_tick(p_hug, p_look, p_in, p_out);
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
