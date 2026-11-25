/** @file scan.c
 * @brief Hug sonar requests, reply matching and contact-face observations.
 */

#include "internal.h"

#include <stddef.h>

#include "geometry.h"
#include "obstacle_config.h"
#include "platform/units.h"

#define RAY_NEAR_N      (((2U * HUG_NEAR_ZONE_DECI) / HUG_NEAR_STEP_DECI) + 1U)
#define SIDE_BODY_DECI  (HUG_NEAR_ZONE_DECI - OBSTACLE_SERVO_LIMIT_DECI)
#define RAY_CONTACT_N   ((uint32_t)(sizeof(g_contact_deci) / sizeof(g_contact_deci[0])))
#define CENTER_STEP_DECI (20)
#define CENTER_RAYS      (((2U * OBSTACLE_SERVO_LIMIT_DECI) / CENTER_STEP_DECI) + 1U)

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
STATIC_ASSERT(RAY_REFINE_N == RAY_REFINE_FAR_N, "both refine windows hold the same rays");
STATIC_ASSERT((2U * RAY_REFINE_N) <= LOOK_RAYS_MAX, "two refine windows fit the look");
STATIC_ASSERT(RAY_NEAR_N <= LOOK_RAYS_MAX, "a look fits the look");
STATIC_ASSERT(CENTER_RAYS <= LOOK_RAYS_MAX, "the central servo sweep fits the look");
STATIC_ASSERT((SIDE_BODY_DECI > 0) && (SIDE_BODY_DECI <= OBSTACLE_SERVO_LIMIT_DECI),
              "two side-look headings cover the sweep");
STATIC_ASSERT(PROBE_STEP_DECI <= OBSTACLE_SERVO_LIMIT_DECI, "probe rays fit servo travel");

static int16_t const g_contact_deci[] = {HUG_CONTACT_RAYS};

STATIC_ASSERT(RAY_CONTACT_N <= LOOK_RAYS_MAX, "the contact look fits the look");

void hug_start_scan(hug_t * p_hug, look_t * p_look, uint32_t purpose, int32_t center)
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
    if (!hug_flag_get(p_hug, F_WIN2 | F_RECHECK))
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

bool hug_facet_clearance(hug_t const * p_hug)
{
    return (PU_FACET_CLEAR == p_hug->purpose) || (PU_FACET_RETRY == p_hug->purpose);
}

bool hug_corner_scan(hug_t const * p_hug)
{
    return (PU_CORNER == p_hug->purpose) || (PU_CORNER_RETRY == p_hug->purpose) || hug_facet_clearance(p_hug);
}

bool hug_probe_scan(hug_t const * p_hug)
{
    return hug_corner_scan(p_hug) || (PU_CHECK == p_hug->purpose);
}

bool hug_edge_sweep(hug_t const * p_hug)
{
    return (PU_EDGE_LOOK == p_hug->purpose) || (PU_EDGE_RETRY == p_hug->purpose) ||
           (PU_MID_LOOK == p_hug->purpose);
}

static bool side_sweep(hug_t const * p_hug)
{
    return (PU_LOOK == p_hug->purpose) || hug_edge_sweep(p_hug);
}

static uint32_t ray_count(hug_t const * p_hug)
{
    uint32_t n = RAY_NEAR_N;

    if (PU_CONTACT == p_hug->purpose)
    {
        n = hug_flag_get(p_hug, F_RECHECK) ? RECHECK_RAYS : RAY_CONTACT_N;
    }
    else if (PU_CENTER == p_hug->purpose)
    {
        n = CENTER_RAYS;
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        n = RAY_REFINE_N;
    }
    else if (hug_probe_scan(p_hug))
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
int32_t hug_ray_offset(hug_t const * p_hug, uint32_t i)
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
    else if (hug_corner_scan(p_hug))
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
        offset = hug_flag_get(p_hug, F_RECHECK)
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
static uint8_t hug_body_step(hug_t const * p_hug, int32_t offset)
{
    if ((PU_REFINE == p_hug->purpose) || (PU_CENTER == p_hug->purpose) ||
        hug_probe_scan(p_hug) || hug_flag_get(p_hug, F_RECHECK))
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

static int32_t hug_body_deci(hug_t const * p_hug, uint8_t step)
{
    int32_t const step_deci = side_sweep(p_hug) ? SIDE_BODY_DECI : HUG_PIVOT_STEP_DECI;

    return ((int32_t)step - BODY_BIAS) * step_deci;
}

int32_t hug_scan_body_aim(hug_t const * p_hug, int32_t offset)
{
    return (0 != p_hug->scan_lead_deci) ? offset + p_hug->scan_lead_deci
                                        : hug_body_deci(p_hug, hug_body_step(p_hug, offset));
}

void hug_look_along_face(hug_t * p_hug, uint8_t purpose)
{
    p_hug->b_close_front = false;
    p_hug->purpose = purpose;
    p_hug->center = p_hug->look_head;
    if (PU_MID_LOOK == purpose)
    {
        p_hug->center = (int16_t)geom_wrap_deci(p_hug->look_head + p_hug->side * (QUARTER_DECI / HALVES));
    }
    p_hug->idx = 0U;
    hug_flag_set(p_hug, F_AIMED, true);
    hug_start_pivot(p_hug, p_hug->center + hug_body_deci(p_hug, hug_body_step(p_hug, hug_ray_offset(p_hug, 0U))),
                THEN_SCAN);
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

bool hug_prior_near_return(look_t const * p_look, int32_t bearing, int32_t range_mm)
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
           hug_prior_near_return(p_look, bearing, p_in->ping_mm);
}

void hug_restart_contact(hug_t * p_hug, look_t * p_look)
{
    hug_flag_set(p_hug, F_RECHECK, false);
    p_hug->recheck_hits = 0U;
    hug_start_scan(p_hug, p_look, PU_CONTACT, 0);
}

void hug_find_edges(look_t const * p_look, look_face_t const * p_face,
                    hug_t const * p_hug, hug_edges_t * p_edges)
{
    int32_t const left_deci = ((PU_CONTACT == p_hug->purpose) || (PU_REFINE == p_hug->purpose)
                               ? 0
                               : p_hug->center)
                              + QUARTER_DECI;

    p_edges->b_narrow = hug_valid_rays(p_look) < HUG_NORMAL_RAYS_MIN;
    look_edges(p_look, p_face, p_edges->edge, p_edges->have);
    for (uint8_t s = 0U; s < HUG_SIDES; s++)
    {
        p_edges->lat_mm[s] = 0;
        if (p_edges->have[s])
        {
            p_edges->lat_mm[s] = geom_abs(geom_dot_mm(left_deci,
                                                 p_edges->edge[s].x_mm - hug_pos_x_mm(p_hug),
                                                 p_edges->edge[s].y_mm - hug_pos_y_mm(p_hug)));
        }
    }
}

int32_t hug_edges_apart_mm(hug_edges_t const * p_edges)
{
    int32_t const dx = p_edges->edge[LEFT].x_mm - p_edges->edge[RIGHT].x_mm;
    int32_t const dy = p_edges->edge[LEFT].y_mm - p_edges->edge[RIGHT].y_mm;

    return (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy)));
}

/* The nearer edge, else the more open side; a side whose gap beside the
 * edge is too narrow for the car is passed over. A tie goes left. */
int8_t hug_pick_side(hug_t const * p_hug, hug_edges_t const * p_edges)
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

/* The sonar reads the nearest point inside its beam, so a ray more than the
 * half-angle off the face normal has hit the face half a beam nearer the
 * normal, and a ray within it has hit the foot of the normal. Fit the face
 * with the rays moved there; the rays go back afterwards. */
static bool hug_fit_beam(hug_t const * p_hug, look_t * p_look, look_face_t * p_face)
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

static bool hug_refit_beam(hug_t const * p_hug, look_t * p_look, look_face_t * p_face,
                           hug_edges_t * p_edges)
{
    bool b_ok = hug_fit_beam(p_hug, p_look, p_face);

    if (b_ok)
    {
        hug_find_edges(p_look, p_face, p_hug, p_edges);
        b_ok = p_edges->have[LEFT] && p_edges->have[RIGHT];
    }
    return b_ok;
}

/* Readings that are farther than the nearest one by more than the depth of
 * a face are what lies behind it, not the face: they count as OPEN. */
static void hug_prune_far(look_t * p_look, bool b_check_cluster)
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
static void hug_drop_wide(look_t * p_look)
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
static void hug_save_guard_ranges(look_t const * p_look, uint16_t ranges[RAY_CONTACT_N])
{
    for (uint8_t i = 0U; i < RAY_CONTACT_N; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];

        ranges[i] = ((LOOK_ECHO_VALID == look_ray_echo(p_ray)) && (p_ray->range_mm <= HUG_GUARD_MM))
                        ? p_ray->range_mm : 0U;
    }
}

static void hug_restore_guard_ranges(look_t * p_look, uint16_t const ranges[RAY_CONTACT_N])
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

/* Both wheel deltas are modular low-word readings. Bound them before
 * converting to signed travel or subtracting them for the heading. */
bool hug_near_survey_pose(hug_t const * p_hug, hug_in_t const * p_in)
{
    uint32_t const dl = (uint32_t)p_in->left_um - p_hug->origin.survey.left_um;
    uint32_t const dr = (uint32_t)p_in->right_um - p_hug->origin.survey.right_um;
    bool const b_near = ((dl + RECHECK_WHEEL_UM) <= (2U * RECHECK_WHEEL_UM)) &&
                        ((dr + RECHECK_WHEEL_UM) <= (2U * RECHECK_WHEEL_UM));

    return b_near && (geom_abs((int32_t)dr - (int32_t)dl) * DECI_PER_RAD <=
                      (HUG_HEAD_TOL_DECI * CHASSIS_WHEEL_TRACK_MM * UM_PER_MM));
}

bool hug_can_recheck(hug_t const * p_hug, look_t const * p_look, hug_in_t const * p_in)
{
    return (RECHECK_READY == p_hug->recheck_hits) && !p_in->b_stall && !p_in->b_relook &&
           !p_in->b_ping_reply && (0 == p_in->ping_servo_deci) &&
           (LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > 0U) &&
           (p_in->ping_mm <= HUG_STANDOFF_MM) &&
           ((p_in->now_ms - p_hug->t_ms) <= HUG_REPLY_TIMEOUT_MS) &&
           ((p_in->now_ms - p_hug->contact_scan_ms) <= HUG_NOTE_MS) &&
           hug_near_survey_pose(p_hug, p_in) && hug_prior_near_return(p_look, 0, p_in->ping_mm);
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
    hug_flag_set(p_hug, F_COARSE_NARROW, p_edges->b_narrow);
}

static void restore_edges(hug_t const * p_hug, hug_edges_t * p_edges, look_face_t * p_face)
{
    int32_t const left_deci = QUARTER_DECI;

    p_edges->b_narrow = hug_flag_get(p_hug, F_COARSE_NARROW);
    for (uint8_t s = 0U; s < HUG_SIDES; s++)
    {
        p_edges->edge[s].x_mm = p_hug->keep_x_mm[s];
        p_edges->edge[s].y_mm = p_hug->keep_y_mm[s];
        p_edges->have[s] = true;
        p_edges->lat_mm[s] = geom_abs(geom_dot_mm(left_deci, p_edges->edge[s].x_mm - hug_pos_x_mm(p_hug),
                                             p_edges->edge[s].y_mm - hug_pos_y_mm(p_hug)));
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
        int32_t const ox = hug_pos_x_mm(p_hug) + geom_mul_q14(LOOK_SONAR_FWD_MM, geom_cos_q14(center));
        int32_t const oy = hug_pos_y_mm(p_hug) + geom_mul_q14(LOOK_SONAR_FWD_MM, geom_sin_q14(center));

        center = geom_wrap_deci(geom_atan_deci(p_hug->keep_y_mm[side] - oy,
                                               p_hug->keep_x_mm[side] - ox)
                                + out);
    }
    hug_flag_set(p_hug, F_WIN2, RIGHT == side);
    hug_start_scan(p_hug, p_look, PU_REFINE, center);
}

static void decide_contact(hug_t * p_hug, look_t const * p_look, look_face_t const * p_face,
                           hug_edges_t const * p_edges)
{
    /* A narrow face gives edges too poor to report a width from. */
    int32_t const span = (p_edges->have[LEFT] && p_edges->have[RIGHT]) ? hug_edges_apart_mm(p_edges) : 0;
    p_hug->b_close_front = (span > 0) && (span <= (int32_t)OBSTACLE_CAR_HALF_W_MM) &&
        (p_face->dist_mm >= HUG_SCAN_CLEARANCE_MM) &&
        !hug_flag_get(p_hug, F_STALL) && (geom_abs(p_face->normal_deci) <= HUG_FACE_ANGLE_DECI) &&
        (geom_dot_mm(p_face->normal_deci + QUARTER_DECI, p_edges->edge[LEFT].x_mm - p_edges->edge[RIGHT].x_mm,
                     p_edges->edge[LEFT].y_mm - p_edges->edge[RIGHT].y_mm) > 0);
    p_hug->width_mm = p_edges->b_narrow ? 0U : (uint16_t)span;
    p_hug->side = hug_pick_side(p_hug, p_edges);
    hug_plan_leg(p_hug, p_face, p_edges, true, p_look);
}

/* The first look of a face. With both edges in trust each edge is looked at
 * once more with finer rays, which fixes it to a few mm. Without them the
 * width stays unknown and the car goes on from what it saw. */
static void hug_after_contact(hug_t * p_hug, look_t * p_look, look_face_t const * p_face)
{
    hug_edges_t edges;

    hug_find_edges(p_look, p_face, p_hug, &edges);
    if (edges.have[LEFT] && edges.have[RIGHT])
    {
        keep_edges(p_hug, &edges, p_face);
        hug_flag_set(p_hug, F_REFINE, true);
        start_refine(p_hug, p_look, LEFT);
    }
    else if (edges.b_narrow && (edges.have[LEFT] || edges.have[RIGHT]))
    {
        /* With only one edge, compare the two halves of the beam from one
         * stationary, centred pose. Preserve the measured edge for the leg. */
        keep_edges(p_hug, &edges, p_face);
        p_hug->side = edges.have[LEFT] ? (int8_t)1 : (int8_t)-1;
        hug_start_scan(p_hug, p_look, PU_CENTER, 0);
    }
    else
    {
        decide_contact(p_hug, p_look, p_face, &edges);
    }
}

static void hug_after_center(hug_t * p_hug, look_t const * p_look, look_face_t const * p_fit, bool b_found)
{
    hug_edges_t edges;
    look_face_t face = {0, 0, 0, 0};
    uint8_t const missing = (p_hug->side > 0) ? RIGHT : LEFT;
    int32_t balance = 0;

    restore_edges(p_hug, &edges, &face);
    edges.have[missing] = false;
    p_hug->side = hug_pick_side(p_hug, &edges);
    /* Only a complete symmetric sweep can compare the two sides. The wide
     * scan's side-wall clearance retains priority. */
    if ((CENTER_RAYS == p_look->count) && (hug_valid_rays(p_look) >= HUG_FACE_RAYS_MIN) &&
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
    hug_plan_leg(p_hug, &face, &edges, true, p_look);
}

static void hug_after_refine(hug_t * p_hug, look_t * p_look)
{
    if (!hug_flag_get(p_hug, F_WIN2))
    {
        start_refine(p_hug, p_look, RIGHT);
    }
    else
    {
        look_pose_t const pose = hug_axle_pose(p_hug);
        look_face_t face = {0, 0, 0, 0};
        hug_edges_t edges;
        bool b_fine = look_face(p_look, &pose, &face);

        hug_flag_set(p_hug, F_REFINE | F_WIN2, false);
        if (b_fine)
        {
            hug_find_edges(p_look, &face, p_hug, &edges);
            b_fine = edges.have[LEFT] && edges.have[RIGHT];
        }

        if (b_fine && ((p_hug->keep_dist_mm - LOOK_SONAR_FWD_MM) > HUG_FAR_MM))
        {
            /* Far off the beam sags the points by a few percent. A near face is
             * narrow in rays and the correction would only thin them out. */
            b_fine = hug_refit_beam(p_hug, p_look, &face, &edges);
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
static void hug_end_stationary_batch(hug_t * p_hug, look_t * p_look)
{
    if (hug_flag_get(p_hug, F_RECHECK))
    {
        hug_restart_contact(p_hug, p_look);
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        look_face_t face = {0};
        hug_edges_t edges;
        restore_edges(p_hug, &edges, &face);
        hug_flag_set(p_hug, F_REFINE | F_WIN2, false);
        decide_contact(p_hug, p_look, &face, &edges);
    }
    else if (PU_CENTER == p_hug->purpose)
    {
        /* The saved single edge still determines a candidate side. */
        hug_after_center(p_hug, p_look, NULL, false);
    }
    else if (PU_CHECK == p_hug->purpose)
    {
        hug_look_along_face(p_hug, PU_LOOK);
    }
    else if (!hug_corner_scan(p_hug))
    {
        int32_t const offset = hug_ray_offset(p_hug, p_hug->idx);
        int32_t const delta = geom_wrap_deci(p_hug->center + offset - hug_pose_heading_deci(p_hug));
        p_hug->scan_lead_deci = (int8_t)((delta >= 0) ? HUG_NEAR_STEP_DECI : -HUG_NEAR_STEP_DECI);
        p_hug->sub = (uint8_t)SUB_START;
        hug_flag_set(p_hug, F_AIMED, false);
    }
    else
    {
        /* All three corner-clearance rays must share the stationary pose. */
    }
}

/* After a scan the car has the look, and the next move is planned from it. */
static void hug_after_scan(hug_t * p_hug, look_t * p_look, hug_out_t * p_out)
{
    uint16_t guard_ranges[RAY_CONTACT_N];
    look_pose_t const pose = hug_axle_pose(p_hug);

    if (hug_corner_scan(p_hug))
    {
        hug_after_corner(p_hug, p_look, p_out);
        return;
    }
    if (PU_CHECK == p_hug->purpose)
    {
        hug_after_side_check(p_hug, p_look);
        return;
    }
    if (PU_CONTACT == p_hug->purpose)
    {
        if (p_look->count == RAY_CONTACT_N)
        {
            hug_save_guard_ranges(p_look, guard_ranges);
        }
        hug_drop_wide(p_look);
    }
    hug_prune_far(p_look, ((PU_CONTACT == p_hug->purpose) && (hug_valid_rays(p_look) < HUG_NORMAL_RAYS_MIN)) ||
              (PU_REFINE == p_hug->purpose) || (PU_CENTER == p_hug->purpose) || hug_edge_sweep(p_hug));
    look_face_t face = {0, 0, 0, 0};
    bool const b_found = look_face(p_look, &pose, &face);

    /* A face turned from the heading reads as an arc: rays within the beam of
     * the normal all read its foot. The first fit is a few degrees off, so the
     * rays are moved to the face again with each better normal. */
    for (uint32_t i = 0U; b_found && (PU_REFINE != p_hug->purpose) && (i < BEAM_PASSES); i++)
    {
        look_face_t next = face;

        if (hug_fit_beam(p_hug, p_look, &next))
        {
            face = next;
        }
    }
    if (PU_CENTER == p_hug->purpose)
    {
        hug_after_center(p_hug, p_look, &face, b_found);
    }
    else if (PU_REFINE == p_hug->purpose)
    {
        hug_after_refine(p_hug, p_look);
    }
    else if (PU_CONTACT == p_hug->purpose)
    {
        bool const b_seen = b_found && (hug_valid_rays(p_look) >= HUG_FACE_RAYS_MIN);

        if (!hug_flag_get(p_hug, F_BACKED) && hug_flag_get(p_hug, F_STALL))
        {
            /* A stall is a face at range 0. */
            hug_start_back(p_hug, 0);
        }
        else if (!hug_flag_get(p_hug, F_BACKED) && b_seen && (face.dist_mm < HUG_AXLE_OFF_MM))
        {
            hug_start_back(p_hug, face.dist_mm - LOOK_SONAR_FWD_MM);
        }
        else if (b_seen)
        {
            hug_after_contact(p_hug, p_look, &face);
        }
        else if (hug_flag_get(p_hug, F_STALL) && hug_flag_get(p_hug, F_REPEAT))
        {
            hug_plan_assumed(p_hug, true);
        }
        else
        {
            /* The hit is gone, or the stall was a block that went away: nothing
             * to go round. */
            if (hug_flag_get(p_hug, F_STALL) && (p_hug->n_rel < UINT8_MAX))
            {
                p_hug->n_rel++;
            }
            if ((p_look->count == RAY_CONTACT_N) && (hug_valid_rays(p_look) < HUG_FACE_RAYS_MIN))
            {
                hug_restore_guard_ranges(p_look, guard_ranges);
                if (!hug_flag_get(p_hug, F_STALL | F_BACKED))
                {
                    p_hug->recheck_hits = RECHECK_READY;
                }
            }
            hug_release(p_hug);
        }
    }
    else if (PU_LOOK == p_hug->purpose)
    {
        hug_after_look(p_hug, p_look, &face, b_found, p_out);
    }
    else if (hug_edge_sweep(p_hug))
    {
        hug_after_edge_sweep(p_hug, p_look, &face, b_found, p_out);
    }
    else
    {
        hug_after_inner(p_hug, p_look, &face, b_found);
    }
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
            hug_start_pivot(p_hug, 0, THEN_RELEASE);
        }
        else
        {
            hug_after_scan(p_hug, p_look, p_out);
        }
    }
    else if ((0 != p_hug->scan_lead_deci) || (hug_body_step(p_hug, hug_ray_offset(p_hug, p_hug->idx)) != p_hug->body))
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
            hug_end_stationary_batch(p_hug, p_look);
        }
    }
}

static void scan_reply(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       int32_t offset, hug_out_t * p_out)
{
    int32_t const servo_deci = offset - hug_scan_body_aim(p_hug, offset);
    bool const b_recheck = hug_flag_get(p_hug, F_RECHECK);
    bool const b_expired = (p_in->now_ms - p_hug->t_ms) > HUG_REPLY_TIMEOUT_MS;
    bool const b_short = (PU_CHECK == p_hug->purpose);
    bool const b_match = !b_expired && obstacle_ping_reply_matches(
        p_in->b_ping_reply, p_in->ping_seq, p_in->ping_servo_deci,
        p_hug->req_seq, servo_deci, OBSTACLE_PING_MAX_QUANTIZATION_DECI);
    bool const b_corner_unknown = (LOOK_ECHO_OPEN != p_in->ping_echo) &&
        (LOOK_ECHO_TOO_CLOSE != p_in->ping_echo) &&
        ((LOOK_ECHO_VALID != p_in->ping_echo) || (0U == p_in->ping_mm));

    if (hug_corner_scan(p_hug) && (b_expired || (b_match && b_corner_unknown)))
    {
        if ((PU_CORNER_RETRY == p_hug->purpose) || (PU_FACET_RETRY == p_hug->purpose))
        {
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else
        {
            /* The axle is still at the edge. Keep its position and side,
             * but measure all three clearance rays together again. */
            hug_start_scan(p_hug, p_look, (PU_FACET_CLEAR == p_hug->purpose) ? PU_FACET_RETRY : PU_CORNER_RETRY,
                       p_hug->center);
        }
    }
    else if (b_recheck && (b_expired ||
        (b_match && !recheck_ray(p_hug, p_look, p_in, offset))))
    {
        hug_restart_contact(p_hug, p_look);
    }
    else if (b_short && b_match &&
        ((LOOK_ECHO_VALID != p_in->ping_echo) || (0U == p_in->ping_mm)))
    {
        hug_look_along_face(p_hug, PU_LOOK);
    }
    else if (b_match)
    {
        accept_scan_ray(p_hug, p_look, p_in, offset, b_recheck, p_out);
    }
    else if (b_expired)
    {
        if (hug_probe_scan(p_hug))
        {
            hug_look_along_face(p_hug, PU_LOOK);
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
        if (p_hug->lost >= HUG_LOST_REPLIES)
        {
            p_hug->state = ST_IDLE;
            p_out->b_timeout = 1U;
            return;
        }
        int32_t const next_offset = hug_ray_offset(p_hug, p_hug->idx);
        emit_request(p_hug, p_out, next_offset - hug_scan_body_aim(p_hug, next_offset));
        p_out->b_stop = 0U;
    }
}

void hug_scan_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                   hug_out_t * p_out)
{
    int32_t const offset = hug_ray_offset(p_hug, p_hug->idx);
    uint8_t const want = hug_body_step(p_hug, offset);

    if (SUB_MOVE == p_hug->sub)
    {
        bool const b_aimed = hug_flag_get(p_hug, F_AIMED);

        hug_flag_set(p_hug, F_AIMED, false);
        if (hug_pivot_cmd(p_hug, p_hug->center + hug_scan_body_aim(p_hug, offset), p_out))
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
        if (hug_probe_scan(p_hug) || (PU_CENTER == p_hug->purpose))
        {
            hug_start_scan(p_hug, p_look, p_hug->purpose, p_hug->center);
        }
        p_hug->sub = (uint8_t)SUB_START;
    }
    else
    {
        scan_reply(p_hug, p_look, p_in, offset, p_out);
    }
}

uint32_t hug_valid_rays(look_t const * p_look)
{
    uint32_t n = 0U;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        n += (LOOK_ECHO_OPEN != look_ray_echo(&p_look->rays[i])) ? 1U : 0U;
    }
    return n;
}
