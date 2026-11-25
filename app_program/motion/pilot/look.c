/** @file look.c
 * @brief Fit a face to one stationary sonar sweep and find its edges.
 *
 * The rays are ordered by where they meet the face line, measured along the
 * face from its centre, so the walk to one side does not depend on the order
 * the pings arrived in.
 */

#include "motion/pilot/look.h"

#include "obstacle/obstacle.h"
#include "platform/units.h"

#define DECI_STRAIGHT  (1800)
#define DECI_RIGHT     (900)
#define INT16_MAX_MM   (32767)
#define INT16_MIN_MM   (-32768)
#define FIT_LIMIT      ((int64_t)1073741824)
#define HALVES         (2)
#define TWICE          (2)
#define FACE_POINTS_MIN (2)
#define NO_RAY         (0xFFU)
#define LOOK_SIDES     (2U)
#define Q14_SHIFT      (14U)

typedef struct
{
    int32_t t_mm; /* along the face from its centre, positive to the left */
    uint8_t idx;
} look_cand_t;

/* The face normal and the direction along the face, from the car side, as
 * Q14 cosine and sine: worked out once for all rays. */
typedef struct
{
    int32_t cos_n;
    int32_t sin_n;
    int32_t cos_t;
    int32_t sin_t;
} look_axes_t;

/* Per ray: whether it meets the face line, where along the face, and whether
 * it reads past the face. */
typedef struct
{
    int32_t  t_mm[LOOK_RAYS_MAX];
    uint32_t hit;
    uint32_t past;
} look_hits_t;

STATIC_ASSERT(LOOK_RAYS_MAX <= 32U, "look rays fit the hit masks");

static int16_t clamp_mm(int32_t value);
static void    ray_point(look_ray_t const * p_ray, int32_t * p_x_mm, int32_t * p_y_mm);
static bool    hit_face(look_face_t const * p_face, look_axes_t const * p_axes, int32_t ox_mm,
                        int32_t oy_mm, int32_t angle_deci, int32_t * p_x_mm, int32_t * p_y_mm);
static int32_t incidence_deci(look_face_t const * p_face, int32_t angle_deci);
static bool    reads_past_face(look_face_t const * p_face, look_axes_t const * p_axes,
                               look_ray_t const * p_ray);

void look_clear(look_t * p_look)
{
    p_look->count = 0U;
}

bool look_add(look_t * p_look, look_pose_t const * p_pose, int32_t servo_deci,
              uint8_t echo, int32_t range_mm)
{
    bool stored = false;

    if ((echo <= LOOK_ECHO_TOO_CLOSE) && ((LOOK_ECHO_OPEN == echo) || (range_mm > 0)))
    {
        if (p_look->count < LOOK_RAYS_MAX)
        {
            look_ray_t * const p_ray = &p_look->rays[p_look->count];
            int32_t const      sx    = p_pose->x_mm
                               + geom_mul_q14(LOOK_SONAR_FWD_MM,
                                              geom_cos_q14(p_pose->head_deci));
            int32_t const sy = p_pose->y_mm
                               + geom_mul_q14(LOOK_SONAR_FWD_MM, geom_sin_q14(p_pose->head_deci));

            p_ray->x_mm       = clamp_mm(sx);
            p_ray->y_mm       = clamp_mm(sy);
            p_ray->angle_echo = (uint16_t)((uint32_t)echo << LOOK_RAY_ECHO_SHIFT);
            look_ray_set_angle(p_ray, geom_wrap_deci(p_pose->head_deci + servo_deci));
            if (LOOK_ECHO_OPEN == echo)
            {
                p_ray->range_mm = 0U;
            }
            else
            {
                p_ray->range_mm = (range_mm > (int32_t)UINT16_MAX) ? UINT16_MAX
                                                                   : (uint16_t)range_mm;
            }
            p_look->count++;
            stored = true;
        }
    }
    return (stored);
}

bool look_face(look_t const * p_look, look_pose_t const * p_from,
               look_face_t * p_face)
{
    return look_face_near(p_look, p_from, UINT16_MAX, p_face);
}

static bool fit_ray(look_t const * p_look, uint8_t i, uint16_t max_range_mm)
{
    return ((p_look->rays[i].range_mm <= max_range_mm)
            && look_echo_is_point(look_ray_echo(&p_look->rays[i])));
}

/* Least-squares face through the points in range, leaving out ray skip. */
static bool fit_near(look_t const * p_look, look_pose_t const * p_from,
                     uint16_t max_range_mm, uint8_t skip, look_face_t * p_face)
{
    int64_t sum_x  = 0;
    int64_t sum_y  = 0;
    int64_t sum_xx = 0;
    int64_t sum_yy = 0;
    int64_t sum_xy = 0;
    int32_t points = 0;
    bool    found  = false;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        if ((i != skip) && fit_ray(p_look, i, max_range_mm))
        {
            int32_t x = 0;
            int32_t y = 0;

            ray_point(&p_look->rays[i], &x, &y);
            sum_x += x;
            sum_y += y;
            sum_xx += (int64_t)x * x;
            sum_yy += (int64_t)y * y;
            sum_xy += (int64_t)x * y;
            points++;
        }
    }

    if (points >= FACE_POINTS_MIN)
    {
        int32_t const cx  = (int32_t)(sum_x / points);
        int32_t const cy  = (int32_t)(sum_y / points);
        /* The sums of squares about the centre, from the raw sums: exact, as
         * the centre is a whole number. */
        int64_t const sxx = sum_xx - (TWICE * cx * sum_x) + ((int64_t)points * cx * cx);
        int64_t const syy = sum_yy - (TWICE * cy * sum_y) + ((int64_t)points * cy * cy);
        int64_t const sxy = sum_xy - ((int64_t)cx * sum_y) - ((int64_t)cy * sum_x)
                            + ((int64_t)points * cx * cy);

        if ((sxx + syy) > 0)
        {
            int64_t num = TWICE * sxy;
            int64_t den = sxx - syy;

            while (((num > FIT_LIMIT) || (num < -FIT_LIMIT))
                   || ((den > FIT_LIMIT) || (den < -FIT_LIMIT)))
            {
                num /= HALVES;
                den /= HALVES;
            }

            {
                /* The line runs along half the angle of (2 sxy, sxx - syy). */
                int32_t const line   = geom_atan_deci((int32_t)num, (int32_t)den) / HALVES;
                int32_t       normal = geom_wrap_deci(line + DECI_RIGHT);
                int32_t       dist_q = geom_dot_mm(normal, cx - p_from->x_mm, cy - p_from->y_mm);

                if (dist_q < 0)
                {
                    normal = geom_wrap_deci(normal + DECI_STRAIGHT);
                    dist_q = -dist_q;
                }
                p_face->x_mm        = cx;
                p_face->y_mm        = cy;
                p_face->normal_deci = normal;
                p_face->dist_mm     = dist_q;
                found               = true;
            }
        }
    }
    return (found);
}

bool look_face_near(look_t const * p_look, look_pose_t const * p_from,
                    uint16_t max_range_mm, look_face_t * p_face)
{
    bool const found = fit_near(p_look, p_from, max_range_mm, NO_RAY, p_face);

    if (found)
    {
        uint8_t stray = NO_RAY;
        int32_t worst = LOOK_FACE_MATCH_MM;

        for (uint8_t i = 0U; i < p_look->count; i++)
        {
            look_face_t loo;

            if (fit_ray(p_look, i, max_range_mm)
                && fit_near(p_look, p_from, max_range_mm, i, &loo))
            {
                int32_t x        = 0;
                int32_t y        = 0;
                int32_t residual = 0;

                ray_point(&p_look->rays[i], &x, &y);
                residual = geom_abs(geom_dot_mm(loo.normal_deci, x - loo.x_mm,
                                                y - loo.y_mm));
                if (residual > worst)
                {
                    worst = residual;
                    stray = i;
                }
            }
        }
        if (NO_RAY != stray)
        {
            (void)fit_near(p_look, p_from, max_range_mm, stray, p_face);
        }
    }
    return (found);
}

int32_t look_face_reach_mm(look_t const * p_look, look_pose_t const * p_from,
                           look_face_t const * p_face, look_side_t side)
{
    int32_t const normal = p_face->normal_deci;
    int32_t const sign = (LOOK_SIDE_LEFT == side) ? 1 : -1;
    int32_t const along = normal + sign * DECI_RIGHT;
    int32_t reach_mm = 0;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        if (LOOK_ECHO_VALID == look_ray_echo(p_ray))
        {
            int32_t const offset = geom_wrap_deci(look_ray_angle_deci(p_ray) - normal);
            int32_t const hit = offset - clamp_i32(offset, -LOOK_BEAM_DECI, LOOK_BEAM_DECI);
            int32_t const dx = p_ray->x_mm - p_from->x_mm;
            int32_t const dy = p_ray->y_mm - p_from->y_mm;
            int32_t const depth = geom_dot_mm(normal, dx, dy) + geom_mul_q14(p_ray->range_mm, geom_cos_q14(hit));
            int32_t const ahead = geom_dot_mm(along, dx, dy) + sign * geom_mul_q14(p_ray->range_mm, geom_sin_q14(hit));
            if ((geom_abs(depth - p_face->dist_mm) <= LOOK_FACE_MATCH_MM) && (ahead > reach_mm))
            {
                reach_mm = ahead;
            }
        }
    }
    return reach_mm;
}

/* The rays that meet the face on one side, nearest to the face centre first;
 * the edge is the ray after the last one on the face. */
static bool side_edge(look_t const * p_look, look_face_t const * p_face,
                      look_axes_t const * p_axes, look_hits_t const * p_hits,
                      look_side_t side, look_edge_t * p_edge)
{
    look_cand_t cand[LOOK_RAYS_MAX];
    uint8_t     last_on = NO_RAY;
    bool        found   = false;
    uint8_t     n       = 0U;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        int32_t const out = (LOOK_SIDE_LEFT == side) ? p_hits->t_mm[i] : -p_hits->t_mm[i];

        if ((0U != (p_hits->hit & (UINT32_C(1) << i))) && (out > 0))
        {
            uint8_t j = n;

            while ((j > 0U) && (geom_abs(cand[j - 1U].t_mm) > out))
            {
                cand[j] = cand[j - 1U];
                j--;
            }
            cand[j].t_mm = p_hits->t_mm[i];
            cand[j].idx  = i;
            n++;
        }
    }

    /* The edge is the ray after the last one on the face: a lone OPEN ray
     * among rays on the face is a dropout, not the end of the face. */
    for (uint8_t k = 0U; k < n; k++)
    {
        if (0U == (p_hits->past & (UINT32_C(1) << cand[k].idx)))
        {
            last_on = k;
        }
    }
    if ((NO_RAY != last_on) && ((uint8_t)(last_on + 1U) < n))
    {
        look_ray_t const * const p_ray = &p_look->rays[cand[last_on + 1U].idx];
        int32_t const ed_deci = incidence_deci(p_face, look_ray_angle_deci(p_ray));

        if (ed_deci <= LOOK_TRUST_DECI)
        {
            /* The beam sees the nearest face inside its half-angle, so the
             * edge lies half a beam nearer the face centre than each ray. */
            int32_t const on_deci = incidence_deci(p_face,
                                                   look_ray_angle_deci(&p_look->rays[cand[last_on].idx]));
            int32_t const mid     = (on_deci + ed_deci - (TWICE * LOOK_BEAM_DECI)) / HALVES;
            int32_t const world   = p_face->normal_deci
                                  + ((LOOK_SIDE_LEFT == side) ? mid : -mid);

            found = hit_face(p_face, p_axes, p_ray->x_mm, p_ray->y_mm, world, &p_edge->x_mm,
                             &p_edge->y_mm);
        }
    }
    return (found);
}

void look_edges(look_t const * p_look, look_face_t const * p_face,
                look_edge_t * p_edge, bool * p_have)
{
    look_axes_t axes;
    look_hits_t hits;

    axes.cos_n = geom_cos_q14(p_face->normal_deci);
    axes.sin_n = geom_sin_q14(p_face->normal_deci);
    axes.cos_t = geom_cos_q14(p_face->normal_deci + DECI_RIGHT);
    axes.sin_t = geom_sin_q14(p_face->normal_deci + DECI_RIGHT);
    hits.hit   = 0U;
    hits.past  = 0U;

    /* Where each ray meets the face, once for both sides. */
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        int32_t                  hx    = 0;
        int32_t                  hy    = 0;

        hits.t_mm[i] = 0;
        if (hit_face(p_face, &axes, p_ray->x_mm, p_ray->y_mm, look_ray_angle_deci(p_ray), &hx, &hy))
        {
            hits.hit |= UINT32_C(1) << i;
            hits.t_mm[i] = geom_dot_q14(axes.cos_t, axes.sin_t, hx - p_face->x_mm,
                                        hy - p_face->y_mm);
            if (reads_past_face(p_face, &axes, p_ray))
            {
                hits.past |= UINT32_C(1) << i;
            }
        }
    }
    for (uint8_t s = 0U; s < LOOK_SIDES; s++)
    {
        p_have[s] = side_edge(p_look, p_face, &axes, &hits, (look_side_t)s, &p_edge[s]);
    }
}

static int16_t clamp_mm(int32_t value)
{
    int32_t clamped = value;

    if (clamped > INT16_MAX_MM)
    {
        clamped = INT16_MAX_MM;
    }
    else if (clamped < INT16_MIN_MM)
    {
        clamped = INT16_MIN_MM;
    }
    else
    {
        /* In range. */
    }
    return ((int16_t)clamped);
}

bool look_echo_is_point(uint8_t echo)
{
    return ((LOOK_ECHO_VALID == echo) || (LOOK_ECHO_TOO_CLOSE == echo));
}

static void ray_point(look_ray_t const * p_ray, int32_t * p_x_mm, int32_t * p_y_mm)
{
    int32_t const angle_deci = look_ray_angle_deci(p_ray);

    *p_x_mm = p_ray->x_mm + geom_mul_q14((int32_t)p_ray->range_mm, geom_cos_q14(angle_deci));
    *p_y_mm = p_ray->y_mm + geom_mul_q14((int32_t)p_ray->range_mm, geom_sin_q14(angle_deci));
}

/* Where a ray from (ox, oy) at angle_deci meets the face line. */
static bool hit_face(look_face_t const * p_face, look_axes_t const * p_axes, int32_t ox_mm,
                     int32_t oy_mm, int32_t angle_deci, int32_t * p_x_mm, int32_t * p_y_mm)
{
    int32_t const slant = geom_cos_q14(angle_deci - p_face->normal_deci);
    int64_t const along = ((int64_t)p_axes->cos_n * (p_face->x_mm - ox_mm))
                          + ((int64_t)p_axes->sin_n * (p_face->y_mm - oy_mm));
    bool hit = false;

    if ((slant > 0) && (along >= 0))
    {
        int32_t const reach = (int32_t)(along / slant);

        *p_x_mm = ox_mm + geom_mul_q14(reach, geom_cos_q14(angle_deci));
        *p_y_mm = oy_mm + geom_mul_q14(reach, geom_sin_q14(angle_deci));
        hit     = true;
    }
    return (hit);
}

static int32_t incidence_deci(look_face_t const * p_face, int32_t angle_deci)
{
    return (geom_abs(geom_wrap_deci(angle_deci - p_face->normal_deci)));
}

static bool reads_past_face(look_face_t const * p_face, look_axes_t const * p_axes,
                            look_ray_t const * p_ray)
{
    bool past = true;

    if (look_echo_is_point(look_ray_echo(p_ray)))
    {
        int32_t x = 0;
        int32_t y = 0;

        ray_point(p_ray, &x, &y);
        past = geom_dot_q14(p_axes->cos_n, p_axes->sin_n, x - p_face->x_mm, y - p_face->y_mm)
               > LOOK_EDGE_STEP_MM;
    }
    return (past);
}
