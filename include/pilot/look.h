/** @file look.h
 * @brief Fit a face to one stationary sonar sweep and find its edges.
 *
 * Pure C99, integer only, no kernel and no driver calls. The caller feeds
 * each ping of a sweep to look_add(), then asks look_face() for the face the
 * VALID pings lie on and look_edges() for where that face ends on each side.
 *
 * Frame: x and y in mm with x ahead of the car at heading 0 and y to its
 * left, angles in deci-degrees, counter-clockwise positive, 0 along +x, 900
 * along +y. A servo angle is relative to the car front, positive to the left.
 * The legacy navigator uses x left and y ahead. To exchange points with it,
 * swap x and y and keep the heading.
 *
 * Trigonometry comes from src/navigation/obstacle/geometry.c, which must be
 * linked next to look.c (include path src/navigation/obstacle).
 */

#ifndef PILOT_LOOK_H
#define PILOT_LOOK_H

#include <stdbool.h>
#include <stdint.h>

#include "navigation/obstacle_ping.h"
#include "platform/chassis.h"
#include "platform/static_assert.h"

/** Rays in one look. A full look refuses the new ray. */
#define LOOK_RAYS_MAX (24U)

/* The face fitter consumes the sonar service's observation codes. */
#define LOOK_ECHO_VALID     OBSTACLE_CLASS_VALID
#define LOOK_ECHO_OPEN      OBSTACLE_CLASS_OPEN
#define LOOK_ECHO_TOO_CLOSE OBSTACLE_CLASS_TOO_CLOSE
/** No usable reading (no echo, a glitch, a stuck echo line). It says nothing
 *  about the scene: look_add() refuses it and it is never OPEN. */
#define LOOK_ECHO_NONE      OBSTACLE_ECHO_UNUSABLE

/** Sonar position ahead of the axle. */
#define LOOK_SONAR_FWD_MM CHASSIS_SONAR_FORWARD_MM

/** A ray whose incidence on the face is above this is never an edge. */
#define LOOK_TRUST_DECI (250)

/** Half-angle of the sonar beam. The reading is the nearest face inside it. */
#define LOOK_BEAM_DECI CHASSIS_SONAR_BEAM_HALF_DECI

/** A reading this far beyond the face line, or more, ends the face. */
#define LOOK_EDGE_STEP_MM (60)

/** Normal-distance tolerance for associating a return with a measured face. */
#define LOOK_FACE_MATCH_MM (25)

/* Offset the wrapped bearing (-1800, 1800] to keep all deci-degrees in 12 bits. */
#define LOOK_RAY_ANGLE_BIAS  (1800)
#define LOOK_RAY_ANGLE_MASK  (0x0FFFU)
#define LOOK_RAY_ECHO_SHIFT  (12U)
#define LOOK_RAY_ECHO_MASK   (0x3U)

typedef struct
{
    int16_t  x_mm; /* sonar position when the ping was sent */
    int16_t  y_mm;
    uint16_t range_mm; /* 0 for OPEN */
    uint16_t angle_echo;
} look_ray_t;

typedef struct
{
    look_ray_t rays[LOOK_RAYS_MAX];
    uint8_t    count;
} look_t;

STATIC_ASSERT((2U * LOOK_RAY_ANGLE_BIAS) <= LOOK_RAY_ANGLE_MASK, "bearing fits 12 bits");
STATIC_ASSERT(LOOK_ECHO_NONE <= LOOK_RAY_ECHO_MASK, "echo fits 2 bits");
STATIC_ASSERT(sizeof(look_ray_t) == 8U, "look_ray_t must stay 8 bytes");
STATIC_ASSERT(sizeof(look_t) == 194U, "look_t must stay 194 bytes");

static inline int32_t look_ray_angle_deci(look_ray_t const * p_ray)
{
    return ((int32_t)(p_ray->angle_echo & LOOK_RAY_ANGLE_MASK) - LOOK_RAY_ANGLE_BIAS);
}

static inline uint8_t look_ray_echo(look_ray_t const * p_ray)
{
    return ((uint8_t)(((uint32_t)p_ray->angle_echo >> LOOK_RAY_ECHO_SHIFT) & LOOK_RAY_ECHO_MASK));
}

/** Update the bearing after wrapping it into (-1800, 1800]. */
static inline void look_ray_set_angle(look_ray_t * p_ray, int32_t angle_deci)
{
    p_ray->angle_echo = (uint16_t)((p_ray->angle_echo & (LOOK_RAY_ECHO_MASK << LOOK_RAY_ECHO_SHIFT))
                                  | (uint32_t)(angle_deci + LOOK_RAY_ANGLE_BIAS));
}

static inline void look_ray_mark_open(look_ray_t * p_ray)
{
    p_ray->angle_echo = (uint16_t)((p_ray->angle_echo & LOOK_RAY_ANGLE_MASK)
                                  | (LOOK_ECHO_OPEN << LOOK_RAY_ECHO_SHIFT));
    p_ray->range_mm   = 0U;
}

typedef struct
{
    int32_t x_mm;
    int32_t y_mm;
    int32_t head_deci;
} look_pose_t;

STATIC_ASSERT(sizeof(look_pose_t) == 12U, "look_pose_t size");

typedef struct
{
    int32_t x_mm;        /* centre of the VALID points */
    int32_t y_mm;
    int32_t normal_deci; /* from the position toward the face */
    int32_t dist_mm;     /* from the position to the face line */
} look_face_t;

STATIC_ASSERT(sizeof(look_face_t) == 16U, "look_face_t size");

typedef enum
{
    LOOK_SIDE_LEFT = 0, /* left of the normal, seen from the car */
    LOOK_SIDE_RIGHT
} look_side_t;

STATIC_ASSERT(sizeof(look_side_t) <= 4U, "look_side_t size");

typedef struct
{
    int32_t x_mm; /* beam-corrected point on the face line */
    int32_t y_mm;
} look_edge_t;

STATIC_ASSERT(sizeof(look_edge_t) == 8U, "look_edge_t size");

/** A VALID or TOO_CLOSE reading is a point; OPEN and NONE are not. */
bool look_echo_is_point(uint8_t echo);

void look_clear(look_t * p_look);

/**
 * Store one ping as a ray.
 *
 * @return true when the ray was stored. false for a full look, for an echo
 *         class other than the three above, and for a
 *         VALID or TOO_CLOSE reading that is not positive.
 */
bool look_add(look_t * p_look, look_pose_t const * p_pose, int32_t servo_deci,
              uint8_t echo, int32_t range_mm);

/**
 * Fit the face through the VALID points (orthogonal least squares).
 *
 * @param p_from Position the distance and normal refer to. Only x and y are
 *               used.
 * @return false for fewer than 2 VALID points or points on one spot.
 */
bool look_face(look_t const * p_look, look_pose_t const * p_from,
               look_face_t * p_face);

/** Fit the face using only positive echoes up to max_range_mm. The complete
 *  look remains available for checking which rays pass beyond that face.
 *  A ray that lies more than LOOK_FACE_MATCH_MM off the face fitted without
 *  it is a stray: the single worst one is dropped and the face refitted once. */
bool look_face_near(look_t const * p_look, look_pose_t const * p_from,
                    uint16_t max_range_mm, look_face_t * p_face);

/** Furthest beam-corrected on-face return along the selected side, measured
 *  from the axle. Zero means no positive extent was observed there. */
int32_t look_face_reach_mm(look_t const * p_look, look_pose_t const * p_from,
                           look_face_t const * p_face, look_side_t side);

/**
 * Find where the face ends on each side.
 *
 * Going outward from the face centre, the edge is the first ray that reads
 * OPEN or more than LOOK_EDGE_STEP_MM beyond the face line, provided its
 * incidence on the face is at most LOOK_TRUST_DECI. The point is on the face
 * line at the middle of the beam edges facing the face of the last ray on the
 * face and of the edge ray.
 *
 * @param[out] p_edge Two edges, indexed by look_side_t; set only where the side
 *                    has an edge.
 * @param[out] p_have Two flags: whether each side has an edge.
 */
void look_edges(look_t const * p_look, look_face_t const * p_face,
                look_edge_t * p_edge, bool * p_have);

#endif /* PILOT_LOOK_H */
