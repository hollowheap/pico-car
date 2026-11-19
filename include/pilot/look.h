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
 * The geometry.h pose frame swaps the axes (x left, y ahead): pass
 * x = geom_pose_y_mm(), y = geom_pose_x_mm() with the heading unchanged, and
 * swap the face and edge points back the same way.
 *
 * Trigonometry comes from src/navigation/obstacle/geometry.c, which must be
 * linked next to look.c (include path src/navigation/obstacle).
 */

#ifndef PILOT_LOOK_H
#define PILOT_LOOK_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/** Rays in one look. A full look refuses the new ray. */
#define LOOK_RAYS_MAX (24U)

/**
 * Echo classes of a ping. Copies of OBSTACLE_ECHO_IDX_VALID, _OPEN and
 * _TOO_CLOSE in include/navigation/obstacle.h (that header needs the kernel);
 * keep the values in step. TOO_CLOSE counts as a VALID point at its range.
 */
#define LOOK_ECHO_VALID     (0U)
#define LOOK_ECHO_OPEN      (1U)
#define LOOK_ECHO_TOO_CLOSE (2U)
/** No usable reading (no echo, a glitch, a stuck echo line). It says nothing
 *  about the scene: look_add() refuses it and it is never OPEN. */
#define LOOK_ECHO_NONE      (3U)

/** Sonar position ahead of the axle. Copy of OBSTACLE_SONAR_FWD_MM. */
#define LOOK_SONAR_FWD_MM (70)

/** A ray whose incidence on the face is above this is never an edge. */
#define LOOK_TRUST_DECI (250)

/** Half-angle of the sonar beam. The reading is the nearest face inside it. */
#define LOOK_BEAM_DECI (95)

/** A reading this far beyond the face line, or more, ends the face. */
#define LOOK_EDGE_STEP_MM (60)

typedef struct
{
    int16_t  x_mm;       /* sonar position when the ping was sent */
    int16_t  y_mm;
    int16_t  angle_deci; /* world angle of the beam, (-1800, 1800] */
    uint16_t range_mm;   /* 0 for OPEN */
    uint8_t  echo;       /* LOOK_ECHO_* */
    uint8_t  spare;
} look_ray_t;

typedef struct
{
    look_ray_t rays[LOOK_RAYS_MAX];
    uint8_t    count;
} look_t;

STATIC_ASSERT(sizeof(look_ray_t) == 10U, "look_ray_t must stay 10 bytes");
STATIC_ASSERT(sizeof(look_t) == 242U, "look_t must stay 242 bytes");

typedef struct
{
    int32_t x_mm;
    int32_t y_mm;
    int32_t head_deci;
} look_pose_t;

typedef struct
{
    int32_t x_mm;        /* centre of the VALID points */
    int32_t y_mm;
    int32_t normal_deci; /* from the position toward the face */
    int32_t dist_mm;     /* from the position to the face line */
} look_face_t;

typedef enum
{
    LOOK_SIDE_LEFT = 0, /* left of the normal, seen from the car */
    LOOK_SIDE_RIGHT
} look_side_t;

typedef struct
{
    int32_t x_mm; /* beam-corrected point on the face line */
    int32_t y_mm;
} look_edge_t;

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
