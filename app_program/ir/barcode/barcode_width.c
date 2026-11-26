/** @file barcode_width.c
 *
 * @brief Convert barcode element pulse intervals to distance with motion
 * feedback snapshot.
 */

#include "ir/barcode/barcode.h"
#include "platform/units.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define US_PER_S  (UINT64_C(1000000))
#define PM_PER_UM (INT64_C(1000000))
#define DECI_PER_TURN (3600)
#define RAD_PER_DECI (0.00174532925f)
/* A turn moves a point at radius r along a chord of 2 r sin(half turn). */
#define CHORD_PER_SINE (2.0f)

bool barcode_element_is_block (bool is_space, uint16_t width_um)
{
    return (!is_space) && (width_um > BARCODE_BLOCK_UM);
}

#define ODO_MASK       (0x7FFFFFFFUL)
/* Half of the 31-bit range: a larger difference is travel backward. */
#define ODO_BACKWARD   (0x40000000UL)

void barcode_block_init (barcode_block_t * p_block)
{
    *p_block = (barcode_block_t)BARCODE_BLOCK_INIT;
}

uint16_t barcode_block_edge (barcode_block_t * p_block, bool is_black, uint32_t odo_mm,
                            const uint16_t * p_width_um, bool * p_measured)
{
    uint16_t report_mm = 0U;
    bool b_measured = (NULL != p_width_um) && barcode_element_is_block(is_black, *p_width_um);

    if (UINT32_MAX == odo_mm)
    {
        p_block->b_black = 0U;
    }
    else if (is_black)
    {
        p_block->black_mm = odo_mm & ODO_MASK;
        p_block->b_black  = 1U;
    }
    else if (0U != p_block->b_black)
    {
        uint32_t length_mm = (odo_mm - p_block->black_mm) & ODO_MASK;
        /* Travel since the last event; behind it the difference wraps high. */
        uint32_t const since_mm = odo_mm - p_block->last_event_mm;

        p_block->b_black = 0U;
        if (NULL == p_width_um)
        {
            b_measured = (length_mm < ODO_BACKWARD) && (length_mm > (BARCODE_BLOCK_UM / UM_PER_MM));
        }
        else if ((*p_width_um != UINT16_MAX) || (length_mm >= ODO_BACKWARD) ||
                 (length_mm < (UINT16_MAX / UM_PER_MM)))
        {
            length_mm = (uint32_t)*p_width_um / UM_PER_MM;
        }
        if (b_measured && (since_mm >= BARCODE_BLOCK_EVENT_MM) && (since_mm <= ODO_MASK))
        {
            p_block->last_event_mm = odo_mm;
            report_mm = (length_mm > (uint32_t)INT16_MAX) ? (uint16_t)INT16_MAX
                                                          : (uint16_t)length_mm;
        }
    }
    else
    {
        /* A white edge with no black edge before it. */
    }

    if (p_measured != NULL)
    {
        *p_measured = b_measured;
    }
    return report_mm;
}

bool barcode_width_um (uint32_t                  interval_us,
                       uint32_t                  end_us,
                       motion_feedback_t const * p_feedback,
                       uint16_t                * p_width_um)
{
    int64_t  mean_speed_um_s = 0;
    uint64_t width           = 0ULL;
    size_t   i               = 0U;
    uint32_t start_us        = end_us - interval_us;

    if ((NULL == p_feedback) || (NULL == p_width_um))
    {
        return false;
    }

    if (!p_feedback->fresh)
    {
        return false;
    }

    /* Modular time: after the start by at most the element plus the slack,
     * or before it by at most the slack. */
    if (((p_feedback->timestamp_us - start_us) > ((uint64_t)interval_us + BARCODE_SPEED_SLACK_US)) &&
        ((start_us - p_feedback->timestamp_us) > BARCODE_SPEED_SLACK_US))
    {
        return false;
    }

    for (i = 0U; i < (size_t)MOTION_WHEELS; i++)
    {
        if (!p_feedback->speed_valid[i])
        {
            return false;
        }
    }

    if ((p_feedback->left_speed_um_s < 0) || (p_feedback->right_speed_um_s < 0))
    {
        return false;
    }

    mean_speed_um_s = ((int64_t)p_feedback->left_speed_um_s +
                       (int64_t)p_feedback->right_speed_um_s) /
                      (int64_t)MOTION_WHEELS;

    if (mean_speed_um_s <= 0)
    {
        return false;
    }

    width = ((uint64_t)interval_us * (uint64_t)mean_speed_um_s) / US_PER_S;

    if (width > (uint64_t)UINT16_MAX)
    {
        *p_width_um = UINT16_MAX;
    }
    else
    {
        *p_width_um = (uint16_t)width;
    }

    return true;
}

/* The two bounded unsigned differences also cover a wrapping device clock. */
static bool edge_offset_us (uint32_t edge_us, uint32_t sample_us, int32_t * p_offset)
{
    uint32_t const after = edge_us - sample_us;
    uint32_t const before = sample_us - edge_us;

    if (after <= BARCODE_SPEED_SLACK_US)
    {
        *p_offset = (int32_t)after;
        return true;
    }
    if (before <= BARCODE_SPEED_SLACK_US)
    {
        *p_offset = -(int32_t)before;
        return true;
    }
    return false;
}

bool barcode_odometry_covers_start (uint32_t start_us, const motion_feedback_t * p_feedback)
{
    return ((p_feedback->timestamp_us - start_us) <= p_feedback->odometry_span_us) ||
           ((start_us - p_feedback->timestamp_us) <= BARCODE_SPEED_SLACK_US);
}

static bool wheel_travel (int64_t start_um, int64_t end_um, int64_t correction_pm,
                          int64_t * p_distance_pm)
{
    uint64_t const delta = (uint64_t)end_um - (uint64_t)start_um;

    if ((end_um < start_um) || (delta > UINT32_MAX))
    {
        return false;
    }
    *p_distance_pm = ((int64_t)delta * PM_PER_UM) + correction_pm;
    return *p_distance_pm >= 0;
}

static float sensor_travel_on_tape (const motion_feedback_t * p_start, int32_t start_offset_us,
                                   int32_t tape_heading_deci, int64_t left_pm, int64_t right_pm)
{
    float const track_pm = (float)(CHASSIS_WHEEL_TRACK_MM * UM_PER_MM) * (float)PM_PER_UM;
    int64_t const yaw_deci = wheel_yaw_deci(p_start->right_displacement_um -
                                           p_start->left_displacement_um);
    int64_t const yaw_speed = (int64_t)p_start->right_speed_um_s - p_start->left_speed_um_s;
    float const start_rad = (float)((yaw_deci - tape_heading_deci) % DECI_PER_TURN) * RAD_PER_DECI +
                            (float)(yaw_speed * start_offset_us) / track_pm;
    float const half_turn = (float)(right_pm - left_pm) / (2.0f * track_pm);
    float const mid_rad = start_rad + half_turn;
    float const sin_half = sinf(half_turn);
    float const arc_um = (float)(left_pm + right_pm) / (2.0f * (float)PM_PER_UM);
    float const chord_um = (fabsf(half_turn) < 0.001f) ? arc_um : arc_um * sin_half / half_turn;

    /* A left turn carries the offset left sensor backwards along the tape. */
    float const side_um = (float)(CHASSIS_LEFT_REACH_MM * UM_PER_MM);
    float const front_um = (float)(CHASSIS_BARCODE_FORWARD_MM * UM_PER_MM);
    return (chord_um - CHORD_PER_SINE * side_um * sin_half) * cosf(mid_rad) -
           CHORD_PER_SINE * front_um * sin_half * sinf(mid_rad);
}

bool barcode_width_between_edges (uint32_t start_us, uint32_t end_us,
                                  const motion_feedback_t * p_start,
                                  const motion_feedback_t * p_end,
                                  int32_t tape_heading_deci,
                                  uint16_t * p_width_um)
{
    int32_t start_offset_us = 0;
    int32_t end_offset_us = 0;
    int64_t left_pm = 0;
    int64_t right_pm = 0;

    if ((NULL == p_start) || (NULL == p_end) || (NULL == p_width_um))
    {
        return false;
    }
    for (size_t wheel = 0U; wheel < (size_t)MOTION_WHEELS; wheel++)
    {
        if (p_start->invalid_samples[wheel] != p_end->invalid_samples[wheel])
        {
            return false;
        }
    }
    /* A known invalid or reversing opening interval cannot use the speed fallback. */
    if (p_start->fresh && (!p_start->speed_valid[0] || !p_start->speed_valid[1] ||
                          (p_start->left_speed_um_s < 0) || (p_start->right_speed_um_s < 0)))
    {
        return false;
    }
    bool const b_end_odometry = p_end->fresh && p_end->odometry_valid[0] && p_end->odometry_valid[1];

    if (b_end_odometry && !barcode_odometry_covers_start(start_us, p_end))
    {
        return false;
    }
    if (!p_start->fresh || !p_start->odometry_valid[0] || !p_start->odometry_valid[1] || !b_end_odometry)
    {
        return barcode_width_um(end_us - start_us, end_us, p_end, p_width_um);
    }
    if (!p_end->speed_valid[0] || !p_end->speed_valid[1] ||
        (p_end->left_speed_um_s < 0) || (p_end->right_speed_um_s < 0) ||
        !edge_offset_us(start_us, p_start->timestamp_us, &start_offset_us) ||
        !edge_offset_us(end_us, p_end->timestamp_us, &end_offset_us))
    {
        return false;
    }
    /* A speed in um/s times an offset in us gives picometres. Preserve
     * positive sub-micrometre travel so the parser can merge a short pulse. */
    int64_t const left_correction_pm = ((int64_t)p_end->left_speed_um_s * end_offset_us) -
                                      ((int64_t)p_start->left_speed_um_s * start_offset_us);
    int64_t const right_correction_pm = ((int64_t)p_end->right_speed_um_s * end_offset_us) -
                                       ((int64_t)p_start->right_speed_um_s * start_offset_us);

    if (!wheel_travel(p_start->left_displacement_um, p_end->left_displacement_um,
                      left_correction_pm, &left_pm) ||
        !wheel_travel(p_start->right_displacement_um, p_end->right_displacement_um,
                      right_correction_pm, &right_pm))
    {
        return false;
    }
    if (BARCODE_TAPE_HEADING_UNKNOWN != tape_heading_deci)
    {
        float const width_um = sensor_travel_on_tape(p_start, start_offset_us, tape_heading_deci,
                                                     left_pm, right_pm);
        if (width_um <= 0.0f)
        {
            return false;
        }
        *p_width_um = (width_um > (float)UINT16_MAX) ? UINT16_MAX : (uint16_t)width_um;
        return true;
    }

    int64_t const travel_pm = left_pm + right_pm;
    int64_t const width_um = travel_pm / ((int64_t)MOTION_WHEELS * PM_PER_UM);

    *p_width_um = (width_um > UINT16_MAX) ? UINT16_MAX : (uint16_t)width_um;
    return travel_pm > 0;
}

/*** end of file ***/
