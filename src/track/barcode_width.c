/** @file barcode_width.c
 *
 * @brief Convert barcode element pulse intervals to distance with motion
 * feedback snapshot (D81, S33).
 */

#include "track/barcode.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define US_PER_S (UINT64_C(1000000))

bool barcode_element_is_block (bool is_space, uint16_t width_um)
{
    return (!is_space) && (width_um > BARCODE_BLOCK_UM);
}

#define UM_PER_MM      (1000U)
#define ODO_MASK       (0x7FFFFFFFUL)
/* Half of the 31-bit range: a larger difference is travel backward. */
#define ODO_BACKWARD   (0x40000000UL)

void barcode_block_init (barcode_block_t * p_block)
{
    *p_block = (barcode_block_t)BARCODE_BLOCK_INIT;
}

uint16_t barcode_block_edge (barcode_block_t * p_block, bool is_black, uint32_t odo_mm,
                            bool * p_measured)
{
    uint16_t report_mm = 0U;
    bool b_measured = false;

    if (is_black)
    {
        p_block->black_mm = odo_mm & ODO_MASK;
        p_block->b_black  = 1U;
    }
    else if (0U != p_block->b_black)
    {
        uint32_t const length_mm = (odo_mm - p_block->black_mm) & ODO_MASK;
        /* Travel since the last event; behind it the difference wraps high. */
        uint32_t const since_mm = odo_mm - p_block->last_event_mm;

        p_block->b_black = 0U;
        b_measured = (length_mm < ODO_BACKWARD) && (length_mm > (BARCODE_BLOCK_UM / UM_PER_MM));
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

/*** end of file ***/
