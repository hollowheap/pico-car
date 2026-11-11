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

bool barcode_width_um (uint32_t                  interval_us,
                       motion_feedback_t const * p_feedback,
                       uint16_t                * p_width_um)
{
    int64_t  mean_speed_um_s = 0;
    uint64_t width           = 0ULL;
    size_t   i               = 0U;

    if ((NULL == p_feedback) || (NULL == p_width_um))
    {
        return false;
    }

    if (!p_feedback->fresh)
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
