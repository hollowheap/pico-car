#include <stddef.h>

#include "track/line_recovery.h"

bool line_recovery_init (line_recovery_t * p_recovery, int32_t track_um)
{
    if ((p_recovery == NULL) || (track_um <= 0))
    {
        return false;
    }

    *p_recovery = (line_recovery_t){
        .track_um      = track_um,
        .last_left_um  = 0,
        .last_right_um = 0,
        .heading_urad  = 0,
        .initialised   = true,
    };
    return true;
}

void line_recovery_reset (line_recovery_t * p_recovery, int64_t left_um,
                          int64_t right_um)
{
    if ((p_recovery == NULL) || !p_recovery->initialised)
    {
        return;
    }
    p_recovery->last_left_um  = left_um;
    p_recovery->last_right_um = right_um;
    p_recovery->heading_urad  = 0;
}

void line_recovery_update (line_recovery_t * p_recovery, int64_t left_um,
                           int64_t right_um)
{
    if ((p_recovery == NULL) || !p_recovery->initialised)
    {
        return;
    }

    const int64_t delta_left  = left_um - p_recovery->last_left_um;
    const int64_t delta_right = right_um - p_recovery->last_right_um;
    p_recovery->last_left_um  = left_um;
    p_recovery->last_right_um = right_um;
    p_recovery->heading_urad +=
        ((delta_right - delta_left) * INT64_C(1000000)) / p_recovery->track_um;
}

int32_t line_recovery_heading_mdeg (const line_recovery_t * p_recovery)
{
    if ((p_recovery == NULL) || !p_recovery->initialised)
    {
        return 0;
    }
    return (int32_t)((p_recovery->heading_urad * INT64_C(180000)) /
                     INT64_C(3141593));
}
