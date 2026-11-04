#include <stddef.h>

#include "track/barcode_filter.h"

bool barcode_filter_init (barcode_filter_t * p_filter, int64_t rearm_distance_um)
{
    if ((p_filter == NULL) || (rearm_distance_um <= 0))
    {
        return false;
    }

    *p_filter = (barcode_filter_t){
        .suppress_until_us = 0U,
        .last_marker_um    = 0,
        .rearm_distance_um = rearm_distance_um,
        .armed             = true,
        .initialised       = true,
    };
    return true;
}

void barcode_filter_note_marker (barcode_filter_t * p_filter, uint32_t now_us,
                                 int64_t distance_um)
{
    if ((p_filter == NULL) || !p_filter->initialised)
    {
        return;
    }
    p_filter->suppress_until_us = now_us + BARCODE_SUPPRESS_US;
    p_filter->last_marker_um    = distance_um;
    p_filter->armed             = false;
}

void barcode_filter_update (barcode_filter_t * p_filter, int64_t distance_um)
{
    if ((p_filter == NULL) || !p_filter->initialised || p_filter->armed)
    {
        return;
    }
    if ((distance_um - p_filter->last_marker_um) >= p_filter->rearm_distance_um)
    {
        p_filter->armed = true;
    }
}

bool barcode_filter_suppresses_line_lost (const barcode_filter_t * p_filter,
                                          uint32_t now_us)
{
    if ((p_filter == NULL) || !p_filter->initialised)
    {
        return false;
    }
    return (int32_t)(now_us - p_filter->suppress_until_us) < 0;
}

bool barcode_filter_is_armed (const barcode_filter_t * p_filter)
{
    return (p_filter != NULL) && p_filter->initialised && p_filter->armed;
}
