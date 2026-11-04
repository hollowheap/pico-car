#ifndef TRACK_BARCODE_FILTER_H
#define TRACK_BARCODE_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#define BARCODE_SUPPRESS_US 250000U
#define BARCODE_REARM_UM    100000

typedef struct
{
    uint32_t suppress_until_us;
    int64_t  last_marker_um;
    int64_t  rearm_distance_um;
    bool     armed;
    bool     initialised;
} barcode_filter_t;

bool barcode_filter_init (barcode_filter_t * p_filter, int64_t rearm_distance_um);
void barcode_filter_note_marker (barcode_filter_t * p_filter, uint32_t now_us,
                                 int64_t distance_um);
void barcode_filter_update (barcode_filter_t * p_filter, int64_t distance_um);
bool barcode_filter_suppresses_line_lost (const barcode_filter_t * p_filter,
                                          uint32_t now_us);
bool barcode_filter_is_armed (const barcode_filter_t * p_filter);

#endif /* TRACK_BARCODE_FILTER_H */
