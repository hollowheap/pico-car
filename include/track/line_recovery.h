#ifndef TRACK_LINE_RECOVERY_H
#define TRACK_LINE_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t  track_um;
    int64_t  last_left_um;
    int64_t  last_right_um;
    int64_t  heading_urad;
    bool     initialised;
} line_recovery_t;

bool line_recovery_init (line_recovery_t * p_recovery, int32_t track_um);
void line_recovery_reset (line_recovery_t * p_recovery, int64_t left_um,
                          int64_t right_um);
void line_recovery_update (line_recovery_t * p_recovery, int64_t left_um,
                           int64_t right_um);
int32_t line_recovery_heading_mdeg (const line_recovery_t * p_recovery);

#endif /* TRACK_LINE_RECOVERY_H */
