#ifndef NAVIGATION_STOP_CONFIRM_H
#define NAVIGATION_STOP_CONFIRM_H

#include <stdbool.h>
#include <stdint.h>

#define STOP_CONFIRM_QUIET_US 50000U

typedef struct
{
    int64_t  left_um;
    int64_t  right_um;
    uint32_t quiet_since_us;
    bool     have_last;
} stop_confirm_t;

void stop_confirm_init (stop_confirm_t * p_confirm, uint32_t now_us,
                        int64_t left_um, int64_t right_um);
void stop_confirm_update (stop_confirm_t * p_confirm, uint32_t now_us,
                          int64_t left_um, int64_t right_um);
bool stop_confirm_ready (const stop_confirm_t * p_confirm, uint32_t now_us);

#endif /* NAVIGATION_STOP_CONFIRM_H */
