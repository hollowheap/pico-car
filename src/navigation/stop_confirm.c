#include <stddef.h>

#include "navigation/stop_confirm.h"

void stop_confirm_init (stop_confirm_t * p_confirm, uint32_t now_us,
                        int64_t left_um, int64_t right_um)
{
    if (p_confirm == NULL)
    {
        return;
    }
    *p_confirm = (stop_confirm_t){
        .left_um         = left_um,
        .right_um        = right_um,
        .quiet_since_us  = now_us,
        .have_last       = true,
    };
}

void stop_confirm_update (stop_confirm_t * p_confirm, uint32_t now_us,
                          int64_t left_um, int64_t right_um)
{
    if (p_confirm == NULL)
    {
        return;
    }
    if (!p_confirm->have_last ||
        (left_um != p_confirm->left_um) ||
        (right_um != p_confirm->right_um))
    {
        p_confirm->left_um        = left_um;
        p_confirm->right_um       = right_um;
        p_confirm->quiet_since_us = now_us;
        p_confirm->have_last      = true;
    }
}

bool stop_confirm_ready (const stop_confirm_t * p_confirm, uint32_t now_us)
{
    if ((p_confirm == NULL) || !p_confirm->have_last)
    {
        return false;
    }
    return (uint32_t)(now_us - p_confirm->quiet_since_us) >=
           STOP_CONFIRM_QUIET_US;
}
