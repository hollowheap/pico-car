/** @file b4_shared.c
 *
 * @brief Packing helper for B4 telemetry copies.
 */

#include <stddef.h>

#include "b4_shared.h"

void b4_pack_msg (uint32_t run_id, uint32_t event_id, uint32_t now_ms,
                  float peak_mm, float pitch_deg, float run_peak_mm,
                  uint8_t state, uint8_t motion, uint8_t fault,
                  b4_hump_msg_t * p_msg)
{
    if (p_msg == NULL)
    {
        return;
    }

    p_msg->run_id         = run_id;
    p_msg->event_id       = event_id;
    p_msg->timestamp_ms   = now_ms;
    p_msg->peak_height_mm = peak_mm;
    p_msg->peak_pitch_deg = pitch_deg;
    p_msg->run_peak_mm    = run_peak_mm;
    p_msg->state          = state;
    p_msg->motion         = motion;
    p_msg->fault          = fault;
}

/*** end of file ***/
