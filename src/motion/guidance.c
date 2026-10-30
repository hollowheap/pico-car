/** @file guidance.c
 * @brief Translate high-level guidance without reading another buddy's GPIO.
 */
#include <limits.h>
#include <stddef.h>
#include "guidance.h"

bool guidance_plan (const guidance_request_t * p_request,
                    const uint32_t limits_um_s[ENCODER_WHEELS],
                    uint32_t max_ttl_us, uint32_t now_us,
                    motion_command_t * p_command)
{
    motion_command_t command = {0};
    unsigned int     wheel;

    if ((p_request == NULL) || (limits_um_s == NULL) || (p_command == NULL) ||
        (max_ttl_us == 0U) || (max_ttl_us > (uint32_t)INT32_MAX) ||
        (p_request->valid_for_us == 0U) ||
        (p_request->valid_for_us > max_ttl_us) ||
        ((uint32_t)(now_us - p_request->sensed_us) >= p_request->valid_for_us))
    {
        return false;
    }
    command.issued_us = p_request->sensed_us;
    command.ttl_us    = p_request->valid_for_us;
    switch (p_request->action)
    {
        case GUIDANCE_STRAIGHT:
            command.target_um_s[0] = p_request->speed_um_s;
            command.target_um_s[1] = p_request->speed_um_s;
            break;
        case GUIDANCE_STEER_LEFT:
        case GUIDANCE_STEER_RIGHT:
            if ((p_request->inside_um_s < 0) ||
                (p_request->inside_um_s > p_request->speed_um_s))
            {
                return false;
            }
            command.target_um_s[0] = p_request->action == GUIDANCE_STEER_LEFT
                                         ? p_request->inside_um_s
                                         : p_request->speed_um_s;
            command.target_um_s[1] = p_request->action == GUIDANCE_STEER_RIGHT
                                         ? p_request->inside_um_s
                                         : p_request->speed_um_s;
            break;
        case GUIDANCE_HOLD:
            break;
        default:
            return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if ((limits_um_s[wheel] == 0U) ||
            (limits_um_s[wheel] > (uint32_t)INT32_MAX) ||
            (command.target_um_s[wheel] < 0) ||
            ((uint32_t)command.target_um_s[wheel] > limits_um_s[wheel]))
        {
            return false;
        }
    }
    *p_command = command;
    return true;
}
