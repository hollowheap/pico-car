/** @file test_motion_guidance.c
 * @brief Decoded steering and hold/stop interleavings, without real IR GPIO.
 */
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>
#include "guidance.h"
#include "command_slot.h"

int main (void)
{
    const uint32_t limits[2] = {50000U, 50000U};
    guidance_request_t request = {GUIDANCE_STRAIGHT, 40000, 20000,
                                      UINT32_MAX - 100U, 1000U};
    motion_command_t command;
    motion_command_t taken;
    motion_command_t saved;
    command_slot_t inbox;
    bool stop;
    unsigned int mask;

    assert(guidance_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 40000 && command.target_um_s[1] == 40000);
    assert(command.issued_us == request.sensed_us && command.ttl_us == 1000U);
    request.action = GUIDANCE_STEER_LEFT;
    assert(guidance_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 20000 && command.target_um_s[1] == 40000);
    request.action = GUIDANCE_STEER_RIGHT;
    assert(guidance_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 40000 && command.target_um_s[1] == 20000);
    request.action = GUIDANCE_HOLD;
    assert(guidance_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 0 && command.target_um_s[1] == 0);
    saved = command;
    request.action = (guidance_action_t)99;
    assert(!guidance_plan(&request, limits, 1000U, 20U, &command));
    request.action = GUIDANCE_STEER_LEFT;
    request.inside_um_s = 40001;
    assert(!guidance_plan(&request, limits, 1000U, 20U, &command));
    request.inside_um_s = -1;
    assert(!guidance_plan(&request, limits, 1000U, 20U, &command));
    request.action = GUIDANCE_STRAIGHT;
    request.speed_um_s = -1;
    assert(!guidance_plan(&request, limits, 1000U, 20U, &command));
    request.speed_um_s = 50001;
    assert(!guidance_plan(&request, limits, 1000U, 20U, &command));
    request.speed_um_s = 40000;
    assert(!guidance_plan(&request, limits, 1000U, 899U, &command)); /* At expiry. */
    assert(!guidance_plan(NULL, limits, 1000U, 20U, &command));
    assert(memcmp(&command, &saved, sizeof(command)) == 0);

    assert(!command_slot_init(NULL, 1000U));
    assert(!command_slot_init(&inbox, 0U));
    assert(!command_slot_init(&inbox, UINT32_MAX));
    assert(command_slot_init(&inbox, 1000U));
    command = (motion_command_t){1U, 100U, 1000U, {10000, 10000}};
    assert(command_slot_post(&inbox, &command, 110U) == COMMAND_SLOT_ACCEPTED);
    command.id = 2U;
    command.target_um_s[0] = 5000;
    assert(command_slot_post(&inbox, &command, 120U) == COMMAND_SLOT_ACCEPTED);
    assert(command_slot_take(&inbox, 130U, &taken, &stop));
    assert(!stop && taken.id == 2U && taken.issued_us == 100U);
    assert(!command_slot_take(&inbox, 140U, &taken, &stop));
    assert(command_slot_hold(&inbox, MOTION_HOLD_OBSTACLE, true, 150U));
    assert(command_slot_post(&inbox, &command, 160U) == COMMAND_SLOT_HELD);
    assert(command_slot_hold(&inbox, MOTION_HOLD_LINE_LOST, true, 160U));
    assert(command_slot_hold(&inbox, MOTION_HOLD_OBSTACLE, false, 170U));
    assert(inbox.hold_mask == MOTION_HOLD_LINE_LOST);
    assert(command_slot_take(&inbox, 180U, &taken, &stop));
    assert(!stop && taken.target_um_s[0] == 0 && taken.target_um_s[1] == 0);
    assert(command_slot_hold(&inbox, MOTION_HOLD_LINE_LOST, false, 200U));
    assert(command_slot_hold(&inbox, MOTION_HOLD_LINE_LOST, false, 201U));
    assert(inbox.released_us == 200U); /* Repeated clear does not delay resume. */
    assert(command_slot_post(&inbox, &command, 210U) == COMMAND_SLOT_STALE);
    command.issued_us = 205U;
    assert(command_slot_post(&inbox, &command, 210U) == COMMAND_SLOT_ACCEPTED);
    assert(command_slot_take(&inbox, 211U, &taken, &stop));
    assert(taken.target_um_s[0] == 0); /* Brake event before a fresh restart. */
    assert(command_slot_take(&inbox, 212U, &taken, &stop));
    assert(taken.target_um_s[0] == 5000 && taken.issued_us == 205U);
    command.issued_us = 199U;
    assert(command_slot_post(&inbox, &command, 215U) == COMMAND_SLOT_STALE);
    /* First fresh frame must not make a still-live old frame replayable. */

    /* Hold set+clear between owner polls must not lose the brake event. */
    assert(command_slot_hold(&inbox, MOTION_HOLD_OPERATOR, true, 220U));
    assert(command_slot_hold(&inbox, MOTION_HOLD_OPERATOR, false, 221U));
    command.issued_us = 222U;
    assert(command_slot_post(&inbox, &command, 222U) == COMMAND_SLOT_ACCEPTED);
    assert(command_slot_take(&inbox, 223U, &taken, &stop) && taken.target_um_s[0] == 0);
    assert(command_slot_take(&inbox, 224U, &taken, &stop) && taken.target_um_s[0] == 5000);
    command_slot_stop(&inbox);
    assert(!command_slot_take(&inbox, 225U, &taken, &stop) && stop);
    assert(command_slot_post(&inbox, &command, 225U) == COMMAND_SLOT_STOPPED);
    assert(command_slot_hold(&inbox, MOTION_HOLD_OPERATOR, false, 225U));
    assert(inbox.stopped);

    /* All combinations: clearing one reason never clears another. */
    for (mask = 0U; mask < 8U; mask++)
    {
        unsigned int bit;

        assert(command_slot_init(&inbox, 1000U));
        for (bit = 0U; bit < 3U; bit++)
        {
            if ((mask & (1U << bit)) != 0U)
            {
                assert(command_slot_hold(&inbox, 1U << bit, true, 0U));
            }
        }
        assert(command_slot_hold(&inbox, MOTION_HOLD_OBSTACLE, false, 10U));
        assert(inbox.hold_mask == (mask & ~MOTION_HOLD_OBSTACLE));
        assert(!command_slot_hold(&inbox, 3U, false, 10U));
        assert(!command_slot_hold(&inbox, 8U, false, 10U));
    }
    assert(command_slot_init(&inbox, 1000U));
    assert(command_slot_hold(&inbox, MOTION_HOLD_LINE_LOST, true, UINT32_MAX - 10U));
    assert(command_slot_hold(&inbox, MOTION_HOLD_LINE_LOST, false, UINT32_MAX - 5U));
    command.issued_us = UINT32_MAX - 6U;
    assert(command_slot_post(&inbox, &command, 20U) == COMMAND_SLOT_STALE);
    command.issued_us = 10U;
    assert(command_slot_post(&inbox, &command, 20U) == COMMAND_SLOT_ACCEPTED);
    command.issued_us = 2000U;
    assert(command_slot_post(&inbox, &command, 2000U) == COMMAND_SLOT_ACCEPTED);
    assert(!inbox.require_fresh); /* Old leases all expired; no long-uptime trap. */
    assert(command_slot_post(&inbox, NULL, 2000U) == COMMAND_SLOT_STALE);
    assert(command_slot_take(&inbox, 2000U, &taken, &stop));
    assert(taken.target_um_s[0] == 0 && taken.target_um_s[1] == 0);
    return 0;
}
