/** @file test_motion_guidance.c
 * @brief Decoded steering and hold/stop interleavings, without real IR GPIO.
 */
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>
#include "motion_request.h"
#include "motion_inbox.h"

int main (void)
{
    const uint32_t limits[2] = {50000U, 50000U};
    motion_drive_request_t request = {MOTION_DRIVE_STRAIGHT, 40000, 20000,
                                      UINT32_MAX - 100U, 1000U};
    motion_command_t command;
    motion_command_t taken;
    motion_command_t saved;
    motion_inbox_t inbox;
    bool stop;
    unsigned int mask;

    assert(motion_request_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 40000 && command.target_um_s[1] == 40000);
    assert(command.issued_us == request.sensed_us && command.ttl_us == 1000U);
    request.action = MOTION_STEER_LEFT;
    assert(motion_request_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 20000 && command.target_um_s[1] == 40000);
    request.action = MOTION_STEER_RIGHT;
    assert(motion_request_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 40000 && command.target_um_s[1] == 20000);
    request.action = MOTION_DRIVE_HOLD;
    assert(motion_request_plan(&request, limits, 1000U, 20U, &command));
    assert(command.target_um_s[0] == 0 && command.target_um_s[1] == 0);
    saved = command;
    request.action = (motion_drive_action_t)99;
    assert(!motion_request_plan(&request, limits, 1000U, 20U, &command));
    request.action = MOTION_STEER_LEFT;
    request.inside_um_s = 40001;
    assert(!motion_request_plan(&request, limits, 1000U, 20U, &command));
    request.inside_um_s = -1;
    assert(!motion_request_plan(&request, limits, 1000U, 20U, &command));
    request.action = MOTION_DRIVE_STRAIGHT;
    request.speed_um_s = -1;
    assert(!motion_request_plan(&request, limits, 1000U, 20U, &command));
    request.speed_um_s = 50001;
    assert(!motion_request_plan(&request, limits, 1000U, 20U, &command));
    request.speed_um_s = 40000;
    assert(!motion_request_plan(&request, limits, 1000U, 899U, &command)); /* At expiry. */
    assert(!motion_request_plan(NULL, limits, 1000U, 20U, &command));
    assert(memcmp(&command, &saved, sizeof(command)) == 0);

    assert(!motion_inbox_init(NULL, 1000U));
    assert(!motion_inbox_init(&inbox, 0U));
    assert(!motion_inbox_init(&inbox, UINT32_MAX));
    assert(motion_inbox_init(&inbox, 1000U));
    command = (motion_command_t){1U, 100U, 1000U, {10000, 10000}};
    assert(motion_inbox_post(&inbox, &command, 110U) == MOTION_INBOX_ACCEPTED);
    command.id = 2U;
    command.target_um_s[0] = 5000;
    assert(motion_inbox_post(&inbox, &command, 120U) == MOTION_INBOX_ACCEPTED);
    assert(motion_inbox_take(&inbox, 130U, &taken, &stop));
    assert(!stop && taken.id == 2U && taken.issued_us == 100U);
    assert(!motion_inbox_take(&inbox, 140U, &taken, &stop));
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_OBSTACLE, true, 150U));
    assert(motion_inbox_post(&inbox, &command, 160U) == MOTION_INBOX_HELD);
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_LINE_LOST, true, 160U));
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_OBSTACLE, false, 170U));
    assert(inbox.hold_mask == MOTION_HOLD_LINE_LOST);
    assert(motion_inbox_take(&inbox, 180U, &taken, &stop));
    assert(!stop && taken.target_um_s[0] == 0 && taken.target_um_s[1] == 0);
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_LINE_LOST, false, 200U));
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_LINE_LOST, false, 201U));
    assert(inbox.released_us == 200U); /* Repeated clear does not delay resume. */
    assert(motion_inbox_post(&inbox, &command, 210U) == MOTION_INBOX_STALE);
    command.issued_us = 205U;
    assert(motion_inbox_post(&inbox, &command, 210U) == MOTION_INBOX_ACCEPTED);
    assert(motion_inbox_take(&inbox, 211U, &taken, &stop));
    assert(taken.target_um_s[0] == 0); /* Brake event before a fresh restart. */
    assert(motion_inbox_take(&inbox, 212U, &taken, &stop));
    assert(taken.target_um_s[0] == 5000 && taken.issued_us == 205U);
    command.issued_us = 199U;
    assert(motion_inbox_post(&inbox, &command, 215U) == MOTION_INBOX_STALE);
    /* First fresh frame must not make a still-live old frame replayable. */

    /* Hold set+clear between owner polls must not lose the brake event. */
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_OPERATOR, true, 220U));
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_OPERATOR, false, 221U));
    command.issued_us = 222U;
    assert(motion_inbox_post(&inbox, &command, 222U) == MOTION_INBOX_ACCEPTED);
    assert(motion_inbox_take(&inbox, 223U, &taken, &stop) && taken.target_um_s[0] == 0);
    assert(motion_inbox_take(&inbox, 224U, &taken, &stop) && taken.target_um_s[0] == 5000);
    motion_inbox_stop(&inbox);
    assert(!motion_inbox_take(&inbox, 225U, &taken, &stop) && stop);
    assert(motion_inbox_post(&inbox, &command, 225U) == MOTION_INBOX_STOPPED);
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_OPERATOR, false, 225U));
    assert(inbox.stopped);

    /* All combinations: clearing one reason never clears another. */
    for (mask = 0U; mask < 8U; mask++)
    {
        unsigned int bit;

        assert(motion_inbox_init(&inbox, 1000U));
        for (bit = 0U; bit < 3U; bit++)
        {
            if ((mask & (1U << bit)) != 0U)
            {
                assert(motion_inbox_hold(&inbox, 1U << bit, true, 0U));
            }
        }
        assert(motion_inbox_hold(&inbox, MOTION_HOLD_OBSTACLE, false, 10U));
        assert(inbox.hold_mask == (mask & ~MOTION_HOLD_OBSTACLE));
        assert(!motion_inbox_hold(&inbox, 3U, false, 10U));
        assert(!motion_inbox_hold(&inbox, 8U, false, 10U));
    }
    assert(motion_inbox_init(&inbox, 1000U));
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_LINE_LOST, true, UINT32_MAX - 10U));
    assert(motion_inbox_hold(&inbox, MOTION_HOLD_LINE_LOST, false, UINT32_MAX - 5U));
    command.issued_us = UINT32_MAX - 6U;
    assert(motion_inbox_post(&inbox, &command, 20U) == MOTION_INBOX_STALE);
    command.issued_us = 10U;
    assert(motion_inbox_post(&inbox, &command, 20U) == MOTION_INBOX_ACCEPTED);
    command.issued_us = 2000U;
    assert(motion_inbox_post(&inbox, &command, 2000U) == MOTION_INBOX_ACCEPTED);
    assert(!inbox.require_fresh); /* Old leases all expired; no long-uptime trap. */
    assert(motion_inbox_post(&inbox, NULL, 2000U) == MOTION_INBOX_STALE);
    assert(motion_inbox_take(&inbox, 2000U, &taken, &stop));
    assert(taken.target_um_s[0] == 0 && taken.target_um_s[1] == 0);
    return 0;
}
