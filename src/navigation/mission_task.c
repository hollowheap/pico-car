/** @file mission_task.c
 * @brief Mission lifecycle task owned by Navigation (Buddy 5).
 */
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "motion/motion.h"
#include "navigation/mission_events.h"
#include "navigation/navigation.h"
#include "system/startup_task.h"

#define MISSION_PERIOD_MS 10U
#define MISSION_START_PIN      20U
#define MISSION_LINE_SPEED_UM_S UINT32_C(40841)
#define MISSION_LINE_VALID_US  UINT32_C(50000)
#define MISSION_STOP_TTL_US    UINT32_C(20000)
#define TIMER_RAW_LOW          (TIMER_BASE + 0x28U)

static T_SPLOCK             event_lock;
static mission_state_t      current_state;
static uint32_t             pending_events;
static mission_line_state_t latest_line;
static bool                 line_valid;
static bool                 initialised;

static uint32_t take_events (void)
{
    UINT     saved;
    uint32_t events;

    (void)ISpinLock(&event_lock, &saved);
    events         = pending_events;
    pending_events = 0U;
    (void)ISpinUnlock(&event_lock, saved);
    return events;
}

static void apply_pending (uint32_t events)
{
    UINT            saved;
    mission_state_t before;
    mission_state_t after;
    bool            stop_action;

    if (events == 0U)
    {
        return;
    }

    (void)ISpinLock(&event_lock, &saved);
    before = current_state;
    current_state = mission_apply_events(before, events, &stop_action);
    after = current_state;
    (void)ISpinUnlock(&event_lock, saved);

    if (stop_action)
    {
        (void)motion_stop();
    }
    if (after != before)
    {
        (void)tm_printf((UB *)"mission state: %d\n", (int)after);
    }
}

ER mission_init (void)
{
    if (initialised)
    {
        return E_OBJ;
    }

    (void)InitSpinLock(&event_lock);
    current_state  = MISSION_STATE_READY;
    pending_events = 0U;
    line_valid     = false;
    (void)gpio_set_pin(MISSION_START_PIN, GPIO_MODE_IN);
    clr_w(GPIO(MISSION_START_PIN), GPIO_PDE);
    set_w(GPIO(MISSION_START_PIN), GPIO_PUE | GPIO_SHEMITT);
    initialised = true;
    return E_OK;
}

static bool take_line (mission_line_state_t * p_line)
{
    UINT saved;
    bool valid;

    (void)ISpinLock(&event_lock, &saved);
    valid = line_valid;
    if (valid)
    {
        *p_line = latest_line;
    }
    (void)ISpinUnlock(&event_lock, saved);
    return valid;
}

static void drive_motion (void)
{
    mission_line_state_t line;
    const bool           has_line = take_line(&line);
    const uint32_t       now      = in_w(TIMER_RAW_LOW);
    mission_state_t      state;
    UINT                 saved;

    (void)ISpinLock(&event_lock, &saved);
    state = current_state;
    (void)ISpinUnlock(&event_lock, saved);

    if ((state != MISSION_STATE_RUNNING) || !has_line ||
        ((uint32_t)(now - line.sensed_us) >= MISSION_LINE_VALID_US))
    {
        guidance_request_t hold = {
            .action       = GUIDANCE_HOLD,
            .sensed_us    = now,
            .valid_for_us = MISSION_STOP_TTL_US,
        };
        (void)motion_submit(&hold);
        return;
    }

    guidance_request_t request = {0};
    request.speed_um_s             = MISSION_LINE_SPEED_UM_S;
    request.inside_um_s            = 0;
    request.sensed_us              = line.sensed_us;
    request.valid_for_us           = MISSION_LINE_VALID_US;
    if (line.left && line.right)
    {
        request.action = GUIDANCE_HOLD;
    }
    else if (line.left)
    {
        request.action = GUIDANCE_STEER_LEFT;
    }
    else if (line.right)
    {
        request.action = GUIDANCE_STEER_RIGHT;
    }
    else
    {
        request.action = GUIDANCE_STRAIGHT;
    }
    (void)motion_submit(&request);
}

void mission_run (void)
{
    if (!initialised)
    {
        (void)tm_printf((UB *)"mission_init not called\n");
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }

    while (!startup_ready())
    {
        if (startup_failed())
        {
            (void)tm_printf((UB *)"mission task: startup failed\n");
            for (;;)
            {
                (void)tk_slp_tsk(TMO_FEVR);
            }
        }
        (void)tk_dly_tsk(10);
    }

    if (motion_bind(tk_get_tid()) != E_OK)
    {
        (void)tm_printf((UB *)"mission task: motion bind failed\n");
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }

    for (;;)
    {
        motion_status_t status;

        if (gpio_get_val(MISSION_START_PIN) == 0U)
        {
            (void)mission_arm_start();
        }
        apply_pending(take_events());
        if ((motion_status(&status) == E_OK) &&
            (status.stop_requested ||
             (status.state == MOTION_FAULT)))
        {
            (void)mission_post(MISSION_EVENT_STOP);
        }
        drive_motion();
        (void)tk_dly_tsk((RELTIM)MISSION_PERIOD_MS);
    }
}

static ER queue_event (mission_event_t event)
{
    UINT saved;

    (void)ISpinLock(&event_lock, &saved);
    pending_events |= (uint32_t)event;
    (void)ISpinUnlock(&event_lock, saved);
    return E_OK;
}

ER mission_post (mission_event_t event)
{
    if (!initialised)
    {
        return E_OBJ;
    }
    if ((event != MISSION_EVENT_START) && (event != MISSION_EVENT_STOP) &&
        (event != MISSION_EVENT_FAULT))
    {
        return E_PAR;
    }
    return queue_event(event);
}

ER mission_arm_start (void)
{
    return mission_post(MISSION_EVENT_START);
}

ER mission_post_line (const mission_line_state_t * p_line)
{
    UINT saved;

    if (p_line == NULL)
    {
        return E_PAR;
    }
    if (!initialised)
    {
        return E_OBJ;
    }

    (void)ISpinLock(&event_lock, &saved);
    latest_line = *p_line;
    line_valid  = true;
    (void)ISpinUnlock(&event_lock, saved);
    return E_OK;
}

ER mission_emergency_stop (void)
{
    (void)motion_stop();
    return mission_post(MISSION_EVENT_STOP);
}

ER mission_reset (void)
{
    motion_status_t status;

    /* Motion's emergency latch is reboot-only. */
    if ((motion_status(&status) == E_OK) && status.stop_requested)
    {
        return E_OBJ;
    }
    return queue_event(MISSION_EVENT_RESET);
}

mission_state_t mission_get_state (void)
{
    UINT            saved;
    mission_state_t state;

    if (!initialised)
    {
        return MISSION_STATE_INIT;
    }
    (void)ISpinLock(&event_lock, &saved);
    state = current_state;
    (void)ISpinUnlock(&event_lock, saved);
    return state;
}

bool mission_is_running (void)
{
    return mission_get_state() == MISSION_STATE_RUNNING;
}

/*** end of file ***/
