/** @file mission_task.h
 * @brief Navigation-owned mission lifecycle and supervisor.
 *
 * Only src/main.c calls mission_init()/mission_run(). Runtime
 * producers use mission_post() or the S05 commands. Emergency stop stays
 * on the direct motion path.
 */
#ifndef MISSION_TASK_H
#define MISSION_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    MISSION_STATE_INIT = 0,
    MISSION_STATE_READY,
    MISSION_STATE_RUNNING,
    MISSION_STATE_STOPPED
} mission_state_t;

typedef enum
{
    MISSION_EVENT_START = 1U << 0,
    MISSION_EVENT_STOP  = 1U << 1,
    MISSION_EVENT_RESET = 1U << 2,
    MISSION_EVENT_FAULT = 1U << 3
} mission_event_t;

/** Latest line observation; the mission task owns the decision. */
typedef struct
{
    bool     left;
    bool     right;
    uint32_t sensed_us;
} mission_line_state_t;

/* Composition root only. */
ER mission_init (void);
void mission_run (void);

/* Runtime ingress: task context only. */
ER mission_post (mission_event_t event);
ER mission_post_line (const mission_line_state_t * p_line);

ER mission_arm_start (void);
ER mission_emergency_stop (void);
/* E_OBJ while the motion emergency latch is set. */
ER mission_reset (void);

mission_state_t mission_get_state (void);
bool            mission_is_running (void);

#endif /* MISSION_TASK_H */

/*** end of file ***/
