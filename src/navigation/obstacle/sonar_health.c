/** @file sonar_health.c
 *
 * @brief Sonar echo health tracking and packed status/ping publication state.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "sonar_health.h"

static uint8_t run_count (uint8_t prev, bool b_match)
{
    uint8_t next = 0U;

    if (b_match)
    {
        next = (prev < UINT8_MAX) ? (uint8_t)(prev + 1U) : UINT8_MAX;
    }

    return (next);
}

static uint16_t fault_bit (uint8_t cause)
{
    uint16_t bit = 0U;

    if (OBSTACLE_CAUSE_NO_ECHO == cause)
    {
        bit = (uint16_t)OBSTACLE_FAULT_NO_ECHO;
    }
    else if (OBSTACLE_CAUSE_ECHO_STUCK == cause)
    {
        bit = (uint16_t)OBSTACLE_FAULT_ECHO_STUCK;
    }
    else if (OBSTACLE_CAUSE_ECHO_NOISE == cause)
    {
        bit = (uint16_t)OBSTACLE_FAULT_ECHO_NOISE;
    }
    else
    {
        /* Other causes carry no echo fault bit. */
    }

    return (bit);
}

static uint16_t ranged_dist (uint8_t cls, uint16_t dist_mm)
{
    uint16_t dist = 0U;

    if (OBSTACLE_CLASS_VALID == cls)
    {
        dist = dist_mm;
    }
    else if (OBSTACLE_CLASS_TOO_CLOSE == cls)
    {
        dist = (uint16_t)OBSTACLE_MIN_RANGE_MM;
    }
    else
    {
        dist = 0U;
    }

    return (dist);
}

static void clear_counters (sonar_health_t * p_health)
{
    p_health->no_echo = 0U;
    p_health->stuck   = 0U;
    p_health->glitch  = 0U;
    p_health->fail    = 0U;
}

static uint8_t health_cause (sonar_health_t * p_health, uint8_t cls)
{
    uint8_t    cause        = OBSTACLE_CAUSE_NONE;
    bool const b_unresolved = obstacle_pack_class_is_unresolved(cls);

    p_health->no_echo = run_count(p_health->no_echo, OBSTACLE_CLASS_NO_ECHO == cls);
    p_health->stuck   = run_count(p_health->stuck, OBSTACLE_CLASS_STUCK == cls);
    p_health->glitch  = run_count(p_health->glitch, OBSTACLE_CLASS_GLITCH == cls);
    p_health->fail    = run_count(p_health->fail, b_unresolved);

    if (p_health->no_echo >= OBSTACLE_NO_ECHO_COUNT)
    {
        cause = OBSTACLE_CAUSE_NO_ECHO;
    }
    else if (p_health->stuck >= OBSTACLE_STUCK_COUNT)
    {
        cause = OBSTACLE_CAUSE_ECHO_STUCK;
    }
    else if ((p_health->glitch >= OBSTACLE_GLITCH_COUNT) ||
             (p_health->fail >= OBSTACLE_FAIL_COUNT))
    {
        cause = OBSTACLE_CAUSE_ECHO_NOISE;
    }
    else
    {
        /* The echo line is healthy enough. */
    }

    if (!b_unresolved)
    {
        p_health->echo_faults = 0U;
    }
    else if (OBSTACLE_CAUSE_NONE != cause)
    {
        p_health->echo_faults |= fault_bit(cause);
    }
    else
    {
        /* Unresolved ping below fault count; keep prior echo faults. */
    }

    return (cause);
}

void sonar_health_init (sonar_health_t * p_health)
{
    (void)memset(p_health, 0, sizeof(*p_health));
    p_health->state = OBSTACLE_STATE_STOPPED;
    p_health->cause = OBSTACLE_CAUSE_BOOT;
}

void sonar_health_command (sonar_health_t * p_health, uint8_t cmd)
{
    uint8_t const op = (uint8_t)obstacle_pack_get(cmd, OBSTACLE_CMD_TYPE_SH, OBSTACLE_CMD_TYPE_MK);

    if (OBSTACLE_CMD_STOP == op)
    {
        p_health->state      = OBSTACLE_STATE_STOPPED;
        p_health->cause      = OBSTACLE_CAUSE_EXTERNAL;
        p_health->b_reset_ok = false;
        clear_counters(p_health);
        p_health->seq = (uint8_t)(((uint32_t)p_health->seq + 1U) & OBSTACLE_ST_SEQ_MK);
    }
    else if ((OBSTACLE_CMD_RESET == op) && (OBSTACLE_STATE_STOPPED == p_health->state))
    {
        p_health->faults     = 0U;
        p_health->cause      = OBSTACLE_CAUSE_NONE;
        p_health->b_reset_ok = true;
        clear_counters(p_health);
        p_health->seq = (uint8_t)(((uint32_t)p_health->seq + 1U) & OBSTACLE_ST_SEQ_MK);
    }
    else if ((OBSTACLE_CMD_START == op) && (OBSTACLE_STATE_STOPPED == p_health->state) &&
             p_health->b_reset_ok)
    {
        p_health->state      = OBSTACLE_STATE_MONITOR;
        p_health->cause      = OBSTACLE_CAUSE_NONE;
        p_health->b_reset_ok = false;
        clear_counters(p_health);
        p_health->seq = (uint8_t)(((uint32_t)p_health->seq + 1U) & OBSTACLE_ST_SEQ_MK);
    }
    else
    {
        /* Ignore commands that do not apply in the current state. */
    }
}

void sonar_health_ping (sonar_health_t * p_health, uint8_t cls,
                        uint16_t dist_mm, bool b_ahead, int16_t look_deci)
{
    uint16_t const dist  = ranged_dist(cls, dist_mm);
    uint8_t const  cause = health_cause(p_health, cls);

    p_health->look_deci  = look_deci;
    p_health->seq        = (uint8_t)(((uint32_t)p_health->seq + 1U) & OBSTACLE_ST_SEQ_MK);
    p_health->last_class = cls;
    p_health->last_dist  = dist;

    if (b_ahead)
    {
        p_health->ping = obstacle_pack_ping(
            obstacle_pack_get(p_health->ping, OBSTACLE_PING_SEQ_SH, OBSTACLE_PING_SEQ_MK) + 1U,
            cls, dist, look_deci);
    }

    if (OBSTACLE_STATE_STOPPED != p_health->state)
    {
        if (OBSTACLE_CAUSE_NONE != cause)
        {
            p_health->state      = OBSTACLE_STATE_STOPPED;
            p_health->cause      = cause;
            p_health->faults    |= fault_bit(cause);
            p_health->b_reset_ok = true;
        }
    }
    else
    {
        bool const b_health_stop = p_health->b_reset_ok &&
                                   ((OBSTACLE_CAUSE_NO_ECHO == p_health->cause) ||
                                    (OBSTACLE_CAUSE_ECHO_STUCK == p_health->cause) ||
                                    (OBSTACLE_CAUSE_ECHO_NOISE == p_health->cause));

        p_health->faults = (uint16_t)((p_health->faults & ~OBSTACLE_FAULTS_ECHO) |
                                      p_health->echo_faults);
        if (b_health_stop && (0U == p_health->faults))
        {
            p_health->cause      = OBSTACLE_CAUSE_NONE;
            p_health->b_reset_ok = false;
            p_health->state      = OBSTACLE_STATE_MONITOR;
        }
    }
}

uint32_t sonar_health_status (sonar_health_t const * p_health)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK, p_health->state);
    word = obstacle_pack_put(word, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK, p_health->cause);
    word = obstacle_pack_put(word, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK, p_health->seq);
    word = obstacle_pack_put(word, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK, p_health->last_class);
    word = obstacle_pack_put(word, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK, p_health->last_dist);

    return (word);
}

uint8_t sonar_health_flags (sonar_health_t const * p_health)
{
    (void)p_health;
    return 0U;
}

/*** end of file ***/
