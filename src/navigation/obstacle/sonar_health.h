/** @file sonar_health.h
 *
 * @brief Sonar echo health tracking and packed status/ping publication state.
 *
 * Pure C99, no kernel or hardware calls. Replaces the legacy obstacle
 * statechart now that the pilot owns avoidance.
 */

#ifndef OBSTACLE_SONAR_HEALTH_H
#define OBSTACLE_SONAR_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t ping;        /* Latest packed free-running ping word.       */
    uint16_t faults;      /* OBSTACLE_FAULT_* bits.                      */
    uint16_t echo_faults; /* Current unresolved echo fault bits.         */
    uint16_t last_dist;   /* Latest ranged distance in mm.               */
    int16_t  look_deci;   /* Current servo angle in deci-degrees.        */
    uint8_t  state;       /* OBSTACLE_STATE_STOPPED or _MONITOR.         */
    uint8_t  cause;       /* OBSTACLE_CAUSE_* for STOPPED.               */
    uint8_t  seq;         /* Packed status sequence (3 bits).            */
    uint8_t  last_class;  /* Latest OBSTACLE_CLASS_*.                    */
    uint8_t  no_echo;     /* Consecutive NO_ECHO pings.                  */
    uint8_t  stuck;       /* Consecutive STUCK pings.                    */
    uint8_t  glitch;      /* Consecutive GLITCH pings.                   */
    uint8_t  fail;        /* Consecutive unresolved pings.               */
    bool     b_reset_ok;  /* True when a health stop may auto-resume.    */
} sonar_health_t;

void     sonar_health_init (sonar_health_t * p_health);
void     sonar_health_command (sonar_health_t * p_health, uint8_t cmd);
void     sonar_health_ping (sonar_health_t * p_health, uint8_t cls,
                            uint16_t dist_mm, bool b_ahead, int16_t look_deci);
uint32_t sonar_health_status (sonar_health_t const * p_health);
uint8_t  sonar_health_flags (sonar_health_t const * p_health);

#endif /* OBSTACLE_SONAR_HEALTH_H */

/*** end of file ***/
