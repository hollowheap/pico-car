/** @file avoidance_internal.h
 * @brief Interface between the pilot and its obstacle maneuver.
 */
#ifndef PILOT_AVOIDANCE_INTERNAL_H
#define PILOT_AVOIDANCE_INTERNAL_H

#include "pilot/pilot.h"

typedef enum
{
    AVOIDANCE_IDLE,
    AVOIDANCE_COMMAND,
    AVOIDANCE_RELEASE,
    AVOIDANCE_REJOINED,
    AVOIDANCE_FIND_TAPE,
    AVOIDANCE_NO_EXIT,
    AVOIDANCE_TIMEOUT
} avoidance_result_t;

/* Wheel evidence over the current commanded interval. Travel is the
 * unsigned difference of the low odometer words, including across wrap. */
typedef struct
{
    uint32_t left_um;
    uint32_t right_um;
    uint32_t age_ms;
    bool b_moving;
    bool b_line_reverse;
    bool b_advanced;
    bool b_slip;
    bool b_one_moved;
    bool b_both_commanded;
} avoidance_progress_t;

void avoidance_init(avoidance_t * p_state);
void avoidance_discard_route(avoidance_t * p_state);
void avoidance_restart_stalled(avoidance_t * p_state);
void avoidance_request_survey(avoidance_t * p_state, uint32_t now_ms);
bool avoidance_resume_edges(avoidance_t * p_state, pilot_meas_t const * p_meas);
bool avoidance_backing_off(avoidance_t const * p_state);
/* True starts a new wheel observation interval at this tick. */
bool avoidance_recover(avoidance_t * p_state, avoidance_progress_t const * p_progress,
                       pilot_meas_t const * p_meas, pilot_cmd_t * p_cmd);
avoidance_result_t avoidance_step(avoidance_t * p_state, pilot_meas_t const * p_meas,
                                  uint16_t bend_radius_mm, int16_t drift_corr_um, bool b_inhibited,
                                  bool b_fresh_forward, bool b_backoff_done, pilot_cmd_t * p_cmd);

#endif /* PILOT_AVOIDANCE_INTERNAL_H */
