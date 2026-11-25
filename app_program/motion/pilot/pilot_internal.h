#ifndef PILOT_INTERNAL_H
#define PILOT_INTERNAL_H

#include "motion/pilot/pilot.h"

/* Start a new command and clear any sonar request or flags from the previous owner. */
void pilot_set_command(pilot_cmd_t * p_cmd, int32_t speed_mm_s, int32_t turn_deci_s,
                       pilot_behaviour_t behaviour, pilot_reason_t reason);

bool pilot_forward_clear(pilot_meas_t const * p_meas, uint16_t min_range);
bool pilot_contour_step(pilot_meas_t const * p_meas, pilot_contour_t * p_run,
                        pilot_contour_plan_t const * p_offer, bool b_inhibited,
                        bool b_course_hold, bool b_fresh_forward, pilot_cmd_t * p_cmd);

#endif /* PILOT_INTERNAL_H */
