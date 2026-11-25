#ifndef PILOT_HUG_INTERNAL_H
#define PILOT_HUG_INTERNAL_H

#include "motion/pilot/hug.h"

/* Axle pose relative to obstacle contact, with x forward and y left. */
int32_t hug_pose_heading_deci(hug_t const * p_hug);
look_pose_t hug_axle_pose(hug_t const * p_hug);

void bypass_begin(hug_t * p_hug, hug_in_t const * p_in, uint16_t range_mm);
void hug_begin_bypass(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, uint16_t range_mm);
void bypass_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out);
#endif
