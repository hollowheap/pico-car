#ifndef PILOT_HUG_IMPLEMENTATION_H
#define PILOT_HUG_IMPLEMENTATION_H

#include "../hug_internal.h"

#define HALVES          (2)
#define EXIT_LEFT       (1U)
#define EXIT_RIGHT      (2U)
#define EXIT_WAIT_CLEAR (EXIT_LEFT | EXIT_RIGHT)
#define QUARTER_DECI    (900)
#define SEQ_MASK        (0x3FU)
#define HIST_BITS       (3U)
#define PROBE_RAYS      (3U)
#define PROBE_STEP_DECI (150)
#define HUG_FLANK_DECI (250)
#define HUG_SCAN_CLEARANCE_MM (HUG_SWEEP_RADIUS_MM + (int32_t)OBSTACLE_PLAN_MARGIN_MM)
#define ROUND_HALF      (HUG_PIVOT_STEP_DECI / HALVES)

/* States. */
#define ST_IDLE         (0U)
#define ST_SCAN         (1U)
#define ST_PIVOT        (2U)
#define ST_DRIVE        (3U)

/* Sub-states. A pivot or a drive turns or drives, then settles. A scan
 * pivots to its body step, settles, then asks for a ping. A move is set up in
 * START, and its clock starts at its first tick, in MOVE. */
#define SUB_MOVE        (0U)
#define SUB_SETTLE      (1U)
#define SUB_ASK         (2U)
#define SUB_START       (3U)

/* What follows a motion or a scan. The wire state reads THEN_BACK as the
 * back-off (hug_word.h). */
#define THEN_SCAN       (0U)
#define THEN_BACK       (1U)
#define THEN_APPROACH   (2U)
#define THEN_ALIGN      (3U)
#define THEN_LEG        (4U)
#define THEN_LOOK       (5U)
#define THEN_TURN       (6U)
#define THEN_EXIT_TURN  (7U)
#define THEN_RELEASE    (8U)
#define THEN_ADVANCE    (9U)
#define THEN_REAR_LATERAL (10U)
#define THEN_REAR_ALIGN   (11U)
#define THEN_REAR_READY   (12U)
#define THEN_FLANK_LOOK   (13U)

/* Why the car looks. */
#define PU_CONTACT      (0U)
#define PU_LOOK         (1U)
#define PU_INNER        (2U)
#define PU_REFINE       (3U)
#define PU_CORNER       (4U)
#define PU_CHECK        (5U)
#define PU_CENTER       (6U)
#define PU_CORNER_RETRY (7U)
#define PU_EDGE_LOOK    (8U)
#define PU_EDGE_RETRY   (9U)
#define PU_MID_LOOK     (10U)
#define PU_FACET_CLEAR    (11U)
#define PU_FACET_RETRY    (12U)

#define F_PAST          (0x01U) /* the leg ended the body and the gap past the corner     */
#define F_EXIT_TAPE     (0x02U) /* a sensor located tape outside the approach line */
#define F_EXIT          (0x04U) /* the tape ended the hug: the turn out stops at the tape */
#define F_TURNED        (0x08U) /* the last motion was the turn round a corner    */
#define F_REFINE        (0x10U) /* the finer looks of a face whose edges are known */
#define F_WIN2          (0x20U) /* the second finer look adds to the first        */
#define F_STALL         (0x40U) /* the hit or the leg ended in a wheel stall      */
#define F_ASSUME        (0x80U) /* the face in use is the one assumed at a stall  */
#define F_BACKED        (0x100U) /* the first contact has backed off to the standoff */
#define F_REPEAT        (0x200U) /* a stall at the place of the last one, which may be a wall */
#define F_RECHECK       (0x400U) /* stationary near-sector check against the last contact survey */
#define F_AIMED         (0x800U) /* the pivot before the look has stopped at its first body step and settled */
#define F_REAPPROACH    (0x1000U) /* the last contact look released after an unseen stall */
#define F_EDGE_LEG      (0x2000U) /* the current leg ends beyond a measured edge */
#define F_COARSE_NARROW (0x4000U) /* width confidence of the saved coarse fit */
#define F_MEASURED_FACE (0x8000U) /* a full fit supplies the face line */

#define LEFT            ((uint8_t)LOOK_SIDE_LEFT)
#define RIGHT           ((uint8_t)LOOK_SIDE_RIGHT)

/* Lateral distances are measured across the heading the car looked along, not
 * across the fitted normal: the fit of a narrow face is a few degrees off. */
typedef struct
{
    look_edge_t edge[HUG_SIDES];
    bool        have[HUG_SIDES];
    int32_t     lat_mm[HUG_SIDES]; /* across the heading the car looked along, from the car, absolute */
    bool        b_narrow;          /* too few rays for the normal and the edges of the face           */
} hug_edges_t;

/* hug.c */
bool hug_flag_get(hug_t const * p_hug, uint32_t mask);
void hug_flag_set(hug_t * p_hug, uint32_t mask, bool b_on);
int32_t hug_along_head(hug_t const * p_hug);
void hug_after_corner(hug_t * p_hug, look_t const * p_look, hug_out_t * p_out);
void hug_after_side_check(hug_t * p_hug, look_t const * p_look);
void hug_after_look(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                    bool b_found, hug_out_t * p_out);
void hug_after_edge_sweep(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                          bool b_found, hug_out_t * p_out);
void hug_release(hug_t * p_hug);
void hug_finish(hug_t * p_hug);
void hug_plan_assumed(hug_t * p_hug, bool b_first);
void hug_after_inner(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                     bool b_found);
void hug_proceed(hug_t * p_hug, look_t * p_look);

/* scan.c */
void hug_start_scan(hug_t * p_hug, look_t * p_look, uint32_t purpose, int32_t center);
bool hug_facet_clearance(hug_t const * p_hug);
bool hug_corner_scan(hug_t const * p_hug);
bool hug_probe_scan(hug_t const * p_hug);
bool hug_edge_sweep(hug_t const * p_hug);
int32_t hug_ray_offset(hug_t const * p_hug, uint32_t i);
int32_t hug_scan_body_aim(hug_t const * p_hug, int32_t offset);
void hug_look_along_face(hug_t * p_hug, uint8_t purpose);
bool hug_prior_near_return(look_t const * p_look, int32_t bearing, int32_t range_mm);
void hug_restart_contact(hug_t * p_hug, look_t * p_look);
void hug_find_edges(look_t const * p_look, look_face_t const * p_face,
                    hug_t const * p_hug, hug_edges_t * p_edges);
int32_t hug_edges_apart_mm(hug_edges_t const * p_edges);
int8_t hug_pick_side(hug_t const * p_hug, hug_edges_t const * p_edges);
bool hug_near_survey_pose(hug_t const * p_hug, hug_in_t const * p_in);
bool hug_can_recheck(hug_t const * p_hug, look_t const * p_look, hug_in_t const * p_in);
void hug_scan_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                   hug_out_t * p_out);
uint32_t hug_valid_rays(look_t const * p_look);

/* motion.c */
int32_t hug_pos_x_mm(hug_t const * p_hug);
int32_t hug_pos_y_mm(hug_t const * p_hug);
void hug_zero_out(hug_out_t * p_out);
void hug_odo_update(hug_t * p_hug, hug_in_t const * p_in);
void hug_note_update(hug_t * p_hug, hug_in_t const * p_in);
bool hug_forward_ping(hug_t * p_hug, hug_in_t const * p_in, bool * p_b_guard);
bool hug_triggered(hug_t const * p_hug);
bool hug_recheck_on_bend(hug_t * p_hug, hug_in_t const * p_in, bool b_forward);
void hug_start_pivot(hug_t * p_hug, int32_t head, uint32_t then);
void hug_start_drive(hug_t * p_hug, int32_t len_mm, int32_t head, uint32_t then);
void hug_start_back(hug_t * p_hug, int32_t range_mm);
bool hug_pivot_cmd(hug_t const * p_hug, int32_t target, hug_out_t * p_out);
uint32_t hug_move_limit_ms(hug_t const * p_hug);
int32_t hug_from_edge_mm(hug_t const * p_hug, int32_t axis);
int32_t hug_obstacle_side_reach_mm(hug_t const * p_hug);
void hug_plan_leg(hug_t * p_hug, look_face_t const * p_face,
                  hug_edges_t const * p_edges, bool b_first, look_t const * p_look);
void hug_observe_exit_tape(hug_t * p_hug, hug_in_t const * p_in);
void hug_drive_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                    bool b_guard, hug_out_t * p_out);
void hug_pivot_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                    bool b_forward, bool b_guard, hug_out_t * p_out);

#endif
