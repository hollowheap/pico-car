#ifndef PILOT_BYPASS_H
#define PILOT_BYPASS_H

#include <stdbool.h>
#include <stdint.h>
#include "platform/static_assert.h"

#define BYPASS_SIDES (2U)
#define BYPASS_FAN_DEADLINE_MS (1000U)

/* The axle origin is the obstacle contact, with x ahead and y left. */
typedef enum
{
    BYPASS_IDLE, BYPASS_CONTACT, BYPASS_ALIGN, BYPASS_AHEAD, BYPASS_AWAY,
    BYPASS_OUT, BYPASS_PARALLEL, BYPASS_LEVEL, BYPASS_FACE, BYPASS_SWEEP,
    BYPASS_TRACK_OUT, BYPASS_TRACK_BACK, BYPASS_BACK, BYPASS_MOVE,
    BYPASS_TO_LINE, BYPASS_CENTRE, BYPASS_MAP, BYPASS_RETRACE, BYPASS_RESCAN,
    BYPASS_LOOK_BACK, BYPASS_EXIT_AHEAD, BYPASS_WALL_BACK, BYPASS_WALL_CHECK,
    BYPASS_SIDE_STEP, BYPASS_SIDE_FAN, BYPASS_RETREAT_ALIGN
} bypass_phase_t;

STATIC_ASSERT(sizeof(bypass_phase_t) <= 4U, "bypass_phase_t size");

typedef struct
{
    uint32_t request_ms;
    uint32_t clearance_ms;
    uint32_t survey_ms;
    uint32_t look_start_ms;
    int32_t map_diff_um;
    int16_t clear_until_remaining_mm;
    int16_t axis_start_mm;
    int16_t target_mm;
    int16_t parallel_origin_mm;
    int16_t return_origin_mm;
    int16_t ahead_mm;
    int16_t out_mm;
    int16_t out_origin_mm;
    int16_t contact_x_mm;
    int16_t contact_y_mm;
    int16_t level_mm;
    int16_t move_mm;
    int16_t black_mm;
    int16_t requested_head;
    uint16_t contact_mm;
    uint16_t nearest_mm;
    int16_t last_hit_deci;
    int16_t edge_deci[BYPASS_SIDES];
    uint8_t phase;
    uint8_t ray;
    uint8_t seq;
    uint8_t last_forward;
    uint8_t unknown;
    uint8_t sector : 2;
    uint8_t blocked_pings : 2;
    uint8_t failed_sides : 2;
    uint8_t views : 2;
    uint8_t fan_left_step : 3;
    uint8_t fan_right_step : 3;
    uint8_t fan_gap_left : 2;
    uint8_t fan_repeat_left : 3;
    uint8_t fan_repeat_right : 3;
    uint8_t fan_gap_right : 2;
    uint8_t fan_repeat_slot : 4;
    bool b_pending : 1;
    bool b_clear : 1;
    bool b_black : 1;
    bool b_white : 1;
    bool b_blocked : 1;
    bool b_swing_blocked : 1;
    bool b_edge_gap_left : 1;
    bool b_edge_gap_right : 1;
    bool b_retried : 1;
    bool b_wide : 1;
    bool b_backed : 1;
    bool b_front_seen : 1;
    bool b_center_clear : 1;
    bool b_interrupted : 1;
    bool b_motor : 1;
    bool b_probe : 1;
    bool b_stalled : 1;
    bool b_survey_timed : 1;
    bool b_drive_aligned : 1;
    bool b_anchor : 1;
    bool b_look_ready : 1;
    bool b_exit_tape : 1;
    bool b_wall : 1;
    bool b_wall_margin : 1;
    bool b_saw_side : 1;
    bool b_progress_timed : 1;
    bool b_scan_pause : 1;
    bool b_left_reverse : 1;
    bool b_right_reverse : 1;
    bool b_contact_moving : 1;
    bool b_fan_done : 1;
    bool b_fan_repeated : 1;
    bool b_fan_repeating : 1;
    bool b_fan_pose_set : 1;
    bool b_fan_center_gap : 1;
} bypass_t;

STATIC_ASSERT(sizeof(bypass_t) == 72U, "bypass state size");

/** Both wheels are at least the move minimum away from a settled pose, in either direction. */
bool bypass_pose_moved(int32_t dl_um, int32_t dr_um);
#endif
