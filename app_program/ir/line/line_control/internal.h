/** @file internal.h
 * @brief Private state fields and helpers shared by the line controller.
 */

#ifndef TRACK_LINE_CONTROL_INTERNAL_H
#define TRACK_LINE_CONTROL_INTERNAL_H

#include "ir/line/line_control.h"

#include "platform/units.h"

#define TRACK_UM             ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
#define BLIND_SETTLED        (4U)
#define FOLLOW_STRIPE_RIGHT  (8U)
#define FOLLOW_STRIPE_READY  (16U)
#define FOLLOW_MARKER_RECENT (32U)
#define FOLLOW_SUSTAINED_EDGE (16U) /* retained in stripe.detector_bits during a gap */
#define FOLLOW_ROUTE_CANDIDATE (64U)
#define FOLLOW_LOCAL_SEARCH    (128U)

/* line_follow_t.sweep: the search turn in progress. */
#define SWEEP_NONE           (0U)
#define SWEEP_REVERSE        (4U)

#define ROUTE_MIDPOINT_DIVISOR   (2)
#define ROUTE_MAX_CROSS_MM       (2 * LINE_WIDTH_MM)
/* A 60-degree transverse crossing projects 15 mm of tape to about 17 mm.
 * Allow another couple of millimetres for the sampled entry and exit. */
#define ROUTE_STRIPE_MAX_MM      (LINE_WIDTH_MM + LINE_WIDTH_MM / 3)
/* A lone detector can graze the through tape while unwinding a bend.
 * Require a near-full-width traversal. The 5 mm allowance covers steering,
 * debounce and quantized encoder travel in the simulated placements. */
#define SINGLE_STRIPE_MIN_MM     (LINE_WIDTH_MM - LINE_WIDTH_MM / 3)

static inline void set_targets(line_targets_t * p_targets, int32_t left_um_s,
                               int32_t right_um_s)
{
    p_targets->left_um_s  = left_um_s;
    p_targets->right_um_s = right_um_s;
}

static inline int64_t abs_i64(int64_t value)
{
    return (value < 0) ? -value : value;
}

void line_follow_slew_ref(line_follow_t * p_follow, int32_t rate, uint32_t dt_ms,
                           int32_t heading_deci, int32_t cap_deci);
void line_follow_relax_ref(line_follow_t * p_follow, uint32_t dt_ms, int32_t heading_deci);
int32_t line_follow_corner_speed(int32_t ref_deci, int32_t heading_deci);
void line_follow_steer_to(int32_t ref_deci, int32_t heading_deci, int32_t base_um_s,
                          line_targets_t * p_targets);

bool line_recovery_past_loss(uint8_t loss_cm, int64_t travel_um);
void line_recovery_trail_record(line_follow_t * p_follow, int32_t heading_deci, int64_t travel_um);
void line_recovery_found(line_follow_t * p_follow, int32_t heading_deci);
int32_t line_recovery_blind(line_follow_t * p_follow, int32_t heading_deci,
                            int64_t travel_um, uint32_t dt_ms, bool b_continuous_floor);
bool line_recovery_local_stopped(line_follow_t * p_follow, int32_t heading_deci, uint32_t now_ms);
int32_t line_recovery_spin(line_follow_t const * p_follow, int32_t heading_deci);

#endif /* TRACK_LINE_CONTROL_INTERNAL_H */
