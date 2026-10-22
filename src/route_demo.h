/** @file route_demo.h
 * @brief Once-only encoder route for an explicitly selected demonstration.
 */
#ifndef ROUTE_DEMO_H
#define ROUTE_DEMO_H

#include "motion_sampler.h"

#define ROUTE_DEMO_SPEED_UM_S INT32_C(50000)
#define ROUTE_DEMO_PAUSE_US UINT32_C(400000)
#define ROUTE_DEMO_MAX_GAP_US UINT32_C(100000)
#define ROUTE_DEMO_MAX_AGE_US UINT32_C(60000)
#define ROUTE_DEMO_TOTAL_US UINT32_C(65000000)

typedef enum
{
    ROUTE_DEMO_FIRST = 0,
    ROUTE_DEMO_PAUSE_FIRST,
    ROUTE_DEMO_RIGHT,
    ROUTE_DEMO_PAUSE_RIGHT,
    ROUTE_DEMO_LAST,
    ROUTE_DEMO_DONE,
    ROUTE_DEMO_ABORTED
} route_demo_phase_t;

typedef enum
{
    ROUTE_DEMO_OK = 0,
    ROUTE_DEMO_STOP,
    ROUTE_DEMO_QUALITY,
    ROUTE_DEMO_TIMING,
    ROUTE_DEMO_TIMEOUT,
    ROUTE_DEMO_RANGE
} route_demo_error_t;

typedef struct
{
    route_demo_phase_t phase;
    route_demo_error_t error;
    int64_t goals[3][ENCODER_WHEELS];
    int64_t baseline[ENCODER_WHEELS];
    int64_t progress[ENCODER_WHEELS];
    bool reached[ENCODER_WHEELS];
    uint32_t started_us;
    uint32_t phase_us;
    uint32_t previous_us;
    uint32_t last_sample_us;
    bool initialised;
} route_demo_t;

/* Caller must establish a NEW, braked capture/sampler baseline first.
 * Baseline net counts are zero. No global quality flags are cleared here.
 * Scales may be provisional: this demo is approximate, not accepted navigation.
 * Goals: each wheel500mm; left3revolutions/right0; each wheel100mm. */
bool route_demo_start (route_demo_t * p_route,
    const motion_scale_t scales[ENCODER_WHEELS], uint32_t baseline_us);

/* One owner, every fresh sample. Failure/stop/deadline is terminal.
 * Each driven wheel stops after reaching its own goal; no wheel restarts
 * within a leg after rollback. Pauses take a fresh leg baseline, never reset
 * sampler quality. Intervals/age/gaps must be valid, even while paused.
 * This does not check IR/sonar, heading, obstacles or the USB cable. */
void route_demo_update (route_demo_t * p_route, const motion_sample_t * p_sample,
                        uint32_t now_us, bool stop_requested);
void route_demo_targets (const route_demo_t * p_route,
                         int32_t * p_left, int32_t * p_right);

#endif /* ROUTE_DEMO_H */
