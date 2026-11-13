/** @file test_assemble.c
 * @brief Host test for the telemetry assembler (S46): the mapping from the
 *        subsystem snapshots to the packet structs.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry/assemble.h"

static unsigned g_fail;
static unsigned g_checks;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        g_checks++;                                                          \
        if (!(cond))                                                         \
        {                                                                    \
            g_fail++;                                                        \
            (void)printf("FAIL line %d: %s\n", __LINE__, #cond);             \
        }                                                                    \
    } while (0)

#define BIT(n) (1u << (n))

static motion_status_t running_motion (void)
{
    motion_status_t m;

    memset(&m, 0, sizeof m);
    m.state = MOTION_STATE_RUNNING;
    m.ready = true;
    return m;
}

static motion_feedback_t fresh_feedback (void)
{
    motion_feedback_t f;

    memset(&f, 0, sizeof f);
    f.left_speed_um_s         = 160000;
    f.right_speed_um_s        = 164000;
    f.left_displacement_um    = 1000000;
    f.right_displacement_um   = 1002000;
    f.fresh                   = true;
    return f;
}

static obstacle_status_t obstacle_in (uint8_t state)
{
    obstacle_status_t o;

    memset(&o, 0, sizeof o);
    o.state = state;
    o.ready = true;
    return o;
}

static void test_status_basic (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    motion_feedback_t  f = fresh_feedback();
    physics_status_t   p;
    obstacle_status_t  o = obstacle_in(TELEMETRY_OBSTACLE_STATE_MONITOR);

    memset(&p, 0, sizeof p);
    p.ready         = true;
    p.heading_valid = true;
    p.heading_ddeg  = 3590U; /* above 255 degrees: the old byte wrapped */

    m.hold_mask = MOTION_HOLD_OBSTACLE;
    telemetry_assemble_status(&out, &m, &f, &p, &o, 0x0020U, 7U, 0x03U, false);

    CHECK(out.speed_mms == 162);         /* mean of 160 and 164 mm/s */
    CHECK(out.odometer_mm == 1001U);     /* mean of 1000 and 1002 mm */
    CHECK(out.heading_ddeg == 3590U);
    CHECK(out.state == TELEMETRY_RUN_STATE_RUNNING);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_FOLLOW);
    CHECK(out.faults == 0x0020U); /* imu_nack: not a line-lost mode */
    CHECK(out.seq == 7U);
    CHECK(out.line_bits == 0x03U);
    CHECK(out.hold_mask == MOTION_HOLD_OBSTACLE);
}

static void test_status_stale_feedback_keeps_odometer (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    motion_feedback_t  f = fresh_feedback();

    f.fresh = false;
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.speed_mms == 0);
    CHECK(out.odometer_mm == 1001U);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
}

static void test_status_run_state_and_estop (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();

    m.state = MOTION_STATE_IDLE;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.state == TELEMETRY_RUN_STATE_READY);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_STOPPED);

    m.state = MOTION_STATE_REVERSING;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.state == TELEMETRY_RUN_STATE_RUNNING);

    m.state = MOTION_STATE_FAULT;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.state == TELEMETRY_RUN_STATE_STOPPED);

    /* The motion latch adds the estop bit to the level word. */
    m = running_motion();
    m.stop_requested = true;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0x0001U, 0U, 0U, false);
    CHECK(out.state == TELEMETRY_RUN_STATE_STOPPED);
    CHECK(out.faults == (0x0001U | BIT(TELEMETRY_FAULT_BIT_ESTOP_LATCHED)));

    /* No motion service at all. */
    telemetry_assemble_status(&out, NULL, NULL, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.state == TELEMETRY_RUN_STATE_INIT);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_STOPPED);
    CHECK(out.hold_mask == 0U);
}

static void test_status_mode (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    obstacle_status_t  o;

    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_SCANNING);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_HOLD);
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_WAIT_STOP);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_HOLD);
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_BYPASS);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, true);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_BYPASS); /* beats a turn */
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_REACQUIRE);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_BYPASS);

    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_MONITOR);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, true);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_TURN);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o,
                              BIT(TELEMETRY_FAULT_BIT_LINE_LOST), 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_SEARCH);

    /* Lift and overturn win over everything else. */
    telemetry_assemble_status(&out, &m, NULL, NULL, &o,
                              BIT(TELEMETRY_FAULT_BIT_LIFTED), 0U, 0U, true);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_LIFTED);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o,
                              BIT(TELEMETRY_FAULT_BIT_LIFTED) |
                                  BIT(TELEMETRY_FAULT_BIT_OVERTURNED),
                              0U, 0U, true);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_OVERTURNED);

    /* The holds the supervisor sets show in the mode byte. */
    m.hold_mask = MOTION_HOLD_BARCODE;
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_HOLD_BARCODE);
    m.hold_mask = MOTION_HOLD_LIFT;
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_LIFTED);
    m.hold_mask = MOTION_HOLD_OPERATOR;
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_STOPPED);
    CHECK(out.state == TELEMETRY_RUN_STATE_STOPPED);
}

static void test_status_clamps_and_nulls (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    motion_feedback_t  f = fresh_feedback();

    f.left_speed_um_s  = 2000000000;
    f.right_speed_um_s = 2000000000;
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.speed_mms == INT16_MAX);

    f.left_speed_um_s  = -2000000000;
    f.right_speed_um_s = -2000000000;
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0U, 0U, 0U, false);
    CHECK(out.speed_mms == INT16_MIN);

    /* A negative mean travel reads as zero. */
    f.left_displacement_um  = -5000000;
    f.right_displacement_um = -5000000;
    CHECK(telemetry_odometer_mm(&f) == 0U);
    CHECK(telemetry_odometer_mm(NULL) == 0U);
    f.left_displacement_um  = INT64_C(9000000000000);
    f.right_displacement_um = INT64_C(9000000000000);
    CHECK(telemetry_odometer_mm(&f) == UINT32_MAX);

    telemetry_assemble_status(NULL, &m, &f, NULL, NULL, 0U, 0U, 0U, false);
    telemetry_assemble_motion(NULL, &m, &f, NULL);
    telemetry_assemble_physics(NULL, NULL);
    telemetry_assemble_obstacle(NULL, NULL, NULL);
}

static void test_motion (void)
{
    telemetry_motion_t out;
    motion_status_t    m = running_motion();
    motion_feedback_t  f = fresh_feedback();
    motion_diag_t      d;

    memset(&d, 0, sizeof d);
    d.left_target_um_s     = 163364;
    d.right_target_um_s    = -80000;
    d.left_duty_permille   = 312;
    d.right_duty_permille  = -45;
    d.overruns             = 70000U; /* saturates at u16 */
    m.state                = MOTION_STATE_REVERSING;
    m.fault                = 5U;

    telemetry_assemble_motion(&out, &m, &f, &d);
    CHECK(out.left_mms == 160);
    CHECK(out.right_mms == 164);
    CHECK(out.left_target_mms == 163);
    CHECK(out.right_target_mms == -80);
    CHECK(out.left_duty_pm == 312);
    CHECK(out.right_duty_pm == -45);
    CHECK(out.overruns == UINT16_MAX);
    CHECK(out.motion_state == MOTION_STATE_REVERSING);
    CHECK(out.motion_fault == 5U);

    f.fresh = false;
    telemetry_assemble_motion(&out, &m, &f, &d);
    CHECK(out.left_mms == 0);
    CHECK(out.right_mms == 0);
    CHECK(out.left_target_mms == 163); /* targets do not depend on feedback */

    telemetry_assemble_motion(&out, NULL, NULL, NULL);
    CHECK(out.motion_state == 0U);
    CHECK(out.left_target_mms == 0);
    CHECK(out.overruns == 0U);
}

static void test_physics (void)
{
    telemetry_physics_t out;
    physics_status_t    p;

    memset(&p, 0, sizeof p);
    p.ready          = true;
    p.pitch_cdeg     = -450;
    p.roll_cdeg      = 6200;
    p.az_mg          = -240;
    p.amag_mg        = 1031U;
    p.attitude_state = TELEMETRY_ATTITUDE_STATE_OVERTURNED;
    p.yaw_rate_cdps  = 1234;
    p.a_fwd_mg       = -87;
    p.hump_peak_mm   = 41U;
    p.run_peak_mm    = 63U;
    p.state          = 2U;
    p.motion         = 5U;
    p.faults         = 3U;
    p.heading_valid  = true;
    p.heading_ddeg   = 2700U;
    p.hump_active    = true;
    p.height_valid   = true;
    p.calibrated     = true;

    telemetry_assemble_physics(&out, &p);
    CHECK(out.pitch_cdeg == -450);
    CHECK(out.yaw_rate_cdps == 1234);
    CHECK(out.a_fwd_mg == -87);
    CHECK(out.hump_peak_mm == 41U);
    CHECK(out.run_peak_mm == 63U);
    CHECK(out.hump_state == 2U);
    CHECK(out.motion_class == 5U);
    CHECK(out.physics_faults == 3U);
    CHECK(out.heading_ddeg == 2700U);
    CHECK(out.flags == (BIT(TELEMETRY_PHYSICS_FLAGS_BIT_HUMP_ACTIVE) |
                        BIT(TELEMETRY_PHYSICS_FLAGS_BIT_HEIGHT_VALID) |
                        BIT(TELEMETRY_PHYSICS_FLAGS_BIT_CALIBRATED)));
    CHECK(out.roll_cdeg == 6200);
    CHECK(out.az_mg == -240);
    CHECK(out.amag_mg == 1031U);
    CHECK(out.attitude_state == TELEMETRY_ATTITUDE_STATE_OVERTURNED);

    p.finished      = true;
    p.hump_active   = false;
    p.height_valid  = false;
    p.calibrated    = false;
    p.heading_valid = false;
    telemetry_assemble_physics(&out, &p);
    CHECK(out.flags == BIT(TELEMETRY_PHYSICS_FLAGS_BIT_FINISHED));
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);

    /* Not ready or missing: zeros and an invalid heading. */
    p.ready = false;
    telemetry_assemble_physics(&out, &p);
    CHECK(out.pitch_cdeg == 0);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
    telemetry_assemble_physics(&out, NULL);
    CHECK(out.flags == 0U);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
}

static void test_obstacle (void)
{
    telemetry_obstacle_t    out;
    obstacle_status_t       o;
    obstacle_plan_summary_t plan;

    memset(&o, 0, sizeof o);
    o.ready       = true;
    o.state       = 3U;
    o.cause       = 7U;
    o.faults      = 0x181U;
    o.echo_class  = 2U;
    o.distance_mm = 215U;
    o.seq         = 29U;
    o.req_id      = 9U;
    o.servo_deci  = -300;
    o.body_deci   = 600;
    o.flags       = 0x48U;

    telemetry_assemble_obstacle(&out, &o, NULL);
    CHECK(out.state == 3U);
    CHECK(out.cause == 7U);
    CHECK(out.faults == 0x181U);
    CHECK(out.echo_class == 2U);
    CHECK(out.dist_mm == 215U);
    CHECK(out.seq == 29U);
    CHECK(out.req_id == 9U);
    CHECK(out.servo_deci == -300);
    CHECK(out.body_deci == 600);
    CHECK(out.flags == 0x48U);
    CHECK(out.plan_width_mm == 0U);

    plan.closest_mm = 180U;
    plan.width_mm   = 205U;
    plan.need_mm    = 190U;
    plan.status     = 0U;
    plan.side       = 1U;
    plan.start_deg  = 60U;
    plan.b_wall     = false;
    telemetry_assemble_obstacle(&out, &o, &plan);
    CHECK(out.closest_mm == 180U);
    CHECK(out.plan_width_mm == 205U);
    CHECK(out.plan_need_mm == 190U);
    CHECK(out.plan_side == 1U);
    CHECK(out.plan_start_deg == 60U);

    telemetry_assemble_obstacle(&out, NULL, NULL);
    CHECK(out.state == 0U);
    CHECK(out.dist_mm == 0U);
}

int main (void)
{
    test_status_basic();
    test_status_stale_feedback_keeps_odometer();
    test_status_run_state_and_estop();
    test_status_mode();
    test_status_clamps_and_nulls();
    test_motion();
    test_physics();
    test_obstacle();

    (void)printf("%u checks, %u failed\n", g_checks, g_fail);
    return (g_fail == 0U) ? 0 : 1;
}
