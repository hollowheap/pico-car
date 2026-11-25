/** @file assemble_test.c
 * @brief Host test for the telemetry assembler (S46): the mapping from the
 *        subsystem snapshots to the packet structs.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "comms/telemetry/assemble.h"
#include "obstacle/obstacle_flags.h"
#include "obstacle/obstacle_ping.h"
#include "comms/telemetry/hug_word.h"

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
    telemetry_assemble_status(&out, &m, &f, &p, &o, 0x0020U, 7U, 0x03U, false, NULL);

    CHECK(out.speed_mms == 162);         /* mean of 160 and 164 mm/s */
    CHECK(out.odometer_mm == 1001U);     /* mean of 1000 and 1002 mm */
    CHECK(out.heading_ddeg == 3590U);
    CHECK(out.state == TELEMETRY_RUN_STATE_RUNNING);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_FOLLOW);
    CHECK(out.faults == 0x0020U); /* imu_nack: not a line-lost mode */
    CHECK(out.seq == 7U);
    CHECK(out.line_bits == 0x03U);
    CHECK(out.hold_mask == MOTION_HOLD_OBSTACLE);

    p.heading_valid = false;
    p.pivot_scale_pct = 67U;
    telemetry_assemble_status(&out, &m, &f, &p, &o, 0x0020U, 8U, 0x03U, false, NULL);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
    CHECK(out.pivot_scale_pct == 67U);
    p.ready = false;
    telemetry_assemble_status(&out, &m, &f, &p, &o, 0x0020U, 9U, 0x03U, false, NULL);
    CHECK(out.pivot_scale_pct == 0U);
}

static void test_status_stale_feedback_keeps_odometer (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    motion_feedback_t  f = fresh_feedback();

    f.fresh = false;
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.speed_mms == 0);
    CHECK(out.odometer_mm == 1001U);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
}

static void test_status_run_state_and_estop (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    motion_feedback_t  f = fresh_feedback();

    m.state = MOTION_STATE_IDLE;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.state == TELEMETRY_RUN_STATE_READY);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_STOPPED);

    m.state = MOTION_STATE_REVERSING;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.state == TELEMETRY_RUN_STATE_RUNNING);

    m.state = MOTION_STATE_FAULT;
    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.state == TELEMETRY_RUN_STATE_STOPPED);

    /* An accepted stop request inhibits drive and invalidates wheel speed. */
    m = running_motion();
    m.stop_requested = true;
    m.ready = false;
    f.fresh = false;
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0x0001U, 0U, 0U, false, NULL);
    CHECK(out.state == TELEMETRY_RUN_STATE_STOPPED);
    CHECK(out.faults == (0x0001U | BIT(TELEMETRY_FAULT_BIT_ESTOP_LATCHED)));
    CHECK(out.speed_mms == 0);

    /* No motion service at all. */
    telemetry_assemble_status(&out, NULL, NULL, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.state == TELEMETRY_RUN_STATE_INIT);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_STOPPED);
    CHECK(out.hold_mask == 0U);
}

static void test_status_link_fault_keeps_running (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    uint16_t const     wifi_fault = BIT(TELEMETRY_FAULT_BIT_WIFI_DISCONN);

    telemetry_assemble_status(&out, &m, NULL, NULL, NULL, wifi_fault, 0U, 0U, false, NULL);
    CHECK(out.state == TELEMETRY_RUN_STATE_RUNNING);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_FOLLOW);
    CHECK(out.faults == wifi_fault);
    CHECK(out.hold_mask == 0U);
}

static void test_status_mode (void)
{
    telemetry_status_t out;
    motion_status_t    m = running_motion();
    obstacle_status_t  o;

    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_SCANNING);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_HOLD);
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_WAIT_STOP);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_HOLD);
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_BYPASS);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, true, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_BYPASS); /* beats a turn */
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_REACQUIRE);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_BYPASS);

    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_MONITOR);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, true, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_TURN);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o,
                              BIT(TELEMETRY_FAULT_BIT_LINE_LOST), 0U, 0U, false, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_SEARCH);

    /* Lift and overturn win over everything else. */
    telemetry_assemble_status(&out, &m, NULL, NULL, &o,
                              BIT(TELEMETRY_FAULT_BIT_LIFTED), 0U, 0U, true, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_LIFTED);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o,
                              BIT(TELEMETRY_FAULT_BIT_LIFTED) |
                                  BIT(TELEMETRY_FAULT_BIT_OVERTURNED),
                              0U, 0U, true, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_OVERTURNED);

    /* The default pilot owns the wheels while the legacy scanner monitors. */
    uint32_t hug[HUG_WORD_COUNT] = {0};
    hug[0] = hug_word_put(1U, HUG_WORD_VALID_SH, HUG_WORD_VALID_MK) |
             hug_word_put(TELEMETRY_HUG_STATE_LEG, HUG_WORD_STATE_SH, HUG_WORD_STATE_MK);
    o = obstacle_in(TELEMETRY_OBSTACLE_STATE_MONITOR);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, hug);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_BYPASS);
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, BIT(TELEMETRY_FAULT_BIT_LIFTED), 0U, 0U, false, hug);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_LIFTED);
    /* The holds the supervisor sets show in the mode byte. */
    m.hold_mask = MOTION_HOLD_BARCODE;
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_HOLD_BARCODE);
    m.hold_mask = MOTION_HOLD_LIFT;
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, NULL);
    CHECK(out.mode == TELEMETRY_VEHICLE_MODE_LIFTED);
    m.hold_mask = MOTION_HOLD_OPERATOR;
    telemetry_assemble_status(&out, &m, NULL, NULL, &o, 0U, 0U, 0U, false, NULL);
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
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.speed_mms == INT16_MAX);

    f.left_speed_um_s  = -2000000000;
    f.right_speed_um_s = -2000000000;
    telemetry_assemble_status(&out, &m, &f, NULL, NULL, 0U, 0U, 0U, false, NULL);
    CHECK(out.speed_mms == INT16_MIN);

    /* A negative mean travel reads as zero. */
    f.left_displacement_um  = -5000000;
    f.right_displacement_um = -5000000;
    CHECK(telemetry_odometer_mm(&f) == 0U);
    CHECK(telemetry_odometer_mm(NULL) == 0U);
    f.left_displacement_um  = INT64_C(9000000000000);
    f.right_displacement_um = INT64_C(9000000000000);
    CHECK(telemetry_odometer_mm(&f) == UINT32_MAX);

    telemetry_assemble_status(NULL, &m, &f, NULL, NULL, 0U, 0U, 0U, false, NULL);
    telemetry_assemble_motion(NULL, &m, &f, NULL);
    telemetry_assemble_physics(NULL, NULL);
    telemetry_assemble_obstacle(NULL, NULL, NULL, 0U, NULL);
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
    p.roll_cdeg      = -4000;
    p.report_roll_cdeg = -2141;
    p.attitude_sample_valid = true;
    p.az_mg          = -271;
    p.amag_mg        = 1293U;
    p.attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED;
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
    CHECK(out.roll_cdeg == -2141);
    CHECK(out.az_mg == -271);
    CHECK(out.amag_mg == 1293U);
    CHECK(out.attitude_state == TELEMETRY_ATTITUDE_STATE_LIFTED);

    p.attitude_sample_valid = false;
    telemetry_assemble_physics(&out, &p);
    CHECK(!(out.flags & BIT(TELEMETRY_PHYSICS_FLAGS_BIT_CALIBRATED)));
    p.finished      = true;
    p.hump_active   = false;
    p.height_valid  = false;
    p.calibrated    = false;
    p.heading_valid = false;
    p.attitude_unknown = true;
    telemetry_assemble_physics(&out, &p);
    CHECK(out.flags == BIT(TELEMETRY_PHYSICS_FLAGS_BIT_FINISHED));
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
    /* Ready loss snapshots retain the last known attitude with stale faults. */
    CHECK(out.physics_faults == (BIT(TELEMETRY_PHYSICS_FAULTS_BIT_I2C) |
                                 BIT(TELEMETRY_PHYSICS_FAULTS_BIT_STALE)));
    CHECK(out.roll_cdeg == -2141);
    CHECK(out.az_mg == -271);
    CHECK(out.amag_mg == 1293U);
    CHECK(out.attitude_state == TELEMETRY_ATTITUDE_STATE_LIFTED);

    /* Not ready or missing: zeros and an invalid heading. */
    p.ready = false;
    telemetry_assemble_physics(&out, &p);
    CHECK(out.pitch_cdeg == 0);
    CHECK(out.roll_cdeg == 0);
    CHECK(out.az_mg == 0);
    CHECK(out.amag_mg == 0U);
    CHECK(out.attitude_state == 0U);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
    telemetry_assemble_physics(&out, NULL);
    CHECK(out.flags == 0U);
    CHECK(out.heading_ddeg == TELEMETRY_HEADING_INVALID);
}

static uint32_t ping_word (uint32_t seq, uint32_t echo, uint32_t distance_mm, int32_t servo_deci)
{
    uint32_t const servo = (uint32_t)((servo_deci / OBSTACLE_PING_DECI_PER_DEG) +
                                      OBSTACLE_PING_SERVO_OFFSET);

    return ((seq << OBSTACLE_PING_SEQ_SH) | (echo << OBSTACLE_PING_CLASS_SH) |
            (distance_mm << OBSTACLE_PING_DIST_SH) | (servo << OBSTACLE_PING_SERVO_SH));
}

static void test_obstacle (void)
{
    telemetry_obstacle_t    out;
    obstacle_status_t       o;
    obstacle_plan_summary_t plan;
    uint32_t const          ping = ping_word(61U, OBSTACLE_CLASS_VALID, 215U, -200);

    memset(&o, 0, sizeof o);
    o.ready       = true;
    o.state       = 3U;
    o.cause       = 7U;
    o.faults      = 0x181U;
    o.echo_class  = 2U;
    o.distance_mm = 20U;
    o.seq         = 29U;
    o.req_id      = 9U;
    o.servo_deci  = -300;
    o.body_deci   = 600;
    o.flags       = 0x48U;

    telemetry_assemble_obstacle(&out, &o, NULL, ping, NULL);
    CHECK(out.state == 3U);
    CHECK(out.cause == 7U);
    CHECK(out.faults == 0x181U);
    CHECK(out.echo_class == TELEMETRY_ECHO_CLASS_VALID);
    CHECK(out.dist_mm == 215U);
    CHECK(out.seq == 61U);
    CHECK(out.servo_deci == -200);
    CHECK(out.body_deci == 600);
    CHECK(out.flags == 0x48U);
    CHECK(out.width_mm == 0U);
    CHECK(out.hug_state == TELEMETRY_HUG_STATE_IDLE);
    CHECK(out.hug_side == TELEMETRY_HUG_SIDE_NONE);

    plan.closest_mm = 180U;
    plan.width_mm   = 205U;
    plan.need_mm    = 190U;
    plan.status     = 0U;
    plan.side       = 1U;
    plan.start_deg  = 60U;
    plan.b_wall     = false;
    telemetry_assemble_obstacle(&out, &o, &plan, ping, NULL);
    CHECK(out.closest_mm == 180U);
    CHECK(out.width_mm == 0U);
    CHECK(out.breadth_mm == 0U);
    CHECK(out.face_mm == 0U);

    telemetry_assemble_obstacle(&out, NULL, NULL, 0U, NULL);
    CHECK(out.state == 0U);
    CHECK(out.echo_class == TELEMETRY_ECHO_CLASS_NO_ECHO);
    CHECK(out.dist_mm == 0U);
}

/* The hug words independently fill the phase and geometry. */
static void test_obstacle_hug (void)
{
    telemetry_obstacle_t out;
    obstacle_status_t    o;
    hug_t                hug;
    uint32_t             words[HUG_WORD_COUNT];
    uint32_t const       none[HUG_WORD_COUNT] = {0U, 0U};
    uint32_t const       ping = ping_word(23U, OBSTACLE_CLASS_OPEN, 0U, 120);

    memset(&o, 0, sizeof o);
    o.ready      = true;
    o.servo_deci = 120;
    o.body_deci  = 50;

    /* No words, or words never published: the frame is as before. */
    telemetry_assemble_obstacle(&out, &o, NULL, ping, NULL);
    CHECK(out.hug_state == TELEMETRY_HUG_STATE_IDLE);
    CHECK(out.hug_side == TELEMETRY_HUG_SIDE_NONE);
    CHECK(out.servo_deci == 120);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, none);
    CHECK(out.hug_state == TELEMETRY_HUG_STATE_IDLE);
    CHECK(out.hug_side == TELEMETRY_HUG_SIDE_NONE);
    CHECK(out.servo_deci == 120);
    CHECK(out.width_mm == 0U);

    /* An idle hug shows idle and no side, but keeps the sizes it has seen. */
    memset(&hug, 0, sizeof hug);
    hug.width_mm   = 310U;
    hug.breadth_mm = 205U;
    hug.face_p_mm  = 249;
    hug.seen_mm    = 180;
    hug_word_pack(words, &hug, -150, 700);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.hug_state == TELEMETRY_HUG_STATE_IDLE);
    CHECK(out.hug_side == TELEMETRY_HUG_SIDE_NONE);
    CHECK(out.width_mm == 310U);
    CHECK(out.breadth_mm == 205U);
    CHECK(out.face_mm == 0U);
    CHECK(out.servo_deci == 120);
    CHECK(out.body_deci == 0);

    /* An active hug going right: state and side come from the hug. */
    hug.state   = 3U;  /* a drive */
    hug.then    = 5U;  /* then a look: a leg */
    hug.side    = -1;
    hug_word_pack(words, &hug, 300, -450);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.hug_state == TELEMETRY_HUG_STATE_LEG);
    CHECK(out.face_mm == 180U);
    CHECK(out.body_deci == -450);
    CHECK(out.hug_side == TELEMETRY_HUG_SIDE_RIGHT);
    CHECK(out.servo_deci == 120);
    hug.side = 1;
    hug_word_pack(words, &hug, 0, 1800);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.hug_side == TELEMETRY_HUG_SIDE_LEFT);
    CHECK(out.body_deci == 1800);
    hug_word_pack(words, &hug, 0, -1800);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.body_deci == -1800);
    CHECK(out.servo_deci == 120);

    /* Large values saturate in their own field and never spill. */
    hug.width_mm   = 60000U;
    hug.breadth_mm = 7U;
    hug.seen_mm    = -20;
    hug_word_pack(words, &hug, 5000, 0);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.width_mm == 4095U);
    CHECK(out.breadth_mm == 7U);
    CHECK(out.face_mm == 0U);
    CHECK(hug_word_servo_deci(words) == 300);

    /* The ranges of the pack: each distance saturates at 4095 mm, and the
     * servo goes in whole degrees, cut toward zero, within +-30 degrees. */
    hug.width_mm   = 4096U;
    hug.breadth_mm = 65535U;
    hug.seen_mm    = 4096;
    hug.bypass.phase = BYPASS_OUT;
    hug.bypass.b_wall = true;
    hug_word_pack(words, &hug, 0, 0);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK((out.flags & (1U << TELEMETRY_OBSTACLE_FLAGS_BIT_WALL)) != 0U);
    hug.bypass.phase = BYPASS_IDLE;
    hug_word_pack(words, &hug, 0, 0);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK((out.flags & (1U << TELEMETRY_OBSTACLE_FLAGS_BIT_WALL)) == 0U);
    hug_word_pack(words, &hug, 157, 0);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.width_mm == 4095U);
    CHECK(out.breadth_mm == 4095U);
    CHECK(out.face_mm == 4095U);
    CHECK(hug_word_servo_deci(words) == 150);
    hug.seen_mm = 4095;
    hug_word_pack(words, &hug, -157, 0);
    telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
    CHECK(out.face_mm == 4095U);
    CHECK(hug_word_servo_deci(words) == -150);
    hug_word_pack(words, &hug, -5000, 0);
    CHECK(hug_word_servo_deci(words) == -300);

    /* The hug sequence is two bits, shows in the frame, and wraps. It does
     * not disturb the other fields. */
    hug.width_mm = 310U;
    hug.seen_mm  = 180;
    hug_word_pack(words, &hug, 0, 0);
    CHECK(0U == hug_word_get(words[0], HUG_WORD_SEQ_SH, HUG_WORD_SEQ_MK));
    for (uint32_t seq = 0U; seq < 6U; seq++)
    {
        words[0] |= hug_word_put(seq, HUG_WORD_SEQ_SH, HUG_WORD_SEQ_MK);
        telemetry_assemble_obstacle(&out, &o, NULL, ping, words);
        CHECK(out.hug_seq == (seq & 3U));
        CHECK(out.width_mm == 310U);
        CHECK(out.face_mm == 180U);
        CHECK(out.hug_side == TELEMETRY_HUG_SIDE_LEFT);
        CHECK(hug_word_is_valid(words));
        words[0] &= ~hug_word_put(HUG_WORD_SEQ_MK, HUG_WORD_SEQ_SH, HUG_WORD_SEQ_MK);
    }
    telemetry_assemble_obstacle(&out, &o, NULL, ping, none);
    CHECK(out.hug_seq == 0U);
}

static void test_obstacle_sonar_sample (void)
{
    telemetry_obstacle_t out;
    obstacle_status_t    sonar = {0};
    hug_t                hug  = {0};
    uint32_t             words[HUG_WORD_COUNT];

    /* The line snapshot still sees a centered ray when sonar publishes a
     * side return. The old bearing makes this look like a forward obstacle. */
    sonar.state       = TELEMETRY_OBSTACLE_STATE_MONITOR;
    sonar.echo_class  = TELEMETRY_ECHO_CLASS_OPEN;
    sonar.seq         = 8U;
    hug.width_mm      = 420U;
    hug_word_pack(words, &hug, 0, 0);
    uint32_t const reply = ping_word(9U, OBSTACLE_CLASS_VALID, 180U, -120) |
                           (OBSTACLE_PING_REPLY_MK << OBSTACLE_PING_REPLY_SH);
    telemetry_assemble_obstacle(&out, &sonar, NULL, reply, words);
    CHECK(out.echo_class == TELEMETRY_ECHO_CLASS_VALID);
    CHECK(out.dist_mm == 180U);
    CHECK(out.seq == 9U);
    CHECK(out.servo_deci == -120);
    CHECK(out.hug_state == TELEMETRY_HUG_STATE_IDLE);
    CHECK(out.width_mm == 420U);

    /* The opposite writer order must not hide a new centered return. */
    sonar.distance_mm = 180U;
    sonar.seq         = 9U;
    sonar.servo_deci  = -120;
    hug_word_pack(words, &hug, -120, 0);
    telemetry_assemble_obstacle(&out, &sonar, NULL,
                                ping_word(10U, OBSTACLE_CLASS_VALID, 210U, 0), words);
    CHECK(out.echo_class == TELEMETRY_ECHO_CLASS_VALID);
    CHECK(out.dist_mm == 210U);
    CHECK(out.seq == 10U);
    CHECK(out.servo_deci == 0);

    telemetry_assemble_obstacle(&out, &sonar, NULL, 0U, words);
    CHECK(out.echo_class == TELEMETRY_ECHO_CLASS_NO_ECHO);
    CHECK(out.dist_mm == 0U);
    CHECK(out.seq == 0U);
    CHECK(out.servo_deci == 0);
    CHECK(out.width_mm == 420U);

    telemetry_assemble_obstacle(&out, NULL, NULL,
                                ping_word(62U, OBSTACLE_CLASS_TOO_CLOSE, 0U, -120), words);
    CHECK(out.echo_class == TELEMETRY_ECHO_CLASS_TOO_CLOSE);
    CHECK(out.dist_mm == OBSTACLE_PING_TOO_CLOSE_MM);
    CHECK(out.seq == 62U);
    CHECK(out.servo_deci == -120);

    const struct
    {
        uint8_t source;
        uint8_t wire;
    } failed_echoes[] = {{OBSTACLE_CLASS_NO_ECHO, TELEMETRY_ECHO_CLASS_NO_ECHO},
                         {OBSTACLE_CLASS_GLITCH, TELEMETRY_ECHO_CLASS_GLITCH},
                         {OBSTACLE_CLASS_STUCK, TELEMETRY_ECHO_CLASS_STUCK}};
    for (size_t i = 0; i < (sizeof failed_echoes / sizeof failed_echoes[0]); i++)
    {
        telemetry_assemble_obstacle(&out, &sonar, NULL,
                                    ping_word(12U, failed_echoes[i].source, 0U, 0), words);
        CHECK(out.echo_class == failed_echoes[i].wire);
        CHECK(out.dist_mm == 0U);
        CHECK(out.seq == 12U);
        CHECK(out.servo_deci == 0);
    }
}

/* The profile header comes from the plan of the same scan, so a client
 * pairs the profile with that scan by req_id. */
static void test_profile (void)
{
    telemetry_profile_t     out;
    obstacle_plan_summary_t plan;

    memset(&plan, 0, sizeof plan);
    plan.closest_mm = 180U;
    plan.width_mm   = 205U;
    plan.status     = 2U;
    plan.side       = 1U;
    plan.b_wall     = true;
    plan.req_id     = 11U;
    memset(&out, 0xA5, sizeof out);
    telemetry_assemble_profile(&out, &plan);
    CHECK(out.req_id == 11U);
    CHECK(out.status == 2U);
    CHECK(out.side == 1U);
    CHECK(out.closest_mm == 180U);
    CHECK(out.width_mm == 205U);
    CHECK(out.flags == (1U << TELEMETRY_OBSTACLE_FLAGS_BIT_WALL));
}

static void test_status_bundle_details (void)
{
    telemetry_status_bundle_t out;
    motion_feedback_t feedback = fresh_feedback();
    physics_status_t const physics = {
        .ready = true,
        .calibrated = true,
        .attitude_sample_valid = true,
    };
    obstacle_forward_sample_t forward = {
        .trigger_us = UINT32_MAX - 3999U,
        .distance_mm = 432U,
        .echo_class = OBSTACLE_CLASS_VALID,
    };

    feedback.timestamp_us = UINT32_MAX - 499U;
    feedback.left_displacement_um = -1201999;
    feedback.right_displacement_um = 991500;
    feedback.speed_valid[1] = true;
    feedback.odometry_valid[0] = true;
    telemetry_assemble_bundle_details(&out, 1000U, &feedback, &physics, &forward);
    CHECK(out.device_sample_us == 1000U);
    CHECK(out.left_travel_mm == -1201 && out.right_travel_mm == 991);
    CHECK(out.wheel_age_us == 1500U);
    CHECK(out.sample_flags == (BIT(TELEMETRY_STATUS_SAMPLE_FLAGS_BIT_WHEEL_FRESH) |
                               BIT(TELEMETRY_STATUS_SAMPLE_FLAGS_BIT_RIGHT_SPEED_VALID) |
                               BIT(TELEMETRY_STATUS_SAMPLE_FLAGS_BIT_LEFT_TRAVEL_VALID) |
                               BIT(TELEMETRY_STATUS_SAMPLE_FLAGS_BIT_ATTITUDE_VALID)));
    CHECK(out.forward_dist_mm == 432U && out.forward_echo_class == OBSTACLE_CLASS_VALID);
    CHECK(out.forward_age_ms == 5U);

    forward.echo_class = OBSTACLE_CLASS_NO_ECHO;
    telemetry_assemble_bundle_details(&out, 1000U, NULL, NULL, &forward);
    CHECK(out.forward_dist_mm == UINT16_MAX && out.forward_echo_class == OBSTACLE_CLASS_NO_ECHO);
    CHECK(out.forward_age_ms == 5U);
    CHECK(out.wheel_age_us == UINT16_MAX && out.sample_flags == 0U);

    feedback.fresh = false;
    feedback.left_displacement_um = INT64_MIN;
    feedback.right_displacement_um = INT64_MAX;
    telemetry_assemble_bundle_details(&out, 251000U, &feedback, NULL, &forward);
    CHECK(out.left_travel_mm == INT32_MIN && out.right_travel_mm == INT32_MAX);
    CHECK(out.wheel_age_us == UINT16_MAX);
    CHECK((out.sample_flags & BIT(TELEMETRY_STATUS_SAMPLE_FLAGS_BIT_WHEEL_FRESH)) == 0U);
    CHECK(out.forward_dist_mm == UINT16_MAX);
    CHECK(out.forward_echo_class == UINT8_MAX && out.forward_age_ms == UINT8_MAX);
}

int main (int argc, char ** argv)
{
    if ((2 == argc) && (0 == strcmp(argv[1], "link_fault")))
    {
        test_status_link_fault_keeps_running();
        return (g_fail == 0U) ? 0 : 1;
    }
    if ((2 == argc) && (0 == strcmp(argv[1], "estop_status")))
    {
        test_status_run_state_and_estop();
        return (g_fail == 0U) ? 0 : 1;
    }
    if (1 != argc)
    {
        (void)fprintf(stderr, "Usage: %s [link_fault|estop_status]\n", argv[0]);
        return 2;
    }
    test_status_basic();
    test_status_stale_feedback_keeps_odometer();
    test_status_run_state_and_estop();
    test_status_link_fault_keeps_running();
    test_status_mode();
    test_status_clamps_and_nulls();
    test_motion();
    test_physics();
    test_status_bundle_details();
    test_obstacle();
    test_obstacle_hug();
    test_obstacle_sonar_sample();
    test_profile();

    (void)printf("%u checks, %u failed\n", g_checks, g_fail);
    return (g_fail == 0U) ? 0 : 1;
}
