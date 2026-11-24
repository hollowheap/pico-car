/* Direct measurements supplied to the production pilot, hug and drive. */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "navigation/obstacle_ping.h"
#include "pilot/hug.h"
#include "telemetry/hug_word.h"
#include "pilot/look.h"
#include "pilot/pilot.h"
#include "platform/chassis.h"
#include "pilot/drive.h"
#include "platform/units.h"

static void check(bool b_ok, char const * p_what)
{
    if (!b_ok)
    {
        (void)printf("FAIL: %s\n", p_what);
        exit(EXIT_FAILURE);
    }
}

static pilot_meas_t fresh_meas(uint32_t now_ms)
{
    pilot_meas_t m = {0};

    m.now_ms = now_ms;
    m.stamp_ms = now_ms;
    m.b_line_left = true;

    return m;
}

static void test_direct(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = fresh_meas(1000U);

    pilot_init(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "fresh data drives");
    check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "follow line wins");
    check(cmd.speed_mm_s > 0, "drives forward");

    m.now_ms = m.stamp_ms + PILOT_STALE_MS;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "at the limit is still fresh");
    m.now_ms = m.stamp_ms + PILOT_STALE_MS + 1U;
    pilot_step(&st, &m, &cmd);
    check((PILOT_REASON_STALE == cmd.reason) && (0 == cmd.speed_mm_s) &&
              (0 == cmd.turn_deci_s) && (PILOT_BEHAVIOUR_SAFETY == cmd.behaviour),
          "stale data stops");
    m = fresh_meas(1200U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "stale ends by itself");

    /* The clock wraps. */
    pilot_init(&st);
    m = fresh_meas(0xFFFFFFF0U);
    m.now_ms = 5U;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "a wrapped clock is fresh");

    pilot_estop(&st);
    m = fresh_meas(1300U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_ESTOP == cmd.reason, "estop stops");
    m = fresh_meas(1400U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_ESTOP == cmd.reason, "estop latches");
    pilot_init(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "init (a reboot) ends estop");
    (void)printf("direct cases ok\n");
}

#define TICK_MS          (10U)
#define SLOW_OUTER_MM_S  (20)  /* travel of the outer wheel, mm/s */
#define UM_PER_MM_S_TICK (10)  /* travel of 1 mm/s over one tick, um */

/* Keep unrelated sonar evidence flowing in the direct motion fixtures. */
static void open_sonar(pilot_meas_t * p_meas)
{
    p_meas->ping_echo = LOOK_ECHO_OPEN;
    p_meas->ping_seq = (uint8_t)((p_meas->now_ms / 60U) & OBSTACLE_PING_SEQ_MK);
}

/* Wheel speeds the command asks for, mm/s. */
static void wheel_speeds(pilot_cmd_t const * p_cmd, int32_t * p_left, int32_t * p_right)
{
    int32_t const half = ((int32_t)p_cmd->turn_deci_s * CHASSIS_WHEEL_TRACK_MM) / 1146;

    *p_left = p_cmd->speed_mm_s - half;
    *p_right = p_cmd->speed_mm_s + half;
}

/* The inner wheel is commanded below the deadband and does not turn, while
 * the outer wheel turns slowly: a pivot about one wheel with healthy
 * encoders. Neither a dead encoder nor stuck. */
static void test_slow_wheel_seam(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    unsigned t;
    unsigned n_slow = 0U;

    pilot_init(&st);
    for (t = 0U; t < 500U; t++)
    {
        int32_t l, r;

        m.now_ms = 1000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        m.b_line_left = false;
        m.b_line_right = true;
        open_sonar(&m);
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_ENCODER != cmd.reason, "slow wheel read as a dead encoder");
        check(PILOT_REASON_STUCK != cmd.reason, "slow wheel read as stuck");
        wheel_speeds(&cmd, &l, &r);
        n_slow += ((r < PILOT_WHEEL_MIN_MM_S) && (l >= PILOT_WHEEL_MIN_MM_S)) ? 1U : 0U;
        m.left_um += (int64_t)SLOW_OUTER_MM_S * UM_PER_MM_S_TICK;
    }
    check(n_slow > ((1000U / TICK_MS) + 100U), "no slow inner wheel for over a second");
    (void)printf("slow wheel seam ok\n");
}

/* With an obstruction confirmed and both encoders fixed, a retry must
 * remeasure from rest. Changing course cannot inherit the failed drive. */
static void test_retry_label_seam(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    bool saw_drive = false;
    bool saw_probe = false;
    pilot_init(&st);
    m.b_stationary = true;
    for (unsigned tick = 0; tick < 800 && !saw_probe; tick++)
    {
        m.now_ms = m.stamp_ms = 1000U + tick * TICK_MS;
        open_sonar(&m);
        m.b_ping_reply = cmd.b_ping_req != 0;
        m.ping_servo_deci = cmd.b_ping_req ? cmd.ping_servo_deci : 0;
        if (cmd.b_ping_req)
        {
            m.ping_seq = cmd.ping_seq;
        }
        if (m.ping_servo_deci < 100)
        {
            m.ping_echo = LOOK_ECHO_VALID;
            m.ping_mm = 205U;
        }
        pilot_step(&st, &m, &cmd);
        check(cmd.speed_mm_s >= 0, "a frozen outward drive reversed without a new measurement");
        saw_probe |= saw_drive && cmd.b_ping_req && cmd.speed_mm_s == 0 && cmd.turn_deci_s == 0;
        saw_drive |= cmd.behaviour == PILOT_BEHAVIOUR_HUG && cmd.speed_mm_s > 0;
    }
    check(saw_drive && saw_probe, "a stationary axle must get a new look after the drive request");
    pilot_change_floor(&st, true);
    m.b_ping_reply = false;
    m.ping_servo_deci = 0;
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms - PILOT_STALE_MS - 1U;
    open_sonar(&m);
    pilot_step(&st, &m, &cmd);
    check(cmd.reason == PILOT_REASON_STALE && !cmd.speed_mm_s && !cmd.turn_deci_s,
          "stale wheels cannot carry the obstacle drive onto a new course");
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms;
    open_sonar(&m);
    pilot_step(&st, &m, &cmd);
    check(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE && cmd.speed_mm_s > 0,
          "clear course with fresh feedback must start forward");
}

/* While motion holds the wheels the pilot stops with HELD and does not read
 * the missing travel as stuck. */
static void test_held(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    unsigned t;

    pilot_init(&st);
    for (t = 0U; t < 50U; t++)
    {
        m.now_ms = 1000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        m.left_um += 3000;
        m.right_um += 3000;
        open_sonar(&m);
        pilot_step(&st, &m, &cmd);
    }
    m.b_held = true;
    for (t = 0U; t < 300U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        open_sonar(&m);
        pilot_step(&st, &m, &cmd);
        check((PILOT_REASON_HELD == cmd.reason) && (0 == cmd.speed_mm_s) &&
                  (0 == cmd.turn_deci_s) && (PILOT_BEHAVIOUR_SAFETY == cmd.behaviour),
              "a hold stops with HELD");
    }
    m.b_held = false;
    for (t = 0U; t < 150U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        m.left_um += 3000;
        m.right_um += 3000;
        open_sonar(&m);
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_NONE == cmd.reason, "STUCK or ENCODER after a hold");
        check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "follower runs after a hold");
    }

    pilot_init(&st);
    pilot_estop(&st);
    m = fresh_meas(5000U);
    m.b_held = true;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_ESTOP == cmd.reason, "estop wins over a hold");

    /* A held tick restarts a stuck window that is nearly over. */
    pilot_init(&st);
    m = fresh_meas(8000U);
    for (t = 0U; t < 90U; t++)
    {
        m.now_ms = 8000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        open_sonar(&m);
        pilot_step(&st, &m, &cmd);
    }
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms;
    m.b_held = true;
    open_sonar(&m);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_HELD == cmd.reason, "held tick in a stuck window");
    m.b_held = false;
    for (t = 0U; t < 50U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        open_sonar(&m);
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_NONE == cmd.reason, "a hold did not restart the stuck window");
    }
    (void)printf("held ok\n");
}

#define HUG_CODE_ST_PIVOT    (2U)
#define HUG_CODE_THEN_APPROACH (2U)
#define HUG_CODE_SUB_SETTLE  (1U)

static void hug_feed(hug_t * p_hug, look_t * p_look, uint8_t seq, uint8_t echo,
                     uint16_t mm, bool b_reply, bool * p_b_takes)
{
    hug_in_t in = {0};
    hug_out_t out;

    in.now_ms = (uint32_t)seq * 60U;
    in.ping_seq = seq;
    in.ping_echo = echo;
    in.ping_mm = mm;
    in.b_ping_reply = b_reply;
    *p_b_takes = hug_step(p_hug, p_look, &in, &out);
}

static void test_hug_trigger(void)
{
    static look_t look;
    hug_t hug;
    bool b_takes = false;

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, HUG_STANDOFF_MM, false, &b_takes);
    check(!b_takes, "one hit does not trigger");
    hug_feed(&hug, &look, 2U, LOOK_ECHO_VALID, HUG_STANDOFF_MM + 1, false, &b_takes);
    check(!b_takes, "a miss does not trigger");
    hug_feed(&hug, &look, 3U, LOOK_ECHO_VALID, HUG_STANDOFF_MM, false, &b_takes);
    check(b_takes && hug_active(&hug), "two of the last three trigger");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    hug_feed(&hug, &look, 2U, LOOK_ECHO_OPEN, 0U, false, &b_takes);
    hug_feed(&hug, &look, 3U, LOOK_ECHO_OPEN, 0U, false, &b_takes);
    hug_feed(&hug, &look, 4U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    check(!b_takes, "hits three pings apart do not trigger");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, true, &b_takes);
    hug_feed(&hug, &look, 2U, LOOK_ECHO_VALID, 100U, true, &b_takes);
    hug_feed(&hug, &look, 3U, LOOK_ECHO_VALID, 100U, true, &b_takes);
    check(!b_takes, "replies to requests do not trigger");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    check(!b_takes, "one ping seen again does not count twice");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_TOO_CLOSE, 15U, false, &b_takes);
    hug_feed(&hug, &look, 2U, LOOK_ECHO_VALID, HUG_STANDOFF_MM, false, &b_takes);
    check(b_takes && hug_active(&hug), "a TOO_CLOSE ping counts as a hit");
    (void)printf("hug trigger ok\n");
}

static void test_sonar_fault(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = fresh_meas(1000U);

    pilot_init(&st);
    m.b_sonar_fault = true;
    pilot_step(&st, &m, &cmd);
    check((PILOT_REASON_SONAR == cmd.reason) && (0 == cmd.speed_mm_s) && (0 == cmd.turn_deci_s),
          "a sonar fault stops with SONAR");
    m.b_sonar_fault = false;
    m = fresh_meas(1010U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "SONAR ends when the fault clears");
    (void)printf("sonar fault ok\n");
}

#define HUG_CODE_ST_SCAN   (1U)
#define HUG_CODE_SUB_ASK   (2U)
#define HUG_CODE_PU_LOOK   (1U)
#define HUG_CODE_F_TURNED  (0x08U)
#define BREADTH_RAYS       (15U)
#define BREADTH_STEP_DECI  (50)
#define BREADTH_FACE_MM    (250.0)
#define BREADTH_LEFT_MM    (40.0)
#define BREADTH_MEASURED   (55U)

static void breadth_ray(int32_t servo_deci, uint8_t * p_echo, int32_t * p_mm)
{
    double const rad = 3.14159265358979323846 / 1800.0;
    double const hi = BREADTH_FACE_MM * tan(((double)servo_deci + (double)LOOK_BEAM_DECI) * rad);
    double const lo = BREADTH_FACE_MM * tan(((double)servo_deci - (double)LOOK_BEAM_DECI) * rad);
    double const a = lo;
    double const b = (hi < BREADTH_LEFT_MM) ? hi : BREADTH_LEFT_MM;
    double y = 0.0;

    *p_echo = (uint8_t)LOOK_ECHO_OPEN;
    *p_mm = 0;
    if (a <= b)
    {
        y = (a > 0.0) ? a : ((b < 0.0) ? b : 0.0);
        *p_echo = (uint8_t)LOOK_ECHO_VALID;
        *p_mm = (int32_t)(sqrt((BREADTH_FACE_MM * BREADTH_FACE_MM) + (y * y)) + 0.5);
    }
}

static void test_breadth_kept(void)
{
    static look_t look;
    look_pose_t const pose = {0, 0, 0};
    hug_t hug;
    hug_in_t in = {.b_stationary = true};
    hug_out_t out;
    uint8_t echo = 0U;
    int32_t mm = 0;

    hug_init(&hug);
    look_clear(&look);
    for (uint32_t i = 0U; i < (BREADTH_RAYS - 1U); i++)
    {
        int32_t const servo = ((int32_t)i * BREADTH_STEP_DECI) - 350;

        breadth_ray(servo, &echo, &mm);
        (void)look_add(&look, &pose, servo, echo, mm);
    }
    hug.state = (uint8_t)HUG_CODE_ST_SCAN;
    hug.sub = (uint8_t)HUG_CODE_SUB_ASK;
    hug.purpose = (uint8_t)HUG_CODE_PU_LOOK;
    hug.flags = (uint8_t)HUG_CODE_F_TURNED;
    hug.idx = (uint8_t)(BREADTH_RAYS - 1U);
    hug.body = 11U;
    hug.req_seq = 5U;
    hug.breadth_mm = BREADTH_MEASURED;
    breadth_ray(350, &echo, &mm);
    in.now_ms = 1000U;
    in.b_ping_reply = true;
    in.ping_seq = 5U;
    in.ping_echo = echo;
    in.ping_mm = (uint16_t)mm;
    /* The last 35 degree bearing is sampled at 15 degrees of chassis yaw. */
    hug.diff_um = (int32_t)(150 * CHASSIS_WHEEL_TRACK_MM * 1000 / DECI_PER_RAD);
    hug.t_ms = in.now_ms;
    in.ping_servo_deci = 200;
    (void)hug_step(&hug, &look, &in, &out);
    check(hug.state != (uint8_t)HUG_CODE_ST_SCAN, "the last ray ends the look");
    check(BREADTH_MEASURED == hug.breadth_mm, "a look with one edge keeps the breadth");
    (void)printf("breadth kept ok\n");
}

/* The pivot before a look has stopped at the first body step and settled, so
 * the scan does not settle again: only when the car is already on that step at
 * the first tick of the scan. A pivot that has to turn settles after it. The
 * hug is put in the first move of a look by hand (codes as in hug.c). */
#define HUG_CODE_SUB_START   (3U)
#define HUG_CODE_F_AIMED     (0x800U)
#define AIM_STEP_DECI        (OBSTACLE_SERVO_LIMIT_DECI - HUG_NEAR_ZONE_DECI)
#define AIM_TICK_MS          (10U)

static void aim_head(hug_t * p_hug, int32_t deci)
{
    double const rad = ((double)deci / 1800.0) * 3.14159265358979323846;

    p_hug->diff_um = (int32_t)(rad * (double)CHASSIS_WHEEL_TRACK_MM * 1000.0);
}

static void test_missed_rear_leg_reply(uint8_t next_echo)
{
    hug_t hug;
    look_t look;
    hug_in_t in = {.now_ms = 1200U, .b_stationary = true, .ping_echo = LOOK_ECHO_NONE, .ping_seq = 1U};
    hug_out_t out;
    hug_init(&hug);
    look_clear(&look);
    hug.state = HUG_CODE_ST_PIVOT;
    hug.sub = HUG_CODE_SUB_SETTLE;
    hug.then = 10U; /* Return to the entry lateral coordinate. */
    hug.t_ms = in.now_ms - HUG_SETTLE_MS;
    hug.last_motion_ms = hug.t_ms;
    hug.goal_head = -900;
    hug.y_um = 60000;
    aim_head(&hug, hug.goal_head);
    check(hug_step(&hug, &look, &in, &out) && !out.b_timeout && !out.speed_mm_s,
          "one unresolved rear ping keeps the planned leg waiting for clearance");
    in.now_ms += (next_echo == LOOK_ECHO_NONE) ? HUG_REPLY_TIMEOUT_MS : 60U;
    in.ping_seq++;
    in.ping_echo = next_echo;
    (void)hug_step(&hug, &look, &in, &out);
    if (next_echo == LOOK_ECHO_OPEN)
    {
        in.now_ms += 10U;
        (void)hug_step(&hug, &look, &in, &out);
        check(out.speed_mm_s > 0 && !out.b_timeout, "the next fresh clear ping starts the rear leg");
    }
    else
    {
        check(out.b_timeout && !out.speed_mm_s, "an actual obstruction or expired rear reply window ends the leg");
    }
}

static void test_leg_pivot_coast(void)
{
    hug_t hug;
    look_t look;
    hug_in_t in = {.now_ms = 1000U, .b_stationary = true, .ping_echo = LOOK_ECHO_OPEN};
    hug_out_t out;
    hug_init(&hug);
    look_clear(&look);
    hug.state = HUG_CODE_ST_PIVOT;
    hug.then = 4U; /* Leg follows alignment. */
    hug.leg_mm = HUG_LEG_MIN_MM;
    hug.u_head = hug.goal_head = 900;
    hug.last_motion_ms = in.now_ms;
    hug.t_ms = in.now_ms + 1000U;
    aim_head(&hug, 900);
    (void)hug_step(&hug, &look, &in, &out);
    aim_head(&hug, 935);
    in.now_ms += HUG_SETTLE_MS;
    in.ping_seq++;
    (void)hug_step(&hug, &look, &in, &out);
    in.now_ms += 10U;
    (void)hug_step(&hug, &look, &in, &out);
    check(out.turn_deci_s < 0 && !out.speed_mm_s, "correct the coast after a planned leg pivot");
    aim_head(&hug, 900);
    in.now_ms += 10U;
    (void)hug_step(&hug, &look, &in, &out);
    in.now_ms += HUG_SETTLE_MS;
    (void)hug_step(&hug, &look, &in, &out);
    check(!out.speed_mm_s, "pre-correction sonar cannot start a leg");
    in.ping_seq++;
    in.now_ms += 10U;
    (void)hug_step(&hug, &look, &in, &out);
    in.now_ms += 10U;
    (void)hug_step(&hug, &look, &in, &out);
    check(out.speed_mm_s > 0, "a corrected, clear leg advances");
}

static void test_contact_side_uses_usable_rays(void)
{
    int16_t const contact_angles[] = {HUG_CONTACT_RAYS};
    uint8_t const wide_echoes[] = {LOOK_ECHO_OPEN, LOOK_ECHO_NONE};
    look_pose_t const pose = {0};

    for (size_t i = 0U; i < sizeof(wide_echoes) / sizeof(wide_echoes[0]); i++)
    {
        hug_t hug;
        look_t look;
        hug_in_t in = {.b_stationary = true};
        hug_out_t out;

        hug_init(&hug);
        look_clear(&look);
        /* Neither edge of this wide face is in the usable contact look. */
        for (int32_t angle = -HUG_CONTACT_NEAR_DECI; angle <= HUG_CONTACT_NEAR_DECI;
             angle += HUG_NEAR_STEP_DECI)
        {
            double const off_face = fmax(0.0, (double)abs(angle) - LOOK_BEAM_DECI);
            double const range = 300.0 / cos(off_face * 3.14159265358979323846 / 1800.0);

            check(look_add(&look, &pose, angle, LOOK_ECHO_VALID, (int32_t)(range + 0.5)),
                  "contact face ray is recorded");
        }
        hug.state = (uint8_t)HUG_CODE_ST_SCAN;
        hug.sub = (uint8_t)HUG_CODE_SUB_ASK;
        hug.idx = (uint8_t)((sizeof(contact_angles) / sizeof(contact_angles[0])) - 1U);
        hug.req_seq = 5U;
        hug.open_mm[LOOK_SIDE_RIGHT] = 1000U; /* Two usable 500 mm wide returns. */
        aim_head(&hug, 750);
        in.b_ping_reply = true;
        in.ping_seq = 5U;
        in.ping_servo_deci = (int16_t)(contact_angles[hug.idx] - 750);
        in.ping_echo = wide_echoes[i];
        (void)hug_step(&hug, &look, &in, &out);
        check(hug.side == ((LOOK_ECHO_OPEN == in.ping_echo) ? 1 : -1),
              "an unresolved wide ray cannot win the side choice over valid returns");
    }
}

/* Ticks from the pivot done to the request for a ping; -1 for none. */
static int aim_run(int32_t head_deci)
{
    static look_t look;
    hug_t hug;
    hug_in_t in = {.b_stationary = true};
    hug_out_t out;
    int done_ms = -1;
    int req_ms = -1;
    unsigned t;

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_SCAN;
    hug.sub = (uint8_t)HUG_CODE_SUB_START;
    hug.purpose = (uint8_t)HUG_CODE_PU_LOOK;
    hug.flags = (uint16_t)HUG_CODE_F_AIMED;
    aim_head(&hug, head_deci);
    in.now_ms = 1000U;
    for (t = 0U; (t < 60U) && (req_ms < 0); t++)
    {
        (void)hug_step(&hug, &look, &in, &out);
        if ((done_ms < 0) && (0 == out.turn_deci_s))
        {
            done_ms = (int)(t * AIM_TICK_MS);
            aim_head(&hug, AIM_STEP_DECI);
        }
        else if (0U != out.b_req)
        {
            req_ms = (int)(t * AIM_TICK_MS);
        }
        else if (0 != out.turn_deci_s)
        {
            aim_head(&hug, AIM_STEP_DECI);
        }
        in.now_ms += AIM_TICK_MS;
    }
    return (req_ms < 0) ? -1 : (req_ms - done_ms);
}

static void test_aimed_settle(void)
{
    int const settled = aim_run(AIM_STEP_DECI);
    int const turned = aim_run(0);

    (void)printf("aimed settle: on the step %d ms, turned %d ms "
                 "(settle %u ms)\n",
                 settled, turned, (unsigned)HUG_SETTLE_MS);
    check((settled >= 0) && (settled < (int)HUG_SETTLE_MS),
          "a scan already on its step does not settle twice");
    check(turned >= (int)HUG_SETTLE_MS,
          "a scan that had to turn settles after the turn");
}

static void test_scan_requests_the_next_bearing(void)
{
    hug_t hug;
    look_t look;
    hug_in_t in = {.b_stationary = true};
    hug_out_t out = {0};

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_SCAN;
    hug.sub = (uint8_t)HUG_CODE_SUB_START;
    hug.purpose = (uint8_t)HUG_CODE_PU_LOOK;
    hug.flags = (uint16_t)HUG_CODE_F_AIMED;
    aim_head(&hug, AIM_STEP_DECI);
    for (unsigned i = 0U; (i < 40U) && (0U == out.b_req); i++)
    {
        in.now_ms += AIM_TICK_MS;
        (void)hug_step(&hug, &look, &in, &out);
    }
    check(0U != out.b_req, "the scan requests its first ray");
    in.now_ms += AIM_TICK_MS;
    in.b_ping_reply = true;
    in.ping_seq = out.seq;
    in.ping_servo_deci = out.servo_deci;
    in.ping_echo = LOOK_ECHO_OPEN;
    (void)hug_step(&hug, &look, &in, &out);
    check((0U != out.b_req) && (out.seq != in.ping_seq) &&
          (out.servo_deci == (in.ping_servo_deci + HUG_NEAR_STEP_DECI)),
          "the next scan sequence requests the next bearing");
}

static void test_side_look_uses_two_headings(int8_t direction)
{
    hug_t hug;
    look_t look;
    hug_in_t in = {.b_stationary = true};
    hug_out_t out = {0};
    int32_t const ray_count = (2 * HUG_NEAR_ZONE_DECI / HUG_NEAR_STEP_DECI) + 1;
    int32_t rays = 0;
    int32_t headings = 0;
    int32_t last_head = 0;
    double head_deci = 0.0;

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_SCAN;
    hug.sub = (uint8_t)HUG_CODE_SUB_START;
    hug.purpose = (uint8_t)HUG_CODE_PU_LOOK;
    hug.side = direction;
    for (unsigned i = 0U; (i < 400U) && (rays < ray_count); i++)
    {
        in.now_ms += AIM_TICK_MS;
        (void)hug_step(&hug, &look, &in, &out);
        check(out.b_scan_pivot == (0 != out.turn_deci_s),
              "the scan pivot flag follows the current turn command");
        in.b_ping_reply = (0U != out.b_req);
        if (in.b_ping_reply)
        {
            int32_t const head = hug_heading_deci(&hug);
            int32_t const bearing = direction * ((rays * HUG_NEAR_STEP_DECI) - HUG_NEAR_ZONE_DECI);

            check((0 == out.turn_deci_s) && (0 == out.speed_mm_s),
                  "the side look only pings while stationary");
            check(abs(out.servo_deci) <= OBSTACLE_SERVO_LIMIT_DECI,
                  "the side look fits the physical servo travel");
            check(abs(head + out.servo_deci - bearing) <= HUG_HEAD_TOL_DECI,
                  "the side look keeps every planned bearing");
            headings += ((0 == rays) || (last_head != head)) ? 1 : 0;
            last_head = head;
            rays++;
            in.ping_seq = out.seq;
            in.ping_servo_deci = out.servo_deci;
            in.ping_echo = LOOK_ECHO_OPEN;
        }
        head_deci += (double)out.turn_deci_s * (double)AIM_TICK_MS / 1000.0;
        aim_head(&hug, (int32_t)head_deci);
    }
    check(ray_count == rays, "the side look requests its full sweep");
    check(2 == headings, "the side look uses two chassis headings");
}

#define HUG_CODE_ST_DRIVE (3U)
#define HUG_CODE_THEN_LOOK (5U)
#define HUG_CODE_F_PAST (0x0001U)
#define HUG_CODE_F_EDGE_LEG (0x2000U)

typedef enum { CORNER_CLEAR, CORNER_BLOCKED, CORNER_DELAYED, CORNER_UNKNOWN, CORNER_INTERRUPTED,
               CORNER_DRIVE_HELD } corner_check_t;

static void test_measured_corner_checks_next_leg(corner_check_t scenario)
{
    hug_t hug;
    look_t look;
    hug_in_t in = {.b_stationary = true};
    hug_out_t out = {0};

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_DRIVE;
    hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    hug.then = (uint8_t)HUG_CODE_THEN_LOOK;
    hug.flags = (uint16_t)(HUG_CODE_F_PAST | HUG_CODE_F_EDGE_LEG);
    hug.side = 1;
    hug.face_p_mm = HUG_AXLE_OFF_MM;
    hug.keep_x_mm[LOOK_SIDE_LEFT] = HUG_AXLE_OFF_MM;
    hug.y_um = HUG_AXLE_OFF_MM * 1000;
    in.now_ms = HUG_SETTLE_MS;

    for (unsigned i = 0U; (i < 40U) && (0U == out.b_req); i++)
    {
        in.now_ms += AIM_TICK_MS;
        (void)hug_step(&hug, &look, &in, &out);
        check((0 == out.speed_mm_s) && (0 == out.turn_deci_s),
              "a reached corner checks the next leg without turning back to the old face");
    }
    unsigned attempt = 0U;
    for (int16_t ray = -150; ray <= 150; ray += 150)
    {
        check((0U != out.b_req) && (ray == out.servo_deci), "the next leg is checked at three bearings");
        in.now_ms += AIM_TICK_MS;
        in.b_ping_reply = true;
        in.ping_seq = out.seq;
        in.ping_servo_deci = out.servo_deci;
        in.ping_echo = (uint8_t)(((CORNER_BLOCKED == scenario) && (0 == ray)) ? LOOK_ECHO_VALID : LOOK_ECHO_OPEN);
        in.ping_mm = (LOOK_ECHO_VALID == in.ping_echo) ? 200U : 0U;
        if ((150 == ray) && (((CORNER_DELAYED == scenario) && (0U == attempt)) || (CORNER_UNKNOWN == scenario) ||
                             ((CORNER_INTERRUPTED == scenario) && (0U == attempt))))
        {
            uint8_t const expired_seq = in.ping_seq;
            in.now_ms += (CORNER_DELAYED == scenario) ? HUG_REPLY_TIMEOUT_MS + 1U : 0U;
            if (CORNER_DELAYED != scenario) in.ping_echo = LOOK_ECHO_NONE;
            (void)hug_step(&hug, &look, &in, &out);
            check(!out.speed_mm_s && !out.turn_deci_s, "an uncertain corner bearing cannot start a drive");
            if (attempt > 0U)
            {
                check(out.b_timeout && !hug_active(&hug), "exhausted corner evidence budgets end the drive under a stop");
                return;
            }
            check(hug.side == 1 && hug.keep_x_mm[LOOK_SIDE_LEFT] == HUG_AXLE_OFF_MM && look.count == 0U,
                  "a local retry remembers the measured edge and discards the partial clearance rays");
            if (CORNER_INTERRUPTED == scenario)
            {
                in.b_other = true;
                in.now_ms += AIM_TICK_MS;
                (void)hug_step(&hug, &look, &in, &out);
                check(!out.speed_mm_s && !out.turn_deci_s && HUG_CODE_PU_LOOK == hug.purpose,
                      "an independent wheel owner cancels the retained corner probe for a fresh look");
                return;
            }
            for (unsigned tick = 0U; tick < 40U && !out.b_req; tick++)
            {
                in.b_ping_reply = false;
                in.now_ms += AIM_TICK_MS;
                (void)hug_step(&hug, &look, &in, &out);
                check(!out.speed_mm_s && !out.turn_deci_s, "a local retry keeps the wheels still at the measured edge");
            }
            check(out.b_req && out.seq != expired_seq && out.servo_deci == -150,
                  "a late or unusable ray starts a fresh three-bearing check at the saved corner");
            attempt++;
            ray = -300;
            continue;
        }
        (void)hug_step(&hug, &look, &in, &out);
        check(0 == out.speed_mm_s, "the hug holds until it has checked all three bearings");
    }
    in.now_ms += AIM_TICK_MS;
    in.b_ping_reply = false;
    if (CORNER_DRIVE_HELD == scenario)
    {
        in.b_other = true;
        (void)hug_step(&hug, &look, &in, &out);
        check(!out.speed_mm_s && !out.turn_deci_s, "another wheel owner takes priority over the edge leg");
        in.b_other = false;
        in.now_ms += AIM_TICK_MS;
        (void)hug_step(&hug, &look, &in, &out);
        check(!out.speed_mm_s && HUG_CODE_PU_LOOK == hug.purpose,
              "clearance measured before losing the wheels cannot authorize the edge leg");
        return;
    }
    (void)hug_step(&hug, &look, &in, &out);
    if (CORNER_BLOCKED == scenario)
    {
        check((0 == out.speed_mm_s) && (0 != out.turn_deci_s),
              "a blocked corner falls back to a wider look");
    }
    else
    {
        check((out.speed_mm_s > 0) && (0 == out.turn_deci_s),
              "the hug drives the next leg after three clear readings");
    }
}

/* The note of a stall hug that gave the wheels back: the car moves about while
 * the hug is idle (wheel travel only), then stalls. A stall at the place of the
 * last one is a repeat (flag code as in hug.c); one anywhere else is a first
 * stall. The car is put at the pose the release leaves it in, 260 mm before the
 * stall. */
#define HUG_CODE_F_REPEAT    (0x200U)
#define HUG_CODE_F_EXIT      (0x04U)
#define HUG_CODE_THEN_RELEASE (8U)
#define NOTE_RELEASE_X_MM    (-260)
#define NOTE_TICK_MS         (50U)
#define NOTE_PIVOT_MS        (10U)
#define NOTE_STEP_MM         (10)
#define NOTE_STEP_UM         ((int64_t)NOTE_STEP_MM * 1000)
#define NOTE_WAIT_MS         (60000U)
#define NOTE_ARC_RADIUS_MM   (300)
#define NOTE_LONG_HUG_MS     (20000U)

typedef struct
{
    hug_t    hug;
    hug_in_t in;
} note_car_t;

static look_t g_note_look;

static void note_start(note_car_t * p_car)
{
    hug_init(&p_car->hug);
    look_clear(&g_note_look);
    p_car->in = (hug_in_t){0};
    p_car->in.now_ms = 1000U;
    p_car->hug.b_note = 1U;
    p_car->hug.t_ms = p_car->in.now_ms;
    p_car->hug.x_um = NOTE_RELEASE_X_MM * 1000;
}

static void note_tick(note_car_t * p_car, int64_t left_um, int64_t right_um,
                      uint32_t dt_ms)
{
    hug_out_t out;

    p_car->in.left_um += left_um;
    p_car->in.right_um += right_um;
    p_car->in.now_ms += dt_ms;
    (void)hug_step(&p_car->hug, &g_note_look, &p_car->in, &out);
}

static void note_forward(note_car_t * p_car, int mm)
{
    for (int i = 0; i < (mm / NOTE_STEP_MM); i++)
    {
        note_tick(p_car, NOTE_STEP_UM, NOTE_STEP_UM, NOTE_TICK_MS);
    }
}

/* Turn in place, degrees to the left, a degree a tick; the hug reads 80 percent
 * of the wheel travel of a pivot as turning. */
static void note_turn(note_car_t * p_car, int deg)
{
    double const pct = (double)HUG_PIVOT_PCT / 100.0;
    double const um = (3.14159265358979323846 / 180.0) *
                      (double)CHASSIS_WHEEL_TRACK_MM * 1000.0 / pct;
    int const n = (deg < 0) ? -deg : deg;
    int64_t const half = (int64_t)(um / 2.0) * ((deg < 0) ? -1 : 1);

    for (int i = 0; i < n; i++)
    {
        note_tick(p_car, -half, half, NOTE_PIVOT_MS);
    }
}

static void note_wait(note_car_t * p_car, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += NOTE_TICK_MS)
    {
        note_tick(p_car, 0, 0, NOTE_TICK_MS);
    }
}

/* A gentle arc to the left, a tick a step, the wheels apart by the track over
 * the radius. The car circles near the place of the first stall. */
static void note_arc(note_car_t * p_car, int ticks)
{
    double const half = (double)NOTE_STEP_UM * (double)CHASSIS_WHEEL_TRACK_MM /
                        (2.0 * (double)NOTE_ARC_RADIUS_MM);

    for (int i = 0; i < ticks; i++)
    {
        note_tick(p_car, NOTE_STEP_UM - (int64_t)half,
                  NOTE_STEP_UM + (int64_t)half, NOTE_TICK_MS);
    }
}

/* The hug is put in the settle before its release by hand. */
static void note_release(note_car_t * p_car, uint16_t flags)
{
    p_car->hug.state = (uint8_t)HUG_CODE_ST_PIVOT;
    p_car->hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    p_car->hug.then = (uint8_t)HUG_CODE_THEN_RELEASE;
    p_car->hug.flags = flags;
    p_car->hug.t_ms = p_car->in.now_ms;
    p_car->in.b_stationary = true;
    p_car->in.ping_echo = LOOK_ECHO_OPEN;
    p_car->in.ping_seq++;
    if (0U != (flags & HUG_CODE_F_EXIT))
    {
        p_car->hug.tape.crossing.heading_deci = hug_heading_deci(&p_car->hug);
        p_car->hug.origin.drive.x_mm = p_car->hug.x_um / 1000;
        p_car->hug.origin.drive.y_mm = p_car->hug.y_um / 1000;
    }
    note_tick(p_car, 0, 0, HUG_SETTLE_MS);
}

static bool note_stall_is_repeat(note_car_t * p_car)
{
    p_car->in.b_stall = true;
    note_tick(p_car, 0, 0, NOTE_TICK_MS);
    p_car->in.b_stall = false;
    return hug_active(&p_car->hug) &&
           (0U != (p_car->hug.flags & HUG_CODE_F_REPEAT));
}

static void test_note(void)
{
    static note_car_t car;
    bool b_repeat = false;

    /* Round and back to the wall: the same place, though the wheels travelled
     * 900 mm. */
    note_start(&car);
    note_turn(&car, 180);
    note_forward(&car, 300);
    note_turn(&car, 180);
    note_forward(&car, 600);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall at the same place after a turn round is a repeat");

    /* 450 mm on, past the 150 mm: a new place. */
    note_start(&car);
    note_forward(&car, 450);
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall far past the first is not a repeat");

    /* The line turns away and the car goes far along it, within the note's
     * time. */
    note_start(&car);
    note_turn(&car, 90);
    note_forward(&car, 800);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall within the note's life, line turned away: repeat");

    /* The same path, and the note has run out. */
    note_start(&car);
    note_turn(&car, 90);
    note_forward(&car, 800);
    note_wait(&car, NOTE_WAIT_MS);
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall long after the first is not a repeat");

    /* A hug that ends at the tape clears the note, whatever the car did. */
    note_start(&car);
    note_release(&car, (uint16_t)HUG_CODE_F_EXIT);
    check(!hug_active(&car.hug) && (0U == car.hug.b_note),
          "a hug that ends at the tape clears the note");
    note_start(&car);
    note_release(&car, 0U);
    check(!hug_active(&car.hug) && (0U != car.hug.b_note),
          "a release with no wrap keeps the note");

    /* A hug that ends at the tape clears the count of releases as well. */
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    note_release(&car, (uint16_t)HUG_CODE_F_EXIT);
    check(!hug_active(&car.hug) && (0U == car.hug.n_rel),
          "a hug that ends at the tape clears the count of releases");
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall after the tape is not a repeat, count or not");

    /* Past the releases allowed, the next stall wraps whatever the note says;
     * the count outlives the note cleared by travel, and times out alone. */
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    note_forward(&car, 450);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "the count stands after the note was cleared by travel");
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    note_forward(&car, 450);
    note_wait(&car, NOTE_WAIT_MS);
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "the count times out after the note was cleared");

    /* A hug that began with no stall (the sonar) starts the count again. */
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    car.in.b_stationary = true;
    car.in.ping_echo = LOOK_ECHO_VALID;
    car.in.ping_mm = 205U;
    car.in.ping_seq++;
    note_tick(&car, 0, 0, NOTE_TICK_MS);
    car.in.ping_seq++;
    note_tick(&car, 0, 0, NOTE_TICK_MS);
    check(hug_active(&car.hug) && (0U == car.hug.n_rel),
          "a hug started by the sonar starts the count again");
    hug_out_t out = {0};
    car.in.ping_mm = 0U;
    car.in.ping_echo = LOOK_ECHO_OPEN;
    for (unsigned tick = 0U; hug_active(&car.hug) && tick < 40U; tick++)
    {
        car.in.now_ms += NOTE_TICK_MS;
        car.in.b_ping_reply = out.b_req != 0U;
        car.in.ping_seq = out.seq;
        car.in.ping_servo_deci = out.servo_deci;
        (void)hug_step(&car.hug, &g_note_look, &car.in, &out);
    }
    check(!hug_active(&car.hug), "clear readings from the stopped pose release the old contact");
    car.in.b_ping_reply = false;
    car.in.ping_servo_deci = 0;
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall after a hug the sonar started is not a repeat");

    /* The note's time runs from the end of the hug, not from its start. */
    note_start(&car);
    car.in.now_ms += NOTE_LONG_HUG_MS;
    note_release(&car, 0U);
    note_wait(&car, 1000U);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall just after a long hug is still a repeat");

    /* The car circles near the first stall and only the time clears the note:
     * a repeat at exactly HUG_NOTE_MS, none one tick past it. */
    note_start(&car);
    note_arc(&car, (int)(HUG_NOTE_MS / NOTE_TICK_MS) - 1);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall at exactly HUG_NOTE_MS is still a repeat");
    note_start(&car);
    note_arc(&car, (int)(HUG_NOTE_MS / NOTE_TICK_MS));
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat,
          "a stall one tick after the note runs out is not a repeat");
    (void)printf("stall note ok\n");
}

/* A move of the hug that creeps without reaching its goal ends after HUG_MOVE_TIMES
 * times its planned time, which is at least HUG_MOVE_FLOOR_MS. The hug is put
 * in the settle before the move by hand. With other_ms > 0, the tick that
 * many ms into the move is another behaviour's, and the clock of the move
 * starts again after it. Returns the time from the first tick of the move, or
 * of the restart, to the timeout; 0 for none within a minute. */
#define HUG_CODE_THEN_ALIGN  (3U)
#define APPROACH_MM          (100)
#define MOVE_TICKS_MAX       (6000U)

static uint32_t move_limit(uint32_t plan_ms)
{
    return HUG_MOVE_TIMES * ((plan_ms < HUG_MOVE_FLOOR_MS) ? HUG_MOVE_FLOOR_MS : plan_ms);
}

static uint32_t pivot_plan_ms(uint32_t angle_deci)
{
    return (angle_deci * 1000U * 100U) / (uint32_t)(HUG_PIVOT_MIN_DECI_S * HUG_PIVOT_PCT);
}

static uint32_t stall_ms(uint8_t then, int16_t u_head, uint32_t other_ms)
{
    static look_t look;
    hug_t hug;
    hug_in_t in = {0};
    hug_out_t out;
    uint32_t start_ms = 0U;
    uint32_t got = 0U;
    unsigned t;

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_PIVOT;
    hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    hug.then = then;
    hug.u_head = u_head;
    hug.app_mm = APPROACH_MM;
    in.now_ms = HUG_SETTLE_MS;
    check(hug_step(&hug, &look, &in, &out), "the settle ends in the move");
    start_ms = in.now_ms + TICK_MS;
    for (t = 0U; (t < MOVE_TICKS_MAX) && (0U == got); t++)
    {
        /* Encoder creep keeps the independent three-second motion timer
         * alive while the commanded move misses its own deadline. */
        in.left_um++;
        in.right_um++;
        in.now_ms += TICK_MS;
        in.b_other = (other_ms > 0U) && ((in.now_ms - start_ms) == other_ms);
        (void)hug_step(&hug, &look, &in, &out);
        if (in.b_other)
        {
            start_ms = in.now_ms + TICK_MS;
            other_ms = 0U;
        }
        else if (0U != out.b_timeout)
        {
            check(!hug_active(&hug), "a move timeout ends the hug");
            check((0 == out.speed_mm_s) && (0 == out.turn_deci_s), "a move timeout stands");
            got = in.now_ms - start_ms;
        }
        else
        {
            check(hug_active(&hug) && ((0 != out.speed_mm_s) || (0 != out.turn_deci_s)),
                  "a stalled move goes on until it times out");
        }
    }
    return got;
}

static void check_limit(uint32_t got_ms, uint32_t limit_ms, char const * p_what)
{
    (void)printf("move timeout %s: %u ms (limit %u ms)\n", p_what, (unsigned)got_ms,
                 (unsigned)limit_ms);
    check((got_ms > limit_ms) && (got_ms <= (limit_ms + TICK_MS)), p_what);
}

static void test_move_timeout(void)
{
    uint32_t const pivot_90 = move_limit(pivot_plan_ms(900U));
    uint32_t const pivot_10 = move_limit(pivot_plan_ms(100U));
    uint32_t const approach = move_limit(((uint32_t)APPROACH_MM * 1000U) /
                                            (uint32_t)HUG_SHORT_MM_S);

    check(pivot_10 == (HUG_MOVE_TIMES * HUG_MOVE_FLOOR_MS), "a short pivot gets the floor");
    check_limit(stall_ms(HUG_CODE_THEN_ALIGN, 900, 0U), pivot_90, "pivot of 90 degrees");
    check_limit(stall_ms(HUG_CODE_THEN_ALIGN, 100, 0U), pivot_10, "pivot of 10 degrees");
    check_limit(stall_ms(HUG_CODE_THEN_APPROACH, 0, 0U), approach, "approach drive");
    check_limit(stall_ms(HUG_CODE_THEN_ALIGN, 900, pivot_90 - 100U), pivot_90,
                "pivot restarted by another behaviour");
}

/* A pivot that crawls: both wheels turn at 3 mm/s, which the stuck check takes
 * for travel, so the first pivot of the contact look never ends. The move
 * timeout stops the car with HUG_TIMEOUT, and the stop holds. */
#define CRAWL_UM_TICK   (30)
#define HIT_MM          (200U)
#define FREE_PING_TICKS (5U)

static void test_hug_timeout(void)
{
    pilot_state_t st;
    pilot_meas_t m = fresh_meas(1000U);
    pilot_cmd_t cmd = {0};
    bool requested = false;
    bool resumed = false;
    pilot_init(&st);
    m.b_stationary = true;
    for (unsigned i = 0; i < 650U && !resumed; i++)
    {
        m.now_ms = m.stamp_ms = 1000U + i * TICK_MS;
        m.ping_seq = (uint8_t)((i / 6U) & 63U);
        m.b_ping_reply = i > 400U && cmd.b_ping_req;
        m.ping_servo_deci = m.b_ping_reply ? cmd.ping_servo_deci : 0;
        if (m.b_ping_reply)
        {
            m.ping_seq = cmd.ping_seq;
        }
        m.ping_echo = (m.ping_servo_deci >= 100) ? LOOK_ECHO_OPEN : LOOK_ECHO_VALID;
        m.ping_mm = (m.ping_echo == LOOK_ECHO_OPEN) ? 0U : 205U;
        pilot_step(&st, &m, &cmd);
        requested |= cmd.b_ping_req != 0U;
        if (requested && i <= 400U)
        {
            check(!cmd.speed_mm_s && !cmd.turn_deci_s, "a missing requested scan cannot authorize motion");
        }
        resumed |= requested && cmd.behaviour == PILOT_BEHAVIOUR_HUG && cmd.speed_mm_s > 0;
    }
    check(resumed, "fresh matching replies must resume a transiently silent survey without START");
}

static void test_slip_evidence(void)
{
    enum { PLATEAU, WRAP, REPLAY, INTERRUPTED, HEALTHY, TURNING, DISAGREEING, REPLY,
           ANGLED, REPEATED, STATIONARY };

    for (int scenario = PLATEAU; scenario <= STATIONARY; scenario++)
    {
        pilot_state_t pilot;
        pilot_cmd_t command = {0};
        pilot_meas_t meas = fresh_meas(1000U);
        int64_t const step_um = 5000;
        uint16_t const face_mm = HUG_STANDOFF_MM + 150U;
        bool b_survey = false;

        pilot_init(&pilot);
        if (WRAP == scenario)
        {
            meas.left_um = (int64_t)UINT32_MAX - (5 * step_um);
            meas.right_um = meas.left_um;
        }
        meas.b_line_right = (TURNING != scenario);
        meas.ping_echo = LOOK_ECHO_VALID;
        for (uint32_t sample = 0U; sample <= 22U; sample++)
        {
            meas.now_ms = ((WRAP == scenario) ? (UINT32_MAX - 500U) : 1000U) + (sample * 50U);
            if (STATIONARY == scenario && sample >= 18U)
            {
                break;
            }
            /* Keep each case's namesake as the only invalid evidence. Start
             * with a usable sample so a replay can exercise the stale path. */
            meas.stamp_ms = meas.now_ms;
            meas.ping_seq = (uint8_t)((REPEATED == scenario && sample > 0U) ? 1U : (sample + 1U));
            if (WRAP == scenario)
            {
                meas.ping_seq = (uint8_t)((60U + sample) & 0x3FU);
            }
            else if (REPLAY == scenario && sample == 21U)
            {
                meas.ping_seq = 21U;
            }
            meas.ping_mm = (HEALTHY == scenario) ? (uint16_t)(face_mm - (5U * sample)) : face_mm;
            meas.b_ping_reply = (REPLY == scenario);
            meas.ping_servo_deci = (ANGLED == scenario) ? 50 : 0;
            if (STATIONARY != scenario)
            {
                meas.left_um += step_um;
                meas.right_um += (DISAGREEING == scenario) ? (step_um / 2) : step_um;
            }
            pilot_step(&pilot, &meas, &command);
            check(command.speed_mm_s >= 0, "a distant echo authorized a blind reverse");
            if (PILOT_REASON_COURSE_RECOVERY == command.reason)
            {
                check((((PLATEAU == scenario || WRAP == scenario || INTERRUPTED == scenario) &&
                        sample == 21U) ||
                        (REPLAY == scenario && sample == 22U)) && command.turn_deci_s == 0 &&
                        command.speed_mm_s == 0,
                      "slip survey lacked 100 mm of corroborated straight-forward travel");
                b_survey = true;
                break;
            }
            if (REPEATED == scenario && sample >= 5U)
            {
                check(command.speed_mm_s == 0 && (PILOT_REASON_SONAR == command.reason),
                      "cached forward samples bypassed the stale-sonar stop");
            }
        }
        if (scenario <= INTERRUPTED)
        {
            check(b_survey, "the fresh stable range did not trigger a stationary survey");
        }
        if (INTERRUPTED == scenario)
        {
            meas.b_sonar_fault = true;
            meas.stamp_ms = meas.now_ms += 50U;
            meas.ping_seq++;
            pilot_step(&pilot, &meas, &command);
            check(PILOT_REASON_SONAR == command.reason && command.speed_mm_s == 0,
                  "sonar loss did not take priority over the slip survey");

            meas.b_sonar_fault = false;
            meas.stamp_ms = meas.now_ms += 50U;
            meas.ping_seq++;
            pilot_step(&pilot, &meas, &command);
            check(PILOT_REASON_COURSE_RECOVERY == command.reason && command.speed_mm_s == 0 &&
                      command.turn_deci_s == 0 && command.b_ping_req == 0U,
                  "sonar recovery took the wheels before the survey could settle");
        }
    }
}

static void test_stale_forward_ping(bool b_check_latch)
{
    pilot_state_t state;
    pilot_cmd_t command = {0};
    pilot_meas_t meas = fresh_meas(1000U);
    bool b_stopped = false;

    pilot_init(&state);
    meas.b_line_right = true;
    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_seq = 1U;
    pilot_step(&state, &meas, &command);
    check(command.speed_mm_s > 0, "fresh forward data should permit motion");

    /* The wheel source stays fresh, and the sonar task emits no fault. Its
     * last sequence is republished unchanged while the car approaches. */
    for (uint32_t elapsed = 10U; elapsed <= 4000U; elapsed += 10U)
    {
        meas.now_ms = 1000U + elapsed;
        meas.stamp_ms = meas.now_ms;
        meas.left_um += 1250;
        meas.right_um += 1250;
        pilot_step(&state, &meas, &command);
        b_stopped = b_stopped || ((PILOT_REASON_SONAR == command.reason ||
                                   PILOT_REASON_SONAR_LATCHED == command.reason) &&
                                  0 == command.speed_mm_s && 0 == command.turn_deci_s);
    }
    if (b_check_latch)
    {
        meas.now_ms += 10U;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq++;
        pilot_step(&state, &meas, &command);
        /* Past the rest of the latch the continuous floor crawls blind. */
        check(((PILOT_REASON_SONAR_LATCHED == command.reason) &&
               (0 == command.speed_mm_s) && (0 == command.turn_deci_s)) ||
              ((PILOT_REASON_BLIND_CRAWL == command.reason) &&
               (command.speed_mm_s > 0) && (command.speed_mm_s <= PILOT_BLIND_CRAWL_MM_S)),
              "a new forward ping did not leave a long dropout latched");
        pilot_release_sonar(&state);
        pilot_step(&state, &meas, &command);
        check((PILOT_REASON_NONE == command.reason) && (command.speed_mm_s > 0),
              "START did not release a recovered forward sonar");
    }
    else
    {
        check(b_stopped, "the car drove with a silent forward sonar for four seconds");
    }
}

static void test_forward_ping_recovery_window(uint32_t recovery_ms, bool b_latches)
{
    pilot_state_t state;
    pilot_cmd_t command;
    pilot_meas_t meas = fresh_meas(UINT32_MAX - 100U);

    pilot_init(&state);
    meas.b_bounded_course = true;
    meas.b_line_right = true;
    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_seq = 63U;
    pilot_step(&state, &meas, &command);
    meas.now_ms += 150U;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq = 0U;
    meas.left_um += 20000;
    meas.right_um += 20000;
    pilot_step(&state, &meas, &command);
    check(command.speed_mm_s > 0, "a wrapped forward ping sequence did not refresh the sonar");

    meas.now_ms += 249U;
    meas.stamp_ms = meas.now_ms;
    meas.left_um += 30000;
    meas.right_um += 30000;
    pilot_step(&state, &meas, &command);
    check(command.speed_mm_s > 0, "a recent forward ping should permit motion");

    ++meas.now_ms;
    meas.stamp_ms = meas.now_ms;
    meas.b_ping_reply = true;
    meas.ping_seq = 1U;
    pilot_step(&state, &meas, &command);
    check((PILOT_REASON_SONAR == command.reason) && (0 == command.speed_mm_s) &&
          (0 == command.turn_deci_s), "an unrelated ping reply hid the stale forward data");

    meas.now_ms += recovery_ms;
    meas.stamp_ms = meas.now_ms;
    meas.b_ping_reply = false;
    pilot_step(&state, &meas, &command);
    if (b_latches)
    {
        check((PILOT_REASON_SONAR_LATCHED == command.reason) && (0 == command.speed_mm_s),
              "forward data arriving after the recovery deadline released the stop");
    }
    else
    {
        check((PILOT_REASON_NONE == command.reason) && (command.speed_mm_s > 0),
              "forward data arriving on the recovery deadline did not release the stop");
    }
}

/* The pose of the hug and the contour turns a pivot by the measured witness
 * share when physics reports one, else by HUG_PIVOT_PCT. */
#define WITNESS_TICK_UM      (1000)
#define WITNESS_SHARE_PCT    (60U)
#define WITNESS_TRAVEL_UM    (2 * WITNESS_TICK_UM)

static void test_pivot_witness_pose(void)
{
    static note_car_t car;
    pilot_state_t pilot;
    pilot_contour_t contour = {0};
    pilot_meas_t meas = {0};
    pilot_cmd_t cmd;
    pilot_contour_plan_t offer = {
        .id = 1U, .issued_ms = 1000U,
        .steps = {{100U, 300}, {100U, -300}, {100U, -300}, {100U, 300}}, .count = 4U,
    };

    note_start(&car);
    note_tick(&car, -WITNESS_TICK_UM, WITNESS_TICK_UM, NOTE_PIVOT_MS);
    check((WITNESS_TRAVEL_UM * HUG_PIVOT_PCT / PERCENT) == car.hug.diff_um,
          "a hug pivot without a witness turns the pose by the fixed share");
    car.in.pivot_scale_pct = WITNESS_SHARE_PCT;
    note_tick(&car, -WITNESS_TICK_UM, WITNESS_TICK_UM, NOTE_PIVOT_MS);
    check((WITNESS_TRAVEL_UM * (HUG_PIVOT_PCT + (int32_t)WITNESS_SHARE_PCT) / PERCENT) == car.hug.diff_um,
          "a hug pivot with a witness turns the pose by the measured share");

    pilot_init(&pilot);
    meas.now_ms = 1000U;
    meas.stamp_ms = meas.now_ms;
    meas.b_stationary = true;
    meas.b_line_left = true;
    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_seq = 1U;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_WAIT == contour.status, "the contour fixture waits");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.left_um = -WITNESS_TICK_UM;
    meas.right_um = WITNESS_TICK_UM;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check((WITNESS_TRAVEL_UM * HUG_PIVOT_PCT / PERCENT) == contour.diff_um,
          "a contour pivot without a witness turns by the fixed share");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.left_um -= WITNESS_TICK_UM;
    meas.right_um += WITNESS_TICK_UM;
    meas.pivot_scale_pct = WITNESS_SHARE_PCT;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check((WITNESS_TRAVEL_UM * (HUG_PIVOT_PCT + (int32_t)WITNESS_SHARE_PCT) / PERCENT) == contour.diff_um,
          "a contour pivot with a witness turns by the measured share");
}

static void test_contour_executor(void)
{
    pilot_state_t pilot;
    pilot_contour_t contour = {0};
    pilot_meas_t meas = {0};
    pilot_cmd_t cmd;
    drive_state_t wheels = {100000, 100000};
    int32_t left = 0, right = 0;
    pilot_contour_plan_t offer = {
        .id = 1U, .issued_ms = 1000U,
        .steps = {{100U, 300}, {100U, -300}, {100U, -300}, {100U, 300}}, .count = 4U,
    };

    pilot_init(&pilot);
    meas.now_ms = 1000U;
    meas.stamp_ms = meas.now_ms;
    meas.b_stationary = true;
    meas.b_line_left = true;
    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_seq = 1U;
    meas.b_bounded_course = true;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_IDLE == contour.status && PILOT_BEHAVIOUR_HUG != cmd.behaviour,
          "the installed course refuses an otherwise valid contour");

    pilot_init(&pilot);
    meas.b_bounded_course = false;
    offer.left_um = 1U;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_IDLE == contour.status && PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour,
          "an offer for the wrong axle pose leaves the existing pilot in control");

    pilot_init(&pilot);
    offer.left_um = 0U;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_WAIT == contour.status && 0 == cmd.speed_mm_s,
          "a verified contour waits before moving");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(0 == cmd.speed_mm_s, "the pre-offer forward ping cannot start the contour");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq++;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_MOVE == contour.status && PILOT_BEHAVIOUR_HUG == cmd.behaviour &&
          cmd.speed_mm_s > 0 && cmd.turn_deci_s > 0 && 0U == cmd.b_ping_req,
          "fresh forward clearance admits the certified arc without pausing sonar liveness");

    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq++;
    meas.ping_echo = LOOK_ECHO_VALID;
    meas.ping_mm = HUG_GUARD_MM;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    pilot_drive(&wheels, &cmd, 10U, &left, &right);
    check(PILOT_CONTOUR_STOPPED == contour.status && PILOT_BEHAVIOUR_SAFETY == cmd.behaviour &&
          0 == left && 0 == right, "close forward evidence aborts the arc and brakes without slew");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq++;
    meas.ping_echo = LOOK_ECHO_OPEN;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_STOPPED == contour.status && 0 == cmd.speed_mm_s,
          "a cached route cannot restart after a stop");
}

static void contour_fixture_first_leg(pilot_state_t * p_pilot, pilot_contour_t * p_contour,
                                      pilot_meas_t * p_meas, pilot_contour_plan_t const * p_offer,
                                      pilot_cmd_t * p_cmd)
{
    pilot_init(p_pilot);
    pilot_step_with_contour(p_pilot, p_meas, p_contour, p_offer, p_cmd);
    for (unsigned i = 0U; i <= 40U; i++)
    {
        p_meas->now_ms += 10U;
        p_meas->stamp_ms = p_meas->now_ms;
        p_meas->left_um = (int64_t)i * 1000;
        p_meas->right_um = p_meas->left_um;
        p_meas->b_stationary = (0U == i);
        p_meas->b_line_left = (35U == i);
        p_meas->ping_seq = (uint8_t)(2U + (i / 5U));
        pilot_step_with_contour(p_pilot, p_meas, p_contour, p_offer, p_cmd);
    }
}

static void test_contour_terminal_line(void)
{
    pilot_state_t pilot;
    pilot_contour_t contour = {0};
    pilot_meas_t meas = {.b_stationary = true, .ping_echo = LOOK_ECHO_OPEN, .ping_seq = 1U};
    pilot_contour_plan_t const offer = {.id = 1U, .steps = {{40U, 0}}, .count = 1U};
    pilot_cmd_t cmd;

    contour_fixture_first_leg(&pilot, &contour, &meas, &offer, &cmd);
    check(PILOT_CONTOUR_LINE == contour.status, "the end of a measured route stops before handoff");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_LINE == contour.status && 0 == cmd.speed_mm_s,
          "a transient late tape hit does not survive arrival at the route end");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.b_stationary = true;
    meas.b_line_left = true;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_LINE == contour.status && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s,
          "a centered echo seen during the previous leg cannot admit terminal handoff");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq++;
    meas.b_line_left = false;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_LINE == contour.status && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s,
          "a verified route cannot add an unsurveyed terminal turn while waiting for tape");
    meas.now_ms += 10U;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq++;
    meas.b_line_left = true;
    pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
    check(PILOT_CONTOUR_DONE == contour.status, "stationary tape and fresh centered clearance admit handoff");
}

static void test_contour_wait_envelope(void)
{
    for (unsigned yaw = 0U; yaw < 2U; yaw++)
    {
        pilot_state_t pilot;
        pilot_contour_t contour = {0};
        pilot_meas_t meas = {.b_stationary = true, .ping_echo = LOOK_ECHO_OPEN, .ping_seq = 1U};
        pilot_contour_plan_t const offer = {.id = 1U, .steps = {{40U, 0}, {150U, 0}}, .count = 2U};
        pilot_cmd_t cmd;
        unsigned const ticks = (0U == yaw) ? 4U : 2U;
        int64_t const left_step = (0U == yaw) ? 7500 : -7500;

        contour_fixture_first_leg(&pilot, &contour, &meas, &offer, &cmd);
        check(PILOT_CONTOUR_WAIT == contour.status, "the next segment is waiting for a new return");
        for (unsigned i = 0U; i < ticks; i++)
        {
            meas.now_ms += 30U;
            meas.stamp_ms = meas.now_ms;
            meas.left_um += left_step;
            meas.right_um += 7500;
            pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
        }
        check(PILOT_CONTOUR_STOPPED == contour.status && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s,
              (0U == yaw) ? "coasting beyond the 25 mm position envelope aborts while waiting" :
                            "coasting beyond the 8 degree yaw envelope aborts while waiting");
        for (unsigned i = 0U; i < ticks; i++)
        {
            meas.now_ms += 10U;
            meas.stamp_ms = meas.now_ms;
            meas.left_um -= left_step;
            meas.right_um -= 7500;
            meas.ping_seq++;
            pilot_step_with_contour(&pilot, &meas, &contour, &offer, &cmd);
        }
        check(PILOT_CONTOUR_STOPPED == contour.status && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s,
              "returning in bounds with a fresh echo cannot restart a failed contour");
    }
}

static void test_contour_guard_stops(void)
{
    pilot_reason_t const reasons[] = {PILOT_REASON_SONAR, PILOT_REASON_COURSE_CLEARANCE,
        PILOT_REASON_COURSE_CLEARANCE, PILOT_REASON_HELD, PILOT_REASON_ESTOP};

    for (unsigned fault = 0U; fault < sizeof(reasons) / sizeof(reasons[0]); fault++)
    {
        pilot_state_t pilot;
        pilot_contour_t contour = {0};
        pilot_meas_t meas = {.b_stationary = true, .ping_echo = LOOK_ECHO_OPEN, .ping_seq = 1U};
        pilot_contour_plan_t offer = {.id = 1U, .steps = {{300U, 0}}, .count = 1U};
        pilot_contour_plan_t const * p_offer = &offer;
        pilot_cmd_t cmd;
        drive_state_t wheels = {100000, 100000};
        int32_t left = 0, right = 0;

        pilot_init(&pilot);
        pilot_step_with_contour(&pilot, &meas, &contour, p_offer, &cmd);
        meas.now_ms = 10U;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq = 2U;
        pilot_step_with_contour(&pilot, &meas, &contour, p_offer, &cmd);
        check(cmd.speed_mm_s > 0, "the contour is travelling before each stop condition");

        if (0U == fault)
        {
            for (unsigned i = 1U; i <= (PILOT_FORWARD_PING_STALE_MS / 10U); i++)
            {
                meas.now_ms += 10U;
                meas.stamp_ms = meas.now_ms;
                meas.left_um = (int64_t)i * 1000;
                meas.right_um = meas.left_um;
                meas.b_stationary = false;
                pilot_step_with_contour(&pilot, &meas, &contour, p_offer, &cmd);
            }
        }
        else
        {
            if (1U == fault)
            {
                p_offer = NULL;
            }
            else if (2U == fault)
            {
                offer.id++;
            }
            else if (3U == fault)
            {
                meas.b_held = true;
            }
            else
            {
                pilot_estop(&pilot);
            }
            meas.now_ms += 10U;
            meas.stamp_ms = meas.now_ms;
            meas.left_um = 1000;
            meas.right_um = meas.left_um;
            meas.b_stationary = false;
            pilot_step_with_contour(&pilot, &meas, &contour, p_offer, &cmd);
        }
        pilot_drive(&wheels, &cmd, 10U, &left, &right);
        check(PILOT_CONTOUR_STOPPED == contour.status && reasons[fault] == cmd.reason &&
              0 == left && 0 == right, "stale, revoked, replaced, held and emergency routes brake immediately");
    }
}
int main(void)
{
    test_direct();
    test_slow_wheel_seam();
    test_retry_label_seam();
    test_held();
    test_hug_trigger();
    test_sonar_fault();
    test_stale_forward_ping(false);
    test_stale_forward_ping(true);
    test_forward_ping_recovery_window(1000U, false);
    test_forward_ping_recovery_window(1001U, true);
    test_breadth_kept();
    test_leg_pivot_coast();
    test_missed_rear_leg_reply(LOOK_ECHO_OPEN);
    test_missed_rear_leg_reply(LOOK_ECHO_TOO_CLOSE);
    test_missed_rear_leg_reply(LOOK_ECHO_NONE);
    test_contact_side_uses_usable_rays();
    test_aimed_settle();
    test_scan_requests_the_next_bearing();
    test_side_look_uses_two_headings(1);
    test_side_look_uses_two_headings(-1);
    for (corner_check_t scenario = CORNER_CLEAR; scenario <= CORNER_DRIVE_HELD; scenario++)
    {
        test_measured_corner_checks_next_leg(scenario);
    }
    test_note();
    test_move_timeout();
    test_hug_timeout();
    test_slip_evidence();
    test_pivot_witness_pose();
    test_contour_executor();
    test_contour_terminal_line();
    test_contour_wait_envelope();
    test_contour_guard_stops();
    return EXIT_SUCCESS;
}
