/* Line behaviour of the pilot path: a barcode is driven straight through, and
 * the line-lost bookkeeping counts a lost line only while the follower has the
 * wheels. */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pilot/pilot.h"
#include "navigation/obstacle_ping.h"
#include "platform/chassis.h"
#include "platform/units.h"


#define TICK_MS           (10U)
#define MS_PER_S_I        (1000)
#define BLIND_RUN_TICKS   (6000U)
#define BARCODE_TICKS     (25U)
#define REVERSE_TOL_MM    (10)
static void check(bool b_ok, char const * p_what, uint32_t seed)
{
    if (!b_ok)
    {
        (void)printf("FAIL: %s (seed %u)\n", p_what, (unsigned)seed);
        exit(EXIT_FAILURE);
    }
}

static line_follow_status_t status_of(pilot_state_t const * p_st)
{
    return (line_follow_status_t)p_st->follow.status;
}

/* Direct line fixtures have an open course with the normal sonar cadence. */
static void open_course(pilot_meas_t * p_meas)
{
    p_meas->ping_echo = LOOK_ECHO_OPEN;
    p_meas->ping_seq = (uint8_t)((p_meas->now_ms / 60U) & OBSTACLE_PING_SEQ_MK);
}

/* A hug that ends while the follower was lost: the follower starts afresh, and
 * that is no line-found. A stop is reported once. */
static void test_watch_direct(void)
{
    line_watch_t w = {0U, 0U};

    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_FOLLOWING), "idle", 0U);
    check(0U == w.b_lost, "following is not lost", 0U);
    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_SEARCHING), "search", 0U);
    check(0U != w.b_lost, "searching is lost", 0U);
    check(0U == line_watch_step(&w, false, false, LINE_FOLLOW_FOLLOWING), "safety tick", 0U);
    check(0U != w.b_lost, "a safety stop does not end a lost line", 0U);
    check(0U != (LINE_WATCH_QUIET & line_watch_step(&w, false, true, LINE_FOLLOW_SEARCHING)),
          "hug ends the lost state quietly", 0U);
    check(0U == w.b_lost, "hug clears the lost state", 0U);
    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_FOLLOWING), "after the hug", 0U);
    check(0U == w.b_lost, "no lost line after the hug", 0U);
    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_SEARCHING), "search again", 0U);
    check(0U != (LINE_WATCH_STOP & line_watch_step(&w, true, false, LINE_FOLLOW_STOPPED)),
          "first stop is reported", 0U);
    check(0U == (LINE_WATCH_STOP & line_watch_step(&w, true, false, LINE_FOLLOW_STOPPED)),
          "stop is reported once", 0U);
}

/* No line under the sensors, a barcode edge seen: straight at cruise, and the
 * search state stays. A stopped follower stays stopped. */
static void test_barcode_direct(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = {0};

    m.now_ms = 1000U;
    m.stamp_ms = 1000U;
    m.b_barcode = true;
    pilot_init(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "barcode: follower drives", 0U);
    check(cmd.speed_mm_s == (LINE_CRUISE_UM_S / 1000), "barcode: cruise speed", 0U);
    check(0 == cmd.turn_deci_s, "barcode: straight", 0U);
    check(st.follow.last_ms == m.now_ms && st.follow.anchor_um == 0,
          "barcode: next correction starts from the marker sample", 0U);

    st.follow.status = (uint8_t)LINE_FOLLOW_STOPPED;
    pilot_step(&st, &m, &cmd);
    check((0 == cmd.speed_mm_s) && (PILOT_REASON_LINE_LOST_STOP == cmd.reason),
          "barcode: a stopped follower stays stopped", 0U);
}

static void test_barcode_reader_crosses_unmarked_bend(bool left)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = {.b_barcode = true, .b_line_left = left, .b_line_right = !left};
    pilot_init(&st);
    for (unsigned tick = 0U; tick < 4U; tick++)
    {
        m.now_ms = 1000U + tick * TICK_MS;
        m.stamp_ms = m.now_ms;
        m.left_um = (int64_t)tick * 1250;
        m.right_um = m.left_um;
        pilot_step(&st, &m, &cmd);
    }
    check(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE && cmd.speed_mm_s > 0,
          "side reader crosses bend: still following", 0U);
    check(left ? cmd.turn_deci_s > 0 : cmd.turn_deci_s < 0,
          "side reader crosses bend: steer toward the line detector on tape", 0U);
}

/* Accepted elements during a verified marker renew the travel anchor. After it the
 * follower has the whole LINE_GAP_MM before it searches. */
static void test_barcode_straight_keeps_the_gap(void)
{
    enum { STEP_MS = 10, STRAIGHT_STEPS = 25, MAX_STEPS = 400 };
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = {.b_bounded_course = true};
    int32_t travel_mm = 0;
    int32_t resume_mm;
    int i;

    pilot_init(&st);
    m.now_ms = 1000U;
    m.stamp_ms = 1000U;
    m.b_line_left = true;
    m.b_line_right = true;
    open_course(&m);
    pilot_step(&st, &m, &cmd);
    m.b_line_left = false;
    m.b_line_right = false;
    m.b_barcode = true;
    m.b_marker_progress = true;
    for (i = 0; i < STRAIGHT_STEPS; i++)
    {
        m.now_ms += STEP_MS;
        m.stamp_ms = m.now_ms;
        travel_mm++;
        m.left_um = (int64_t)travel_mm * 1000;
        m.right_um = m.left_um;
        open_course(&m);
        pilot_step(&st, &m, &cmd);
    }
    m.b_barcode = false;
    m.b_marker_progress = false;
    resume_mm = travel_mm;
    for (i = 0; (i < MAX_STEPS) && ((uint8_t)LINE_FOLLOW_SEARCHING != st.follow.status); i++)
    {
        m.now_ms += STEP_MS;
        m.stamp_ms = m.now_ms;
        travel_mm++;
        m.left_um = (int64_t)travel_mm * 1000;
        m.right_um = m.left_um;
        open_course(&m);
        pilot_step(&st, &m, &cmd);
    }
    check((uint8_t)LINE_FOLLOW_SEARCHING == st.follow.status, "barcode gap: searches", 0U);
    check((travel_mm - resume_mm) >= (LINE_GAP_MM - 2), "barcode gap: full bridge after the straight", 0U);
}

/* The pilot on an ideal car: each command moves the wheels exactly, and
 * b_block holds the car still while it is told to reverse. */
typedef struct
{
    pilot_state_t st;
    pilot_meas_t  m;
    pilot_cmd_t   cmd;
    int64_t       travel_um;
    bool          b_block;
    bool          b_jam_left;
    int64_t       left_um;
    int64_t       right_um;
} ideal_t;

static void ideal_init(ideal_t * p_i)
{
    *p_i = (ideal_t){0};
    p_i->m.b_bounded_course = true;
    pilot_init(&p_i->st);
    p_i->m.now_ms = 1000U;
    p_i->m.stamp_ms = 1000U;
    p_i->m.b_line_left = true;
    p_i->m.b_line_right = true;
    open_course(&p_i->m);
    pilot_step(&p_i->st, &p_i->m, &p_i->cmd);
    p_i->m.b_line_left = false;
    p_i->m.b_line_right = false;
}

static bool ideal_reversing(ideal_t const * p_i)
{
    return (PILOT_BEHAVIOUR_FOLLOW_LINE == p_i->cmd.behaviour) &&
           (p_i->cmd.speed_mm_s < 0);
}

static void ideal_step(ideal_t * p_i)
{
    p_i->m.now_ms += TICK_MS;
    p_i->m.stamp_ms = p_i->m.now_ms;
    open_course(&p_i->m);
    pilot_step(&p_i->st, &p_i->m, &p_i->cmd);
    if (!(p_i->b_block && ideal_reversing(p_i)))
    {
        int64_t const dist_um = ((int64_t)p_i->cmd.speed_mm_s * UM_PER_MM *
                                 TICK_MS) / MS_PER_S_I;
        int64_t const turn_um = ((int64_t)p_i->cmd.turn_deci_s *
                                 CHASSIS_WHEEL_TRACK_MM * UM_PER_MM * TICK_MS) /
                                ((int64_t)DECI_PER_RAD * MS_PER_S_I);

        p_i->left_um += p_i->b_jam_left ? 0 : (dist_um - (turn_um / 2));
        p_i->right_um += dist_um + (turn_um / 2);
    }
    p_i->travel_um = (p_i->left_um + p_i->right_um) / 2;
    p_i->m.left_um = p_i->left_um;
    p_i->m.right_um = p_i->right_um;
}

/* The car is against something behind it: every reverse of the search stands
 * still. The pilot must not back off into it, must not hand it to the hug,
 * and the follower must give up after its retries. */
static void test_blocked_reverse_stops_after_the_retries(void)
{
    ideal_t i;
    unsigned tick;
    unsigned reversing = 0U;

    ideal_init(&i);
    i.b_block = true;
    for (tick = 0U;
         (tick < BLIND_RUN_TICKS) && (LINE_FOLLOW_STOPPED != status_of(&i.st));
         tick++)
    {
        ideal_step(&i);
        reversing += ideal_reversing(&i) ? 1U : 0U;
        check(PILOT_REASON_STUCK != i.cmd.reason,
              "blocked reverse: a back-off", 0U);
        check(PILOT_BEHAVIOUR_HUG != i.cmd.behaviour,
              "blocked reverse: the hug has the wheels", 0U);
        check(0U == i.st.b_encoder, "blocked reverse: an encoder stop", 0U);
    }
    check(0U != reversing, "blocked reverse: the car never reversed", 0U);
    check(LINE_FOLLOW_STOPPED == status_of(&i.st),
          "blocked reverse: no stop after the retries", 0U);
    check(LINE_RETRY_COUNT == i.st.follow.retry,
          "blocked reverse: not all retries used", 0U);
    ideal_step(&i);
    check((0 == i.cmd.speed_mm_s) &&
              (PILOT_REASON_LINE_LOST_STOP == i.cmd.reason),
          "blocked reverse: the stop is not the line-lost stop", 0U);
}

/* The left wheel jams as the first reverse of the search begins. The pilot does
 * not back off in the reverse, the follower ends it as a failed retry, and the
 * spin that follows shows one wheel still: ENCODER. */
static void test_one_wheel_jam_after_a_reverse_latches_encoder(void)
{
    ideal_t i;
    unsigned tick;
    unsigned reverse_ticks = 0U;
    bool b_jam = false;

    ideal_init(&i);
    for (tick = 0U; (tick < BLIND_RUN_TICKS) && (0U == i.st.b_encoder); tick++)
    {
        ideal_step(&i);
        if (ideal_reversing(&i))
        {
            b_jam = true;
            i.b_jam_left = true;
            reverse_ticks++;
            check(PILOT_REASON_STUCK != i.cmd.reason,
                  "one wheel jam: a back-off in the reverse", 0U);
        }
    }
    check(b_jam, "one wheel jam: the car never reversed", 0U);
    check(0U != i.st.b_encoder, "one wheel jam: no encoder stop", 0U);
    check(reverse_ticks <= ((LINE_REVERSE_STALL_MS / TICK_MS) + 2U),
          "one wheel jam: the reverse did not end in time", 0U);
}

/* A barcode edge while the car reverses in a search: the reverse goes on, and
 * it covers its distance. */
static void test_barcode_during_the_reverse(void)
{
    ideal_t i;
    unsigned tick;
    unsigned barcode_left = 0U;
    int64_t from_um = 0;
    int64_t back_um = 0;
    bool b_seen = false;
    bool b_done = false;

    ideal_init(&i);
    for (tick = 0U; (tick < BLIND_RUN_TICKS) && !b_done; tick++)
    {
        bool const b_was_reversing = ideal_reversing(&i);

        if (b_was_reversing && !b_seen)
        {
            b_seen = true;
            from_um = i.travel_um;
            barcode_left = BARCODE_TICKS;
        }
        i.m.b_barcode = (0U != barcode_left);
        barcode_left -= (0U != barcode_left) ? 1U : 0U;
        ideal_step(&i);
        if (b_seen && !b_done)
        {
            back_um = from_um - i.travel_um;
            if (i.m.b_barcode)
            {
                check(i.cmd.speed_mm_s < 0,
                      "barcode in the reverse: the car drove forward", 0U);
            }
            b_done = !ideal_reversing(&i) && (back_um > 0);
        }
    }
    check(b_seen, "barcode in the reverse: no reverse", 0U);
    check(b_done, "barcode in the reverse: the reverse did not end", 0U);
    check(back_um >= ((int64_t)(LINE_REVERSE_MM - REVERSE_TOL_MM) * UM_PER_MM),
          "barcode in the reverse: it did not cover its distance", 0U);
}

static void course_hold_tick(pilot_state_t * p_pilot, pilot_meas_t * p_meas, pilot_cmd_t * p_cmd)
{
    p_meas->now_ms += TICK_MS;
    p_meas->stamp_ms = p_meas->now_ms;
    pilot_step(p_pilot, p_meas, p_cmd);
}

/* Use actual request/response and free-running sequence namespaces. More than
 * 45 seconds with the same obstruction crosses both device timer wraps. */
static void course_blocked_episode(pilot_state_t * p_pilot, pilot_meas_t * p_meas,
                                   pilot_cmd_t * p_cmd, uint8_t * p_center_seq, unsigned ticks)
{
    int16_t const expected_angles[] = {-OBSTACLE_SERVO_LIMIT_DECI, 0, OBSTACLE_SERVO_LIMIT_DECI,
        OBSTACLE_SERVO_LIMIT_DECI / 2, 0, -OBSTACLE_SERVO_LIMIT_DECI / 2};
    unsigned completions = 0U;
    unsigned requests = 0U;
    bool b_angles_match = true;
    drive_state_t drive = {120000, 120000};

    for (unsigned tick = 0U; tick < ticks; tick++)
    {
        if (0U != p_cmd->b_ping_req)
        {
            b_angles_match &= p_cmd->ping_servo_deci == expected_angles[requests %
                (sizeof(expected_angles) / sizeof(expected_angles[0]))];
            p_meas->b_ping_reply = true;
            p_meas->ping_servo_deci = p_cmd->ping_servo_deci;
            p_meas->ping_seq = p_cmd->ping_seq;
            requests++;
        }
        else if (0U == (tick % 6U))
        {
            *p_center_seq = (uint8_t)((*p_center_seq + 1U) & OBSTACLE_PING_SEQ_MK);
            p_meas->b_ping_reply = false;
            p_meas->ping_servo_deci = 0;
            p_meas->ping_seq = *p_center_seq;
        }
        p_meas->ping_echo = LOOK_ECHO_VALID;
        p_meas->ping_mm = (uint16_t)(77U + (tick % 24U));
        p_meas->b_held = tick >= 1000U && tick < 1080U;
        p_meas->b_sonar_fault = tick >= 2100U && tick < 2130U;
        if (1020U == tick)
        {
            pilot_clear_latches(p_pilot);
        }
        course_hold_tick(p_pilot, p_meas, p_cmd);
        check(0 == p_cmd->speed_mm_s && 0 == p_cmd->turn_deci_s,
              "a stopped course moved against a 77–100 mm return", 0U);
        int32_t left;
        int32_t right;
        pilot_drive(&drive, p_cmd, TICK_MS, &left, &right);
        check(0 == left && 0 == right, "the parked course did not brake the wheel outputs", 0U);
        completions += (0U != p_cmd->course_scan_samples) ? 1U : 0U;
    }
    (void)printf("blocked for %u ms: %u requested rays, %u completed sweeps\n",
                 ticks * TICK_MS, requests, completions);
    check(6U == requests && 2U == completions && b_angles_match,
          "a blocked episode must request exactly two alternating three-angle sweeps", 0U);
    check(0U == p_cmd->b_ping_req, "the free-running center stream must own the parked servo", 0U);
}

static void course_center_ping(pilot_state_t * p_pilot, pilot_meas_t * p_meas, pilot_cmd_t * p_cmd,
                               uint8_t * p_center_seq, uint8_t echo, uint16_t range)
{
    *p_center_seq = (uint8_t)((*p_center_seq + 1U) & OBSTACLE_PING_SEQ_MK);
    p_meas->ping_seq = *p_center_seq;
    p_meas->ping_servo_deci = 0;
    p_meas->b_ping_reply = false;
    p_meas->ping_echo = echo;
    p_meas->ping_mm = range;
    course_hold_tick(p_pilot, p_meas, p_cmd);
}

static void test_bounded_course_contact_boundary(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t meas = {.b_line_left = true, .b_line_right = true, .b_bounded_course = true};
    uint8_t sequence = 60U;

    pilot_init(&pilot);
    for (unsigned sample = 0U; sample < 4U; sample++)
    {
        course_center_ping(&pilot, &meas, &cmd, &sequence, LOOK_ECHO_VALID,
                           (sample < 2U) ? 250U : 101U);
        check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour && cmd.speed_mm_s > 0,
              "returns outside 100 mm must not park the course", 0U);
        check(0U == cmd.contact_mm, "outside cutoff must not be reported as contact", 0U);
    }

    for (unsigned sample = 0U; sample < 4U; sample++)
    {
        meas.ping_echo = LOOK_ECHO_VALID;
        meas.ping_mm = 95U;
        meas.ping_servo_deci = OBSTACLE_SERVO_LIMIT_DECI;
        meas.b_ping_reply = (sample < 2U);
        meas.ping_seq = (uint8_t)((meas.ping_seq + 1U) & OBSTACLE_PING_SEQ_MK);
        course_hold_tick(&pilot, &meas, &cmd);
        check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour && cmd.speed_mm_s > 0,
              "an uncentered measurement must not park the course", 0U);
    }
    course_center_ping(&pilot, &meas, &cmd, &sequence, LOOK_ECHO_VALID, 0U);
    check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour && cmd.speed_mm_s > 0,
          "a malformed zero valid range must not park the course", 0U);
    meas.ping_mm = 100U;
    course_hold_tick(&pilot, &meas, &cmd);
    check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour && cmd.speed_mm_s > 0,
          "a changed reading with a cached sequence must not park the course", 0U);
    course_center_ping(&pilot, &meas, &cmd, &sequence, LOOK_ECHO_VALID, 100U);
    check(PILOT_REASON_COURSE_RECOVERY == cmd.reason && 0 == cmd.speed_mm_s &&
              0 == cmd.turn_deci_s && 100U == cmd.contact_mm,
          "the first fresh centered 100 mm reading must brake and report contact", 0U);

    pilot_init(&pilot);
    meas.b_commanded_turn = true;
    course_center_ping(&pilot, &meas, &cmd, &sequence, LOOK_ECHO_VALID, 101U);
    check(PILOT_BEHAVIOUR_NONE == cmd.behaviour, "101 mm must not abort a commanded maneuver", 0U);
    course_center_ping(&pilot, &meas, &cmd, &sequence, LOOK_ECHO_VALID, 100U);
    check(PILOT_REASON_COURSE_RECOVERY == cmd.reason && 100U == cmd.contact_mm,
          "a fresh centered 100 mm reading must abort a commanded maneuver", 0U);

    pilot_init(&pilot);
    meas.b_commanded_turn = false;
    course_center_ping(&pilot, &meas, &cmd, &sequence, LOOK_ECHO_TOO_CLOSE, 20U);
    check(PILOT_REASON_COURSE_RECOVERY == cmd.reason && 0 == cmd.speed_mm_s,
          "the first fresh blind-zone contact must brake", 0U);
}

static void test_bounded_course_scan_budget(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd = {0};
    pilot_meas_t meas = {.now_ms = UINT32_MAX - 1000U, .b_line_left = true, .b_line_right = true,
                         .b_stationary = true, .b_bounded_course = true};
    uint8_t center_seq = 59U;

    pilot_init(&pilot);
    for (unsigned sample = 0U; sample < HUG_TRIGGER_HITS; sample++)
    {
        course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 100U);
    }
    check(PILOT_REASON_COURSE_RECOVERY == cmd.reason, "the centered obstacle starts course recovery", 0U);
    course_blocked_episode(&pilot, &meas, &cmd, &center_seq, 4500U);
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_OPEN, 0U);
    for (unsigned tick = 0U; tick < 26U; tick++)
    {
        course_hold_tick(&pilot, &meas, &cmd);
        check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s && 0U == cmd.b_ping_req,
              "a repeated or stale centered ping cannot advance the car or rearm the scan", 0U);
    }
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_NONE, 0U);
    meas.b_held = true;
    for (unsigned sample = 0U; sample < 3U; sample++)
    {
        course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_OPEN, 0U);
        check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s, "clear sonar cannot bypass a motion hold", 0U);
    }
    meas.b_held = false;
    for (unsigned sample = 0U; sample < 3U; sample++)
    {
        course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 100U);
        check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s, "100 mm cannot release a bounded course", 0U);
    }
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 101U);
    check(0 == cmd.speed_mm_s, "one valid clear ping is insufficient", 0U);
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_NONE, 0U);
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_OPEN, 0U);
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 101U);
    check(0 == cmd.speed_mm_s, "invalid ping must reset the consecutive-clear count", 0U);

    meas.b_ping_reply = true;
    meas.ping_servo_deci = OBSTACLE_SERVO_LIMIT_DECI;
    meas.ping_seq = (uint8_t)((center_seq + 1U) & OBSTACLE_PING_SEQ_MK);
    course_hold_tick(&pilot, &meas, &cmd);
    check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s, "a requested side ray cannot release the course", 0U);

    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_OPEN, 0U);
    check(cmd.speed_mm_s > 0 && PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour,
          "three consecutive distinct usable centered pings release the course", 0U);

    meas.left_um = 50000;
    meas.right_um = 50000;
    for (unsigned sample = 0U; sample < HUG_TRIGGER_HITS; sample++)
    {
        course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 100U);
    }
    check(PILOT_REASON_COURSE_RECOVERY == cmd.reason, "a new obstacle starts a new course recovery", 0U);
    course_blocked_episode(&pilot, &meas, &cmd, &center_seq, 500U);
}

/* Neither optic sees tape. A perpendicular face ends on the requested side
 * and continues beyond the opposite outer beam. The encoders start at two
 * different nonzero counts, as on a car which has already followed a route. */
static void test_stationary_course_edge(int side, bool move_encoder, bool far_wall, uint16_t face_mm)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd = {0};
    pilot_meas_t meas = {.b_stationary = true, .b_bounded_course = true,
                         .left_um = 83000, .right_um = 109000};
    uint8_t center_seq = 0U;
    unsigned requests = 0U;
    unsigned completions = 0U;
    int16_t const coarse[] = {-200, 0, 200};
    int16_t const symmetric[] = {100, 0, -100};
    int const expected_side = face_mm > 100U ? 0 : side;

    pilot_init(&pilot);
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 90U);
    for (unsigned tick = 0U; tick < 300U; tick++)
    {
        if (0U != cmd.b_ping_req)
        {
            check(requests < 6U, "a parked survey exceeded its request budget", 0U);
            int const expected = (requests < 3U) ? coarse[requests] :
                ((0 == expected_side || move_encoder) ? symmetric[requests - 3U] :
                 expected_side * 50 * (int)(requests - 2U));
            check(expected == cmd.ping_servo_deci, "the next probe must follow the observed edge", 0U);
            bool const outside = (0 != side) && (side * cmd.ping_servo_deci >= 150);
            meas.b_ping_reply = true;
            meas.ping_servo_deci = cmd.ping_servo_deci;
            meas.ping_seq = cmd.ping_seq;
            meas.ping_echo = (outside && !far_wall) ? LOOK_ECHO_OPEN : LOOK_ECHO_VALID;
            meas.ping_mm = outside ? (far_wall ? 280U : 0U) : ((0 == cmd.ping_servo_deci) ? face_mm : (uint16_t)(face_mm + 6U));
            requests++;
            course_hold_tick(&pilot, &meas, &cmd);
        }
        else
        {
            course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 90U);
        }
        check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s, "a measured edge cannot authorize wheel motion", 0U);
        if (0U != cmd.course_scan_samples)
        {
            completions++;
            unsigned const expected_rays = (1U == completions || move_encoder) ? 3U : 6U;
            check(pilot.avoidance.look.count == expected_rays && cmd.course_scan_samples == expected_rays + 1U,
                  "the profile must retain exactly the measured rays from this stationary pose", 0U);
            if (1U == completions)
            {
                check(pilot.avoidance.course_side == expected_side, "observed geometry must support the provisional side", 0U);
            }
            if (1U == completions && move_encoder)
            {
                /* The speed observation still rounds to zero even though one encoder moved. */
                meas.right_um += 2000;
            }
        }
    }
    check(2U == completions && 6U == requests, "both bounded surveys must complete without tape", 0U);
    meas.b_held = true;
    course_hold_tick(&pilot, &meas, &cmd);
    check(0 == pilot.avoidance.course_side && 0U == pilot.avoidance.look.count && !pilot.avoidance.b_course_anchor,
          "a safety hold must retire the contact survey", 0U);
}

static void test_course_reply_deadline(uint32_t elapsed_ms)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd = {0};
    pilot_meas_t meas = {.b_stationary = true, .b_bounded_course = true, .now_ms = UINT32_MAX - 400U};
    uint8_t center_seq = 0U;
    uint8_t late_seq = 0U;
    unsigned served = 0U;
    bool const fresh = elapsed_ms < 300U;

    pilot_init(&pilot);
    course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 90U);
    for (unsigned ticks = 0U; ticks < 200U; ticks++)
    {
        if (0U == cmd.b_ping_req)
        {
            course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 90U);
        }
        else
        {
            if (3U == served)
            {
                check(cmd.ping_servo_deci == (fresh ? 50 : 100), "a timed-out edge must not direct the next scan", 0U);
                meas.ping_seq = late_seq;
                meas.ping_servo_deci = 200;
                meas.ping_echo = LOOK_ECHO_OPEN;
                meas.ping_mm = 0U;
                meas.b_ping_reply = true;
                course_hold_tick(&pilot, &meas, &cmd);
                check(0U != cmd.b_ping_req && cmd.ping_seq != late_seq && pilot.avoidance.look.count == (fresh ? 3U : 2U),
                      "the old OPEN cannot answer the next scan", 0U);
                return;
            }
            meas.b_ping_reply = true;
            meas.ping_servo_deci = cmd.ping_servo_deci;
            meas.ping_seq = cmd.ping_seq;
            meas.ping_echo = (served == 2U) ? LOOK_ECHO_OPEN : LOOK_ECHO_VALID;
            meas.ping_mm = (served == 0U) ? 96U : ((served == 1U) ? 90U : 0U);
            if (2U == served)
            {
                late_seq = cmd.ping_seq;
                meas.now_ms += elapsed_ms - TICK_MS;
            }
            course_hold_tick(&pilot, &meas, &cmd);
            served++;
            if (3U == served)
            {
                check(cmd.course_scan_samples == (fresh ? 4U : 3U) && pilot.avoidance.course_side == (fresh ? 1 : 0),
                      "a matching OPEN consumed at or after its deadline must not qualify as an edge", 0U);
            }
        }
        check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s, "the reply timing cannot authorize wheel motion", 0U);
    }
    check(false, "the following survey was never requested", 0U);
}

static void test_interrupted_course_scan_budget(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd = {0};
    pilot_meas_t meas = {.b_line_left = true, .b_line_right = true,
                         .b_stationary = true, .b_bounded_course = true};
    uint8_t center_seq = 0U;
    unsigned requests = 0U;
    unsigned hold_until = 0U;
    bool b_angles_match = true;

    pilot_init(&pilot);
    for (unsigned sample = 0U; sample < HUG_TRIGGER_HITS; sample++)
    {
        course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 100U);
    }
    for (unsigned tick = 0U; tick < 1500U; tick++)
    {
        if (0U != cmd.b_ping_req)
        {
            int16_t const angle = (0U == requests) ? -OBSTACLE_SERVO_LIMIT_DECI : (OBSTACLE_SERVO_LIMIT_DECI / 2);

            b_angles_match &= angle == cmd.ping_servo_deci;
            requests++;
            hold_until = tick + 20U;
        }
        meas.b_held = tick < hold_until;
        if (0U == (tick % 6U))
        {
            course_center_ping(&pilot, &meas, &cmd, &center_seq, LOOK_ECHO_VALID, 100U);
        }
        else
        {
            course_hold_tick(&pilot, &meas, &cmd);
        }
        check(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s && 0U == cmd.course_scan_samples,
              "an interrupted sweep leaves the wheels parked and publishes no completion", 0U);
    }
    (void)printf("interrupted sweeps: %u requested rays before the servo stays centered\n", requests);
    check(2U == requests && b_angles_match, "interrupted sweeps must consume their budget", 0U);
}

static void test_tape_is_a_direction_guide(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = {0};
    pilot_init(&st);
    for (unsigned tick = 0; tick <= 1200; tick++)
    {
        m.now_ms = 1000U + tick * 10U;
        m.stamp_ms = m.now_ms;
        m.left_um = (int64_t)tick * 1400;
        m.right_um = m.left_um;
        m.b_line_left = tick >= 400 && tick < 405;
        open_course(&m);
        pilot_step(&st, &m, &cmd);
        check(cmd.speed_mm_s > 0 && cmd.reason == PILOT_REASON_NONE, "clear floor carries forward", tick);
        if (tick == 404) check(cmd.turn_deci_s > 0, "fresh tape guides the heading", tick);
        if (tick == 250) check(cmd.turn_deci_s == 0, "unmarked start holds its bearing", tick);
    }
    m.now_ms += 10U;
    m.stamp_ms = m.now_ms;
    m.b_held = true;
    pilot_step(&st, &m, &cmd);
    check(!cmd.speed_mm_s && !cmd.turn_deci_s, "motor hold owns bare-floor travel", 0U);
}

int main(int argc, char * argv[])
{
    if (2 == argc && 0 == strcmp(argv[1], "course-reply-deadline"))
    {
        test_course_reply_deadline(299U);
        test_course_reply_deadline(300U);
        test_course_reply_deadline(301U);
        return EXIT_SUCCESS;
    }
    if (2 == argc && 0 == strcmp(argv[1], "stationary-course-edge"))
    {
        test_stationary_course_edge(1, false, false, 90U);
        test_stationary_course_edge(-1, false, false, 90U);
        test_stationary_course_edge(-1, false, true, 90U);
        test_stationary_course_edge(0, false, false, 90U);
        test_stationary_course_edge(1, true, false, 90U);
        test_stationary_course_edge(1, false, false, 135U);
        return EXIT_SUCCESS;
    }
    if (2 == argc && 0 == strcmp(argv[1], "bounded-course-contact-boundary"))
    {
        test_bounded_course_contact_boundary();
        return EXIT_SUCCESS;
    }
    if (2 == argc && 0 == strcmp(argv[1], "bounded-course-scan-budget"))
    {
    test_bounded_course_contact_boundary();
    test_bounded_course_scan_budget();
        return EXIT_SUCCESS;
    }
    if (2 == argc && 0 == strcmp(argv[1], "interrupted-course-scan-budget"))
    {
        test_interrupted_course_scan_budget();
        return EXIT_SUCCESS;
    }
    if (1 != argc)
    {
        return EXIT_FAILURE;
    }
    test_watch_direct();
    test_tape_is_a_direction_guide();
    test_barcode_direct();
    test_barcode_reader_crosses_unmarked_bend(true);
    test_barcode_reader_crosses_unmarked_bend(false);
    test_barcode_straight_keeps_the_gap();
    test_blocked_reverse_stops_after_the_retries();
    test_one_wheel_jam_after_a_reverse_latches_encoder();
    test_barcode_during_the_reverse();
    (void)printf("pilot_line: ok\n");
    return EXIT_SUCCESS;
}
