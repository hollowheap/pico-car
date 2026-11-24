/** @file car_bench.c
 *
 * @brief Measurement bench for the car. It collects the numbers that
 *        only the car can give. Standalone application with its own usermain.
 *
 * The bench starts the motion service and the line sensors, nothing else: no
 * obstacle task, no supervisor, no telemetry. Output goes to the USB console.
 * Copy the lines that start with a tag in capitals (STEP, SENS, LINE, DRV,
 * ENC, ODO, PIV) into a file and give the file to the analysis scripts.
 *
 *   GP20 short press = next item (the item is printed)
 *   GP20 long press (1 s) = run it (3 s countdown, a press stops the run)
 *
 * GP21 is not used: it is the emergency-stop input of the motion service, and a
 * press latches the stop until the board reboots.
 *
 * Items:
 *   ENC   Spin each wheel alone and tell which encoder answers.
 *   STEP  Step both wheels from 0 to cruise and log target, speed and duty.
 *   SENS  Log every change of the line sensors while you slide the car by hand.
 *   LINEA Follow a straight line with the OLD law (inner wheel stop), 8 s.
 *   LINEB Follow the same line with the NEW law (heading cascade), 8 s.
 *   LIFT  Drive 14 s, lift the car at 6 s, put it down at 10 s. Logs duty and
 *         speed and shows when the wheels-run-free cue sets.
 *   ODO   The motors stay off for 30 s. Push the car along a ruler, or turn a
 *         wheel one full turn by hand. Prints the distance and the encoder
 *         edges each wheel counted, so the encoder scale can be checked.
 *   PIVOT Pivot on a taped cross for one full turn after the first crossing.
 *         Prints encoder yaw and pivot share at each of the four crossings.
 *   TIME  Capture actual stationary bench pass and feedback-read timing.
 *         Exports the trace as JSONL after the trial.
 *
 * NOTE: usermain() is the name the kernel calls, so it has no module prefix.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Keep the standard size_t when importing the kernel interfaces. */
#define PROHIBIT_DEF_SIZE_T
#include <tk/smp.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "bench/trace.h"
#include "motion/motion.h"
#include "motion/motor_setup.h"
#include "../motion/motion_service.h"
#include "platform/chassis.h"
#include "platform/isr.h"
#include "platform/log.h"
#include "platform/units.h"
#include "platform/watchdog.h"
#include "track/line_control.h"
#include "track/track.h"
#include "../supervisor/health_core.h"
#include "../usb_console_compat.h"

/* Bench buttons, active low with pull-ups. */
#define BTN_NEXT_PIN (20U)
#define LONG_PRESS_MS (1000U)
#define PAD_PULL_UP   (1UL << 3U)
#define PAD_PULL_DOWN (1UL << 2U)

#define STACK_BYTES   (4096U)
#define STACK_ALIGN   (8U)
#define LOOP_MS       (10U)
#define USB_WAIT_LOOPS (200U)
#define USB_WAIT_MS    (100U)
#define USB_UP         (2U)
#define COUNTDOWN_S    (3U)
#define MS_PER_S       (1000U)
#define US_PER_MS      (1000U)
#define UM_PER_MM      (1000)
#define LEASE_US       (200000U)
#define PROBE_UM_S     (100000)
#define PROBE_MS       (1500U)
#define STEP_SAMPLES   (200U)
#define STEP_AT_MS     (200U)
#define STEP_OFF_MS    (1400U)
#define LINE_RUN_MS    (8000U)
#define LINE_SEARCH_ABORT_MS (3000U)
#define LIFT_RUN_MS    (14000U)
#define LIFT_UP_MS     (6000U)
#define LIFT_DOWN_MS   (10000U)
#define LIFT_LOG_MS    (100U)
#define SENS_RUN_MS    (20000U)
#define SENS_POLL_MS   (5U)
#define DECI_PER_DEG   (10)
#define TIMING_PASSES  (64U)
#define TIMING_ROWS_PER_PASS (3U)
#define TRACE_USB_WAIT_POLLS (20U)
#define TRACE_USB_PENDING_LIMIT (1024U)

#ifndef PICO_CAR_BENCH_BUILD
#define PICO_CAR_BENCH_BUILD "unknown"
#endif

typedef enum
{
    ITEM_ENC = 0,
    ITEM_STEP,
    ITEM_SENS,
    ITEM_LINEA,
    ITEM_LINEB,
    ITEM_LIFT,
    ITEM_ODO,
    ITEM_PIVOT,
    ITEM_TIMING,
    ITEM_COUNT
} item_t;

static char const * const ITEM_TEXT[ITEM_COUNT] = {
    "ENC   spin each wheel alone (wheels on a block or the car held up)",
    "STEP  step both wheels 0 to cruise (the car drives about 25 cm)",
    "SENS  log the line sensors (car on the line, slide it by hand)",
    "LINEA OLD law on a straight line, 8 s (needs 1.5 m of straight line)",
    "LINEB NEW law on a straight line, 8 s (put the car back at the start)",
    "LIFT  drive 14 s: lift the car at 6 s, put it down at 10 s (1.5 m free)",
    "ODO   motors off: push the car by hand along a ruler, or turn one wheel one full turn",
    "PIVOT pivot on a taped cross: log encoder yaw and share at four crossings",
    "TIME  motors off: capture bench pass and feedback-read timing as JSONL",
};

typedef struct
{
    int16_t left_mms;
    int16_t right_mms;
    int16_t left_duty;
    int16_t right_duty;
} step_sample_t;

static UB g_stack_motion[STACK_BYTES] __attribute__((aligned(STACK_ALIGN)));
static UB g_stack_bench[STACK_BYTES] __attribute__((aligned(STACK_ALIGN)));
static step_sample_t g_step[STEP_SAMPLES];
static bench_trace_record_t g_timing_rows[TIMING_PASSES * TIMING_ROWS_PER_PASS];

static uint32_t now_ms (void)
{
    SYSTIM tim = {0, 0U};

    (void)tk_get_otm (&tim);
    return tim.lo;
}

static void button_init (UINT pin)
{
    (void)gpio_set_pin (pin, GPIO_MODE_IN);
    clr_w (GPIO (pin), PAD_PULL_DOWN);
    set_w (GPIO (pin), PAD_PULL_UP);
}

/* A stop needs the button held for ABORT_POLLS polls in a row (about 50 ms in
 * a 10 ms loop), so the bounce when GP20 is released to start a run does not
 * stop the run at once. */
#define ABORT_POLLS (5U)
static uint32_t g_abort_count;

static bool abort_requested (void)
{
    bool b_abort = false;

    if (0U == gpio_get_val (BTN_NEXT_PIN))
    {
        g_abort_count++;
        b_abort = (ABORT_POLLS == g_abort_count);
    }
    else
    {
        g_abort_count = 0U;
    }
    return b_abort;
}

static void drive (int32_t left_um_s, int32_t right_um_s)
{
    /* The bench has no supervisor: it renews the run permit itself, or the
       motion service holds the wheels (the permit fails closed). */
    (void)motion_renew_permit (LEASE_US / US_PER_MS);
    (void)motion_set_speed (left_um_s, right_um_s, LEASE_US);
}

static void brake (void)
{
    drive (0, 0);
    (void)tk_dly_tsk (LOOP_MS * 10U);
}

/* Wait for the countdown. Returns false when the abort button is pressed. */
static bool countdown (void)
{
    uint32_t secs = COUNTDOWN_S;

    g_abort_count = 0U;
    for (; secs > 0U; secs--)
    {
        uint32_t waited = 0U;

        LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "start in %d s (press GP20 to stop)", (int) secs);
        log_flush ();
        for (waited = 0U; waited < MS_PER_S; waited += LOOP_MS)
        {
            (void) tk_dly_tsk (LOOP_MS);
            if (abort_requested ())
            {
                LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "aborted");
                log_flush ();
                return false;
            }
        }
    }
    return true;
}

static int32_t mm_of (int64_t um)
{
    return (int32_t) (um / UM_PER_MM);
}

/* ENC: drive each wheel alone for PROBE_MS and compare the encoders. */
static void run_enc (void)
{
    char const * const names[2] = {"left", "right"};
    uint32_t           wheel    = 0U;

    for (wheel = 0U; wheel < 2U; wheel++)
    {
        motion_feedback_t before;
        motion_feedback_t after;
        uint32_t          start = 0U;
        int32_t           driven = 0;
        int32_t           other  = 0;
        drive_probe_t     verdict = DRIVE_PROBE_UNCLEAR;

        (void) motion_feedback_snapshot (&before);
        start = now_ms ();
        while ((now_ms () - start) < PROBE_MS)
        {
            drive ((0U == wheel) ? PROBE_UM_S : 0, (0U == wheel) ? 0 : PROBE_UM_S);
            (void) tk_dly_tsk (LOOP_MS);
        }
        brake ();
        (void) motion_feedback_snapshot (&after);
        driven = mm_of ((0U == wheel) ? (after.left_displacement_um - before.left_displacement_um)
                                      : (after.right_displacement_um - before.right_displacement_um));
        other  = mm_of ((0U == wheel) ? (after.right_displacement_um - before.right_displacement_um)
                                      : (after.left_displacement_um - before.left_displacement_um));
        verdict = health_drive_probe_verdict (driven, other);
        LOG_NOW ("DATA", "bench:enc", "ENC,%s,driven_mm=%d,other_mm=%d,verdict=%d\n", names[wheel],
                 (int) driven, (int) other, (int) verdict);
        LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE,
                  "%s wheel: 0 ok, 1 dead, 2 reversed, 3 crossed, 4 unclear -> %d", names[wheel],
                  (int) verdict);
        log_flush ();
        (void) tk_dly_tsk (MS_PER_S);
    }
}

/* STEP: record 2 s at 10 ms, then print the rows. */
static void run_step (void)
{
    uint32_t idx = 0U;
    int32_t  t63_left  = -1;
    int32_t  t63_right = -1;
    int32_t  final_left  = 0;
    int32_t  final_right = 0;

    for (idx = 0U; idx < STEP_SAMPLES; idx++)
    {
        uint32_t const    t_ms = idx * LOOP_MS;
        int32_t const     cmd  = ((t_ms >= STEP_AT_MS) && (t_ms < STEP_OFF_MS)) ? LINE_CRUISE_UM_S : 0;
        motion_feedback_t fb;
        motion_diag_t     diag;

        drive (cmd, cmd);
        (void) motion_feedback_snapshot (&fb);
        (void) motion_diag_snapshot (&diag);
        g_step[idx].left_mms   = (int16_t) (fb.left_speed_um_s / UM_PER_MM);
        g_step[idx].right_mms  = (int16_t) (fb.right_speed_um_s / UM_PER_MM);
        g_step[idx].left_duty  = diag.left_duty_permille;
        g_step[idx].right_duty = diag.right_duty_permille;
        (void) tk_dly_tsk (LOOP_MS);
    }
    brake ();

    final_left  = g_step[(STEP_OFF_MS / LOOP_MS) - 1U].left_mms;
    final_right = g_step[(STEP_OFF_MS / LOOP_MS) - 1U].right_mms;
    for (idx = STEP_AT_MS / LOOP_MS; idx < (STEP_OFF_MS / LOOP_MS); idx++)
    {
        /* 63 percent of the final value, as a fraction of 100. */
        if ((t63_left < 0) && ((g_step[idx].left_mms * PERCENT) >= (final_left * 63)))
        {
            t63_left = (int32_t) ((idx * LOOP_MS) - STEP_AT_MS);
        }
        if ((t63_right < 0) && ((g_step[idx].right_mms * PERCENT) >= (final_right * 63)))
        {
            t63_right = (int32_t) ((idx * LOOP_MS) - STEP_AT_MS);
        }
    }
    for (idx = 0U; idx < STEP_SAMPLES; idx++)
    {
        LOG_NOW ("DATA", "bench:step", "STEP,%u,%d,%d,%d,%d\n", (unsigned) (idx * LOOP_MS),
                 (int) g_step[idx].left_mms, (int) g_step[idx].right_mms,
                 (int) g_step[idx].left_duty, (int) g_step[idx].right_duty);
        /* The USB console drops rows printed faster than the host reads them. */
        (void) tk_dly_tsk (5U);
    }
    LOG_NOW ("DATA", "bench:step",
             "STEP_RESULT,target_mms=%d,final_left=%d,final_right=%d,t63_left_ms=%d,t63_right_ms=%d\n",
             (int) (LINE_CRUISE_UM_S / UM_PER_MM), (int) final_left, (int) final_right,
             (int) t63_left, (int) t63_right);
    log_flush ();
}

/* SENS: print every change of the three bits for SENS_RUN_MS. */
static void run_sens (void)
{
    uint32_t start = now_ms ();
    uint8_t  last  = 0xFFU;
    uint32_t edges = 0U;

    while (((now_ms () - start) < SENS_RUN_MS) && !abort_requested ())
    {
        ir_sensors_state_t const s = ir_get_state ();
        uint8_t const bits = (uint8_t) ((s.left ? 1U : 0U) | (s.right ? 2U : 0U) | (s.barcode ? 4U : 0U));

        if (bits != last)
        {
            LOG_NOW ("DATA", "bench:sens", "SENS,%u,%u,%u,%u\n", (unsigned) (now_ms () - start),
                     (unsigned) (bits & 1U), (unsigned) ((bits >> 1U) & 1U), (unsigned) ((bits >> 2U) & 1U));
            edges++;
            last = bits;
        }
        (void) tk_dly_tsk (SENS_POLL_MS);
    }
    LOG_NOW ("DATA", "bench:sens", "SENS_RESULT,changes=%u,filtered_edges=%u,raw_edges=%u\n",
             (unsigned) edges, (unsigned) ir_line_edges (), (unsigned) ir_line_raw_edges ());
    log_flush ();
}

/* The old law: inner wheel stop toward the last side, no search limit. */
static void legacy_update (uint8_t * p_side, bool b_left, bool b_right, line_targets_t * p_t)
{
    if (b_left || b_right)
    {
        *p_side = b_left ? (uint8_t) LINE_SIDE_LEFT : (uint8_t) LINE_SIDE_RIGHT;
    }
    p_t->left_um_s  = LINE_CRUISE_UM_S;
    p_t->right_um_s = LINE_CRUISE_UM_S;
    if (!(b_left && b_right))
    {
        if ((uint8_t) LINE_SIDE_LEFT == *p_side)
        {
            p_t->left_um_s = 0;
        }
        else if ((uint8_t) LINE_SIDE_RIGHT == *p_side)
        {
            p_t->right_um_s = 0;
        }
        else
        {
            /* No side yet: straight on. */
        }
    }
}

/* LINEA and LINEB: one run of a law along a straight line. */
static void run_line (bool b_new_law)
{
    line_follow_t     follow;
    uint8_t           legacy_side = (uint8_t) LINE_SIDE_NONE;
    uint32_t          start       = now_ms ();
    uint32_t          searching_ms = 0U;
    uint32_t          passes      = 0U;
    uint32_t          inner_stops = 0U;
    uint32_t          edges0      = ir_line_edges ();
    uint32_t          raw0        = ir_line_raw_edges ();
    int32_t           head_max    = 0;
    int32_t           head_min    = 0;
    int64_t           start_travel = 0;
    int64_t           end_travel   = 0;
    bool              b_zero_prev  = false;
    motion_feedback_t fb;
    bool              b_first = true;

    line_follow_init (&follow);
    while (((now_ms () - start) < LINE_RUN_MS) && !abort_requested ())
    {
        ir_sensors_state_t const s = ir_get_state ();
        line_targets_t           t = {0, 0};
        int32_t                  heading = 0;
        bool                     b_zero = false;

        (void) motion_feedback_snapshot (&fb);
        heading = line_heading_deci (fb.left_displacement_um, fb.right_displacement_um);
        end_travel = (fb.left_displacement_um + fb.right_displacement_um) / 2;
        if (b_first)
        {
            start_travel = end_travel;
            head_max     = heading;
            head_min     = heading;
            b_first      = false;
        }
        if (b_new_law)
        {
            /* The search statistics (searching_ms, the travel and heading range and
               the zero-wheel count) include the reverse of a retry, which drives
               backward at LINE_SEARCH_UM_S. */
            line_follow_status_t const st = line_follow_update (&follow, s.left, s.right, heading,
                                                                end_travel, now_ms (), &t);

            if (LINE_FOLLOW_SEARCHING == st)
            {
                searching_ms += LOOP_MS;
            }
        }
        else
        {
            legacy_update (&legacy_side, s.left, s.right, &t);
        }
        b_zero = ((0 == t.left_um_s) || (0 == t.right_um_s));
        if (b_zero && !b_zero_prev)
        {
            inner_stops++;
        }
        b_zero_prev = b_zero;
        if (heading > head_max)
        {
            head_max = heading;
        }
        if (heading < head_min)
        {
            head_min = heading;
        }
        drive (t.left_um_s, t.right_um_s);
        passes++;
        if (searching_ms > LINE_SEARCH_ABORT_MS)
        {
            LOG_WARN (LOG_TASK_BOOT, LOG_MOD_NONE, "line lost for 3 s, run stopped");
            break;
        }
        (void) tk_dly_tsk (LOOP_MS);
    }
    brake ();
    {
        uint32_t const secs_x10 = ((now_ms () - start) * 10U) / MS_PER_S;
        uint32_t const edges    = ir_line_edges () - edges0;
        uint32_t const raw      = ir_line_raw_edges () - raw0;

        LOG_NOW ("DATA", "bench:line",
                 "LINE_RESULT,law=%s,secs_x10=%u,travel_mm=%d,edges=%u,raw_edges=%u,"
                 "inner_stops=%u,heading_swing_deci=%d,searching_ms=%u,passes=%u\n",
                 b_new_law ? "cascade" : "legacy", (unsigned) secs_x10,
                 (int) mm_of (end_travel - start_travel), (unsigned) edges, (unsigned) raw,
                 (unsigned) inner_stops, (int) (head_max - head_min), (unsigned) searching_ms,
                 (unsigned) passes);
        log_flush ();
    }
}

/* LIFT: constant drive; log duty and speed; run the wheels-free cue on them. */
static void run_lift (void)
{
    drive_t  cue;
    uint32_t start = now_ms ();
    uint32_t next_log = 0U;
    bool     b_was_free = false;

    health_drive_init (&cue);
    while (((now_ms () - start) < LIFT_RUN_MS) && !abort_requested ())
    {
        uint32_t const    t = now_ms () - start;
        motion_feedback_t fb;
        motion_diag_t     diag;
        drive_in_t        in;

        drive (LINE_CRUISE_UM_S, LINE_CRUISE_UM_S);
        (void) motion_feedback_snapshot (&fb);
        (void) motion_diag_snapshot (&diag);
        if (t >= next_log)
        {
            in.duty_pm[0]     = diag.left_duty_permille;
            in.duty_pm[1]     = diag.right_duty_permille;
            in.speed_um_s[0]  = fb.left_speed_um_s;
            in.speed_um_s[1]  = fb.right_speed_um_s;
            in.target_um_s[0] = diag.left_target_um_s;
            in.target_um_s[1] = diag.right_target_um_s;
            in.b_judge        = fb.fresh;
            (void) health_drive_update (&cue, &in, LIFT_LOG_MS);
            LOG_NOW ("DATA", "bench:lift", "DRV,%u,%d,%d,%d,%d,%d,%d,%d\n", (unsigned) t,
                     (int) in.duty_pm[0], (int) in.duty_pm[1], (int) (in.speed_um_s[0] / UM_PER_MM),
                     (int) (in.speed_um_s[1] / UM_PER_MM), (int) (in.target_um_s[0] / UM_PER_MM),
                     (int) (in.target_um_s[1] / UM_PER_MM), (int) (cue.b_free ? 1 : 0));
            if (cue.b_free != b_was_free)
            {
                LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "wheels-free cue %s at %u ms",
                          cue.b_free ? "SET" : "CLEARED", (unsigned) t);
                b_was_free = cue.b_free;
            }
            next_log += LIFT_LOG_MS;
            log_flush ();
        }
        if (((t >= LIFT_UP_MS) && (t < (LIFT_UP_MS + LOOP_MS * 2U))) ||
            ((t >= LIFT_DOWN_MS) && (t < (LIFT_DOWN_MS + LOOP_MS * 2U))))
        {
            LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, (t < LIFT_DOWN_MS) ? "LIFT THE CAR NOW"
                                                                       : "PUT THE CAR DOWN NOW");
            log_flush ();
        }
        (void) tk_dly_tsk (LOOP_MS);
    }
    brake ();
    LOG_NOW ("DATA", "bench:lift", "DRV_RESULT,cue_set=%d\n", (int) (b_was_free ? 1 : 0));
    log_flush ();
}

/* ODO: no motor command; print what the encoders count while the car is moved by hand. */
#define ODO_RUN_MS   (30000U)
#define ODO_PRINT_MS (500U)

static void run_odo (void)
{
    motion_feedback_t start;
    uint32_t          t0 = now_ms ();
    uint32_t          next = 0U;

    (void) motion_feedback_snapshot (&start);
    while (((now_ms () - t0) < ODO_RUN_MS) && !abort_requested ())
    {
        uint32_t const t = now_ms () - t0;

        if (t >= next)
        {
            motion_feedback_t fb;
            int64_t           left_um  = 0;
            int64_t           right_um = 0;

            (void) motion_feedback_snapshot (&fb);
            left_um  = fb.left_displacement_um - start.left_displacement_um;
            right_um = fb.right_displacement_um - start.right_displacement_um;
            /* The edges the firmware counted, from its own scale: um * edges / travel. */
            LOG_NOW ("DATA", "bench:odo", "ODO,%u,left_mm=%d,right_mm=%d,left_edges=%d,right_edges=%d\n",
                     (unsigned) t, (int) mm_of (left_um), (int) mm_of (right_um),
                     (int) ((left_um * (int64_t)CHASSIS_EDGES_PER_REV) / (int64_t)CHASSIS_WHEEL_TRAVEL_UM),
                     (int) ((right_um * (int64_t)CHASSIS_EDGES_PER_REV) / (int64_t)CHASSIS_WHEEL_TRAVEL_UM));
            next += ODO_PRINT_MS;
        }
        (void) tk_dly_tsk (LOOP_MS);
    }
    LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "ODO end: compare left_mm/right_mm with the ruler");
    log_flush ();
}

/* PIVOT: spin on a taped cross and print encoder yaw and share at 4 crossings. */
#define PIVOT_UM_S      (LINE_SEARCH_UM_S)
#define PIVOT_CROSSINGS (4U)
#define PIVOT_STEP_DECI (900)
#define PIVOT_MAX_MS    (12000U)

static void run_pivot (void)
{
    motion_feedback_t start;
    line_follow_t     follow;
    uint32_t const    t0 = now_ms ();
    uint32_t          crossings = 0U;
    int32_t           first_yaw = 0;
    int32_t           prev_yaw = 0;
    bool              b_armed = false;

    (void) motion_feedback_snapshot (&start);
    line_follow_init (&follow);
    while (((now_ms () - t0) < PIVOT_MAX_MS) && (crossings < PIVOT_CROSSINGS) && !abort_requested ())
    {
        motion_feedback_t        fb;
        ir_sensors_state_t const s = ir_get_state ();
        line_targets_t           unused = {0, 0};

        drive (-PIVOT_UM_S, PIVOT_UM_S);
        (void) motion_feedback_snapshot (&fb);
        int64_t const diff_um = (fb.right_displacement_um - start.right_displacement_um) -
                                (fb.left_displacement_um - start.left_displacement_um);
        int32_t const yaw_deci = (int32_t) wheel_yaw_deci (diff_um);
        int64_t const sensor_arc_um = (diff_um * (int64_t) LINE_SENSOR_LEAD_MM) /
                                      (int64_t) CHASSIS_WHEEL_TRACK_MM;

        (void) line_follow_update_floor (&follow, s.left, s.right, yaw_deci, sensor_arc_um,
                                         now_ms (), true, &unused);
        uint8_t const stripe = line_follow_take_stripe (&follow);
        if (0U != stripe)
        {
            if (!b_armed)
            {
                first_yaw = yaw_deci;
                prev_yaw  = yaw_deci;
                b_armed   = true;
            }
            else
            {
                int32_t const step_yaw  = yaw_deci - prev_yaw;
                int32_t const share_pct = (step_yaw > 0) ? ((PIVOT_STEP_DECI * PERCENT) / step_yaw) : 0;

                crossings++;
                LOG_NOW ("DATA", "bench:pivot",
                         "PIV,%u,enc_yaw_deci=%d,share_pct=%d,total_yaw_deci=%d,stripe=%u\n",
                         (unsigned) crossings, (int) step_yaw, (int) share_pct,
                         (int) (yaw_deci - first_yaw), (unsigned) stripe);
                log_flush ();
                prev_yaw = yaw_deci;
            }
        }
        (void) tk_dly_tsk (SENS_POLL_MS);
    }
    brake ();
    {
        int32_t const total_yaw   = b_armed ? (prev_yaw - first_yaw) : 0;
        int32_t const total_share = (total_yaw > 0)
                                        ? (((int32_t) crossings * PIVOT_STEP_DECI * PERCENT) / total_yaw)
                                        : 0;

        LOG_NOW ("DATA", "bench:pivot", "PIV_RESULT,crossings=%u,total_yaw_deci=%d,share_pct=%d\n",
                 (unsigned) crossings, (int) total_yaw, (int) total_share);
        log_flush ();
    }
}

/* Export a complete machine-readable row only when the post-run USB sink has room. */
static bool write_trace_row (void * p_ctx, char const * p_row, size_t length)
{
    uint32_t const dropped = tm_usb_dropped_bytes ();
    uint32_t       polls   = 0U;

    (void) p_ctx;
    while ((polls < TRACE_USB_WAIT_POLLS) && (tm_usb_state () >= USB_UP) &&
           (tm_usb_pending () + length > TRACE_USB_PENDING_LIMIT))
    {
        (void) tk_dly_tsk (LOOP_MS);
        polls++;
    }
    if ((tm_usb_state () < USB_UP) || (tm_usb_pending () + length > TRACE_USB_PENDING_LIMIT))
    {
        return false;
    }
    (void) tm_putstring ((UB const *) p_row);
    return tm_usb_dropped_bytes () == dropped;
}

static void run_timing (void)
{
    char                       trial[sizeof ("timing-00000000")];
    bench_trace_header_t const header = {.build = PICO_CAR_BENCH_BUILD, .fixture = "bench-timing", .trial = trial};
    bench_trace_t              trace;
    uint32_t                   pass = 0U;
    uint32_t                   start = 0U;

    brake ();
    start = in_w (TIMER_TIMERAWL);
    (void) tm_sprintf ((UB *) trial, (UB const *) "timing-%08x", (unsigned int) start);
    if (!bench_trace_start (&trace, &header, g_timing_rows, TIMING_PASSES * TIMING_ROWS_PER_PASS))
    {
        LOG_ERROR (LOG_TASK_BOOT, LOG_MOD_NONE, "cannot start timing trace");
        return;
    }
    for (; (pass < TIMING_PASSES) && !abort_requested (); pass++)
    {
        motion_feedback_t    feedback;
        bench_trace_record_t row = {.time_us = in_w (TIMER_TIMERAWL) - start, .kind = BENCH_TRACE_SCHEDULER,
                                    .marker = BENCH_TRACE_RELEASE};

        (void) bench_trace_record (&trace, &row);
        if (E_OK == motion_feedback_snapshot (&feedback))
        {
            row.time_us = in_w (TIMER_TIMERAWL) - start;
            row.marker  = BENCH_TRACE_CAPTURE;
            (void) bench_trace_record (&trace, &row);
        }
        row.time_us = in_w (TIMER_TIMERAWL) - start;
        row.marker  = BENCH_TRACE_COMPLETE;
        (void) bench_trace_record (&trace, &row);
        (void) tk_dly_tsk (LOOP_MS);
    }
    bench_trace_stop (&trace);
    if (!bench_trace_export (&trace, write_trace_row, NULL))
    {
        LOG_WARN (LOG_TASK_BOOT, LOG_MOD_NONE, "timing trace USB export incomplete");
    }
}

static void run_item (item_t item)
{
    if (!countdown ())
    {
        return;
    }
    switch (item)
    {
        case ITEM_ENC:
            run_enc ();
            break;
        case ITEM_STEP:
            run_step ();
            break;
        case ITEM_SENS:
            run_sens ();
            break;
        case ITEM_LINEA:
            run_line (false);
            break;
        case ITEM_LINEB:
            run_line (true);
            break;
        case ITEM_ODO:
            run_odo ();
            break;
        case ITEM_PIVOT:
            run_pivot ();
            break;
        case ITEM_TIMING:
            run_timing ();
            break;
        default:
            run_lift ();
            break;
    }
    LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "done");
    log_flush ();
}

static void bench_task (INT stacd, void * exinf)
{
    item_t   item    = ITEM_ENC;
    uint32_t down_ms = 0U;
    uint32_t loops   = 0U;

    (void) stacd;
    (void) exinf;
    for (loops = 0U; (loops < USB_WAIT_LOOPS) && (tm_usb_state () < USB_UP); loops++)
    {
        (void) tk_dly_tsk (USB_WAIT_MS);
    }
    LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "car_bench: GP20 short press = next item, hold GP20 for 1 s = run");
    LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "item: %s", ITEM_TEXT[item]);
    log_flush ();
    for (;;)
    {
        bool const b_down = (0U == gpio_get_val (BTN_NEXT_PIN));

        if (b_down)
        {
            down_ms += LOOP_MS;
            if (LONG_PRESS_MS == down_ms)
            {
                LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "release GP20 to run: %s", ITEM_TEXT[item]);
                log_flush ();
            }
        }
        else if (down_ms >= LONG_PRESS_MS)
        {
            down_ms        = 0U;
            run_item (item);
        }
        else if (down_ms > 0U)
        {
            down_ms = 0U;
            item    = (item_t) (((uint32_t) item + 1U) % (uint32_t) ITEM_COUNT);
            LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "item: %s", ITEM_TEXT[item]);
            log_flush ();
        }
        else
        {
            /* Not pressed. */
        }
        (void) tk_dly_tsk (LOOP_MS);
    }
}

static void motion_main (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;
    motion_service_run ();
}

static ID start_task (FP entry, UB * p_stack, PRI priority)
{
    T_CTSK ctsk = {
        .exinf   = NULL,
        .itskpri = priority,
        .stksz   = STACK_BYTES,
        .task    = entry,
        .bufptr  = p_stack,
        .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
        .assprc  = TP_PRC1,
    };
    ID const tid = tk_cre_tsk (&ctsk);

    if (tid > 0)
    {
        (void) tk_sta_tsk (tid, 0);
    }
    return tid;
}

EXPORT INT usermain (void)
{
    ID motion_tid = 0;

    log_init ();
    button_init (BTN_NEXT_PIN);
    if (E_OK != gpio_irq_init ())
    {
        LOG_ERROR (LOG_TASK_BOOT, LOG_MOD_NONE, "gpio_irq_init failed");
        log_flush ();
    }
    else
    {
        T_CTSK ctsk = {
            .exinf   = NULL,
            .itskpri = 1,
            .stksz   = STACK_BYTES,
            .task    = motion_main,
            .bufptr  = g_stack_motion,
            .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
            .assprc  = TP_PRC1,
        };

        motion_tid = tk_cre_tsk (&ctsk);
        if ((motion_tid > 0) && (E_OK == motion_setup (motion_tid)) && (E_OK == ir_capture_init ()))
        {
            (void) tk_sta_tsk (motion_tid, 0);
            (void) start_task (bench_task, g_stack_bench, 3);
        }
        else
        {
            LOG_ERROR (LOG_TASK_BOOT, LOG_MOD_NONE, "bench setup failed");
            log_flush ();
        }
    }
    (void) tk_slp_tsk (TMO_FEVR);
    return 0;
}

/*** end of file ***/
