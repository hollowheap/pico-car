/** Exercise motion feedback and emergency-stop publication. */
#include <assert.h>
#include <setjmp.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform/drv_gpio_irq.h"
#include "comms/telemetry/assemble.h"
#include "comms/telemetry/emit.h"
#include "motion_service.c"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static uint32_t g_test_clock_us;
static uint32_t g_test_clock_high;
static uint32_t g_latch_time_us;
static bool g_b_stop_pressed;
static bool g_b_brake_fails;
static unsigned g_latch_attempts;
static unsigned g_capture_attempts;
static bool g_b_capture_fails;
static uint32_t g_forward_edges;
static unsigned g_pair_writes;
static unsigned g_nonzero_pair_writes;
static INT g_queued_wakes;
static unsigned g_wake_calls;
static unsigned g_stop_pin_reads;
static bool g_b_request_after_admission;
static bool g_b_hold_after_intake;
static bool g_b_inject_remote_in_sleep;
static bool g_b_inject_gpio_in_sleep;
static gpio_irq_handler_t g_stop_edge_handler;
static ER g_gpio_registration_result;
static unsigned g_cyclic_deletes;
static jmp_buf g_parked;
static bool g_b_expect_park;
static struct
{
    int16_t states;
    uint16_t fault_holds;
    uint32_t odo_mm;
} g_motion_events[8];
static unsigned g_motion_event_count;

INT g_shim_prc = TP_PRC1;

static void stop_button_pulse(void)
{
    gpio_edge_snapshot_t const released = {g_test_clock_us, 1U << 21U, 0U};
    gpio_edge_snapshot_t const pulse = {g_test_clock_us, 1U << 21U, 1U << 21U};
    gpio_edge_snapshot_t const bounce = {g_test_clock_us, 0U, 1U << 21U};

    if (g_stop_edge_handler != NULL)
    {
        unsigned const before = g_wake_calls;

        g_stop_edge_handler(&released);
        assert(before == g_wake_calls);
        g_stop_edge_handler(&pulse);
        g_stop_edge_handler(&bounce);
    }
}

uint32_t shim_reg_read(uint32_t address)
{
    if (21U == address)
    {
        return 0U;
    }
    if (TIMER_TIMERAWH == address)
    {
        return g_test_clock_high;
    }
    assert(address == TIMER_TIMERAWL);
    return g_test_clock_us;
}

UINT gpio_get_val (UINT pin)
{
    assert(21U == pin);
    g_stop_pin_reads++;
    if (g_b_request_after_admission && 2U == g_stop_pin_reads)
    {
        assert(E_OK == motion_service_request_stop());
    }
    if (g_b_hold_after_intake && 2U == g_stop_pin_reads)
    {
        assert(E_OK == motion_service_set_hold(MOTION_HOLD_OPERATOR, true));
    }
    return g_b_stop_pressed ? 0U : 1U;
}

ER tk_slp_tsk (TMO timeout)
{
    if (TMO_FEVR == timeout)
    {
        assert(g_b_expect_park);
        longjmp(g_parked, 1);
    }
    assert(!g_b_expect_park);
    if (g_b_inject_remote_in_sleep)
    {
        g_test_clock_us = 1000U;
        assert(E_OK == motion_service_request_stop());
        assert(E_OK == motion_service_request_stop());
    }
    if (g_b_inject_gpio_in_sleep)
    {
        g_test_clock_us = 1000U;
        stop_button_pulse();
    }
    if (g_queued_wakes > 0)
    {
        g_queued_wakes--;
    }
    else if (g_b_inject_gpio_in_sleep || g_b_inject_remote_in_sleep)
    {
        g_test_clock_us = 10000U;
    }
    return E_OK;
}

INT tk_can_wup (ID task_id)
{
    INT const queued = g_queued_wakes;

    (void)task_id;
    g_queued_wakes = 0;
    return queued;
}

ER tk_wup_tsk (ID task_id)
{
    assert(5 == task_id);
    assert(0U == g_service_lock);
    g_wake_calls++;
    g_queued_wakes++;
    return E_OK;
}

ER telemetry_try_emit_event (uint8_t source, uint8_t type, int16_t p1,
                             uint16_t p2, uint32_t odometer_mm)
{
    assert(0U == g_service_lock);
    assert(TELEMETRY_EVENT_SOURCE_MOTION == source);
    assert(TELEMETRY_EVT_MOTION_STATE == type);
    assert(g_motion_event_count < sizeof g_motion_events / sizeof g_motion_events[0]);
    g_motion_events[g_motion_event_count].states = p1;
    g_motion_events[g_motion_event_count].fault_holds = p2;
    g_motion_events[g_motion_event_count].odo_mm = odometer_mm;
    g_motion_event_count++;
    return E_TMOUT;
}

ID tk_get_tid (void)
{
    return 5;
}

ID tk_cre_cyc (const T_CCYC * p_config)
{
    assert(10U == p_config->cyctim && 10U == p_config->cycphs);
    return 6;
}

ER tk_del_cyc (ID cyclic_id)
{
    assert(6 == cyclic_id);
    g_cyclic_deletes++;
    return E_OK;
}

ER gpio_set_pin (UINT pin, UINT mode)
{
    assert(21U == pin && GPIO_MODE_IN == mode);
    return E_OK;
}

UW GPIO (UINT pin)
{
    return pin;
}

void clr_w (UW address, UW bits)
{
    assert(21U == address && MOTION_PAD_PULL_DOWN == bits);
}

void shim_reg_write (UW address, UW bits)
{
    assert(21U == address && 0U != bits);
}

ER drv_gpio_irq_add_handler (UW proc, uint32_t mask, gpio_irq_handler_t handler)
{
    assert(TP_PRC1 == proc && (1U << 21U) == mask && handler != NULL);
    if (E_OK == g_gpio_registration_result)
    {
        g_stop_edge_handler = handler;
    }
    return g_gpio_registration_result;
}

ER encoder_init (void)
{
    return E_OK;
}

ER encoder_read (encoder_sample_t * p_sample)
{
    g_capture_attempts++;
    if (g_b_capture_fails)
    {
        return E_IO;
    }
    *p_sample = (encoder_sample_t){.timestamp_us = g_test_clock_us};
    p_sample->wheel[ENCODER_LEFT].positive = g_forward_edges;
    p_sample->wheel[ENCODER_RIGHT].positive = g_forward_edges;
    return E_OK;
}

void drv_pwm_boot_brake (void)
{
}

ER drv_pwm_init (void)
{
    return E_OK;
}

ER drv_pwm_brake (void)
{
    return E_OK;
}

ER drv_pwm_latch_stop (void)
{
    g_latch_attempts++;
    g_latch_time_us = g_test_clock_us;
    return g_b_brake_fails ? E_IO : E_OK;
}

ER drv_pwm_set_pair (int16_t left, int16_t right)
{
    g_pair_writes++;
    if ((left != 0) || (right != 0))
    {
        g_nonzero_pair_writes++;
    }
    return E_OK;
}

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod, const char * p_format, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_format;
}

static void publish_sample(uint32_t timestamp)
{
    g_start_called = true;
    g_published = (motion_service_status_t){.ready = true};
    g_published.control.sample_available = true;
    g_published.control.sample.timestamp_us = timestamp;
    g_published.control.sample.wheel[ENCODER_LEFT] = (motion_wheel_sample_t){
        .velocity_um_s = -125000, .displacement_um = -10425,
        .speed_valid = true, .odometry_valid = true, .invalid_samples = 2U,
    };
    g_published.control.sample.wheel[ENCODER_RIGHT] = (motion_wheel_sample_t){
        .velocity_um_s = 17000, .displacement_um = 4567,
        .speed_valid = false, .odometry_valid = false, .invalid_samples = 9U,
    };
}

static void test_signed_snapshot(void)
{
    uint32_t const stamp = UINT32_MAX - 499U;
    motion_feedback_t value;

    publish_sample(stamp);
    g_test_clock_us = 500U;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.timestamp_us == stamp);
    assert(value.age_us == 1000U && value.fresh);
    assert(value.left_speed_um_s == -125000 && value.left_displacement_um == -10425);
    assert(value.right_speed_um_s == 17000 && value.right_displacement_um == 4567);
    assert(value.speed_valid[0] && value.odometry_valid[0] && value.invalid_samples[0] == 2U);
    assert(!value.speed_valid[1] && !value.odometry_valid[1] && value.invalid_samples[1] == 9U);

    value.left_speed_um_s = 0;
    value.left_displacement_um = 0;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.left_speed_um_s == -125000 && value.left_displacement_um == -10425);
}

static void test_age_boundary(void)
{
    motion_feedback_t value;

    publish_sample(10U);
    g_test_clock_us = 10U + MOTION_FEEDBACK_MAX_AGE_US;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.fresh && value.age_us == MOTION_FEEDBACK_MAX_AGE_US);
    g_test_clock_us++;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(!value.fresh && value.age_us == MOTION_FEEDBACK_MAX_AGE_US + 1U);
}

static void test_reject_delayed_feedback(void)
{
    uint32_t const stamp = UINT32_MAX - 499U;
    uint32_t const long_age_us = (UINT32_MAX / 2U) + 1U;
    motion_feedback_t value;

    publish_sample(stamp);
    g_test_clock_us = stamp + 26000U;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.age_us == 26000U);
    assert(!value.fresh);

    g_test_clock_us = stamp + long_age_us;
    assert(E_OK == motion_feedback_snapshot(&value));
    assert(value.age_us == long_age_us);
    assert(!value.fresh);
}

static void check_failed_brake_stop (void)
{
    motion_status_t status;
    motion_feedback_t feedback;

    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(MOTION_FAULT_OUTPUT == status.fault);
    assert(status.stop_requested);
    assert(!status.ready);
    assert(!feedback.fresh);
    assert(0U != (system_fault_get() & FAULT_ESTOP_LATCHED_BIT));
}

static controller_config_t test_control_config (void)
{
    controller_config_t cfg = {0};
    unsigned wheel;

    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        cfg.sampling.wheel[wheel] = (motion_scale_t){204204U, 2530U, 1};
        cfg.speed[wheel] =
            (pid_config_t){256U, 256U, 2U, 300000U, 100000U, 600000U, 400U, 0U, 0U, 0U};
    }
    cfg.sampling.max_interval_us = 100000U;
    cfg.max_command_ttl_us = 200000U;
    cfg.max_control_gap_us = 100000U;
    cfg.reversal_brake_us = 100000U;
    return cfg;
}

static void test_estop_brake_failure (bool b_startup)
{
    controller_config_t const cfg = test_control_config();
    motion_input_t input = {0};
    motion_output_t const output = {NULL, apply_output, brake_output};

    assert(controller_init(&g_controller, &cfg, &input, &output));
    assert(motion_inbox_init(&g_inbox, cfg.max_command_ttl_us));
    g_start_called = true;

    g_b_brake_fails = true;
    g_b_stop_pressed = true;
    if (b_startup)
    {
        service_first_status();
    }
    else
    {
        service_cycle(&input);
    }
    assert(1U == g_latch_attempts);
    check_failed_brake_stop();

    g_b_stop_pressed = false;
    service_cycle(&input);
    check_failed_brake_stop();
    assert(0U == g_capture_attempts);
}

static void start_service (motion_input_t * p_input)
{
    motion_service_config_t const config = {test_control_config(), 10U};
    motion_output_t output = {NULL, apply_output, brake_output};

    assert(E_OK == motion_service_init(&config, 5));
    assert(E_OK == service_start(&output, p_input));
    service_first_status();
    assert(E_OK == motion_service_renew_permit(200U));
}

static void test_estop_request_status (void)
{
    uint16_t const inhibit_fault = 1U << TELEMETRY_FAULT_BIT_ESTOP_LATCHED;
    motion_input_t input;
    motion_status_t status;
    motion_feedback_t feedback;
    telemetry_status_t assembled;
    uint8_t packet[TELEMETRY_STATUS_PIVOT_WIRE_SIZE];

    start_service(&input);
    publish_sample(1000U);
    g_published.control.state = MOTION_RUNNING;
    g_published.control.sample.wheel[ENCODER_LEFT].velocity_um_s = 160000;
    g_published.control.sample.wheel[ENCODER_RIGHT].velocity_um_s = 164000;
    g_published.control.sample.wheel[ENCODER_RIGHT].speed_valid = true;
    g_test_clock_us = 1001U;

    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(status.ready && feedback.fresh && !status.stop_requested);
    telemetry_assemble_status(&assembled, &status, &feedback, NULL, NULL, 0U, 0U, 0U, false, NULL);
    assert(TELEMETRY_STATUS_PIVOT_WIRE_SIZE == telemetry_pack_status(packet, &assembled));
    assert(0U == packet[TELEMETRY_STATUS_OFF_SPEED_MMS]);
    assert(162U == packet[TELEMETRY_STATUS_OFF_SPEED_MMS + 1U]);
    assert(TELEMETRY_RUN_STATE_RUNNING == packet[TELEMETRY_STATUS_OFF_STATE]);

    assert(E_OK == motion_request_stop());
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(status.stop_requested && !status.ready && !feedback.fresh);
    assert(MOTION_STATE_RUNNING == status.state && 0U == g_latch_attempts);
    assert(1U == feedback.age_us && 164000 == feedback.right_speed_um_s);
    telemetry_assemble_status(&assembled, &status, &feedback, NULL, NULL, 0U, 0U, 0U, false, NULL);
    assert(TELEMETRY_STATUS_PIVOT_WIRE_SIZE == telemetry_pack_status(packet, &assembled));
    /* The protocol sends zero for unavailable speed before the brake runs. */
    assert(0U == packet[TELEMETRY_STATUS_OFF_SPEED_MMS]);
    assert(0U == packet[TELEMETRY_STATUS_OFF_SPEED_MMS + 1U]);
    assert(TELEMETRY_RUN_STATE_STOPPED == packet[TELEMETRY_STATUS_OFF_STATE]);
    assert((inhibit_fault >> 8U) == packet[TELEMETRY_STATUS_OFF_FAULTS]);
    assert((inhibit_fault & 0xFFU) == packet[TELEMETRY_STATUS_OFF_FAULTS + 1U]);
}

static void test_motion_state_events (void)
{
    motion_input_t input;

    start_service(&input);
    assert(0U == g_motion_event_count);
    assert(E_OK == motion_service_set_speed(125000, 125000, 200000U, false));
    g_test_clock_us = 10000U;
    service_cycle(&input);
    assert(1U == g_motion_event_count);
    assert(0x0001 == g_motion_events[0].states);
    assert(0U == g_motion_events[0].fault_holds);
    assert(0U == g_motion_events[0].odo_mm);

    assert(E_OK == motion_service_set_speed(140000, 130000, 200000U, false));
    g_stop_pin_reads = 0U;
    g_b_hold_after_intake = true;
    g_test_clock_us = 20000U;
    service_cycle(&input);
    g_b_hold_after_intake = false;
    assert(MOTION_HOLD_OPERATOR == g_published.hold_mask);
    assert(1U == g_motion_event_count);
    g_test_clock_us = 30000U;
    service_cycle(&input);
    assert(2U == g_motion_event_count);
    assert(0x0100 == g_motion_events[1].states);
    assert(MOTION_HOLD_OPERATOR == g_motion_events[1].fault_holds);

    assert(E_OK == motion_service_set_hold(MOTION_HOLD_CALIBRATE, true));
    g_test_clock_us = 40000U;
    service_cycle(&input);
    assert(3U == g_motion_event_count);
    assert(0x0000 == g_motion_events[2].states);
    assert((MOTION_HOLD_OPERATOR | MOTION_HOLD_CALIBRATE) ==
           g_motion_events[2].fault_holds);

    g_b_brake_fails = true;
    assert(E_OK == motion_service_request_stop());
    assert(3U == g_motion_event_count);
    g_test_clock_us = 41000U;
    service_cycle(&input);
    assert(4U == g_motion_event_count);
    assert(0x0004 == g_motion_events[3].states);
    assert(((TELEMETRY_MOTION_FAULT_OUTPUT << 8U) |
            MOTION_HOLD_OPERATOR | MOTION_HOLD_CALIBRATE) ==
           g_motion_events[3].fault_holds);
    g_test_clock_us = 50000U;
    service_cycle(&input);
    assert(4U == g_motion_event_count);
}

static void check_latched_stop (uint32_t time_us, unsigned expected_captures)
{
    motion_status_t status;
    motion_feedback_t feedback;

    assert(1U == g_latch_attempts && time_us == g_latch_time_us);
    assert(expected_captures == g_capture_attempts);
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(status.stop_requested && !status.ready && !feedback.fresh);
    assert(MOTION_STOPPED == status.state && MOTION_FAULT_NONE == status.fault);
}

static void test_estop_wake (bool b_physical, bool b_queued)
{
    motion_input_t input;

    start_service(&input);
    if (b_queued)
    {
        g_test_clock_us = 1000U;
        stop_button_pulse();
        /* Periodic wake can race with an already queued urgent wake. */
        g_queued_wakes++;
    }
    else
    {
        g_b_inject_remote_in_sleep = !b_physical;
        g_b_inject_gpio_in_sleep = b_physical;
    }
    service_cycle(&input);
    check_latched_stop(1000U, 1U);
    assert(1U == g_wake_calls);
    assert(0U == g_pair_writes && 0U == g_motion_overruns);
}

static void test_estop_feedback_until_rest (void)
{
    motion_input_t input;
    motion_status_t status;
    motion_feedback_t feedback;
    motion_diag_t diag;

    start_service(&input);
    g_test_clock_us = 1000U;
    stop_button_pulse();
    service_cycle(&input);
    check_latched_stop(1000U, 1U);

    g_test_clock_us = 10000U;
    g_forward_edges = 12U;
    service_cycle(&input);
    assert(E_OK == motion_snapshot(&status, &feedback, &diag));
    assert(status.stop_requested && !status.ready);
    assert(status.stop_age_ms == 9U);
    assert(feedback.fresh && feedback.timestamp_us == 10000U);
    assert(feedback.speed_valid[0] && feedback.speed_valid[1]);
    assert(feedback.left_speed_um_s > 0 && feedback.right_speed_um_s > 0);
    assert(diag.left_duty_permille == 0 && diag.right_duty_permille == 0);

    g_test_clock_us = 20000U;
    g_b_capture_fails = true;
    service_cycle(&input);
    assert(E_OK == motion_feedback_snapshot(&feedback));
    assert(!feedback.fresh);

    g_test_clock_us = 30000U;
    g_b_capture_fails = false;
    service_cycle(&input);
    assert(E_OK == motion_feedback_snapshot(&feedback));
    assert(feedback.fresh && feedback.timestamp_us == 30000U);
    assert(feedback.speed_valid[0] && feedback.speed_valid[1]);
    assert(feedback.left_speed_um_s == 0 && feedback.right_speed_um_s == 0);
    assert(feedback.left_displacement_um > 0 && feedback.right_displacement_um > 0);
    assert(1U == g_latch_attempts && 0U == g_pair_writes);
}

static void test_estop_final_admission (bool b_request_stop)
{
    motion_input_t input;

    start_service(&input);
    assert(E_OK == motion_service_set_speed(125000, 125000, 200000U, false));
    g_stop_pin_reads = 0U;
    g_b_request_after_admission = b_request_stop;
    g_test_clock_us = 10000U;
    service_cycle(&input);

    if (b_request_stop)
    {
        assert(0U == g_pair_writes);
        check_latched_stop(10000U, 2U);
    }
    else
    {
        assert(1U == g_pair_writes && 1U == g_nonzero_pair_writes);
        assert(0U == g_latch_attempts);
    }
}

static void test_feedback_after_hold_release (void)
{
    motion_input_t input;
    motion_status_t status;
    motion_feedback_t feedback;

    g_test_clock_us = UINT32_MAX - 20000U;
    start_service(&input);
    assert(E_OK == motion_service_set_hold(MOTION_HOLD_OPERATOR, true));
    g_test_clock_us += 10000U;
    g_forward_edges = 12U;
    service_cycle(&input);
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(feedback.fresh && status.movement_sequence == 0U);
    assert(feedback.left_speed_um_s > 0 && feedback.right_speed_um_s > 0);
    uint32_t const held_stamp_us = feedback.timestamp_us;

    g_test_clock_us += 5000U;
    assert(E_OK == motion_service_set_hold(MOTION_HOLD_OPERATOR, false));
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(status.ready && 0U == status.hold_mask);
    assert(feedback.fresh && status.movement_sequence == 0U);
    assert(feedback.timestamp_us == held_stamp_us);

    g_test_clock_us += 10000U;
    g_test_clock_high++;
    g_forward_edges += 12U;
    service_cycle(&input);
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(feedback.fresh && status.movement_sequence == 0U);
    assert(feedback.timestamp_us == g_test_clock_us);

    g_test_clock_us += 10000U;
    g_forward_edges += 12U;
    service_cycle(&input);
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(feedback.fresh && status.movement_sequence == 1U);
    assert(feedback.timestamp_us == g_test_clock_us);

    g_test_clock_us += 1000U;
    stop_button_pulse();
    service_cycle(&input);
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(status.stop_requested && !status.ready && status.movement_sequence == 1U);
    g_test_clock_us += 10000U;
    g_forward_edges += 12U;
    service_cycle(&input);
    assert(E_OK == motion_snapshot(&status, &feedback, NULL));
    assert(feedback.fresh && status.movement_sequence == 1U);
    assert(feedback.left_speed_um_s > 0 && feedback.right_speed_um_s > 0);
}

/* Only renewal may release the supervisor hold, including after TIMERAWL wraps. */
static void test_expired_permit (void)
{
    uint32_t const renewed = g_test_clock_us;

    assert(E_OK == motion_service_renew_permit(200U));
    g_test_clock_us += 200001U;
    check_permit();
    assert(0U != (g_inbox.hold_mask & MOTION_HOLD_SUPERVISOR));
    g_test_clock_us = renewed + 1U; /* one complete timer cycle later */
    g_test_clock_high++;
    check_permit();
    assert(0U != (g_inbox.hold_mask & MOTION_HOLD_SUPERVISOR));
    assert(4294968U == motion_service_permit_age_ms());
    assert(E_BUSY == motion_service_set_speed(100000, 100000, 200000U, false));
    assert(E_OK == motion_service_renew_permit(200U));
    check_permit();
    assert(0U == (g_inbox.hold_mask & MOTION_HOLD_SUPERVISOR));
    assert(0U == motion_service_permit_age_ms());
    assert(E_OK == motion_service_set_speed(100000, 100000, 200000U, false));
}

static void test_hold_through_feedback_recovery (void)
{
    motion_input_t input;
    motion_status_t status;
    unsigned const writes = g_nonzero_pair_writes;

    g_test_clock_us += 10000U;
    g_b_capture_fails = true;
    service_cycle(&input);
    g_b_capture_fails = false;
    assert(E_OK == motion_status(&status) && MOTION_FAULT_FEEDBACK == status.fault);
    ER const hold = motion_service_set_hold(MOTION_HOLD_LIFT, true);

    for (uint32_t elapsed = 0U; elapsed < MOTION_FAULT_CLEAR_US; elapsed += 10000U)
    {
        g_test_clock_us += 10000U;
        assert(E_OK == motion_service_renew_permit(200U));
        service_cycle(&input);
    }
    assert(E_OK == motion_status(&status) && MOTION_IDLE == status.state);
    ER const request = motion_service_set_speed(100000, 100000, 200000U, false);
    g_test_clock_us += 10000U;
    service_cycle(&input);
    assert(writes == g_nonzero_pair_writes);
    assert(E_OK == hold && E_BUSY == request);
    assert(0U != (status.hold_mask & MOTION_HOLD_LIFT));
    assert(E_OK == motion_service_set_hold(MOTION_HOLD_LIFT, false));
}

/* A hold posted while a timing fault clears survives the clear, as for FEEDBACK. */
/* A running service pass, its permit and command, and a pass gap over the
 * 100 ms control gap that raises MOTION_FAULT_GAP. */
#define RECOVERY_PASS_US   (10000U)
#define RECOVERY_PERMIT_MS (200U)
#define RECOVERY_SPEED_UM_S (100000)
#define RECOVERY_TTL_US    (200000U)
#define RECOVERY_GAP_US    (150000U)

static void test_hold_through_timing_recovery (void)
{
    motion_input_t input;
    motion_status_t status;

    g_motion_event_count = 0U;
    g_test_clock_us += RECOVERY_PASS_US;
    assert(E_OK == motion_service_renew_permit(RECOVERY_PERMIT_MS));
    assert(E_OK == motion_service_set_speed(RECOVERY_SPEED_UM_S, RECOVERY_SPEED_UM_S, RECOVERY_TTL_US, false));
    service_cycle(&input);
    g_test_clock_us += RECOVERY_PASS_US;
    service_cycle(&input);
    assert(E_OK == motion_status(&status) && MOTION_STATE_RUNNING == status.state);
    g_test_clock_us += RECOVERY_GAP_US;
    assert(E_OK == motion_service_renew_permit(RECOVERY_PERMIT_MS));
    service_cycle(&input);
    assert(E_OK == motion_status(&status) && MOTION_FAULT_GAP == status.fault);
    assert(MOTION_STATE_FAULT == status.state);
    unsigned const writes = g_nonzero_pair_writes;
    ER const hold = motion_service_set_hold(MOTION_HOLD_LIFT, true);

    for (uint32_t elapsed = 0U; elapsed <= MOTION_FAULT_CLEAR_US; elapsed += RECOVERY_PASS_US)
    {
        g_test_clock_us += RECOVERY_PASS_US;
        assert(E_OK == motion_service_renew_permit(RECOVERY_PERMIT_MS));
        service_cycle(&input);
    }
    assert(E_OK == motion_status(&status) && MOTION_IDLE == status.state);
    assert(E_OK == hold);
    assert(0U != (status.hold_mask & MOTION_HOLD_LIFT));
    assert(writes == g_nonzero_pair_writes);
    assert(E_OK == motion_service_set_hold(MOTION_HOLD_LIFT, false));
}

static void test_estop_unavailable_irq (void)
{
    motion_service_config_t const config = {test_control_config(), 10U};
    motion_service_status_t status;

    assert(E_OK == motion_service_init(&config, 5));
    g_gpio_registration_result = E_LIMIT;
    g_b_expect_park = true;
    if (0 == setjmp(g_parked))
    {
        motion_service_run();
        abort();
    }
    assert(E_OK == motion_service_read_status(&status));
    assert(E_LIMIT == status.init_error && !status.ready);
    assert(MOTION_FAULT == status.control.state);
    assert(1U == g_cyclic_deletes && 1U == g_latch_attempts);
    assert(0U == g_pair_writes && NULL == g_stop_edge_handler);
}

int main(int argc, char * argv[])
{
    if (argc == 2 && strcmp(argv[1], "signed-snapshot") == 0)
    {
        test_signed_snapshot();
    }
    else if (argc == 2 && strcmp(argv[1], "feedback-age-boundary") == 0)
    {
        test_age_boundary();
    }
    else if (argc == 2 && strcmp(argv[1], "reject-delayed-feedback") == 0)
    {
        test_reject_delayed_feedback();
    }
    else if (argc == 2 && strcmp(argv[1], "estop-brake-failure") == 0)
    {
        test_estop_brake_failure(false);
    }
    else if (argc == 2 && strcmp(argv[1], "boot-estop-brake-failure") == 0)
    {
        test_estop_brake_failure(true);
    }
    else if (argc == 2 && strcmp(argv[1], "estop-remote-wake") == 0)
    {
        test_estop_wake(false, false);
    }
    else if (argc == 2 && strcmp(argv[1], "estop-gpio-wake") == 0)
    {
        test_estop_wake(true, false);
    }
    else if (argc == 2 && strcmp(argv[1], "estop-feedback-until-rest") == 0)
    {
        test_estop_feedback_until_rest();
    }
    else if (argc == 2 && strcmp(argv[1], "feedback-after-hold-release") == 0)
    {
        test_feedback_after_hold_release();
    }
    else if (argc == 2 && strcmp(argv[1], "estop-gpio-queued") == 0)
    {
        test_estop_wake(true, true);
    }
    else if (argc == 2 && strcmp(argv[1], "estop-final-admission") == 0)
    {
        test_estop_final_admission(true);
    }
    else if (argc == 2 && strcmp(argv[1], "normal-output-admission") == 0)
    {
        test_estop_final_admission(false);
        test_hold_through_feedback_recovery();
        test_hold_through_timing_recovery();
        test_expired_permit();
    }
    else if (argc == 2 && strcmp(argv[1], "estop-unavailable-irq") == 0)
    {
        test_estop_unavailable_irq();
    }
    else if (argc == 2 && strcmp(argv[1], "estop-request-status") == 0)
    {
        test_estop_request_status();
    }
    else if (argc == 2 && strcmp(argv[1], "state-events") == 0)
    {
        test_motion_state_events();
    }
    else
    {
        (void)fprintf(stderr, "Unknown motion feedback case\n");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
