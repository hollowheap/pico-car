/** @file buddy2_speed_check.c
 * @brief Single-start, duty-capped PI trial using the real motor-owner service.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "motion_service.h"
#include "motor_pwm.h"
#include "speed_probe.h"
#include "usb_console_compat.h"

#define START_PIN 20U
#define STOP_PIN 21U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)
#define TRACE_ROWS 192U
#define TRACE_SPACING_US UINT32_C(20000)

typedef struct
{
    uint32_t timestamp_us;
    uint32_t interval_us;
    int32_t velocity[2];
    int16_t duty[2];
    uint32_t invalid[2];
    uint8_t flags;
} speed_row_t;

static speed_row_t trace[TRACE_ROWS];
static unsigned int row_count;
static unsigned int rows_dropped;
static uint32_t last_sample_us;
static uint32_t max_owner_interval_us;
static uint32_t max_sample_age_us;
static bool has_sample;

static void record_sample (const motion_service_status_t * p_status, uint32_t now)
{
    const motion_sample_t * p_sample = &p_status->control.sample;
    unsigned int wheel;
    speed_row_t row = {0};

    if (!p_status->control.sample_available || p_status->control.command_id == 0U)
    {
        return;
    }
    if (p_sample->elapsed_us > max_owner_interval_us)
    {
        max_owner_interval_us = p_sample->elapsed_us;
    }
    if ((uint32_t)(now - p_sample->timestamp_us) > max_sample_age_us)
    {
        max_sample_age_us = now - p_sample->timestamp_us;
    }
    if (has_sample && ((uint32_t)(p_sample->timestamp_us - last_sample_us) < TRACE_SPACING_US))
    {
        return;
    }
    row.timestamp_us = p_sample->timestamp_us;
    row.interval_us = p_sample->elapsed_us;
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        row.velocity[wheel] = p_sample->wheel[wheel].velocity_um_s;
        row.duty[wheel] = p_status->control.duty[wheel];
        row.invalid[wheel] = p_sample->wheel[wheel].invalid_delta;
        if (p_sample->wheel[wheel].speed_valid)
        {
            row.flags |= (uint8_t)(1U << wheel);
        }
        if (p_sample->wheel[wheel].odometry_valid)
        {
            row.flags |= (uint8_t)(1U << (wheel + 2U));
        }
        if (p_sample->wheel[wheel].scale_verified)
        {
            row.flags |= (uint8_t)(1U << (wheel + 4U));
        }
    }
    has_sample = true;
    last_sample_us = p_sample->timestamp_us;
    if (row_count < TRACE_ROWS)
    {
        trace[row_count++] = row;
    }
    else
    {
        rows_dropped++;
    }
}

static void report (const speed_probe_t * p_probe,
                     const motion_service_status_t * p_status, ER api_error,
                     uint32_t stop_wait_us)
{
    unsigned int row;

    tm_printf((UB *)"SPEED TRACE BEGIN rows=%u dropped=%u\n", row_count, rows_dropped);
    tm_printf((UB *)"END state=%u fault=%u init=%d api=%d interrupted=%u producer_gap_fault=%u brake_failed=%u\n",
        (unsigned int)p_status->control.state, (unsigned int)p_status->control.fault,
        p_status->init_error, api_error, p_probe->interrupted ? 1U : 0U,
        p_probe->gap_fault ? 1U : 0U, p_status->control.brake_failed ? 1U : 0U);
    tm_printf((UB *)"SET target_um_s=%d cap_permille=%u kp_q16=64 ki_q16=256 kd_q16=0 start_us=%u duration_us=%u\n",
        SPEED_PROBE_TARGET_UM_S, SPEED_PROBE_DUTY_CAP, p_probe->started_us,
        SPEED_PROBE_DURATION_US);
    tm_printf((UB *)"TIMING max_owner_interval_us=%u max_sample_age_us=%u max_producer_gap_us=%u stop_ack_wait_us=%u\n",
        max_owner_interval_us, max_sample_age_us, p_probe->max_gap_us, stop_wait_us);
    tm_printf((UB *)"ROW sample_us dt_us left_um_s right_um_s left_permille right_permille flags bad_left bad_right\n");
    for (row = 0U; row < row_count; row++)
    {
        tm_printf((UB *)"DATA %u %u %d %d %d %d %u %u %u\n",
            trace[row].timestamp_us, trace[row].interval_us,
            trace[row].velocity[0], trace[row].velocity[1],
            (int)trace[row].duty[0], (int)trace[row].duty[1],
            (unsigned int)trace[row].flags, trace[row].invalid[0], trace[row].invalid[1]);
        /* Pace the frozen trace so the USB ring can drain. Motors have
         * already reached a terminal, latched output state before printing. */
        (void)tk_dly_tsk(10);
    }
    tm_printf((UB *)"SPEED TRACE COMPLETE rows=%u; latched until reboot\n", row_count);
}

static void probe_task (INT start_code, void * p_extra)
{
    motion_service_status_t status = {0};
    speed_probe_t probe;
    ER api_error = E_OK;
    uint32_t ready_us = 0U;
    uint32_t stop_requested_us;
    unsigned int wait;

    (void)start_code;
    (void)p_extra;
    speed_probe_init(&probe);
    (void)gpio_set_pin(START_PIN, GPIO_MODE_IN);
    clr_w(GPIO(START_PIN), GPIO_PDE);
    set_w(GPIO(START_PIN), GPIO_PUE | GPIO_SHEMITT);
    for (wait = 0U; wait < 200U && tm_usb_state() < 2U; wait++)
    {
        (void)tk_dly_tsk(100);
    }
    tm_printf((UB *)"\nBuddy2 PI SPEED CHECK; WHEELS LIFTED; USB bench\n");
    tm_printf((UB *)"Experimental50mm/s, 20percent cap, 3seconds; D off; scales UNVERIFIED\n");
    tm_printf((UB *)"GP20 once to start; GP21 latched STOP; no carrier M1/M2 buttons\n");
    for (;;)
    {
        uint32_t now = in_w(TIMER_RAW_LOW);
        speed_probe_phase_t previous = probe.phase;
        bool stop;

        api_error = motion_service_read_status(&status);
        stop = (api_error != E_OK) || status.init_error != E_OK ||
            status.control.state == MOTION_FAULT ||
            status.control.state == MOTION_STOPPED || gpio_get_val(STOP_PIN) == 0U;
        if (probe.phase == SPEED_PROBE_RUN)
        {
            record_sample(&status, now);
        }
        if (status.ready || stop || probe.phase != SPEED_PROBE_WAIT)
        {
            (void)speed_probe_update(&probe, now, gpio_get_val(START_PIN) == 0U, stop);
        }
        if (probe.phase == SPEED_PROBE_RUN)
        {
            if (previous == SPEED_PROBE_WAIT)
            {
                tm_printf((UB *)"START accepted; bounded PI trial\n");
                /* Start the run clock after acknowledgement, before the
                 * first command; no printing throughout commanded motion. */
                now = in_w(TIMER_RAW_LOW);
                probe.started_us = now;
                probe.previous_us = now;
            }
            if (gpio_get_val(STOP_PIN) == 0U)
            {
                (void)speed_probe_update(&probe, now, false, true);
            }
            else
            {
                api_error = motion_service_set_speed(SPEED_PROBE_TARGET_UM_S,
                    SPEED_PROBE_TARGET_UM_S, SPEED_PROBE_TTL_US);
                if (api_error != E_OK)
                {
                    (void)speed_probe_update(&probe, now, false, true);
                }
            }
        }
        if (probe.phase == SPEED_PROBE_FINISHED)
        {
            break;
        }
        if (probe.phase == SPEED_PROBE_WAIT && (uint32_t)(now - ready_us) >= 1000000U)
        {
            tm_printf((UB *)"READY speed trial service=%u GP20=%u GP21=%u\n",
                status.ready ? 1U : 0U, gpio_get_val(START_PIN), gpio_get_val(STOP_PIN));
            ready_us = now;
        }
        (void)tk_dly_tsk(10);
    }
    stop_requested_us = in_w(TIMER_RAW_LOW);
    (void)motion_service_request_stop();
    /* No trace printing until the owner acknowledges terminal output. An
     * owner/CPU failure is not covered by this software-only stop mechanism. */
    for (;;)
    {
        (void)motion_service_read_status(&status);
        if (status.control.state == MOTION_STOPPED || status.control.state == MOTION_FAULT)
        {
            break;
        }
        (void)tk_dly_tsk(10);
    }
    stop_requested_us = in_w(TIMER_RAW_LOW) - stop_requested_us;
    for (;;)
    {
        report(&probe, &status, api_error, stop_requested_us);
        (void)tk_dly_tsk(2000);
    }
}

static void startup_task (INT start_code, void * p_extra)
{
    motion_service_config_t config;
    T_CTSK task_definition = {
        .task = probe_task, .itskpri = 6, .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC2
    };
    ID task;
    ER result;

    (void)start_code;
    (void)p_extra;
    config.control = speed_probe_configuration();
    config.poll_delay_ms = 10U;
    result = motion_service_start(&config);
    if (result == E_OK)
    {
        task = tk_cre_tsk(&task_definition);
        result = task <= E_OK ? (ER)task : tk_sta_tsk(task, 0);
    }
    if (result != E_OK)
    {
        (void)motion_service_request_stop();
        tm_printf((UB *)"SPEED CHECK startup failed=%d; no motion command sent\n", result);
    }
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

EXPORT INT usermain (void)
{
    T_CTSK task_definition = {
        .task = startup_task, .itskpri = 7, .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1
    };
    ID task;

    motor_pwm_boot_brake();
    task = tk_cre_tsk(&task_definition);
    if (task <= E_OK || tk_sta_tsk(task, 0) != E_OK)
    {
        return 1;
    }
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
