/** Feed measured roll and encoder feedback through the firmware line task. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "navigation/obstacle_flags.h"
#include "navigation/obstacle_ping.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "supervisor/command.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"
#include "track/line_task.h"
#include "track/track.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define TICK_MS 10U
#define LAST_TICK 60U
#define TURN_TICK 12U
#define SONAR_FAULT_TICK 16U

static jmp_buf g_finished;
static unsigned g_tick;
static int16_t g_roll_cdeg;
static bool g_slow_left;
static int32_t g_left_target;
static int32_t g_right_target;
static bool g_turn_case;
static bool g_turning;
static bool g_began_turn;
static bool g_moved_after_fault;
static unsigned g_turn_abort_tick;

ER tk_get_otm(SYSTIM * p_time)
{
    *p_time = (SYSTIM){0, TICK_MS * g_tick};
    return E_OK;
}

int motion_feedback_snapshot(motion_feedback_t * p_feedback)
{
    int32_t const left = g_slow_left ? 100000 : 125000;
    int32_t const right = g_slow_left ? 125000 : 100000;

    *p_feedback = (motion_feedback_t){
        .left_speed_um_s = left, .right_speed_um_s = right,
        .left_displacement_um = (int64_t)left * g_tick / 100,
        .right_displacement_um = (int64_t)right * g_tick / 100,
        .speed_valid = {true, true}, .odometry_valid = {true, true},
        .timestamp_us = TICK_MS * 1000U * g_tick, .fresh = true,
    };
    return E_OK;
}

int motion_set_speed(int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    assert(ttl_us > 0U);
    g_left_target = left_um_s;
    g_right_target = right_um_s;
    if (g_turning && (left_um_s != 0 || right_um_s != 0))
    {
        g_began_turn = true;
        g_moved_after_fault = g_moved_after_fault || (g_tick >= SONAR_FAULT_TICK);
    }
    return E_OK;
}

int motion_status(motion_status_t * p_status)
{
    *p_status = (motion_status_t){.ready = true};
    return E_OK;
}

ER physics_snapshot(physics_status_t * p_status)
{
    *p_status = (physics_status_t){
        .state = TELEMETRY_HUMP_STATE_FLAT, .ready = true, .calibrated = true,
        .roll_cdeg = g_roll_cdeg,
    };
    return E_OK;
}

ir_sensors_state_t ir_get_state(void)
{
    return (ir_sensors_state_t){.left = true, .right = true};
}

ER ir_wait_change(uint8_t type, TMO timeout)
{
    assert(IR_TYPE_LINE == type);
    assert((TMO)TICK_MS == timeout);
    if (++g_tick == LAST_TICK)
    {
        longjmp(g_finished, 1);
    }
    return E_TMOUT;
}

bool ir_barcode_recent(void)
{
    return false;
}

uint8_t supervisor_take_turn(uint8_t * p_source)
{
    if (g_turn_case && g_tick == TURN_TICK)
    {
        *p_source = VEHICLE_SRC_BARCODE;
        return VEHICLE_CMD_LEFT;
    }
    return SUPERVISOR_NO_TURN;
}

void supervisor_turn_refused(uint8_t opcode)
{
    (void)opcode;
    assert(false);
}

void supervisor_set_turning(bool b_turning)
{
    g_turning = b_turning;
}

uint8_t supervisor_reset_count(void)
{
    return 0U;
}

uint8_t supervisor_start_count(void)
{
    return 1U;
}

uint32_t obstacle_ping_read(void)
{
    uint32_t const sequence = (g_tick / 6U) & OBSTACLE_PING_SEQ_MK;

    return (OBSTACLE_CLASS_OPEN << OBSTACLE_PING_CLASS_SH) |
           (sequence << OBSTACLE_PING_SEQ_SH) |
           ((uint32_t)OBSTACLE_PING_SERVO_OFFSET << OBSTACLE_PING_SERVO_SH);
}

ER obstacle_ping_request(int16_t servo_deci, uint8_t seq)
{
    (void)servo_deci;
    (void)seq;
    assert(false);
    return E_OK;
}

uint16_t obstacle_get_faults(void)
{
    return (g_turn_case && g_tick >= SONAR_FAULT_TICK) ? OBSTACLE_FAULT_NO_ECHO : 0U;
}

void liveness_kick(uint8_t task_bit)
{
    assert(LIVE_LINE == task_bit);
}

void system_fault_assign(fault_owner_t owner, uint32_t mask, uint32_t active)
{
    (void)owner;
    (void)mask;
    assert(g_turn_case || 0U == active);
}

uint32_t telemetry_odometer_mm(const motion_feedback_t * p_feedback)
{
    (void)p_feedback;
    return 0U;
}

ER telemetry_emit_event(uint8_t source, uint8_t type, int16_t p1,
                        uint16_t p2, uint32_t odometer_mm)
{
    (void)source;
    if (type == TELEMETRY_EVT_TURN_ABORT)
    {
        g_turn_abort_tick = g_tick;
    }
    (void)p1;
    (void)p2;
    (void)odometer_mm;
    return E_OK;
}

void log_write(log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

static int32_t run_line(bool b_left, int16_t roll_cdeg)
{
    g_tick = 0U;
    g_slow_left = b_left;
    g_roll_cdeg = roll_cdeg;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_left_target > 0 && g_right_target > 0);
    return b_left ? (g_left_target - g_right_target) : (g_right_target - g_left_target);
}

static int turn_aborts_on_sonar(void)
{
    g_turn_case = true;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    if (!g_began_turn || g_moved_after_fault || g_turn_abort_tick != SONAR_FAULT_TICK)
    {
        (void)fprintf(stderr, "Sonar fault failed to abort the active turn at tick %u\n",
                      SONAR_FAULT_TICK);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int main(int argc, char * argv[])
{
    bool b_left;
    int32_t flat;
    int32_t climb;

    if (argc == 2 && strcmp(argv[1], "turn-sonar-fault") == 0)
    {
        return turn_aborts_on_sonar();
    }
    if (argc != 2 || (strcmp(argv[1], "left") != 0 && strcmp(argv[1], "right") != 0))
    {
        (void)fprintf(stderr, "Usage: %s left|right|turn-sonar-fault\n", argv[0]);
        return EXIT_FAILURE;
    }
    b_left = strcmp(argv[1], "left") == 0;
    flat = run_line(b_left, 0);
    climb = run_line(b_left, b_left ? -800 : 800);
    if (climb <= flat)
    {
        (void)fprintf(stderr, "Climbing wheel demand difference %ld vs flat %ld um/s\n",
                      (long)climb, (long)flat);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
