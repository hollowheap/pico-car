/* Exercise the application's task and telemetry boundaries with a simulated car. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comms/telemetry/cmd.h"
#include "comms/telemetry/wire_schema.h"
#include "fakes/world.h"
#include "motion/motion.h"
#include "platform/chassis.h"
#include "supervisor/health_ids.h"

#define BOOT_READY_MS (3000U)
#define START_LIMIT_MS (4000U)
/* Includes the 20 ms supervisor and 10 ms motion task periods. */
#define STOP_LIMIT_MS (60U)
#define WHEEL_SETTLE_MS (350U)
#define START_TRAVEL_MM (75.0)

#define CHECK(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            (void)fprintf(stderr, "%s:%d at %u ms (x=%.1f mm, duty=%d/%d): %s\n", \
                          __FILE__, __LINE__, (unsigned)world_now_ms(), g_world.x_mm, \
                          (int)g_world.duty[0], (int)g_world.duty[1], #condition); \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

static const world_event_t * supervisor_event (uint8_t type, unsigned nth)
{
    return world_find_event(TELEMETRY_EVENT_SOURCE_SUPERVISOR, type, nth);
}

static void expect_ack (uint8_t type, unsigned nth, uint16_t id, uint8_t opcode)
{
    const world_event_t * const p_ack = supervisor_event(type, nth);

    CHECK(NULL != p_ack);
    CHECK(id == p_ack->p2);
    CHECK(opcode == (uint8_t)p_ack->p1);
}

static bool drive_stopped (void)
{
    return (0 == g_world.duty[0]) && (0 == g_world.duty[1]);
}

static bool drove_past_start_check (void)
{
    return (g_world.x_mm > START_TRAVEL_MM) && !drive_stopped();
}

static void boot_parked (void)
{
    world_boot();
    /* Start with the left line sensor on the tape. */
    g_world.y_mm = -((double)CHASSIS_LINE_SENSOR_SPACING_MM / 2.0);
    world_run_ms(BOOT_READY_MS);

    const world_event_t * const p_boot = supervisor_event(TELEMETRY_EVT_BOOT, 0U);

    CHECK((NULL != p_boot) && (0U == p_boot->p2));
    CHECK(NULL == supervisor_event(TELEMETRY_EVT_RUN_START, 0U));
    CHECK(drive_stopped());
    CHECK(0U == g_world.nonzero_duty_ms);
    CHECK(((1U << LIVE_TASKS) - 1U) == g_world.diag_alive_mask);
    CHECK((0U != g_world.heartbeats) && (0U != g_world.heartbeat_run_id));
    CHECK(WORLD_CAR_ID == g_world.heartbeat_car_id);
}

static void expect_started (uint16_t cmd_id)
{
    CHECK(world_run_until(drove_past_start_check, START_LIMIT_MS));
    CHECK(NULL != supervisor_event(TELEMETRY_EVT_RUN_START, 0U));
    CHECK(NULL == supervisor_event(TELEMETRY_EVT_RUN_STOP, 0U));
    expect_ack(TELEMETRY_EVT_CMD_ACCEPT, 0U, cmd_id, TELEMETRY_CMD_START);
}

static void mqtt_start_stop (void)
{
    boot_parked();
    expect_started(world_command(TELEMETRY_CMD_START));
    uint16_t const stop_id = world_command(TELEMETRY_CMD_STOP);

    CHECK(world_run_until(drive_stopped, STOP_LIMIT_MS));
    uint32_t const drive_time = g_world.nonzero_duty_ms;

    world_run_ms(WHEEL_SETTLE_MS);
    CHECK(!world_moving());
    CHECK(drive_time == g_world.nonzero_duty_ms);
    CHECK(NULL != supervisor_event(TELEMETRY_EVT_RUN_STOP, 0U));
    expect_ack(TELEMETRY_EVT_CMD_ACCEPT, 1U, stop_id, TELEMETRY_CMD_STOP);
}

static uint32_t state_field (unsigned offset, unsigned bytes)
{
    uint32_t value = 0U;

    for (unsigned i = 0U; i < bytes; ++i)
    {
        value = (value << 8U) | g_world.status_payload[offset + i];
    }
    return value;
}

static void periodic_state_bundle (void)
{
    boot_parked();
    expect_started(world_command(TELEMETRY_CMD_START));
    uint32_t const samples = g_world.status_frames;
    uint32_t const sample_us = state_field(TELEMETRY_STATUS_BUNDLE_OFF_DEVICE_SAMPLE_US, 4U);
    uint16_t const seq = (uint16_t)state_field(TELEMETRY_STATUS_BUNDLE_OFF_SEQ, 2U);

    world_run_ms(200U);
    CHECK(g_world.status_frames == samples + 1U);
    CHECK(0U == g_world.separate_motion_frames);
    CHECK(0U == g_world.separate_physics_frames);
    CHECK(TELEMETRY_STATUS_BUNDLE_WIRE_SIZE == g_world.status_wire_size);
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_DEVICE_SAMPLE_US, 4U) - sample_us == 200000U);
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_SEQ, 2U) == (uint16_t)(seq + 1U));
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_STATE, 1U) == TELEMETRY_RUN_STATE_RUNNING);
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_WHEEL_AGE_US, 2U) <= MOTION_FEEDBACK_MAX_AGE_US);
    uint32_t const left_travel = state_field(TELEMETRY_STATUS_BUNDLE_OFF_LEFT_TRAVEL_MM, 4U);
    uint32_t const right_travel = state_field(TELEMETRY_STATUS_BUNDLE_OFF_RIGHT_TRAVEL_MM, 4U);
    CHECK((left_travel > 0U) && (left_travel <= (uint32_t)INT32_MAX));
    CHECK((right_travel > 0U) && (right_travel <= (uint32_t)INT32_MAX));
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_FORWARD_ECHO_CLASS, 1U) == TELEMETRY_ECHO_CLASS_OPEN);
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_FORWARD_DIST_MM, 2U) == UINT16_MAX);
    CHECK(state_field(TELEMETRY_STATUS_BUNDLE_OFF_FORWARD_AGE_MS, 1U) < UINT8_MAX);

    /* Expose one real payload for offline replay through the bridge decoder. */
    (void)printf("status bundle ");
    for (unsigned i = 0U; i < sizeof(g_world.status_payload); ++i)
    {
        (void)printf("%02x", (unsigned)g_world.status_payload[i]);
    }
    (void)printf("\n");
}

static void button_start (void)
{
    boot_parked();
    g_world.start_pin_pressed = true;
    world_run_ms(100U);
    g_world.start_pin_pressed = false;
    expect_started(0U);
}

static void mqtt_rejects_stale_commands (void)
{
    boot_parked();
    world_run_ms(1500U);
    uint16_t const run_id = g_world.heartbeat_run_id;
    uint32_t const uptime_s = g_world.heartbeat_uptime_s;

    world_post_frame(1U, (uint16_t)(run_id ^ 1U), TELEMETRY_CMD_START, uptime_s);
    world_post_frame(1U, run_id, TELEMETRY_CMD_START, 0U);
    world_run_ms(1200U);
    CHECK(0U == g_world.nonzero_duty_ms);
    CHECK(NULL == supervisor_event(TELEMETRY_EVT_RUN_START, 0U));
    CHECK(1U == g_world.diag_rejected_run);
    CHECK(1U == g_world.diag_rejected_age);

    uint16_t const cmd_id = world_command(TELEMETRY_CMD_START);

    CHECK(1U == cmd_id);
    expect_started(cmd_id);
    double const x_mm = g_world.x_mm;

    world_post_frame(cmd_id, run_id, TELEMETRY_CMD_STOP, g_world.heartbeat_uptime_s);
    world_run_ms(1200U);
    CHECK(g_world.x_mm > (x_mm + 30.0));
    CHECK(NULL == supervisor_event(TELEMETRY_EVT_RUN_STOP, 0U));
    CHECK(1U == g_world.diag_rejected_old_id);
}

static bool stop_latched (void)
{
    return g_world.latched && drive_stopped();
}

static void mqtt_estop_keeps_latch (void)
{
    boot_parked();
    expect_started(world_command(TELEMETRY_CMD_START));
    uint16_t const estop_id = world_command(TELEMETRY_CMD_ESTOP);

    CHECK(world_run_until(stop_latched, STOP_LIMIT_MS));
    uint32_t const drive_time = g_world.nonzero_duty_ms;

    world_run_ms(WHEEL_SETTLE_MS);
    expect_ack(TELEMETRY_EVT_CMD_ACCEPT, 1U, estop_id, TELEMETRY_CMD_ESTOP);
    uint16_t const restart_id = world_command(TELEMETRY_CMD_START);

    world_run_ms(400U);
    expect_ack(TELEMETRY_EVT_CMD_REJECT, 0U, restart_id, TELEMETRY_CMD_START);
    CHECK(stop_latched());
    CHECK(!world_moving());
    CHECK(drive_time == g_world.nonzero_duty_ms);
    CHECK(NULL == supervisor_event(TELEMETRY_EVT_RUN_START, 1U));
}

static bool obstacle_reported (void)
{
    return NULL != world_find_event(TELEMETRY_EVENT_SOURCE_LINE,
                                    TELEMETRY_EVT_OBSTACLE_SEEN, 0U);
}

static void sonar_approach_stops_for_obstacle (void)
{
    double const box_x_mm = 500.0;

    boot_parked();
    world_add_box(box_x_mm, -170.0, box_x_mm + 120.0, 170.0);
    expect_started(world_command(TELEMETRY_CMD_START));
    CHECK(world_run_until(obstacle_reported, 4000U));
    CHECK(world_run_until(drive_stopped, STOP_LIMIT_MS));
    world_run_ms(WHEEL_SETTLE_MS);
    CHECK(drive_stopped());
    CHECK(!world_moving());
    CHECK(g_world.x_mm > START_TRAVEL_MM);
    CHECK((g_world.x_mm + CHASSIS_FRONT_TIP_MM) < box_x_mm);
}

int main (int argc, char ** argv)
{
    static const struct
    {
        const char * p_name;
        void (*run)(void);
    } cases[] = {
        {"boot_parked", boot_parked},
        {"mqtt_start_stop", mqtt_start_stop},
        {"periodic_state_bundle", periodic_state_bundle},
        {"button_start", button_start},
        {"mqtt_rejects_stale_commands", mqtt_rejects_stale_commands},
        {"mqtt_estop_keeps_latch", mqtt_estop_keeps_latch},
        {"sonar_approach_stops_for_obstacle", sonar_approach_stops_for_obstacle},
    };

    if (argc == 2)
    {
        for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++)
        {
            if (0 == strcmp(argv[1], cases[i].p_name))
            {
                cases[i].run();
                return EXIT_SUCCESS;
            }
        }
    }
    (void)fprintf(stderr, "usage: %s <case>\n", argv[0]);
    return EXIT_FAILURE;
}
