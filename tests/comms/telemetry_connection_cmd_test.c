/* Exercise command dispatch and MQTT session changes through the connection task. */
#include <assert.h>
#include <setjmp.h>
#include <stddef.h>
#include <string.h>

#include "cyw43_utk.h"
#include "mqtt_session.h"
#include "platform/faults.h"
#include "platform/log.h"
#include "platform/liveness.h"
#include "platform/drv_watchdog.h"
#include "comms/telemetry/connection.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/queue.h"
#include "comms/telemetry/drv_wifi_link.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define BOOT_ID (0xFFFFU)
#define COMMAND_ID (73U)
#define ESTOP_ID (COMMAND_ID + 1U)

static bool g_b_locked;
static unsigned g_dispatches;
static unsigned g_estop_dispatches;
static jmp_buf  g_finish;
static unsigned g_pass;
static bool g_b_started;
static unsigned g_commands;
static unsigned g_heartbeats;
static unsigned g_car_ids;
static unsigned g_events;
static uint8_t  g_types[3];
static unsigned g_event_passes[3];
static bool     g_heartbeat_busy;
static bool     g_join_failure;

#define RECOVERED_RSSI (-71)
#define CAR_ID (0xC10ABCF0U)

uint32_t shim_reg_read (uint32_t addr)
{
    (void)addr;
    return 1U;
}

ER ISpinLock (T_SPLOCK * p_lock, UINT * p_saved)
{
    assert(!g_b_locked);
    g_b_locked = true;
    *p_saved = 0U;
    return SpinLock(p_lock);
}

ER ISpinUnlock (T_SPLOCK * p_lock, UINT saved)
{
    (void)saved;
    assert(g_b_locked);
    g_b_locked = false;
    return SpinUnlock(p_lock);
}

int32_t mqtt_session_init (void) { return E_OK; }
void mqtt_session_set_boot_id (uint32_t boot_id) { assert(boot_id != 0U); }

void mqtt_session_start (int32_t task)
{
    assert(task == 1 && !g_b_started);
    g_b_started = true;
}

uint32_t mqtt_session_now_ms (void)
{
    return g_pass * 100U;
}

bool mqtt_session_connected (void)
{
    assert(g_b_started);
    return g_join_failure ? g_pass >= 6U : (g_pass >= 2U && g_pass < 4U) || g_pass >= 6U;
}

uint32_t mqtt_session_drops (void) { return 0U; }

void mqtt_session_set_car_id (uint32_t car_id)
{
    assert(car_id == CAR_ID);
    g_car_ids++;
}

bool mqtt_session_heartbeat (const uint8_t payload[TELEMETRY_HEARTBEAT_WIRE_SIZE])
{
    assert(mqtt_session_connected());
    if (g_join_failure)
    {
        assert(payload[TELEMETRY_HEARTBEAT_OFF_WIFI_RSSI] == (uint8_t)RECOVERED_RSSI);
    }
    assert(payload[TELEMETRY_HEARTBEAT_OFF_CAR_ID] == 0xC1U);
    assert(payload[TELEMETRY_HEARTBEAT_OFF_CAR_ID + 1U] == 0x0AU);
    assert(payload[TELEMETRY_HEARTBEAT_OFF_CAR_ID + 2U] == 0xBCU);
    assert(payload[TELEMETRY_HEARTBEAT_OFF_CAR_ID + 3U] == 0xF0U);
    if (g_heartbeat_busy && g_pass == 2U)
    {
        return false;
    }
    g_heartbeats++;
    return true;
}

bool mqtt_session_command (uint8_t payload[TELEMETRY_BARCODE_MAP_WIRE_SIZE], uint16_t * p_len)
{
    static const uint8_t commands[][TELEMETRY_CMD_WIRE_SIZE] = {
        {0U, COMMAND_ID, 0xFFU, 0xFFU, TELEMETRY_CMD_START, 0U, 0U, 0U, 0U},
        {0U, COMMAND_ID, 0xFFU, 0xFFU, TELEMETRY_CMD_START, 0U, 0U, 0U, 0U},
        {0U, ESTOP_ID, 0U, 0xFFU, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 0U},
        {0U, ESTOP_ID, 0xFFU, 0xFFU, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 0U},
        {0U, ESTOP_ID, 0xFFU, 0xFFU, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 0U},
    };
    static const unsigned arrivals[] = {2U, 2U, 3U, 4U, 4U};

    if (g_join_failure || g_commands == sizeof(arrivals) / sizeof(arrivals[0]) ||
        arrivals[g_commands] != g_pass)
    {
        return false;
    }
    (void)memcpy(payload, commands[g_commands], TELEMETRY_CMD_WIRE_SIZE);
    *p_len = (g_commands == 1U) ? 0U : TELEMETRY_CMD_WIRE_SIZE;
    g_commands++;
    return true;
}

void cyw43_utk_get_status (T_CYW43_UTK_STATUS * p_status)
{
    *p_status = (T_CYW43_UTK_STATUS){
        .link_up = true,
        .dhcp_complete = true,
        .netif_mac = {0x28U, 0xCDU, 0xC1U, 0x0AU, 0xBCU, 0xF0U},
    };
    if (g_join_failure)
    {
        p_status->join_started = 1U;
        p_status->link_up = g_pass >= 5U;
        p_status->dhcp_complete = g_pass >= 6U;
        p_status->join_complete = g_pass >= 2U;
        p_status->join_result = p_status->join_complete ? -1 : -5;
        p_status->link_rssi = INT16_MIN;
    }
}

wifi_join_t wifi_link_join (void)
{
    if (g_join_failure && (g_pass == 2U || g_pass == 3U))
    {
        return WIFI_JOIN_FAILED;
    }
    return (g_join_failure && g_pass == 4U) ? WIFI_JOIN_ACTIVE : WIFI_JOIN_IDLE;
}

wifi_lease_t wifi_link_lease (void)
{
    return (g_join_failure && g_pass < 6U) ? WIFI_LEASE_PENDING : WIFI_LEASE_READY;
}

int32_t wifi_link_rssi (void)
{
    return g_join_failure ? RECOVERED_RSSI : 0;
}

ID tk_get_tid (void) { return 1; }
ID tk_cre_cyc (const T_CCYC * p_config) { (void)p_config; return 1; }
ER tk_wup_tsk (ID task) { (void)task; return E_OK; }
INT tk_can_wup (ID task) { (void)task; return 0; }

ER tk_slp_tsk (TMO timeout)
{
    (void)timeout;
    if (g_join_failure)
    {
        static const connection_state_t expected[] = {
            CONNECTION_DISCONNECTED, CONNECTION_JOINING, CONNECTION_FAULT, CONNECTION_FAULT,
            CONNECTION_JOINING, CONNECTION_WAIT_DHCP, CONNECTION_READY, CONNECTION_READY,
        };
        assert(connection_state() == expected[g_pass]);
    }
    g_pass++;
    if (g_pass == 8U)
    {
        longjmp(g_finish, 1);
    }
    return E_OK;
}

void liveness_kick (uint8_t task) { (void)task; }
uint8_t platform_reset_reason (void) { return 0U; }
UW telemetry_queue_drop_count (void) { return 0U; }
UW telemetry_net_queue_drop_count (void) { return 0U; }

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod,
                const char * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

void system_fault_assign (fault_owner_t owner, uint32_t mask, uint32_t active)
{
    assert(owner == FAULT_OWNER_LINK);
    assert(mask == FAULT_WIFI_DISCONN_BIT && active == 0U);
}

ER telemetry_try_emit_event (uint8_t source, uint8_t type, int16_t p1,
                             uint16_t p2, uint32_t odometer_mm)
{
    assert(!g_b_locked);
    assert(source == TELEMETRY_EVENT_SOURCE_CONNECTION);
    assert(p1 == 0 && p2 == 0U && odometer_mm == 0U);
    assert(g_events < sizeof(g_types) / sizeof(g_types[0]));
    g_types[g_events]        = type;
    g_event_passes[g_events] = g_pass;
    g_events++;

    /* Simulate a busy queue on DOWN. The connection must proceed once. */
    return type == TELEMETRY_EVT_LINK_DOWN ? E_TMOUT : E_OK;
}

telemetry_cmd_result_t __real_telemetry_cmd_receive (
    telemetry_cmd_state_t * p_state, const uint8_t * p_src, uint16_t len,
    uint32_t now_s, telemetry_cmd_dispatch_fn_t dispatch, void * p_context);

telemetry_cmd_result_t __wrap_telemetry_cmd_receive (
    telemetry_cmd_state_t * p_state, const uint8_t * p_src, uint16_t len,
    uint32_t now_s, telemetry_cmd_dispatch_fn_t dispatch, void * p_context)
{
    telemetry_cmd_state_t const before = *p_state;
    telemetry_cmd_state_t observed;

    assert(!g_b_locked);
    /* Let a reader run after the first command-state store. The real decoder
     * below still handles validation, counters and dispatch. */
    if (TELEMETRY_CMD_WIRE_SIZE == len)
    {
        p_state->last_cmd_id++;
    }
    else
    {
        p_state->rejects.length++;
    }
    connection_cmd_state(&observed);
    assert(before.last_cmd_id == observed.last_cmd_id);
    assert(before.has_last == observed.has_last);
    assert(before.rejects.length == observed.rejects.length);
    *p_state = before;
    return __real_telemetry_cmd_receive(p_state, p_src, len, now_s, dispatch, p_context);
}

static void dispatch_command (const telemetry_cmd_t * p_cmd, void * p_context)
{
    telemetry_cmd_state_t observed;

    assert(!g_b_locked);
    assert(p_context == &g_dispatches);
    connection_cmd_state(&observed);
    assert(observed.has_last);
    if (TELEMETRY_CMD_ESTOP == p_cmd->opcode)
    {
        /* Dispatch is the stop handoff. It must run before the command
         * image takes the lock shared with telemetry. */
        assert(COMMAND_ID == observed.last_cmd_id);
        g_estop_dispatches++;
    }
    else
    {
        assert(observed.last_cmd_id == p_cmd->cmd_id);
    }
    g_dispatches++;
}

static void run_connection (void)
{
    if (setjmp(g_finish) == 0)
    {
        connection_run();
    }
}

int main (int argc, char ** argv)
{
    telemetry_cmd_state_t observed;

    g_join_failure = (argc > 1 && strcmp(argv[1], "join-failure") == 0);
    g_heartbeat_busy = (argc > 1 && strcmp(argv[1], "heartbeat-busy") == 0);
    assert(E_OK == connection_init(dispatch_command, &g_dispatches));
    connection_cmd_state(&observed);
    assert(BOOT_ID == observed.expected_run_id && !observed.has_last);
    run_connection();
    assert(connection_mqtt_seen());
    assert(g_car_ids == 1U);
    if (g_join_failure)
    {
        assert(g_events == 1U);
        assert(g_types[0] == TELEMETRY_EVT_LINK_UP && g_event_passes[0] == 6U);
        assert(g_heartbeats == 1U);
        return 0;
    }
    assert(g_events == 3U);
    assert(g_types[0] == TELEMETRY_EVT_LINK_UP && g_event_passes[0] == 2U);
    assert(g_types[1] == TELEMETRY_EVT_LINK_DOWN && g_event_passes[1] == 4U);
    assert(g_types[2] == TELEMETRY_EVT_LINK_UP && g_event_passes[2] == 6U);
    assert(g_heartbeats == 2U);
    connection_cmd_state(&observed);
    assert(1U == g_estop_dispatches && 2U == g_dispatches);
    assert(observed.has_last && ESTOP_ID == observed.last_cmd_id);
    assert(1U == observed.rejects.length);
    assert(1U == observed.rejects.run && 1U == observed.rejects.old_id);
    return 0;
}
