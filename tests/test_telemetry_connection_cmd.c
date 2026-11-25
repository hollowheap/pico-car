/* Exercise command dispatch and MQTT session changes through the connection task. */
#include <assert.h>
#include <setjmp.h>
#include <stddef.h>
#include <string.h>

#include "cyw43_utk.h"
#include "platform/faults.h"
#include "platform/log.h"
#include "platform/liveness.h"
#include "platform/watchdog.h"
#include "telemetry/connection.h"
#include "telemetry/emit.h"
#include "telemetry/mqtt.h"
#include "telemetry/net_queue.h"
#include "telemetry/queue.h"
#include "telemetry/tcp_transport.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define BOOT_ID (0xFFFFU)
#define COMMAND_ID (73U)
#define ESTOP_ID (COMMAND_ID + 1U)

static mqtt_command_fn_t g_mqtt_command;
static void * g_mqtt_context;
static bool g_b_locked;
static unsigned g_dispatches;
static unsigned g_estop_dispatches;
static jmp_buf  g_finish;
static unsigned g_pass;
static unsigned g_transport_open;
static unsigned g_transport_close;
static unsigned g_heartbeats;
static unsigned g_events;
static uint8_t  g_types[3];
static unsigned g_event_passes[3];
static bool     g_publish_failure;
static bool     g_fail_data;
static bool     g_data_pulled;
static bool     g_join_failure;

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

void mqtt_client_init (mqtt_client_t * p_client, const mqtt_config_t * p_config,
                       const mqtt_transport_t * p_transport,
                       mqtt_command_fn_t handler, void * p_context)
{
    (void)p_config;
    (void)p_transport;
    p_client->valid = true;
    g_mqtt_command = handler;
    g_mqtt_context = p_context;
}

uint32_t tcp_transport_now_ms (void)
{
    return g_pass * 100U;
}

void cyw43_utk_get_status (T_CYW43_UTK_STATUS * p_status)
{
    *p_status = (T_CYW43_UTK_STATUS){.link_up = true, .dhcp_complete = true};
    if (g_join_failure)
    {
        p_status->join_started = 1U;
        p_status->link_up = g_pass >= 5U;
        p_status->dhcp_complete = g_pass >= 6U;
        p_status->join_complete = (g_pass == 2U || g_pass == 3U || p_status->link_up);
        p_status->join_result = p_status->link_up ? 0 : (p_status->join_complete ? -1 : -5);
    }
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

INT telemetry_net_queue_pop (uint8_t * p_frame, TMO timeout)
{
    assert(timeout == TMO_POL);
    assert(!g_publish_failure || g_fail_data || g_pass != 2U);
    if (g_fail_data && g_pass == 2U)
    {
        assert(!g_data_pulled);
        g_data_pulled = true;
        (void)memset(p_frame, 0, TELEMETRY_EVENT_QUEUED_SIZE);
        p_frame[0] = TELEMETRY_TOPIC_EVENT;
        return TELEMETRY_EVENT_QUEUED_SIZE;
    }
    return E_TMOUT;
}

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
    if (mask == FAULT_MQTT_NACK_BIT)
    {
        assert(g_fail_data && g_pass == 2U && active == FAULT_MQTT_NACK_BIT);
    }
    else
    {
        assert(mask == FAULT_WIFI_DISCONN_BIT && active == 0U);
    }
}

bool tcp_transport_open (const char * p_host, uint16_t port)
{
    assert(p_host != NULL && port != 0U);
    assert(g_pass == 1U || g_pass >= 5U);
    g_transport_open++;
    return true;
}

void tcp_transport_close (void) { g_transport_close++; }
bool tcp_transport_close_pending (void) { return g_publish_failure && g_pass < 5U; }

mqtt_state_t mqtt_client_state (const mqtt_client_t * p_client)
{
    return p_client->state;
}

mqtt_result_t mqtt_client_poll (mqtt_client_t * p_client, uint32_t now_ms)
{
    static const mqtt_state_t states[] = {
        MQTT_STATE_DISCONNECTED, MQTT_STATE_WAIT_SUBACK, MQTT_STATE_CONNECTED,
        MQTT_STATE_CONNECTED, MQTT_STATE_DISCONNECTED, MQTT_STATE_WAIT_CONNACK,
        MQTT_STATE_CONNECTED, MQTT_STATE_CONNECTED,
    };

    assert(g_pass < sizeof(states) / sizeof(states[0]));
    assert(now_ms == tcp_transport_now_ms());
    if (g_publish_failure && g_pass == 3U)
    {
        assert(g_transport_close == 1U);
        p_client->state = MQTT_STATE_DISCONNECTED;
    }
    else
    {
        p_client->state = states[g_pass];
    }
    return MQTT_OK;
}

mqtt_result_t mqtt_client_publish (mqtt_client_t * p_client, const char * p_topic,
                                   const uint8_t * p_payload, uint16_t len, uint32_t now_ms)
{
    bool const b_heartbeat = (len == TELEMETRY_HEARTBEAT_WIRE_SIZE);

    assert(p_client->state == MQTT_STATE_CONNECTED);
    assert(p_topic != NULL && p_payload != NULL);
    assert((b_heartbeat || (g_fail_data && len == TELEMETRY_EVENT_WIRE_SIZE)) &&
           now_ms == tcp_transport_now_ms());
    if (b_heartbeat)
    {
        g_heartbeats++;
    }
    if (g_publish_failure && g_pass == 2U && b_heartbeat != g_fail_data)
    {
        p_client->state = MQTT_STATE_DISCONNECTED;
        return MQTT_ERR_TRANSPORT;
    }
    return MQTT_OK;
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

int tcp_transport_send (void * p_context, const uint8_t * p_data, uint16_t len)
{
    (void)p_context;
    (void)p_data;
    (void)len;
    return 0;
}

int tcp_transport_recv (void * p_context, uint8_t * p_data, uint16_t len)
{
    (void)p_context;
    (void)p_data;
    (void)len;
    return 0;
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

static void test_command_callback (void)
{
    uint8_t const frame[TELEMETRY_CMD_WIRE_SIZE] = {
        0U, COMMAND_ID, 0xFFU, 0xFFU, TELEMETRY_CMD_START, 0U, 0U, 0U, 0U,
    };
    uint8_t estop[TELEMETRY_CMD_WIRE_SIZE] = {
        0U, ESTOP_ID, 0U, 0xFFU, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 0U,
    };
    telemetry_cmd_state_t observed;

    assert(E_OK == connection_init(dispatch_command, &g_dispatches));
    assert(g_mqtt_command != NULL);
    connection_cmd_state(&observed);
    assert(BOOT_ID == observed.expected_run_id && !observed.has_last);
    g_mqtt_command(frame, TELEMETRY_CMD_WIRE_SIZE, g_mqtt_context);
    g_mqtt_command(frame, TELEMETRY_CMD_WIRE_SIZE - 1U, g_mqtt_context);
    connection_cmd_state(&observed);
    assert(1U == g_dispatches);
    assert(observed.has_last && COMMAND_ID == observed.last_cmd_id);
    assert(1U == observed.rejects.length);

    g_mqtt_command(estop, TELEMETRY_CMD_WIRE_SIZE, g_mqtt_context);
    assert(0U == g_estop_dispatches);
    estop[2] = 0xFFU;
    g_mqtt_command(estop, TELEMETRY_CMD_WIRE_SIZE, g_mqtt_context);
    g_mqtt_command(estop, TELEMETRY_CMD_WIRE_SIZE, g_mqtt_context);
    connection_cmd_state(&observed);
    assert(1U == g_estop_dispatches && 2U == g_dispatches);
    assert(ESTOP_ID == observed.last_cmd_id);
    assert(1U == observed.rejects.run && 1U == observed.rejects.old_id);
}

static void test_session_edges (void)
{
    if (setjmp(g_finish) == 0)
    {
        connection_run();
    }
    assert(g_events == 3U);
    assert(g_types[0] == TELEMETRY_EVT_LINK_UP && g_event_passes[0] == 2U);
    assert(g_types[1] == TELEMETRY_EVT_LINK_DOWN &&
           g_event_passes[1] == (g_publish_failure ? 2U : 4U));
    assert(g_types[2] == TELEMETRY_EVT_LINK_UP && g_event_passes[2] == 6U);
    assert(g_transport_close == 1U && g_transport_open == 2U);
    assert(g_heartbeats == 2U && connection_mqtt_seen());
    assert(g_data_pulled == g_fail_data);
}

int main (int argc, char ** argv)
{
    if (argc > 1 && strcmp(argv[1], "join-failure") == 0)
    {
        g_join_failure = true;
        if (setjmp(g_finish) == 0)
        {
            connection_run();
        }
        assert(g_transport_open == 1U);
        assert(g_transport_close == 1U);
        return 0;
    }
    g_publish_failure = (argc > 1);
    g_fail_data = g_publish_failure && strcmp(argv[1], "data-publish-failure") == 0;
    test_command_callback();
    test_session_edges();
    return 0;
}
