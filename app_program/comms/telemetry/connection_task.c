/** @file connection_task.c
 * @brief Core 1 command ingress, heartbeat assembly, and link reporting.
 */
#include <tk/tkernel.h>
#include <tk/smp.h>

#include "drv_wifi.h"

#include "mqtt_session.h"

#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/drv_watchdog.h"
#include "supervisor/health_ids.h"
#include "platform/log.h"
#include "comms/telemetry/car_id.h"
#include "comms/telemetry/cmd.h"
#include "comms/telemetry/connection.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/packet.h"
#include "comms/telemetry/queue.h"
#include "comms/telemetry/drv_wifi_link.h"

#define CONNECTION_POLL_MS      100U
#define CONNECTION_LOOP_MS      10U
#define CONNECTION_HEARTBEAT_MS 1000U
#define CONNECTION_STATUS_MS    1000U
#define CONNECTION_MS_PER_S     1000U
#define CONNECTION_RSSI_MAX     127
#define CONNECTION_RSSI_MIN     (-128)
#define CONNECTION_DROPS_MAX    255U
#define CONNECTION_COMMANDS_PER_PASS 16U
/* No MQTT session for this long sets the wifi_disconn fault bit. */
#define CONNECTION_LINK_DOWN_MS 3000U

static volatile connection_state_t g_state = CONNECTION_DISCONNECTED;
static volatile bool               g_b_mqtt_seen;
static T_SPLOCK                    g_cmd_lock;
static telemetry_cmd_state_t       g_cmd_state;
static connection_cmd_dispatch_t   gp_dispatch;
static void *                      gp_dispatch_context;
static bool                        g_mqtt_ready;
static uint32_t                    g_heartbeat_ms;
static drv_wifi_status_t           g_status;
static uint32_t                    g_car_id;

static void
publish_cmd_state (const telemetry_cmd_state_t * p_state)
{
    UINT saved = 0U;

    (void)ISpinLock(&g_cmd_lock, &saved);
    g_cmd_state = *p_state;
    (void)ISpinUnlock(&g_cmd_lock, saved);
}

static void
dispatch_command (const telemetry_cmd_t * cmd, void * context)
{
    bool const b_estop = ((uint8_t)TELEMETRY_CMD_ESTOP == cmd->opcode);

    if (!b_estop)
    {
        publish_cmd_state(context);
    }
    if (gp_dispatch != 0)
    {
        gp_dispatch(cmd, gp_dispatch_context);
    }
    if (b_estop)
    {
        /* Hand off the stop before taking the lock shared with diagnostics. */
        publish_cmd_state(context);
    }
}

static void
on_mqtt_command (const uint8_t * payload, uint16_t len, uint32_t now_ms)
{
    /* Only the connection task updates the state after startup. */
    telemetry_cmd_state_t next = g_cmd_state;

    /* Use the clock and units of the heartbeat uptime_s the command echoes.
       The dispatch callback publishes accepted commands. */
    if (TELEMETRY_CMD_OK !=
        telemetry_cmd_receive(&next, payload, len,
                              now_ms / CONNECTION_MS_PER_S,
                              dispatch_command, &next))
    {
        publish_cmd_state(&next);
    }
}

static uint32_t
add_saturating (uint32_t a, uint32_t b)
{
    return (a > (UINT32_MAX - b)) ? UINT32_MAX : (a + b);
}

static void
publish_heartbeat (uint32_t now_ms)
{
    uint8_t               payload[TELEMETRY_HEARTBEAT_WIRE_SIZE];
    telemetry_heartbeat_t heartbeat;
    uint32_t              drops = 0u;
    int32_t               rssi = wifi_link_rssi();

    if ((uint32_t) (now_ms - g_heartbeat_ms) < CONNECTION_HEARTBEAT_MS)
    {
        return;
    }
    if (rssi > CONNECTION_RSSI_MAX)
    {
        rssi = CONNECTION_RSSI_MAX;
    }
    else if (rssi < CONNECTION_RSSI_MIN)
    {
        rssi = CONNECTION_RSSI_MIN;
    }
    drops = add_saturating((uint32_t) telemetry_queue_drop_count(),
                           (uint32_t) telemetry_net_queue_drop_count());
    drops = add_saturating(drops, mqtt_session_drops());
    if (drops > CONNECTION_DROPS_MAX)
    {
        drops = CONNECTION_DROPS_MAX;
    }
    /* The heartbeat is only sent while MQTT is connected, so the link state
       is always "mqtt". The reset reason is filled by the boot checks. */
    heartbeat.uptime_s     = now_ms / CONNECTION_MS_PER_S;
    heartbeat.drops        = (uint8_t) drops;
    heartbeat.wifi_rssi    = (int8_t) rssi;
    heartbeat.reset_reason = platform_reset_reason ();
    heartbeat.link_state   = (uint8_t) TELEMETRY_LINK_STATE_MQTT;
    heartbeat.run_id       = g_cmd_state.expected_run_id;
    heartbeat.car_id       = g_car_id;
    if (telemetry_pack_heartbeat(payload, &heartbeat) == 0u)
    {
        return;
    }
    if (mqtt_session_heartbeat(payload))
    {
        g_heartbeat_ms = now_ms;
    }
}

static connection_state_t
classify (const drv_wifi_status_t * status, wifi_join_t join, wifi_lease_t lease)
{
    if (status->init_result != 0)
    {
        return CONNECTION_FAULT;
    }
    if (!status->link_up)
    {
        switch (join)
        {
        case WIFI_JOIN_ACTIVE:
            return CONNECTION_JOINING;
        case WIFI_JOIN_FAILED:
            return CONNECTION_FAULT;
        case WIFI_JOIN_IDLE:
        default:
            break;
        }
        if (status->join_complete)
        {
            return (status->join_result != 0) ? CONNECTION_FAULT
                                              : CONNECTION_DISCONNECTED;
        }
        return status->join_started ? CONNECTION_JOINING
                                    : CONNECTION_DISCONNECTED;
    }
    switch (lease)
    {
    case WIFI_LEASE_READY:
        return CONNECTION_READY;
    case WIFI_LEASE_TIMEOUT:
        return CONNECTION_FAULT;
    case WIFI_LEASE_PENDING:
    default:
        break;
    }
    return CONNECTION_WAIT_DHCP;
}

ER
connection_init (connection_cmd_dispatch_t dispatch, void * context)
{
    if (g_mqtt_ready)
    {
        return E_OK;
    }
    ER const result = mqtt_session_init();

    if (result != E_OK)
    {
        return result;
    }
    /* A new run ID at each boot: a host must read it from the heartbeat,
       so a command for an earlier boot is refused. */
    (void)InitSpinLock(&g_cmd_lock);
    telemetry_cmd_state_init(&g_cmd_state, drv_wifi_boot_run_id());
    gp_dispatch         = dispatch;
    gp_dispatch_context = context;
    g_mqtt_ready        = true;
    return E_OK;
}

static void
note_mqtt_session (bool b_connected)
{
    uint8_t const type = b_connected ? TELEMETRY_EVT_LINK_UP : TELEMETRY_EVT_LINK_DOWN;

    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_CONNECTION,
                                   type, 0, 0U, 0U);
}

static bool
service_mqtt (uint32_t now_ms, bool b_mqtt_up)
{
    uint8_t command[TELEMETRY_CMD_WIRE_SIZE];
    uint16_t len = 0U;
    bool const b_connected = mqtt_session_connected();

    /* Commands queued before a disconnect have already reached the firmware. */
    for (unsigned n = 0U; n < CONNECTION_COMMANDS_PER_PASS; n++)
    {
        if (!mqtt_session_command(command, &len))
        {
            break;
        }
        on_mqtt_command(command, len, now_ms);
    }

    if (b_connected != b_mqtt_up)
    {
        note_mqtt_session(b_connected);
    }
    if (b_connected)
    {
        g_b_mqtt_seen = true;
        if (!b_mqtt_up)
        {
            g_heartbeat_ms = now_ms - CONNECTION_HEARTBEAT_MS;
        }
        publish_heartbeat(now_ms);
    }
    return b_connected;
}

void
connection_run (void)
{
    connection_state_t previous = CONNECTION_DISCONNECTED;
    bool              b_mqtt_up = false;
    uint32_t           last_poll_ms = 0u;
    uint32_t           last_status_ms = 0u;
    uint32_t           last_mqtt_up_ms = mqtt_session_now_ms();
    bool               first_status = true;

    mqtt_session_start(tk_get_tid());
    for (;;)
    {
        const uint32_t now_ms = mqtt_session_now_ms();

        liveness_kick(LIVE_CONNECTION);

        if ((uint32_t) (now_ms - last_poll_ms) >= CONNECTION_POLL_MS)
        {
            connection_state_t next = CONNECTION_DISCONNECTED;

            last_poll_ms = now_ms;
            drv_wifi_read_status(&g_status);
            if (g_car_id == 0U)
            {
                g_car_id = car_id_from_mac(g_status.netif_mac);
                mqtt_session_set_car_id(g_car_id);
            }
            wifi_join_t const join = wifi_link_join();
            wifi_lease_t const lease = wifi_link_lease();
            next = classify(&g_status, join, lease);
            if ((next != previous) || first_status ||
                ((next != CONNECTION_READY) &&
                 ((uint32_t) (now_ms - last_status_ms) >= CONNECTION_STATUS_MS)))
            {
                previous = next;
                g_state = next;
                first_status = false;
                last_status_ms = now_ms;
                LOG_INFO(LOG_TASK_CONNECTION, LOG_MOD_NONE,
                         "state=%d link=%d join=%d lease=%d init=%d",
                         (int) next, (int) g_status.link_up, (int) join,
                         (int) lease, (int) g_status.init_result);
            }
        }

        b_mqtt_up = service_mqtt(now_ms, b_mqtt_up);
        if (b_mqtt_up)
        {
            last_mqtt_up_ms = now_ms;
        }
        system_fault_assign(FAULT_OWNER_LINK, FAULT_WIFI_DISCONN_BIT,
                            ((uint32_t) (now_ms - last_mqtt_up_ms) >=
                             CONNECTION_LINK_DOWN_MS)
                                ? FAULT_WIFI_DISCONN_BIT
                                : 0u);

        /* Radio callbacks wake the task for commands and session changes. */
        (void)tk_slp_tsk((TMO)CONNECTION_LOOP_MS);
        (void)tk_can_wup(TSK_SELF);
    }
}

bool
connection_mqtt_seen (void)
{
    return g_b_mqtt_seen;
}

connection_state_t
connection_state (void)
{
    return g_state;
}

void
connection_cmd_state (telemetry_cmd_state_t * p_out)
{
    UINT saved = 0U;

    if (p_out != 0)
    {
        (void)ISpinLock(&g_cmd_lock, &saved);
        *p_out = g_cmd_state;
        (void)ISpinUnlock(&g_cmd_lock, saved);
    }
}
