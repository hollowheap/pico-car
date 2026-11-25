/** @file connection_task.c
 * @brief Core 1 Wi-Fi link, MQTT session, telemetry publish, and command ingress.
 */
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>

#include "cyw43_utk.h"

#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/watchdog.h"
#include "supervisor/health_ids.h"
#include "platform/log.h"
#include "telemetry/cmd.h"
#include "telemetry/connection.h"
#include "telemetry/emit.h"
#include "telemetry/mqtt.h"
#include "telemetry/net_queue.h"
#include "telemetry/packet.h"
#include "telemetry/publisher.h"
#include "telemetry/queue.h"
#include "telemetry/tcp_transport.h"
#include "telemetry_config.h"

#define CONNECTION_POLL_MS      100U
#define CONNECTION_LOOP_MS      10U
#define CONNECTION_HEARTBEAT_MS 1000U
#define CONNECTION_ROSC_RANDOMBIT (0x40060000U + 0x1CU) /* ROSC RANDOMBIT */
#define CONNECTION_RUN_ID_BITS  16U
#define CONNECTION_STATUS_MS    1000U
#define CONNECTION_MS_PER_S     1000U
#define CONNECTION_FRAME_MIN    2U
#define CONNECTION_RSSI_MAX     127
#define CONNECTION_RSSI_MIN     (-128)
#define CONNECTION_DROPS_MAX    255U
/* No MQTT session for this long sets the wifi_disconn fault bit. */
#define CONNECTION_LINK_DOWN_MS 3000U

static const char g_topic_status[]    = TELEMETRY_TOPIC_PREFIX "/s";
static const char g_topic_motion[]    = TELEMETRY_TOPIC_PREFIX "/m";
static const char g_topic_physics[]   = TELEMETRY_TOPIC_PREFIX "/p";
static const char g_topic_obstacle[]  = TELEMETRY_TOPIC_PREFIX "/o";
static const char g_topic_diag[]      = TELEMETRY_TOPIC_PREFIX "/d";
static const char g_topic_event[]     = TELEMETRY_TOPIC_PREFIX "/e";
static const char g_topic_profile[]   = TELEMETRY_TOPIC_PREFIX "/f";
static const char g_topic_barcode[]   = TELEMETRY_TOPIC_PREFIX "/b";
static const char g_topic_heartbeat[] = TELEMETRY_TOPIC_PREFIX "/h";
static const char g_topic_command[]   = TELEMETRY_TOPIC_PREFIX "/c";

static volatile connection_state_t g_state = CONNECTION_DISCONNECTED;
static volatile bool               g_b_mqtt_seen;
static mqtt_client_t               g_mqtt;
static T_SPLOCK                    g_cmd_lock;
static telemetry_cmd_state_t       g_cmd_state;
static connection_cmd_dispatch_t   gp_dispatch;
static void *                      gp_dispatch_context;
static bool                        g_mqtt_ready;
static bool                        g_transport_started;
static bool                        g_open_failure_logged;
static bool                        g_transport_closing;
static uint32_t                    g_busy_drops;
static uint32_t                    g_heartbeat_ms;
static T_CYW43_UTK_STATUS          g_status;
/* Run ID from 16 reads of the ring oscillator's noisy bit. The kernel never
   stops the ring oscillator, which runs from reset. */
static uint16_t
boot_run_id (void)
{
    uint32_t id  = 0u;
    uint32_t bit = 0u;

    for (bit = 0u; bit < CONNECTION_RUN_ID_BITS; bit++)
    {
        id = (id << 1u) | (in_w(CONNECTION_ROSC_RANDOMBIT) & 1u);
    }
    /* Cast is safe: 16 bits were read. */
    return (uint16_t) id;
}

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
on_mqtt_command (const uint8_t * payload, uint16_t len, void * context)
{
    /* Only the connection task updates the state after startup. */
    telemetry_cmd_state_t next = g_cmd_state;

    (void) context;

    /* Use the clock and units of the heartbeat uptime_s the command echoes.
       The dispatch callback publishes accepted commands. */
    if (TELEMETRY_CMD_OK !=
        telemetry_cmd_receive(&next, payload, len,
                              tcp_transport_now_ms() / CONNECTION_MS_PER_S,
                              dispatch_command, &next))
    {
        publish_cmd_state(&next);
    }
}

static int
net_pull (uint8_t * frame, uint16_t * len)
{
    const INT got = telemetry_net_queue_pop(frame, TMO_POL);

    if (got <= 0)
    {
        return 0;
    }
    *len = (uint16_t) got;
    return 1;
}

static int
publish_frame (const uint8_t * frame, uint16_t len)
{
    const char * topic = 0;
    mqtt_result_t result = MQTT_OK;

    if ((frame == 0) || (len < CONNECTION_FRAME_MIN))
    {
        return -1;
    }
    switch (frame[0])
    {
    case (uint8_t) TELEMETRY_TOPIC_STATUS:
        topic = g_topic_status;
        break;
    case (uint8_t) TELEMETRY_TOPIC_MOTION:
        topic = g_topic_motion;
        break;
    case (uint8_t) TELEMETRY_TOPIC_PHYSICS:
        topic = g_topic_physics;
        break;
    case (uint8_t) TELEMETRY_TOPIC_OBSTACLE:
        topic = g_topic_obstacle;
        break;
    case (uint8_t) TELEMETRY_TOPIC_DIAG:
        topic = g_topic_diag;
        break;
    case (uint8_t) TELEMETRY_TOPIC_EVENT:
        topic = g_topic_event;
        break;
    case (uint8_t) TELEMETRY_TOPIC_BARCODE_SAMPLE:
        topic = g_topic_barcode;
        break;
    case (uint8_t) TELEMETRY_TOPIC_PROFILE:
        topic = g_topic_profile;
        break;
    case (uint8_t) TELEMETRY_TOPIC_HEARTBEAT:
        topic = g_topic_heartbeat;
        break;
    default:
        return -1;
    }
    result = mqtt_client_publish(&g_mqtt, topic, &frame[1],
                                 (uint16_t) (len - 1u),
                                 tcp_transport_now_ms());
    if (result == MQTT_ERR_BUSY)
    {
        /* The frame is lost, but the broker never refused it. */
        g_busy_drops++;
        return -1;
    }
    if (result != MQTT_OK)
    {
        system_fault_assign(FAULT_OWNER_LINK, FAULT_MQTT_NACK_BIT,
                            FAULT_MQTT_NACK_BIT);
        return -1;
    }
    system_fault_assign(FAULT_OWNER_LINK, FAULT_MQTT_NACK_BIT, 0u);
    return 0;
}

static uint32_t
add_saturating (uint32_t a, uint32_t b)
{
    return (a > (UINT32_MAX - b)) ? UINT32_MAX : (a + b);
}

static void
publish_heartbeat (uint32_t now_ms, const T_CYW43_UTK_STATUS * status)
{
    uint8_t               payload[TELEMETRY_HEARTBEAT_WIRE_SIZE];
    telemetry_heartbeat_t heartbeat;
    uint32_t              drops = 0u;
    int32_t               rssi = status->link_rssi;

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
    drops = add_saturating(drops, g_busy_drops);
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
    if (telemetry_pack_heartbeat(payload, &heartbeat) == 0u)
    {
        return;
    }
    mqtt_result_t const result = mqtt_client_publish(&g_mqtt, g_topic_heartbeat, payload,
                                                     (uint16_t) sizeof(payload), now_ms);
    if (MQTT_OK == result)
    {
        g_heartbeat_ms = now_ms;
    }
}

static connection_state_t
classify (const T_CYW43_UTK_STATUS * status)
{
    if (status->init_result != 0)
    {
        return CONNECTION_FAULT;
    }
    if (!status->link_up)
    {
        if (status->join_complete)
        {
            return (status->join_result != 0) ? CONNECTION_FAULT
                                              : CONNECTION_DISCONNECTED;
        }
        return status->join_started ? CONNECTION_JOINING
                                    : CONNECTION_DISCONNECTED;
    }
    if (!status->dhcp_complete)
    {
        return CONNECTION_WAIT_DHCP;
    }
    if (status->dhcp_result == 0)
    {
        return CONNECTION_READY;
    }
    return CONNECTION_FAULT;
}

ER
connection_init (connection_cmd_dispatch_t dispatch, void * context)
{
    mqtt_config_t    config = {0};
    mqtt_transport_t transport = {0};

    if (g_mqtt_ready)
    {
        return E_OK;
    }

    config.client_id     = TELEMETRY_CLIENT_ID;
    config.command_topic = g_topic_command;
    config.keepalive_s   = MQTT_DEFAULT_KEEPALIVE_S;
    if (TELEMETRY_MQTT_USERNAME[0] != '\0')
    {
        config.username = TELEMETRY_MQTT_USERNAME;
        config.password = TELEMETRY_MQTT_PASSWORD;
    }

    transport.context = 0;
    transport.send    = tcp_transport_send;
    transport.recv    = tcp_transport_recv;

    mqtt_client_init(&g_mqtt, &config, &transport, on_mqtt_command, 0);
    if (!g_mqtt.valid)
    {
        return E_PAR;
    }
    /* A new run ID at each boot: a host must read it from the heartbeat,
       so a command for an earlier boot is refused. */
    (void)InitSpinLock(&g_cmd_lock);
    telemetry_cmd_state_init(&g_cmd_state, boot_run_id());
    gp_dispatch         = dispatch;
    gp_dispatch_context = context;
    g_mqtt_ready        = true;
    return E_OK;
}

/* Open the broker socket once the link is ready; log the first failure. */
static void
open_transport (void)
{
    if (g_transport_closing)
    {
        /* tcp_transport_close only asks: opening again before the owner task
           aborts the old stream would keep it. */
        if (tcp_transport_close_pending())
        {
            return;
        }
        g_transport_closing = false;
    }
    if (g_transport_started)
    {
        return;
    }
    g_transport_started = tcp_transport_open(TELEMETRY_BROKER_HOST,
                                             (uint16_t) TELEMETRY_BROKER_PORT);
    if (!g_transport_started && !g_open_failure_logged)
    {
        g_open_failure_logged = true;
        LOG_ERROR(LOG_TASK_CONNECTION, LOG_MOD_NONE, "transport open failed");
    }
}

static void
close_mqtt_transport (void)
{
    /* The next CONNECT must go out on a new stream. */
    tcp_transport_close();
    g_transport_started = false;
    g_transport_closing = true;
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
    if (!g_mqtt_ready || !g_transport_started)
    {
        return false;
    }

    bool const b_had_session = (mqtt_client_state(&g_mqtt) != MQTT_STATE_DISCONNECTED);

    (void)mqtt_client_poll(&g_mqtt, now_ms);
    mqtt_state_t const state = mqtt_client_state(&g_mqtt);
    bool const b_connected = (state == MQTT_STATE_CONNECTED);

    if (b_had_session && (state == MQTT_STATE_DISCONNECTED))
    {
        close_mqtt_transport();
    }
    if (b_connected != b_mqtt_up)
    {
        note_mqtt_session(b_connected);
    }
    if (!b_connected)
    {
        return false;
    }
    if (!b_mqtt_up)
    {
        g_heartbeat_ms = now_ms - CONNECTION_HEARTBEAT_MS;
    }
    g_b_mqtt_seen = true;
    publish_heartbeat(now_ms, &g_status);
    if (mqtt_client_state(&g_mqtt) == MQTT_STATE_CONNECTED)
    {
        telemetry_publisher_run(net_pull, publish_frame);
    }
    if (mqtt_client_state(&g_mqtt) == MQTT_STATE_DISCONNECTED)
    {
        close_mqtt_transport();
        note_mqtt_session(false);
        return false;
    }
    return true;
}

void
connection_run (void)
{
    connection_state_t previous = CONNECTION_DISCONNECTED;
    bool              b_mqtt_up = false;
    uint32_t           last_poll_ms = 0u;
    uint32_t           last_status_ms = 0u;
    uint32_t           last_mqtt_up_ms = tcp_transport_now_ms();
    bool               first_status = true;

    for (;;)
    {
        const uint32_t now_ms = tcp_transport_now_ms();

        liveness_kick(LIVE_CONNECTION);

        if ((uint32_t) (now_ms - last_poll_ms) >= CONNECTION_POLL_MS)
        {
            connection_state_t next = CONNECTION_DISCONNECTED;

            last_poll_ms = now_ms;
            cyw43_utk_get_status(&g_status);
            next = classify(&g_status);
            if ((next != previous) || first_status ||
                ((next != CONNECTION_READY) &&
                 ((uint32_t) (now_ms - last_status_ms) >= CONNECTION_STATUS_MS)))
            {
                previous = next;
                g_state = next;
                first_status = false;
                last_status_ms = now_ms;
                LOG_INFO(LOG_TASK_CONNECTION, LOG_MOD_NONE,
                         "state=%d link=%d started=%d join=%d dhcp_done=%d "
                         "dhcp=%d init=%d",
                         (int) next, (int) g_status.link_up,
                         (int) g_status.join_started, (int) g_status.join_result,
                         (int) g_status.dhcp_complete, (int) g_status.dhcp_result,
                         (int) g_status.init_result);
            }
            if (next == CONNECTION_READY)
            {
                open_transport();
            }
            else if (next == CONNECTION_FAULT)
            {
                /* One request per close: a repeat every poll would wrap the
                   8-bit request count onto the acknowledged one. */
                if (g_transport_started || !g_transport_closing)
                {
                    tcp_transport_close();
                }
                g_transport_started = false;
                g_transport_closing = true;
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

        /* A queued frame wakes the publisher. The timeout services MQTT and Wi-Fi while idle. */
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
