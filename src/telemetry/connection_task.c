/** @file connection_task.c
 * @brief Core 1 Wi-Fi link, MQTT session, telemetry publish, and command ingress.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "cyw43_utk.h"

#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/watchdog.h"
#include "supervisor/health_ids.h"
#include "platform/log.h"
#include "telemetry/cmd.h"
#include "telemetry/connection.h"
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
#define CONNECTION_STATUS_MS    1000U
#define CONNECTION_MS_PER_S     1000U
#define CONNECTION_FRAME_MIN    2U
#define CONNECTION_RSSI_MAX     127
#define CONNECTION_RSSI_MIN     (-128)
#define CONNECTION_DROPS_MAX    255U
#define CONNECTION_SLEEP_TIMEOUT_MS 50U
/* No MQTT session for this long sets the wifi_disconn fault bit. */
#define CONNECTION_LINK_DOWN_MS 3000U

static const char g_topic_status[]    = TELEMETRY_TOPIC_PREFIX "/s";
static const char g_topic_motion[]    = TELEMETRY_TOPIC_PREFIX "/m";
static const char g_topic_physics[]   = TELEMETRY_TOPIC_PREFIX "/p";
static const char g_topic_obstacle[]  = TELEMETRY_TOPIC_PREFIX "/o";
static const char g_topic_diag[]      = TELEMETRY_TOPIC_PREFIX "/d";
static const char g_topic_event[]     = TELEMETRY_TOPIC_PREFIX "/e";
static const char g_topic_profile[]   = TELEMETRY_TOPIC_PREFIX "/f";
static const char g_topic_heartbeat[] = TELEMETRY_TOPIC_PREFIX "/h";
static const char g_topic_command[]   = TELEMETRY_TOPIC_PREFIX "/c";

static volatile connection_state_t g_state = CONNECTION_DISCONNECTED;
static mqtt_client_t               g_mqtt;
static telemetry_publisher_t       g_publisher;
static telemetry_cmd_state_t       g_cmd_state;
static connection_cmd_dispatch_t   gp_dispatch;
static void *                      gp_dispatch_context;
static bool                        g_mqtt_ready;
static bool                        g_transport_started;
static bool                        g_open_failure_logged;
static uint32_t                    g_heartbeat_ms;
static T_CYW43_UTK_STATUS          g_status;
static ID                          g_conn_task_id;
static ID                          g_conn_cyc_id;
static uint32_t                    g_conn_overruns;

static void
conn_cyclic_wakeup (void * p_exinf)
{
    (void) p_exinf;
    (void) tk_wup_tsk(g_conn_task_id);
}

static void
wait_connection_cycle (void)
{
    INT queued = 0;
    ER const er = tk_slp_tsk((TMO) CONNECTION_SLEEP_TIMEOUT_MS);

    queued = tk_can_wup(TSK_SELF);
    if ((E_OK == er) && (queued > 0))
    {
        g_conn_overruns += (uint32_t) queued;
    }
}

static void
dispatch_command (const telemetry_cmd_t * cmd, void * context)
{
    (void) context;

    if (gp_dispatch != 0)
    {
        gp_dispatch(cmd, gp_dispatch_context);
    }
}

static void
on_mqtt_command (const uint8_t * payload, uint16_t len, void * context)
{
    telemetry_cmd_t cmd;

    (void) context;

    if (telemetry_cmd_parse(payload, len, &cmd) != TELEMETRY_CMD_OK)
    {
        return;
    }
    const telemetry_cmd_result_t result =
        telemetry_cmd_accept(&g_cmd_state, &cmd, tcp_transport_now_ms(),
                             dispatch_command, 0);
    if (result != TELEMETRY_CMD_OK)
    {
        LOG_WARN(LOG_TASK_CONNECTION, LOG_MOD_MQTT,
                 "command opcode=%d refused before supervisor, reason=%d",
                 (int) cmd.opcode, (int) result);
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
    case (uint8_t) TELEMETRY_TOPIC_PROFILE:
        topic = g_topic_profile;
        break;
    case (uint8_t) TELEMETRY_TOPIC_HEARTBEAT:
        topic = g_topic_heartbeat;
        break;
    default:
        return -1;
    }
    const mqtt_result_t result =
        mqtt_client_publish(&g_mqtt, topic, &frame[1], (uint16_t) (len - 1u),
                            tcp_transport_now_ms());
    if (result == MQTT_ERR_BUSY)
    {
        return TELEMETRY_PUBLISH_RETRY;
    }
    if (result != MQTT_OK)
    {
        system_fault_set(FAULT_MQTT_NACK_BIT);
        LOG_WARN(LOG_TASK_CONNECTION, LOG_MOD_MQTT, "publish error=%d", (int) result);
        return -1;
    }
    system_fault_clear(FAULT_MQTT_NACK_BIT);
    return 0;
}

static mqtt_result_t
publish_heartbeat (uint32_t now_ms, const T_CYW43_UTK_STATUS * status)
{
    uint8_t               payload[TELEMETRY_HEARTBEAT_WIRE_SIZE];
    telemetry_heartbeat_t heartbeat;
    uint32_t              drops = 0u;
    int32_t               rssi = status->link_rssi;

    if ((uint32_t) (now_ms - g_heartbeat_ms) < CONNECTION_HEARTBEAT_MS)
    {
        return MQTT_OK;
    }
    if (rssi > CONNECTION_RSSI_MAX)
    {
        rssi = CONNECTION_RSSI_MAX;
    }
    else if (rssi < CONNECTION_RSSI_MIN)
    {
        rssi = CONNECTION_RSSI_MIN;
    }
    drops = (uint32_t) telemetry_queue_drop_count() +
            (uint32_t) telemetry_net_queue_drop_count();
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
    if (telemetry_pack_heartbeat(payload, &heartbeat) == 0u)
    {
        return MQTT_ERR_PROTOCOL;
    }
    const mqtt_result_t result =
        mqtt_client_publish(&g_mqtt, g_topic_heartbeat, payload,
                            (uint16_t) sizeof(payload), now_ms);
    if (result == MQTT_OK)
    {
        g_heartbeat_ms = now_ms;
    }
    return result;
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
        if (status->join_complete && (status->join_result != 0))
        {
            return CONNECTION_FAULT;
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
    transport.restart = tcp_transport_restart;

    mqtt_client_init(&g_mqtt, &config, &transport, on_mqtt_command, 0);
    if (!g_mqtt.valid)
    {
        return E_PAR;
    }
    telemetry_cmd_state_init(&g_cmd_state, 0u);
    gp_dispatch         = dispatch;
    gp_dispatch_context = context;
    g_mqtt_ready        = true;
    return E_OK;
}

/* Open the broker socket once the link is ready; log the first failure. */
static void
open_transport (void)
{
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

void
connection_run (void)
{
    T_CCYC             ccyc = {0};
    connection_state_t previous = CONNECTION_DISCONNECTED;
    uint32_t           last_poll_ms = 0u;
    uint32_t           last_status_ms = 0u;
    uint32_t           last_mqtt_up_ms = tcp_transport_now_ms();
    bool               first_status = true;

    g_conn_task_id = tk_get_tid();
    ccyc.cycatr    = TA_HLNG | TA_STA | TA_ASSPRC;
    ccyc.cychdr    = (FP) conn_cyclic_wakeup;
    ccyc.cyctim    = (RELTIM) CONNECTION_LOOP_MS;
    ccyc.cycphs    = (RELTIM) CONNECTION_LOOP_MS;
    ccyc.assprc    = TP_PRC2;
    g_conn_cyc_id  = tk_cre_cyc(&ccyc);

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
        }

        if (g_mqtt_ready)
        {
            const mqtt_state_t before = mqtt_client_state(&g_mqtt);
            const mqtt_result_t result = mqtt_client_poll(&g_mqtt, now_ms);

            if ((result != MQTT_OK) && (result != MQTT_ERR_BUSY))
            {
                LOG_WARN(LOG_TASK_CONNECTION, LOG_MOD_MQTT, "session error=%d", (int) result);
            }
            if (mqtt_client_state(&g_mqtt) == MQTT_STATE_CONNECTED)
            {
                last_mqtt_up_ms = now_ms;
                if (before != MQTT_STATE_CONNECTED)
                {
                    LOG_INFO(LOG_TASK_CONNECTION, LOG_MOD_MQTT, "session ready");
                    g_heartbeat_ms = now_ms - CONNECTION_HEARTBEAT_MS;
                }
                if ((result == MQTT_OK) && (publish_heartbeat(now_ms, &g_status) == MQTT_OK))
                {
                    telemetry_publisher_run(&g_publisher, net_pull, publish_frame);
                }
            }
        }
        system_fault_assign(FAULT_WIFI_DISCONN_BIT,
                            ((uint32_t) (now_ms - last_mqtt_up_ms) >=
                             CONNECTION_LINK_DOWN_MS)
                                ? FAULT_WIFI_DISCONN_BIT
                                : 0u);

        wait_connection_cycle();
    }
}

connection_state_t
connection_state (void)
{
    return g_state;
}
