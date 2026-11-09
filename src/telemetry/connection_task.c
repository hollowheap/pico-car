/** @file connection_task.c
 * @brief Core 1 Wi-Fi link, MQTT session, telemetry publish, and command ingress.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "cyw43_utk.h"

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

static const char s_topic_status[]    = TELEMETRY_TOPIC_PREFIX "/s";
static const char s_topic_event[]     = TELEMETRY_TOPIC_PREFIX "/e";
static const char s_topic_heartbeat[] = TELEMETRY_TOPIC_PREFIX "/h";
static const char s_topic_command[]   = TELEMETRY_TOPIC_PREFIX "/c";
static const char s_topic_system[]    = TELEMETRY_TOPIC_PREFIX "/t";

static volatile connection_state_t s_state = CONNECTION_DISCONNECTED;
static mqtt_client_t               s_mqtt;
static telemetry_cmd_state_t       s_cmd_state;
static connection_cmd_dispatch_t   s_dispatch;
static void *                      s_dispatch_context;
static bool                        s_mqtt_ready;
static bool                        s_transport_started;
static bool                        s_open_failure_logged;
static uint32_t                    s_heartbeat_ms;
static T_CYW43_UTK_STATUS          s_status;

static void
dispatch_command (const telemetry_cmd_t * cmd, void * context)
{
    (void) context;

    if (s_dispatch != 0)
    {
        s_dispatch(cmd, s_dispatch_context);
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
    (void) telemetry_cmd_accept(&s_cmd_state, &cmd,
                                (uint16_t) tcp_transport_now_ms(),
                                dispatch_command, 0);
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
    const char * topic;

    if ((frame == 0) || (len < 2u))
    {
        return -1;
    }
    switch (frame[0])
    {
    case (uint8_t) 's':
        topic = s_topic_status;
        break;
    case (uint8_t) 'e':
        topic = s_topic_event;
        break;
    case (uint8_t) 'h':
        topic = s_topic_heartbeat;
        break;
    case (uint8_t) 't':
        topic = s_topic_system;
        break;
    default:
        return -1;
    }
    if (mqtt_client_publish(&s_mqtt, topic, &frame[1], (uint16_t) (len - 1u),
                            tcp_transport_now_ms()) != MQTT_OK)
    {
        return -1;
    }
    return 0;
}

static void
publish_heartbeat (uint32_t now_ms, const T_CYW43_UTK_STATUS * status)
{
    uint8_t  payload[TELEMETRY_HEARTBEAT_WIRE_SIZE];
    uint32_t drops;
    int32_t  rssi = status->link_rssi;

    if ((uint32_t) (now_ms - s_heartbeat_ms) < CONNECTION_HEARTBEAT_MS)
    {
        return;
    }
    s_heartbeat_ms = now_ms;
    if (rssi > 127)
    {
        rssi = 127;
    }
    else if (rssi < -128)
    {
        rssi = -128;
    }
    drops = (uint32_t) telemetry_queue_drop_count() +
            (uint32_t) telemetry_net_queue_drop_count();
    if (drops > 255u)
    {
        drops = 255u;
    }
    if (telemetry_pack_heartbeat(payload, (uint16_t) (now_ms / 1000u),
                                 (uint8_t) drops, (int8_t) rssi) == 0u)
    {
        return;
    }
    (void) mqtt_client_publish(&s_mqtt, s_topic_heartbeat, payload,
                               (uint16_t) sizeof(payload), now_ms);
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

    if (s_mqtt_ready)
    {
        return E_OK;
    }

    config.client_id     = TELEMETRY_CLIENT_ID;
    config.command_topic = s_topic_command;
    config.keepalive_s   = MQTT_DEFAULT_KEEPALIVE_S;
    if (TELEMETRY_MQTT_USERNAME[0] != '\0')
    {
        config.username = TELEMETRY_MQTT_USERNAME;
        config.password = TELEMETRY_MQTT_PASSWORD;
    }

    transport.context = 0;
    transport.send    = tcp_transport_send;
    transport.recv    = tcp_transport_recv;

    mqtt_client_init(&s_mqtt, &config, &transport, on_mqtt_command, 0);
    if (!s_mqtt.valid)
    {
        return E_PAR;
    }
    telemetry_cmd_state_init(&s_cmd_state, 0u);
    s_dispatch         = dispatch;
    s_dispatch_context = context;
    s_mqtt_ready       = true;
    return E_OK;
}

void
connection_run (void)
{
    connection_state_t previous = CONNECTION_DISCONNECTED;
    uint32_t           last_poll_ms = 0u;
    uint32_t           last_status_ms = 0u;
    bool               first_status = true;

    for (;;)
    {
        const uint32_t now_ms = tcp_transport_now_ms();

        if ((uint32_t) (now_ms - last_poll_ms) >= CONNECTION_POLL_MS)
        {
            connection_state_t next;

            last_poll_ms = now_ms;
            cyw43_utk_get_status(&s_status);
            next = classify(&s_status);
            if ((next != previous) || first_status ||
                ((next != CONNECTION_READY) &&
                 ((uint32_t) (now_ms - last_status_ms) >= CONNECTION_STATUS_MS)))
            {
                previous = next;
                s_state = next;
                first_status = false;
                last_status_ms = now_ms;
                (void) tm_printf(
                    (UB *) "[conn] state=%d link=%u started=%u join=%d "
                           "dhcp_done=%u dhcp=%d init=%d\n",
                    (INT) next, s_status.link_up, s_status.join_started,
                    s_status.join_result, s_status.dhcp_complete,
                    s_status.dhcp_result, s_status.init_result);
            }
            if (next == CONNECTION_READY)
            {
                if (!s_transport_started)
                {
                    s_transport_started =
                        tcp_transport_open(TELEMETRY_BROKER_HOST,
                                           (uint16_t) TELEMETRY_BROKER_PORT);
                    if (!s_transport_started && !s_open_failure_logged)
                    {
                        s_open_failure_logged = true;
                        (void) tm_printf((UB *)"[conn] transport open failed\n");
                    }
                }
            }
            else if (next == CONNECTION_FAULT)
            {
                tcp_transport_close();
                s_transport_started = false;
            }
        }

        if (s_mqtt_ready)
        {
            (void) mqtt_client_poll(&s_mqtt, now_ms);
            if (mqtt_client_state(&s_mqtt) == MQTT_STATE_CONNECTED)
            {
                telemetry_publisher_run(net_pull, publish_frame);
                publish_heartbeat(now_ms, &s_status);
            }
        }

        (void) tk_dly_tsk(CONNECTION_LOOP_MS);
    }
}

connection_state_t
connection_state (void)
{
    return s_state;
}
