/** @file mqtt_session.c
 * @brief Broker session on the NO_SYS lwIP owner and the core 1 handoffs.
 */
#define PROHIBIT_DEF_SIZE_T
#include <string.h>
#include <tk/smp.h>
#include <tk/tkernel.h>

#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/sys.h"

#include "platform/faults.h"
#include "mqtt_session.h"
#include "comms/telemetry/car_id.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/drv_wifi_link.h"
#include "comms/telemetry/worker.h"
#include "telemetry_config.h"

#define MQTT_SESSION_KEEPALIVE_S    30U
#define MQTT_SESSION_REPLY_MS       10000U
#define MQTT_SESSION_BACKOFF_MIN    1000U
#define MQTT_SESSION_BACKOFF_MAX    30000U
#define MQTT_SESSION_BACKOFF_FACTOR 2U
#define MQTT_SESSION_PUBLISH_QOS    0U
#define MQTT_SESSION_COMMAND_QOS    1U
#define MQTT_SESSION_MBF_ALIGN      4U
#define MQTT_SESSION_COMMAND_BYTES  256U
#define MQTT_SESSION_TOPIC_SIZE     sizeof(TELEMETRY_TOPIC_PREFIX "/?")
#define MQTT_SESSION_SUFFIX_AT      (MQTT_SESSION_TOPIC_SIZE - 2U)
#define MQTT_SESSION_LAST_AT        (MQTT_SESSION_TOPIC_SIZE - 1U)

/* Allow for the fixed header, topic length and a QoS 1 packet identifier. */
#define MQTT_SESSION_WIRE_OVERHEAD 9U
/* Thirty-two four-byte PUBACKs can sit behind a congested publish burst. */
#define MQTT_SESSION_PUBACK_HEADROOM 128U

typedef enum
{
    SESSION_IDLE = 0,
    SESSION_RESOLVING,
    SESSION_CONNECTING,
    SESSION_SUBSCRIBING,
    SESSION_ONLINE
} session_phase_t;

STATIC_ASSERT(sizeof(session_phase_t) <= 4U, "session_phase_t size");
STATIC_ASSERT(MQTT_SESSION_TOPIC_SIZE + TELEMETRY_CMD_WIRE_SIZE +
                      MQTT_SESSION_WIRE_OVERHEAD <=
                  MQTT_VAR_HEADER_BUFFER_LEN,
              "the command packet fits the MQTT receive buffer");
STATIC_ASSERT(MQTT_REQ_MAX_IN_FLIGHT *(MQTT_SESSION_TOPIC_SIZE +
                                       TELEMETRY_WIRE_MAX +
                                       MQTT_SESSION_WIRE_OVERHEAD) +
                      MQTT_SESSION_PUBACK_HEADROOM <=
                  MQTT_OUTPUT_RINGBUF_SIZE,
              "pending telemetry leaves room for a burst of command PUBACKs");

static char g_client_name[sizeof(TELEMETRY_CLIENT_ID) + CAR_ID_HEX_DIGITS + 1U];

static struct mqtt_connect_client_info_t g_client_info = {
    .client_id = g_client_name,
    .client_user =
        (TELEMETRY_MQTT_USERNAME[0] != '\0') ? TELEMETRY_MQTT_USERNAME : NULL,
    .client_pass = ((TELEMETRY_MQTT_USERNAME[0] != '\0') &&
                    (TELEMETRY_MQTT_PASSWORD[0] != '\0'))
                       ? TELEMETRY_MQTT_PASSWORD
                       : NULL,
    .keep_alive  = MQTT_SESSION_KEEPALIVE_S,
};

static uint8_t g_command_queue[MQTT_SESSION_COMMAND_BYTES]
    __attribute__((aligned(MQTT_SESSION_MBF_ALIGN)));
static ID g_command_queue_id;
/* Set last at startup. lwIP state below is owned by the radio task. */
static volatile ID       g_connection_task;
static mqtt_client_t *   gp_client;
static session_phase_t   g_phase;
static uint32_t          g_deadline_ms;
static uint32_t          g_retry_ms;
static uint32_t          g_backoff_ms = MQTT_SESSION_BACKOFF_MIN;
static bool              g_b_retrying;
static bool              g_b_dns_pending;
static bool              g_b_disconnect;
static char              g_command_topic[MQTT_SESSION_TOPIC_SIZE];
static uint8_t           g_incoming[TELEMETRY_CMD_WIRE_SIZE];
static bool              g_b_receiving_command;
static uint16_t          g_incoming_used;
static bool              g_b_heartbeat_sent;
static volatile uint32_t g_drops;
static volatile uint32_t g_car_id;

/* The connection task supplies heartbeat bytes and reads the session level. */
static T_SPLOCK      g_lock;
static volatile bool g_b_connected;
static bool          g_b_heartbeat_pending;
static uint8_t       g_heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE];

static void make_topic (char p_topic[MQTT_SESSION_TOPIC_SIZE], char suffix)
{
    (void)memcpy(p_topic, TELEMETRY_TOPIC_PREFIX "/", MQTT_SESSION_SUFFIX_AT);
    p_topic[MQTT_SESSION_SUFFIX_AT] = suffix;
    p_topic[MQTT_SESSION_LAST_AT]   = '\0';
}

static void publish_session (bool b_connected)
{
    UINT saved = 0U;

    (void)ISpinLock(&g_lock, &saved);
    g_b_heartbeat_pending = false;
    g_b_connected         = b_connected;
    (void)ISpinUnlock(&g_lock, saved);
    (void)tk_wup_tsk(g_connection_task);
}

static void schedule_retry (uint32_t now_ms)
{
    uint32_t const next = g_backoff_ms * MQTT_SESSION_BACKOFF_FACTOR;

    if (g_phase == SESSION_ONLINE)
    {
        publish_session(false);
    }
    g_phase               = SESSION_IDLE;
    g_b_disconnect        = false;
    g_b_receiving_command = false;
    g_b_heartbeat_sent    = false;
    g_b_retrying          = true;
    g_retry_ms            = now_ms + g_backoff_ms;
    g_backoff_ms =
        (next > MQTT_SESSION_BACKOFF_MAX) ? MQTT_SESSION_BACKOFF_MAX : next;
}

static void queue_command (uint16_t len)
{
    /* A one-byte record represents malformed length, including empty
     * publications. Valid commands have a fixed nine-byte payload. */
    INT const size = (len == TELEMETRY_CMD_WIRE_SIZE) ? (INT)len : 1;

    ER result = tk_snd_mbf(g_command_queue_id, g_incoming, size, TMO_POL);

    if ((result == E_TMOUT) && (len == TELEMETRY_CMD_WIRE_SIZE))
    {
        /* The SDK sends PUBACK on return. Keep a complete command in the
         * radio task until core 1 makes room. The reader only takes the
         * kernel queue and does not wait on lwIP. */
        (void)tk_wup_tsk(g_connection_task);
        result = tk_snd_mbf(g_command_queue_id, g_incoming, size, TMO_FEVR);
    }
    if (result != E_OK)
    {
        g_drops++;
        return;
    }
    (void)tk_wup_tsk(g_connection_task);
}

static void on_incoming_publish (void * p_arg, const char * p_topic, u32_t len)
{
    (void)p_arg;
    g_b_receiving_command = false;
    if ((g_phase != SESSION_ONLINE) || (strcmp(p_topic, g_command_topic) != 0))
    {
        return;
    }
    if (len != TELEMETRY_CMD_WIRE_SIZE)
    {
        queue_command(0U);
        return;
    }
    g_b_receiving_command = true;
    g_incoming_used       = 0U;
}

static void on_incoming_data (void * p_arg, const u8_t * p_data, u16_t len,
                              u8_t flags)
{
    (void)p_arg;
    if (!g_b_receiving_command)
    {
        return;
    }
    if (len > (TELEMETRY_CMD_WIRE_SIZE - g_incoming_used))
    {
        queue_command(0U);
        g_b_receiving_command = false;
        return;
    }
    if (len != 0U)
    {
        (void)memcpy(&g_incoming[g_incoming_used], p_data, len);
    }
    g_incoming_used = (uint16_t)(g_incoming_used + len);
    if ((flags & (uint8_t)MQTT_DATA_FLAG_LAST) != 0U)
    {
        queue_command(g_incoming_used);
        g_b_receiving_command = false;
    }
}

static void on_subscribed (void * p_arg, err_t err)
{
    (void)p_arg;
    if (err != ERR_OK)
    {
        /* lwIP is still processing the SUBACK or its timer. */
        g_b_disconnect = true;
        return;
    }
    g_phase      = SESSION_ONLINE;
    g_backoff_ms = MQTT_SESSION_BACKOFF_MIN;
    publish_session(true);
}

static void on_connected (mqtt_client_t * p_client, void * p_arg,
                          mqtt_connection_status_t status)
{
    (void)p_arg;
    if (status != MQTT_CONNECT_ACCEPTED)
    {
        schedule_retry(sys_now());
        return;
    }
    g_phase       = SESSION_SUBSCRIBING;
    g_deadline_ms = sys_now() + MQTT_SESSION_REPLY_MS;
    if (mqtt_subscribe(p_client, g_command_topic, MQTT_SESSION_COMMAND_QOS,
                       on_subscribed, NULL) != ERR_OK)
    {
        g_b_disconnect = true;
    }
}

static void connect_broker (const ip_addr_t * p_addr, uint32_t now_ms)
{
    if (gp_client == NULL)
    {
        gp_client = mqtt_client_new();
        if (gp_client == NULL)
        {
            schedule_retry(now_ms);
            return;
        }
        make_topic(g_command_topic, TELEMETRY_TOPIC_COMMAND);
        mqtt_set_inpub_callback(gp_client, on_incoming_publish,
                                on_incoming_data, NULL);
    }
    g_phase = SESSION_CONNECTING;
    if (mqtt_client_connect(gp_client, p_addr, (uint16_t)TELEMETRY_BROKER_PORT,
                            on_connected, NULL, &g_client_info) != ERR_OK)
    {
        schedule_retry(now_ms);
    }
}

static void on_resolved (const char * p_name, const ip_addr_t * p_addr,
                         void * p_arg)
{
    (void)p_name;
    (void)p_arg;
    g_b_dns_pending = false;
    if (g_phase != SESSION_RESOLVING)
    {
        /* The lease disappeared or the attempt timed out while DNS ran. */
        return;
    }
    uint32_t const now_ms = sys_now();

    if ((p_addr == NULL) || (wifi_link_lease() != WIFI_LEASE_READY))
    {
        schedule_retry(now_ms);
        return;
    }
    connect_broker(p_addr, now_ms);
}

static void begin_attempt (uint32_t now_ms)
{
    ip_addr_t address;

    if (car_id_client_id(g_client_name, sizeof(g_client_name),
                         TELEMETRY_CLIENT_ID, g_car_id) == 0U)
    {
        schedule_retry(now_ms);
        return;
    }
    g_phase       = SESSION_RESOLVING;
    g_deadline_ms = now_ms + MQTT_SESSION_REPLY_MS;
    err_t const err =
        dns_gethostbyname(TELEMETRY_BROKER_HOST, &address, on_resolved, NULL);

    if (err == ERR_OK)
    {
        connect_broker(&address, now_ms);
    }
    else if (err == ERR_INPROGRESS)
    {
        g_b_dns_pending = true;
    }
    else
    {
        schedule_retry(now_ms);
    }
}

static err_t publish_payload (char suffix, const uint8_t * p_payload,
                              uint16_t len)
{
    char topic[MQTT_SESSION_TOPIC_SIZE];

    make_topic(topic, suffix);
    return mqtt_publish(gp_client, topic, p_payload, len,
                        MQTT_SESSION_PUBLISH_QOS, 0U, NULL, NULL);
}

static int publish_frame (const uint8_t * p_frame, uint16_t len)
{
    if (len <= 1U)
    {
        return -1;
    }
    char const suffix = (char)p_frame[0];

    switch (suffix)
    {
        case TELEMETRY_TOPIC_STATUS:
        case TELEMETRY_TOPIC_MOTION:
        case TELEMETRY_TOPIC_PHYSICS:
        case TELEMETRY_TOPIC_OBSTACLE:
        case TELEMETRY_TOPIC_DIAG:
        case TELEMETRY_TOPIC_EVENT:
        case TELEMETRY_TOPIC_BARCODE_SAMPLE:
        case TELEMETRY_TOPIC_PROFILE:
        case TELEMETRY_TOPIC_HEARTBEAT:
            break;
        default:
            return -1;
    }
    err_t const err =
        publish_payload(suffix, &p_frame[1], (uint16_t)(len - 1U));

    if (err == ERR_MEM)
    {
        g_drops++;
        return -1;
    }
    system_fault_assign(FAULT_OWNER_LINK, FAULT_MQTT_NACK_BIT,
                        (err == ERR_OK) ? 0U : FAULT_MQTT_NACK_BIT);
    return (err == ERR_OK) ? 0 : -1;
}

static void service_online (void)
{
    UINT    saved = 0U;
    uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE];

    (void)ISpinLock(&g_lock, &saved);
    bool const b_pending = g_b_heartbeat_pending;

    if (b_pending)
    {
        (void)memcpy(heartbeat, g_heartbeat, sizeof(heartbeat));
        g_b_heartbeat_pending = false;
    }
    (void)ISpinUnlock(&g_lock, saved);

    if (b_pending)
    {
        if (publish_payload(TELEMETRY_TOPIC_HEARTBEAT, heartbeat,
                            (uint16_t)sizeof(heartbeat)) != ERR_OK)
        {
            (void)mqtt_session_heartbeat(heartbeat);
            return;
        }
        g_b_heartbeat_sent = true;
    }
    if (g_b_heartbeat_sent)
    {
        telemetry_worker_run(telemetry_net_queue_pop, publish_frame, TMO_POL);
    }
}

int32_t mqtt_session_init (void)
{
    if (g_command_queue_id > 0)
    {
        return E_OK;
    }
    T_CMBF const config = {
        .mbfatr = TA_USERBUF | TA_TPRI,
        .bufsz  = sizeof(g_command_queue),
        .maxmsz = TELEMETRY_CMD_WIRE_SIZE,
        .bufptr = g_command_queue,
    };
    ID const queue = tk_cre_mbf(&config);

    if (queue <= 0)
    {
        return queue;
    }
    g_command_queue_id = queue;
    (void)InitSpinLock(&g_lock);
    return E_OK;
}

void mqtt_session_set_car_id (uint32_t car_id)
{
    g_car_id = car_id;
}

void mqtt_session_start (int32_t task)
{
    mp_memory_barrir();
    g_connection_task = task;
}

uint32_t mqtt_session_now_ms (void)
{
    return sys_now();
}

bool mqtt_session_connected (void)
{
    return g_b_connected;
}

bool mqtt_session_heartbeat (
    const uint8_t payload[TELEMETRY_HEARTBEAT_WIRE_SIZE])
{
    UINT saved = 0U;

    (void)ISpinLock(&g_lock, &saved);
    bool const b_space = g_b_connected && !g_b_heartbeat_pending;

    if (b_space)
    {
        (void)memcpy(g_heartbeat, payload, sizeof(g_heartbeat));
        g_b_heartbeat_pending = true;
    }
    (void)ISpinUnlock(&g_lock, saved);
    return b_space;
}

bool mqtt_session_command (uint8_t    payload[TELEMETRY_CMD_WIRE_SIZE],
                           uint16_t * p_len)
{
    INT const size = tk_rcv_mbf(g_command_queue_id, payload, TMO_POL);

    *p_len = (size == (INT)TELEMETRY_CMD_WIRE_SIZE) ? (uint16_t)size : 0U;
    return size > 0;
}

uint32_t mqtt_session_drops (void)
{
    return g_drops;
}

void mqtt_session_poll (void)
{
    if (g_connection_task <= 0)
    {
        return;
    }
    mp_memory_barrir();

    uint32_t const now_ms  = sys_now();
    bool const     b_ready = (wifi_link_lease() == WIFI_LEASE_READY);
    bool const     b_waiting =
        (g_phase != SESSION_IDLE) && (g_phase != SESSION_ONLINE);

    if ((g_phase != SESSION_IDLE) &&
        (!b_ready || g_b_disconnect ||
         (b_waiting && ((int32_t)(now_ms - g_deadline_ms) >= 0))))
    {
        if ((g_phase != SESSION_RESOLVING) && (gp_client != NULL))
        {
            mqtt_disconnect(gp_client);
        }
        schedule_retry(now_ms);
    }
    if (g_phase == SESSION_ONLINE)
    {
        service_online();
    }
    else if ((g_phase == SESSION_IDLE) && b_ready && !g_b_dns_pending &&
             (g_car_id != 0U) &&
             (!g_b_retrying || ((int32_t)(now_ms - g_retry_ms) >= 0)))
    {
        begin_attempt(now_ms);
    }
}
