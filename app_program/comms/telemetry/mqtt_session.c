/** @file mqtt_session.c
 * @brief Broker session on the NO_SYS lwIP owner and the core 1 handoffs.
 */
#define PROHIBIT_DEF_SIZE_T
#include <string.h>
#include <tk/smp.h>
#include <tk/tkernel.h>

#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/sys.h"

#include "platform/faults.h"
#include "platform/log.h"
#include "mqtt_session.h"
#include "comms/telemetry/car_id.h"
#include "comms/telemetry/cmd.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/drv_wifi_link.h"
#include "comms/telemetry/worker.h"
#include "telemetry_config.h"

#define MQTT_SESSION_KEEPALIVE_S    30U
/* One failed 4 s exchange, a 1 s retry and a fresh attempt leave time for status. */
#define MQTT_SESSION_REPLY_MS        4000U
#define MQTT_SESSION_RETRY_MS        1000U
#define MQTT_SESSION_PUBLISH_RETRY_MS 20U
#define MQTT_SESSION_RECEIPT_RETRY_MS 1000U
#define MQTT_SESSION_PUBLISH_QOS    0U
#define MQTT_SESSION_STATUS_QOS     1U
#define MQTT_SESSION_RECORD_QOS     1U
#define MQTT_SESSION_COMMAND_QOS    1U
#define MQTT_SESSION_MBF_ALIGN      4U
#define MQTT_SESSION_COMMAND_BYTES  256U
#define MQTT_SESSION_STOP_RECORDS    2U
/* The kernel message buffer stores an INT header and rounds each payload to INT size. */
#define MQTT_SESSION_COMMAND_RECORD_BYTES                                      \
    (sizeof(INT) + ((TELEMETRY_CMD_WIRE_SIZE + sizeof(INT) - 1U) / sizeof(INT)) * \
                       sizeof(INT))
#define MQTT_SESSION_TOPIC_SIZE     sizeof(TELEMETRY_TOPIC_PREFIX "/?")
#define MQTT_SESSION_SUFFIX_AT      (MQTT_SESSION_TOPIC_SIZE - 2U)
#define MQTT_SESSION_LAST_AT        (MQTT_SESSION_TOPIC_SIZE - 1U)
#define MQTT_SESSION_BYTE_BITS      8U

/* Allow for the fixed header, topic length and a QoS 1 packet identifier. */
#define MQTT_SESSION_WIRE_OVERHEAD 9U
/* Allow thirty-two four-byte PUBACKs and one keepalive PINGREQ. */
#define MQTT_SESSION_PUBACK_HEADROOM 128U
#define MQTT_SESSION_PINGREQ_BYTES   2U
#define MQTT_SESSION_CONTROL_HEADROOM                                          \
    (MQTT_SESSION_PUBACK_HEADROOM + MQTT_SESSION_PINGREQ_BYTES)

typedef enum
{
    SESSION_IDLE = 0,
    SESSION_RESOLVING,
    SESSION_CONNECTING,
    SESSION_SUBSCRIBING,
    SESSION_ONLINE
} session_phase_t;

typedef enum
{
    RECEIPT_IDLE = 0,
    RECEIPT_SUBSCRIBING,
    RECEIPT_SUBSCRIBED
} receipt_phase_t;

STATIC_ASSERT(sizeof(session_phase_t) <= 4U, "session_phase_t size");
STATIC_ASSERT(MQTT_SESSION_TOPIC_SIZE + TELEMETRY_BARCODE_RECEIPT_WIRE_SIZE +
                      MQTT_SESSION_WIRE_OVERHEAD <=
                  MQTT_VAR_HEADER_BUFFER_LEN,
              "the command and receipt packets fit the MQTT receive buffer");
STATIC_ASSERT(TELEMETRY_CMD_WIRE_SIZE <= TELEMETRY_BARCODE_RECEIPT_WIRE_SIZE,
              "the command fits the shared receive buffer");
STATIC_ASSERT(MQTT_SESSION_TOPIC_SIZE + TELEMETRY_WIRE_MAX +
                      MQTT_SESSION_WIRE_OVERHEAD +
                      MQTT_SESSION_CONTROL_HEADROOM <
                  MQTT_OUTPUT_RINGBUF_SIZE,
              "the largest publication leaves room for protocol replies");

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
static bool              g_b_retrying;
static bool              g_b_dns_pending;
/* A DHCP lease may carry no DNS server, so a broker name cannot resolve. */
static bool              g_b_lookup_warned;
static bool              g_b_disconnect;
static char              g_command_topic[MQTT_SESSION_TOPIC_SIZE];
static char              g_receipt_topic[MQTT_SESSION_TOPIC_SIZE];
static uint8_t           g_incoming[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
static uint16_t          g_incoming_want;
static char              g_incoming_topic;
static uint16_t          g_incoming_used;
static bool              g_b_heartbeat_sent;
static uint8_t           g_record[TELEMETRY_FRAME_MAX];
static uint16_t          g_record_len;
static bool              g_b_record_inflight;
static bool              g_b_record_retrying;
static bool              g_b_status_inflight;
static uint32_t          g_record_retry_ms;
static uint8_t           g_batch[TELEMETRY_BARCODE_BATCH_WIRE_SIZE];
static uint8_t           g_batch_count;
static uint32_t          g_batch_last_record;
static uint32_t          g_batch_attempt_ms;
static bool              g_b_batch_inflight;
static bool              g_b_batch_waiting;
static bool              g_b_batch_retrying;
static receipt_phase_t   g_receipt_phase;
static uint32_t          g_receipt_retry_ms;
static volatile uint32_t g_drops;
static volatile uint32_t g_publish_retries;
static volatile uint32_t g_car_id;
static uint32_t          g_boot_id;

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

/* The NO_SYS radio task owns both the cursors and the enqueue. The pinned
 * codec can free QoS 0 requests while their bytes still occupy the ring. */
static bool output_has_room (const mqtt_client_t * p_client, uint16_t len)
{
    const struct mqtt_ringbuf_t * const p_ring = &p_client->output;
    uint32_t const                      put    = p_ring->put;
    uint32_t const                      get    = p_ring->get;
    uint32_t const                      used =
        (put >= get) ? (put - get) : (MQTT_OUTPUT_RINGBUF_SIZE - get + put);

    /* Leave one byte unused because the codec treats equal cursors as empty. */
    return (used + MQTT_SESSION_TOPIC_SIZE + MQTT_SESSION_WIRE_OVERHEAD + len +
                MQTT_SESSION_CONTROL_HEADROOM <
            MQTT_OUTPUT_RINGBUF_SIZE);
}

static err_t subscribe_topic (mqtt_client_t * p_client, const char * p_topic,
                              uint8_t qos, mqtt_request_cb_t done)
{
    /* One byte for the requested subscription QoS follows the topic. */
    if (!output_has_room(p_client, 1U))
    {
        return ERR_MEM;
    }
    return mqtt_subscribe(p_client, p_topic, qos, done, NULL);
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
    if (g_phase == SESSION_ONLINE)
    {
        publish_session(false);
    }
    g_phase               = SESSION_IDLE;
    g_b_disconnect        = false;
    g_incoming_topic      = '\0';
    g_b_heartbeat_sent    = false;
    if (g_b_record_inflight)
    {
        /* lwIP clears pending requests without calling their handlers. */
        g_publish_retries++;
    }
    g_b_record_inflight   = false;
    g_b_record_retrying   = false;
    g_b_status_inflight   = false;
    if (g_b_batch_waiting)
    {
        g_publish_retries++;
    }
    g_b_batch_inflight    = false;
    g_b_batch_waiting     = false;
    g_b_batch_retrying    = false;
    g_receipt_phase       = RECEIPT_IDLE;
    g_b_retrying          = true;
    g_retry_ms            = now_ms + MQTT_SESSION_RETRY_MS;
}

static void queue_command (uint16_t len)
{
    /* A one-byte record represents malformed length, including empty publications. */
    INT const size = (len == TELEMETRY_CMD_WIRE_SIZE || len == TELEMETRY_BARCODE_MAP_WIRE_SIZE) ? (INT)len : 1;
    SZ const cost = (SZ)(sizeof(INT) + (((size_t)size + sizeof(INT) - 1U) / sizeof(INT)) * sizeof(INT));
    telemetry_cmd_t command;
    SZ reserve = (SZ)(MQTT_SESSION_STOP_RECORDS * MQTT_SESSION_COMMAND_RECORD_BYTES);

    if (telemetry_cmd_parse(g_incoming, len, &command) == TELEMETRY_CMD_OK)
    {
        if (command.opcode == TELEMETRY_CMD_ESTOP)
        {
            reserve = 0U;
        }
        else if (command.opcode == TELEMETRY_CMD_STOP)
        {
            reserve = (SZ)MQTT_SESSION_COMMAND_RECORD_BYTES;
        }
    }
    /* The radio is the only producer, so the reader can only increase the
     * free space before the polling send. Leave space for the stop commands. */
    T_RMBF info;
    if (((reserve != 0U) &&
         ((tk_ref_mbf(g_command_queue_id, &info) != E_OK) ||
          (info.frbufsz < reserve + cost))) ||
        (tk_snd_mbf(g_command_queue_id, g_incoming, size, TMO_POL) != E_OK))
    {
        g_drops++;
        return;
    }
    (void)tk_wup_tsk(g_connection_task);
}

static void on_incoming_publish (void * p_arg, const char * p_topic, u32_t len)
{
    (void)p_arg;
    bool const b_online = (g_phase == SESSION_ONLINE);

    g_incoming_topic = '\0';
    g_incoming_used  = 0U;
    /* MQTT permits a matching PUBLISH while SUBACK is pending. A failed
     * subscription stays in this phase until the poll closes the session. */
    if (!b_online && ((g_phase != SESSION_SUBSCRIBING) || g_b_disconnect))
    {
        return;
    }
    if (strcmp(p_topic, g_command_topic) == 0)
    {
        if ((len != TELEMETRY_CMD_WIRE_SIZE) && (len != TELEMETRY_BARCODE_MAP_WIRE_SIZE))
        {
            if (b_online)
            {
                queue_command(0U);
            }
            return;
        }
        g_incoming_topic = TELEMETRY_TOPIC_COMMAND;
        g_incoming_want = (uint16_t)len;
    }
    else if (b_online && (len == TELEMETRY_BARCODE_RECEIPT_WIRE_SIZE) &&
             (strcmp(p_topic, g_receipt_topic) == 0))
    {
        g_incoming_topic = TELEMETRY_TOPIC_BARCODE_RECEIPT;
        g_incoming_want = TELEMETRY_BARCODE_RECEIPT_WIRE_SIZE;
    }
    else
    {
        /* Ignore all other publications. */
    }
}

static uint32_t receipt_word (uint32_t offset)
{
    uint32_t word = 0U;

    for (uint32_t i = 0U; i < TELEMETRY_BARCODE_RECORD_ID_BYTES; i++)
    {
        word = (word << MQTT_SESSION_BYTE_BITS) | g_incoming[offset + i];
    }
    return word;
}

static void accept_receipt (void)
{
    if ((g_batch_count != 0U) &&
        (receipt_word(TELEMETRY_BARCODE_RECEIPT_OFF_CAR_ID) == g_car_id) &&
        (receipt_word(TELEMETRY_BARCODE_RECEIPT_OFF_BOOT_ID) == g_boot_id) &&
        (receipt_word(TELEMETRY_BARCODE_RECEIPT_OFF_LAST_RECORD_ID) == g_batch_last_record) &&
        telemetry_accept_barcode_batch(g_batch_count, g_batch_last_record))
    {
        g_batch_count = 0U;
        g_b_batch_waiting = false;
        g_b_batch_retrying = false;
    }
}

static void on_incoming_data (void * p_arg, const u8_t * p_data, u16_t len,
                              u8_t flags)
{
    (void)p_arg;
    if (g_incoming_topic == '\0')
    {
        return;
    }
    bool const b_command = (g_incoming_topic == TELEMETRY_TOPIC_COMMAND);
    uint16_t const want = g_incoming_want;
    if (len > (want - g_incoming_used))
    {
        if (b_command)
        {
            queue_command(0U);
        }
        g_incoming_topic = '\0';
        return;
    }
    if (len != 0U)
    {
        (void)memcpy(&g_incoming[g_incoming_used], p_data, len);
    }
    g_incoming_used = (uint16_t)(g_incoming_used + len);
    if ((flags & (uint8_t)MQTT_DATA_FLAG_LAST) != 0U)
    {
        if (b_command)
        {
            queue_command((g_incoming_used == want) ? g_incoming_used : 0U);
        }
        else if (g_incoming_used == want)
        {
            accept_receipt();
        }
        else
        {
            /* Ignore truncated receipts. */
        }
        g_incoming_topic = '\0';
    }
}

static void receipt_subscribed (void * p_arg, err_t err)
{
    (void)p_arg;
    g_receipt_phase = (err == ERR_OK) ? RECEIPT_SUBSCRIBED : RECEIPT_IDLE;
    g_receipt_retry_ms = sys_now() + MQTT_SESSION_RECEIPT_RETRY_MS;
}

static void service_receipt_subscription (uint32_t now_ms)
{
    if ((g_boot_id == 0U) || (g_receipt_phase != RECEIPT_IDLE) ||
        ((int32_t)(now_ms - g_receipt_retry_ms) < 0))
    {
        return;
    }
    err_t const err =
        subscribe_topic(gp_client, g_receipt_topic, MQTT_SESSION_RECORD_QOS,
                        receipt_subscribed);

    if (err == ERR_OK)
    {
        g_receipt_phase = RECEIPT_SUBSCRIBING;
    }
    else
    {
        g_receipt_retry_ms = now_ms + MQTT_SESSION_RECEIPT_RETRY_MS;
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
    g_phase = SESSION_ONLINE;
    publish_session(true);
    g_receipt_retry_ms = sys_now();
    service_receipt_subscription(g_receipt_retry_ms);
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
    g_phase = SESSION_SUBSCRIBING;
    if (subscribe_topic(p_client, g_command_topic, MQTT_SESSION_COMMAND_QOS,
                        on_subscribed) != ERR_OK)
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
        make_topic(g_receipt_topic, TELEMETRY_TOPIC_BARCODE_RECEIPT);
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

/* Warn once per failure streak; the session keeps retrying. */
static void note_lookup (bool b_failed)
{
    if (b_failed && !g_b_lookup_warned)
    {
        LOG_WARN(LOG_TASK_CONNECTION, LOG_MOD_MQTT, "broker %s did not resolve",
                 TELEMETRY_BROKER_HOST);
    }
    g_b_lookup_warned = b_failed;
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
        note_lookup(p_addr == NULL);
        schedule_retry(now_ms);
        return;
    }
    note_lookup(false);
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
        note_lookup(false);
        connect_broker(&address, now_ms);
    }
    else if (err == ERR_INPROGRESS)
    {
        g_b_dns_pending = true;
    }
    else
    {
        note_lookup(true);
        schedule_retry(now_ms);
    }
}

static err_t publish_payload (char suffix, const uint8_t * p_payload,
                              uint16_t len, uint8_t qos, mqtt_request_cb_t done)
{
    char topic[MQTT_SESSION_TOPIC_SIZE];

    if (!output_has_room(gp_client, len))
    {
        return ERR_MEM;
    }
    make_topic(topic, suffix);
    return mqtt_publish(gp_client, topic, p_payload, len, qos, 0U, done, NULL);
}

static void note_publish (err_t err)
{
    if (err != ERR_MEM)
    {
        system_fault_assign(FAULT_OWNER_LINK, FAULT_MQTT_NACK_BIT,
                            (err == ERR_OK) ? 0U : FAULT_MQTT_NACK_BIT);
    }
}

static int publish_frame (const uint8_t * p_frame, uint16_t len)
{
    if (len <= 1U)
    {
        g_drops++;
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
        case TELEMETRY_TOPIC_PROFILE:
        case TELEMETRY_TOPIC_HEARTBEAT:
            break;
        case TELEMETRY_TOPIC_EVENT:
        case TELEMETRY_TOPIC_BARCODE_SAMPLE:
            (void)memcpy(g_record, p_frame, len);
            g_record_len = len;
            /* No later queued frame can overtake the head record. */
            return -1;
        default:
            g_drops++;
            return -1;
    }
    err_t const err =
        publish_payload(suffix, &p_frame[1], (uint16_t)(len - 1U),
                        MQTT_SESSION_PUBLISH_QOS, NULL);

    note_publish(err);
    if (err != ERR_OK)
    {
        g_drops++;
    }
    return (err == ERR_OK) ? 0 : -1;
}

static void record_acknowledged (void * p_arg, err_t result)
{
    (void)p_arg;
    if (result == ERR_OK)
    {
        g_record_len = 0U;
        g_b_record_inflight = false;
    }
    else
    {
        /* The next session retries the same record. */
        g_b_disconnect = true;
    }
}

static void service_record (uint32_t now_ms)
{
    if ((g_record_len == 0U) || g_b_record_inflight ||
        (g_b_record_retrying &&
         ((uint32_t)(now_ms - g_record_retry_ms) < MQTT_SESSION_PUBLISH_RETRY_MS)))
    {
        return;
    }

    err_t const result = publish_payload((char)g_record[0], &g_record[1],
                                         (uint16_t)(g_record_len - 1U),
                                         MQTT_SESSION_RECORD_QOS,
                                         record_acknowledged);
    note_publish(result);
    g_b_record_inflight = (result == ERR_OK);
    g_b_record_retrying = (result != ERR_OK);
    if (g_b_record_retrying)
    {
        g_record_retry_ms = now_ms;
        g_publish_retries++;
    }
}

static void batch_puback (void * p_arg, err_t result)
{
    (void)p_arg;
    g_b_batch_inflight = false;
    if (result != ERR_OK)
    {
        g_b_disconnect = true;
    }
}

static void service_batch (uint32_t now_ms)
{
    uint32_t const retry_ms = g_b_batch_waiting ? MQTT_SESSION_RECEIPT_RETRY_MS :
                                                MQTT_SESSION_PUBLISH_RETRY_MS;

    if ((g_receipt_phase != RECEIPT_SUBSCRIBED) || g_b_batch_inflight ||
        ((g_b_batch_waiting || g_b_batch_retrying) &&
         ((uint32_t)(now_ms - g_batch_attempt_ms) < retry_ms)))
    {
        return;
    }
    if (g_batch_count == 0U)
    {
        telemetry_barcode_batch_t batch = {.car_id = g_car_id,
                                           .boot_id = g_boot_id};
        telemetry_peek_barcode_batch(&batch);
        if (telemetry_pack_barcode_batch(g_batch, &batch) == 0U)
        {
            return;
        }
        g_batch_count = batch.count;
        g_batch_last_record = batch.records[batch.count - 1U].record_id;
    }

    err_t const result = publish_payload(TELEMETRY_TOPIC_BARCODE_SAMPLE, g_batch,
                                         sizeof(g_batch), MQTT_SESSION_RECORD_QOS,
                                         batch_puback);
    note_publish(result);
    if (g_b_batch_waiting || (result != ERR_OK))
    {
        g_publish_retries++;
    }
    g_b_batch_inflight = (result == ERR_OK);
    g_b_batch_waiting = g_b_batch_waiting || g_b_batch_inflight;
    g_b_batch_retrying = !g_b_batch_inflight;
    g_batch_attempt_ms = now_ms;
}

static void status_acknowledged (void * p_arg, err_t result)
{
    (void)p_arg;
    if (result == ERR_OK)
    {
        g_b_status_inflight = false;
    }
    else
    {
        /* A timed-out request can still have bytes in the MQTT output ring.
         * Reset the session before handing off another bundle. */
        g_b_disconnect = true;
    }
}

static void service_status (uint32_t now_ms)
{
    uint8_t  frame[TELEMETRY_STATUS_BUNDLE_FRAME_SIZE];
    uint32_t version = 0U;

    if (g_b_status_inflight || !telemetry_net_queue_peek_status(frame, &version))
    {
        return;
    }
    /* QoS 0 callbacks can fire on a TCP ACK for an earlier packet. */
    err_t const result = publish_payload(TELEMETRY_TOPIC_STATUS, &frame[1],
                                         TELEMETRY_STATUS_BUNDLE_WIRE_SIZE,
                                         MQTT_SESSION_STATUS_QOS,
                                         status_acknowledged);
    note_publish(result);
    if (result == ERR_OK)
    {
        g_b_status_inflight = true;
        g_deadline_ms = now_ms + MQTT_SESSION_REPLY_MS;
        telemetry_net_queue_accept_status(version);
    }
    else
    {
        g_publish_retries++;
    }
}

static void service_online (uint32_t now_ms)
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
                            (uint16_t)sizeof(heartbeat),
                            MQTT_SESSION_PUBLISH_QOS, NULL) != ERR_OK)
        {
            (void)mqtt_session_heartbeat(heartbeat);
            return;
        }
        g_b_heartbeat_sent = true;
    }
    if (g_b_heartbeat_sent)
    {
        if (g_record_len == 0U)
        {
            telemetry_worker_run(telemetry_net_queue_pop_fifo, publish_frame, TMO_POL);
        }
        service_record(now_ms);
        if ((g_record_len == 0U) || g_b_record_inflight)
        {
            service_batch(now_ms);
            service_status(now_ms);
        }
    }
    service_receipt_subscription(now_ms);
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
        .maxmsz = TELEMETRY_BARCODE_MAP_WIRE_SIZE,
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

void mqtt_session_set_boot_id (uint32_t boot_id)
{
    g_boot_id = (boot_id == 0U) ? 1U : boot_id;
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

bool mqtt_session_command (uint8_t    payload[TELEMETRY_BARCODE_MAP_WIRE_SIZE],
                           uint16_t * p_len)
{
    INT const size = tk_rcv_mbf(g_command_queue_id, payload, TMO_POL);

    *p_len = (size == (INT)TELEMETRY_CMD_WIRE_SIZE || size == (INT)TELEMETRY_BARCODE_MAP_WIRE_SIZE)
                 ? (uint16_t)size : 0U;
    return size > 0;
}

uint32_t mqtt_session_drops (void)
{
    return g_drops;
}

uint32_t mqtt_session_publish_retries (void)
{
    return g_publish_retries;
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
        (g_phase != SESSION_ONLINE) || g_b_status_inflight;
    /* The first SDK tick proves TCP is established. The initial send is
     * the codec's only chance to drain CONNECT before CONNACK. */
    bool const b_connect_stalled =
        (g_phase == SESSION_CONNECTING) && (gp_client != NULL) &&
        (gp_client->cyclic_tick != 0U) &&
        (gp_client->output.put != gp_client->output.get);

    if ((g_phase != SESSION_IDLE) &&
        (!b_ready || g_b_disconnect || b_connect_stalled ||
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
        service_online(now_ms);
    }
    else if ((g_phase == SESSION_IDLE) && b_ready && !g_b_dns_pending &&
             (g_car_id != 0U) &&
             (!g_b_retrying || ((int32_t)(now_ms - g_retry_ms) >= 0)))
    {
        begin_attempt(now_ms);
    }
}
