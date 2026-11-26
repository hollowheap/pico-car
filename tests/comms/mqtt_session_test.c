/* Run the pinned SDK MQTT codec and session adapter against a scripted TCP peer. */
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"

#include "mqtt_session.h"
#include "platform/faults.h"
#include "platform/log.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/queue.h"
#include "comms/telemetry/drv_wifi_link.h"
#include "tk/mbf_fake.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define CAR_ID 0xC10ABCF0U

static uint32_t g_now;
static wifi_lease_t g_lease = WIFI_LEASE_READY;
static struct tcp_pcb g_pcb;
const ip_addr_t ip_addr_any = IPADDR4_INIT(0U);
static ip_addr_t g_addr;
static tcp_connected_fn g_on_connected;
static tcp_recv_fn g_on_recv;
static tcp_err_fn g_on_error;
static tcp_poll_fn g_on_poll;
static tcp_sent_fn g_on_sent;
static void * gp_tcp_arg;
static sys_timeout_handler g_timer;
static void * gp_timer_arg;
static dns_found_callback g_resolve;
static void * gp_dns_arg;
static bool g_async_dns;
static bool g_dns_fail;
static unsigned g_warnings;
static unsigned g_attempts;
static unsigned g_closed;
static unsigned g_data_writes;
static uint8_t g_tx[MQTT_OUTPUT_RINGBUF_SIZE];
static size_t g_tx_len;
static bool g_block_write;
static uint32_t g_fault;

u32_t sys_now (void) { return g_now; }
wifi_lease_t wifi_link_lease (void) { return g_lease; }

ER ISpinLock (T_SPLOCK * p_lock, UINT * p_saved)
{
    *p_saved = 0U;
    return SpinLock(p_lock);
}

ER ISpinUnlock (T_SPLOCK * p_lock, UINT saved)
{
    (void)saved;
    return SpinUnlock(p_lock);
}

ER tk_wup_tsk (ID task) { assert(task == 1); return E_OK; }

ER __real_tk_snd_mbf (ID mbfid, const void * p_msg, INT len, TMO timeout);

ER __wrap_tk_snd_mbf (ID mbfid, const void * p_msg, INT len, TMO timeout)
{
    assert(timeout == TMO_POL);
    return __real_tk_snd_mbf(mbfid, p_msg, len, timeout);
}

void system_fault_assign (fault_owner_t owner, uint32_t mask, uint32_t active)
{
    assert(owner == FAULT_OWNER_LINK && mask == FAULT_MQTT_NACK_BIT);
    g_fault = active;
}

static void offer_event (uint16_t id)
{
    uint8_t frame[TELEMETRY_EVENT_QUEUED_SIZE] = {TELEMETRY_TOPIC_EVENT};

    frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID] = (uint8_t)(id >> 8U);
    frame[2U + TELEMETRY_EVENT_OFF_EVENT_ID] = (uint8_t)id;
    frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] = TELEMETRY_EVT_BARCODE;
    assert(telemetry_net_queue_push(frame, sizeof(frame)) == E_OK);
}

static void offer_profile (void)
{
    const uint8_t frame[TELEMETRY_PROFILE_WIRE_SIZE + 1U] = {TELEMETRY_TOPIC_PROFILE};

    assert(telemetry_net_queue_push(frame, sizeof(frame)) == E_OK);
}

static void offer_status (uint8_t sequence)
{
    uint8_t frame[TELEMETRY_STATUS_BUNDLE_FRAME_SIZE] = {TELEMETRY_TOPIC_STATUS};

    frame[1] = sequence;
    assert(telemetry_net_queue_push(frame, sizeof(frame)) == E_OK);
}

void * mem_calloc (mem_size_t count, mem_size_t size) { return calloc(count, size); }
void mem_free (void * p_mem) { free(p_mem); }

void sys_timeout (u32_t ms, sys_timeout_handler callback, void * p_arg)
{
    assert(ms == 1000U && g_timer == NULL);
    g_timer = callback;
    gp_timer_arg = p_arg;
}

void sys_untimeout (sys_timeout_handler callback, void * p_arg)
{
    if (g_timer == callback && gp_timer_arg == p_arg)
    {
        g_timer = NULL;
        gp_timer_arg = NULL;
    }
}

err_t dns_gethostbyname (const char * p_name, ip_addr_t * p_addr,
                        dns_found_callback found, void * p_arg)
{
    assert(strcmp(p_name, "localhost") == 0);
    if (g_dns_fail)
    {
        /* lwIP returns ERR_VAL when the lease gave no DNS server. */
        return ERR_VAL;
    }
    if (g_async_dns)
    {
        assert(g_resolve == NULL);
        g_resolve = found;
        gp_dns_arg = p_arg;
        return ERR_INPROGRESS;
    }
    *p_addr = g_addr;
    return ERR_OK;
}

struct tcp_pcb * tcp_new_ip_type (u8_t type)
{
    assert(type == IPADDR_TYPE_V4);
    g_pcb.snd_buf = sizeof(g_tx);
    return &g_pcb;
}

void tcp_arg (struct tcp_pcb * p_pcb, void * p_arg)
{
    assert(p_pcb == &g_pcb);
    gp_tcp_arg = p_arg;
}
void tcp_recv (struct tcp_pcb * p_pcb, tcp_recv_fn recv) { assert(p_pcb == &g_pcb); g_on_recv = recv; }
void tcp_err (struct tcp_pcb * p_pcb, tcp_err_fn err) { assert(p_pcb == &g_pcb); g_on_error = err; }
void tcp_poll (struct tcp_pcb * p_pcb, tcp_poll_fn poll, u8_t interval)
{
    (void)interval;
    assert(p_pcb == &g_pcb);
    g_on_poll = poll;
}
void tcp_sent (struct tcp_pcb * p_pcb, tcp_sent_fn sent) { assert(p_pcb == &g_pcb); g_on_sent = sent; }
void tcp_recved (struct tcp_pcb * p_pcb, u16_t len) { assert(p_pcb == &g_pcb && len > 0U); }
void tcp_abort (struct tcp_pcb * p_pcb) { assert(p_pcb == &g_pcb); g_closed++; }
err_t tcp_bind (struct tcp_pcb * p_pcb, const ip_addr_t * p_addr, u16_t port)
{
    assert(p_pcb == &g_pcb && ip_addr_isany(p_addr) && port == 0U);
    return ERR_OK;
}
err_t tcp_close (struct tcp_pcb * p_pcb) { assert(p_pcb == &g_pcb); g_closed++; return ERR_OK; }
err_t tcp_output (struct tcp_pcb * p_pcb) { assert(p_pcb == &g_pcb); return ERR_OK; }

err_t tcp_connect (struct tcp_pcb * p_pcb, const ip_addr_t * p_addr, u16_t port, tcp_connected_fn connected)
{
    assert(p_pcb == &g_pcb && ip_addr_cmp(p_addr, &g_addr) && port == 1883U);
    g_on_connected = connected;
    g_attempts++;
    return ERR_OK;
}

err_t tcp_write (struct tcp_pcb * p_pcb, const void * p_data, u16_t len, u8_t flags)
{
    assert(p_pcb == &g_pcb && g_tx_len + len <= sizeof(g_tx));
    assert((flags == TCP_WRITE_FLAG_COPY) ||
           (flags == (TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE)));
    if (g_block_write)
    {
        return ERR_MEM;
    }
    (void)memcpy(&g_tx[g_tx_len], p_data, len);
    g_tx_len += len;
    g_data_writes++;
    return ERR_OK;
}

u8_t pbuf_free (struct pbuf * p_buf) { assert(p_buf != NULL); return 1U; }

u8_t pbuf_get_at (const struct pbuf * p_buf, u16_t offset)
{
    assert(offset < p_buf->len);
    return ((const uint8_t *)p_buf->payload)[offset];
}

void * pbuf_get_contiguous (const struct pbuf * p_buf, void * p_dst, size_t capacity, u16_t len, u16_t off)
{
    (void)p_dst;
    assert(len <= capacity && off + len <= p_buf->len);
    return &((uint8_t *)p_buf->payload)[off];
}

static void receive (const uint8_t * p_data, size_t len)
{
    uint8_t data[64];
    assert(len <= sizeof(data) && g_on_recv != NULL);
    (void)memcpy(data, p_data, len);
    struct pbuf buf = {.payload = data, .len = (u16_t)len, .tot_len = (u16_t)len};
    assert(ERR_OK == g_on_recv(gp_tcp_arg, &g_pcb, &buf, ERR_OK));
}

static void expect_tx (const uint8_t * p_expected, size_t len)
{
    assert(g_tx_len == len && memcmp(g_tx, p_expected, len) == 0);
    g_tx_len = 0U;
}

static void poll_at (uint32_t now)
{
    g_now = now;
    mqtt_session_poll();
}

static void connack (void)
{
    static const uint8_t accept[] = {0x20U, 2U, 0U, 0U};
    assert(g_on_connected != NULL && ERR_OK == g_on_connected(gp_tcp_arg, &g_pcb, ERR_OK));
    receive(accept, sizeof(accept));
}

static void suback (bool b_ok)
{
    const uint8_t ack[] = {0x90U, 3U, 0U, 1U, b_ok ? 1U : 0x80U};
    receive(ack, sizeof(ack));
}

static void acknowledge (uint16_t packet_id)
{
    const uint8_t puback[] = {0x40U, 2U, (uint8_t)(packet_id >> 8U), (uint8_t)packet_id};
    receive(puback, sizeof(puback));
}

static void test_wire (void)
{
    static const uint8_t connect[] = {
        0x10U, 25U, 0U, 4U, 'M', 'Q', 'T', 'T', 4U, 2U, 0U, 30U, 0U, 13U,
        't', 'e', 's', 't', '-', 'c', '1', '0', 'a', 'b', 'c', 'f', '0',
    };
    static const uint8_t subscribe[] = {0x82U, 11U, 0U, 1U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 1U};
    static const uint8_t heartbeat[] = {
        0x30U, 22U, 0U, 6U, 't', 'e', 's', 't', '/', 'h', 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,
        10U, 11U, 12U, 13U,
    };
    static const uint8_t event[12U + TELEMETRY_EVENT_WIRE_SIZE] = {
        0x32U, 22U, 0U, 6U, 't', 'e', 's', 't', '/', 'e', 0U, 2U,
        0xA5U, 0x5AU, TELEMETRY_EVT_BARCODE,
    };
    static const uint8_t command_start[] = {0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U, 0U, 9U};
    static const uint8_t command_end[] = {0x11U, 0x22U, 6U, 0U, 0U, 0U, 1U};
    static const uint8_t command[] = {0U, 9U, 0x11U, 0x22U, 6U, 0U, 0U, 0U, 1U};
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    static const uint8_t wrong_topic[] = {0x30U, 8U, 0U, 6U, 't', 'e', 's', 't', '/', 'x'};
    static const uint8_t empty[] = {0x30U, 8U, 0U, 6U, 't', 'e', 's', 't', '/', 'c'};
    uint8_t payload[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
    uint16_t len;

    poll_at(0U);
    assert(g_attempts == 1U && !mqtt_session_connected());
    assert(ERR_OK == g_on_connected(gp_tcp_arg, &g_pcb, ERR_OK));
    expect_tx(connect, sizeof(connect));
    const uint8_t accept[] = {0x20U, 2U, 0U, 0U};
    receive(accept, sizeof(accept));
    expect_tx(subscribe, sizeof(subscribe));
    receive(empty, sizeof(empty));
    assert(!mqtt_session_command(payload, &len));
    offer_event(0xA55AU);
    poll_at(2U);
    assert(g_tx_len == 0U && !mqtt_session_connected());
    suback(true);
    assert(mqtt_session_connected());
    poll_at(4U);
    assert(g_tx_len == 0U);
    assert(mqtt_session_heartbeat(&heartbeat[10]));
    poll_at(6U);
    assert(g_tx_len == sizeof(heartbeat) + sizeof(event));
    assert(memcmp(g_tx, heartbeat, sizeof(heartbeat)) == 0);
    assert(memcmp(&g_tx[sizeof(heartbeat)], event, sizeof(event)) == 0);
    assert(g_fault == 0U);
    g_tx_len = 0U;
    acknowledge(2U);

    receive(command_start, sizeof(command_start));
    assert(!mqtt_session_command(payload, &len));
    receive(command_end, sizeof(command_end));
    assert(mqtt_session_command(payload, &len));
    assert(len == sizeof(command) && memcmp(payload, command, sizeof(command)) == 0);
    expect_tx(puback, sizeof(puback));
    receive(wrong_topic, sizeof(wrong_topic));
    assert(!mqtt_session_command(payload, &len));
    receive(empty, sizeof(empty));
    assert(mqtt_session_command(payload, &len) && len == 0U);

    g_block_write = true;
    for (unsigned i = 0U; i < MQTT_REQ_MAX_IN_FLIGHT; i++)
    {
        offer_profile();
        poll_at(8U);
    }
    assert(mqtt_session_drops() != 0U && mqtt_session_connected() && g_fault == 0U);

    /* After the TCP error, the next session must receive commands too. */
    g_block_write = false;
    g_on_error(gp_tcp_arg, ERR_RST);
    assert(!mqtt_session_connected());
    poll_at(1008U);
    connack();
    suback(true);
    assert(mqtt_session_connected());
    g_tx_len = 0U;
    receive(command_start, sizeof(command_start));
    receive(command_end, sizeof(command_end));
    assert(mqtt_session_command(payload, &len));
    assert(len == sizeof(command) && memcmp(payload, command, sizeof(command)) == 0);
    expect_tx(puback, sizeof(puback));
}

static void test_command_before_suback (void)
{
    static const uint8_t subscribe[] = {0x82U, 11U, 0U, 1U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 1U};
    static const uint8_t stop[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U,
        0U, 9U, 0x11U, 0x22U, TELEMETRY_CMD_STOP, 0U, 0U, 0U, 1U,
    };
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {
        [TELEMETRY_HEARTBEAT_OFF_UPTIME_S + 3U] = 1U,
        [TELEMETRY_HEARTBEAT_OFF_RUN_ID] = 0x11U,
        [TELEMETRY_HEARTBEAT_OFF_RUN_ID + 1U] = 0x22U,
        [TELEMETRY_HEARTBEAT_OFF_CAR_ID] = (uint8_t)(CAR_ID >> 24U),
        [TELEMETRY_HEARTBEAT_OFF_CAR_ID + 1U] = (uint8_t)(CAR_ID >> 16U),
        [TELEMETRY_HEARTBEAT_OFF_CAR_ID + 2U] = (uint8_t)(CAR_ID >> 8U),
        [TELEMETRY_HEARTBEAT_OFF_CAR_ID + 3U] = (uint8_t)CAR_ID,
    };
    uint8_t command[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
    uint16_t len;

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(1000U);
    assert(g_tx_len == 10U + sizeof(heartbeat) && g_tx[9U] == TELEMETRY_TOPIC_HEARTBEAT);
    assert(memcmp(&g_tx[10U], heartbeat, sizeof(heartbeat)) == 0);

    g_on_error(gp_tcp_arg, ERR_RST);
    g_tx_len = 0U;
    poll_at(2000U);
    connack();
    assert(!mqtt_session_connected());
    assert(g_tx_len > sizeof(subscribe));
    assert(memcmp(&g_tx[g_tx_len - sizeof(subscribe)], subscribe, sizeof(subscribe)) == 0);
    g_tx_len = 0U;

    /* A broker may send a matching PUBLISH before it sends SUBACK. */
    receive(stop, sizeof(stop));
    expect_tx(puback, sizeof(puback));
    suback(true);
    assert(mqtt_session_connected());
    assert(mqtt_session_command(command, &len) && "PUBACK discarded a valid STOP");
    assert(len == TELEMETRY_CMD_WIRE_SIZE && memcmp(command, &stop[12], len) == 0);
    assert(!mqtt_session_command(command, &len));
}

static void test_command_subscription_recovery (void)
{
    static const uint8_t stop[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U,
        0U, 9U, 0x11U, 0x22U, TELEMETRY_CMD_STOP, 0U, 0U, 0U, 1U,
    };
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    uint8_t command[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
    uint16_t len;

    poll_at(0U);
    connack();
    suback(false);
    g_tx_len = 0U;
    /* lwIP can receive more packets before the deferred disconnect. */
    receive(stop, sizeof(stop));
    expect_tx(puback, sizeof(puback));
    assert(!mqtt_session_command(command, &len));
    poll_at(2U);
    assert(g_closed == 1U && !mqtt_session_connected());
    poll_at(1001U);
    assert(g_attempts == 1U);

    poll_at(1002U);
    connack();
    g_tx_len = 0U;
    receive(stop, sizeof(stop) - 1U);
    assert(g_tx_len == 0U && !mqtt_session_command(command, &len));
    g_on_error(gp_tcp_arg, ERR_RST);

    poll_at(3002U);
    connack();
    g_tx_len = 0U;
    assert(!mqtt_session_command(command, &len));
    receive(stop, sizeof(stop));
    expect_tx(puback, sizeof(puback));
    /* The core 1 reader can drain even while SUBACK is still pending. */
    assert(!mqtt_session_connected());
    assert(mqtt_session_command(command, &len));
    assert(len == TELEMETRY_CMD_WIRE_SIZE && memcmp(command, &stop[12U], len) == 0);
    suback(true);
    assert(mqtt_session_connected() && !mqtt_session_command(command, &len));
}

static void test_command_backpressure (void)
{
    uint8_t command[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U,
        0U, 0U, 0x11U, 0x22U, TELEMETRY_CMD_START, 0U, 0U, 0U, 1U,
    };
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    uint8_t payload[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
    uint16_t len;
    unsigned const attempts = 32U;

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    for (unsigned i = 1U; i <= attempts; i++)
    {
        command[13U] = (uint8_t)i;
        receive(command, sizeof(command));
        expect_tx(puback, sizeof(puback));
    }
    unsigned const rejected = mqtt_session_drops();
    assert(rejected > 0U && rejected < attempts);
    unsigned const queued = attempts - rejected;

    command[13U] = 50U;
    command[16U] = TELEMETRY_CMD_STOP;
    receive(command, sizeof(command));
    expect_tx(puback, sizeof(puback));
    assert(mqtt_session_drops() == rejected);

    command[13U] = 51U;
    receive(command, sizeof(command));
    expect_tx(puback, sizeof(puback));
    assert(mqtt_session_drops() == rejected + 1U);

    command[13U] = 52U;
    command[16U] = TELEMETRY_CMD_ESTOP;
    receive(command, sizeof(command));
    expect_tx(puback, sizeof(puback));
    assert(mqtt_session_drops() == rejected + 1U);

    for (unsigned i = 1U; i <= queued; i++)
    {
        assert(mqtt_session_command(payload, &len));
        assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[4U] == TELEMETRY_CMD_START);
        assert(payload[1U] == i);
    }
    assert(mqtt_session_command(payload, &len));
    assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[4U] == TELEMETRY_CMD_STOP && payload[1U] == 50U);
    assert(mqtt_session_command(payload, &len));
    assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[4U] == TELEMETRY_CMD_ESTOP && payload[1U] == 52U);
    assert(!mqtt_session_command(payload, &len));
}

static void test_puback_space (void)
{
    static const uint8_t estop[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U,
        0U, 9U, 0x11U, 0x22U, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 1U,
    };
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    uint8_t payload[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
    uint16_t len;

    poll_at(0U);
    connack();
    suback(true);
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    g_tx_len = 0U;
    g_block_write = true;
    for (unsigned i = 0U; i < MQTT_REQ_MAX_IN_FLIGHT; i++)
    {
        offer_profile();
        poll_at(4U + i);
    }
    assert(mqtt_session_drops() == 1U);
    receive(estop, sizeof(estop));
    assert(mqtt_session_command(payload, &len));
    assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[4] == TELEMETRY_CMD_ESTOP);
    assert(g_tx_len == 0U);
    g_block_write = false;
    assert(g_on_poll(gp_tcp_arg, &g_pcb) == ERR_OK);
    assert(g_tx_len > 256U && memcmp(&g_tx[g_tx_len - sizeof(puback)], puback, sizeof(puback)) == 0);
}

static void test_output_headroom (void)
{
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE]   = {0};
    const uint8_t diag[1U + TELEMETRY_DIAG_COUNTS_WIRE_SIZE] = {
        TELEMETRY_TOPIC_DIAG};
    uint8_t command[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 0U,
        0U, 0U, 0x11U, 0x22U, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 1U,
    };
    const uint8_t  ping[]          = {0xC0U, 0U};
    uint16_t const profile_bytes   = 10U + TELEMETRY_PROFILE_WIRE_SIZE;
    uint16_t const heartbeat_bytes = 10U + sizeof(heartbeat);
    uint16_t const diag_bytes      = 10U + TELEMETRY_DIAG_COUNTS_WIRE_SIZE;
    unsigned const profile_count   = 15U;
    unsigned const diag_count      = 8U;
    unsigned const ack_count       = 32U;
    size_t const   ack_bytes       = ack_count * 4U;
    uint8_t        payload[TELEMETRY_BARCODE_MAP_WIRE_SIZE];
    uint16_t       len;

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    for (unsigned i = 0U; i < 3U; i++)
    {
        offer_profile();
        poll_at(4U + i);
    }
    assert(g_tx_len == heartbeat_bytes + 3U * profile_bytes);
    g_tx_len      = 0U;
    g_block_write = true;

    /* Old TCP ACKs release entire windows of QoS 0 requests even though the
     * subsequent bytes remain blocked. Without admission control, the first
     * command PUBACK would fill the MQTT output ring to exactly 2048 bytes. */
    unsigned offered = 0U;
    for (unsigned wave = 0U; wave < 3U; wave++)
    {
        uint16_t const acked = (wave == 0U)
                                   ? (uint16_t)(heartbeat_bytes + profile_bytes)
                                   : profile_bytes;
        assert(g_on_sent(gp_tcp_arg, &g_pcb, acked) == ERR_OK);
        for (unsigned slot = 0U; slot < MQTT_REQ_MAX_IN_FLIGHT &&
                                 offered < profile_count + diag_count;
             slot++, offered++)
        {
            if (offered < profile_count)
            {
                offer_profile();
            }
            else
            {
                assert(telemetry_net_queue_push(diag, sizeof(diag)) == E_OK);
            }
            poll_at(10U + offered);
        }
    }
    assert(offered == profile_count + diag_count && g_tx_len == 0U);
    for (uint8_t id = 1U; id <= ack_count; id++)
    {
        command[11] = id;
        command[13] = id;
        receive(command, sizeof(command));
        assert(mqtt_session_command(payload, &len));
        assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[1] == id &&
               payload[4] == TELEMETRY_CMD_ESTOP);
    }
    for (unsigned second = 1U; second <= 30U; second++)
    {
        sys_timeout_handler tick = g_timer;

        g_timer = NULL;
        g_now   = second * 1000U;
        assert(tick != NULL);
        tick(gp_timer_arg);
    }

    unsigned const drops = mqtt_session_drops();
    assert(drops <= diag_count && g_tx_len == 0U);
    size_t const telemetry_bytes =
        (profile_count * profile_bytes) + ((diag_count - drops) * diag_bytes);
    g_block_write = false;
    assert(g_on_poll(gp_tcp_arg, &g_pcb) == ERR_OK);
    assert(g_tx_len == telemetry_bytes + ack_bytes + sizeof(ping));
    assert(g_tx[9] == TELEMETRY_TOPIC_PROFILE && drops > 0U);
    for (uint8_t id = 1U; id <= ack_count; id++)
    {
        const uint8_t ack[] = {0x40U, 2U, 0U, id};
        assert(memcmp(&g_tx[telemetry_bytes + ((id - 1U) * sizeof(ack))], ack,
                      sizeof(ack)) == 0);
    }
    assert(memcmp(&g_tx[telemetry_bytes + ack_bytes], ping, sizeof(ping)) == 0);
    assert(mqtt_session_connected() && g_fault == 0U);
}

static uint16_t expect_retained_event (uint8_t id)
{
    size_t const topic_end = 10U;
    size_t const packet_id_size = (g_tx[0] == 0x32U) ? 2U : 0U;
    size_t const payload = topic_end + packet_id_size;

    assert(g_tx_len >= payload + TELEMETRY_EVENT_WIRE_SIZE);
    assert(g_tx[topic_end - 1U] == TELEMETRY_TOPIC_EVENT);
    assert(g_tx[payload + TELEMETRY_EVENT_OFF_EVENT_ID + 1U] == id &&
           "oldest event disappeared on publish backpressure");
    assert(g_tx_len == payload + TELEMETRY_EVENT_WIRE_SIZE && packet_id_size == 2U);
    g_tx_len = 0U;
    return (uint16_t)(((uint16_t)g_tx[topic_end] << 8U) | g_tx[topic_end + 1U]);
}

static void test_retained_events (void)
{
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    g_block_write = true;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    for (unsigned i = 0U; i < MQTT_REQ_MAX_IN_FLIGHT - 1U; i++)
    {
        offer_profile();
        poll_at(4U + i);
    }
    offer_event(1U);
    offer_event(2U);
    poll_at(12U);
    assert(g_tx_len == 0U);

    g_block_write = false;
    assert(g_on_poll(gp_tcp_arg, &g_pcb) == ERR_OK);
    assert(g_tx_len > 0U && g_on_sent(gp_tcp_arg, &g_pcb, (u16_t)g_tx_len) == ERR_OK);
    g_tx_len = 0U;
    poll_at(14U);
    assert(g_tx_len == 0U);
    poll_at(32U);
    (void)expect_retained_event(1U);
    poll_at(34U);
    assert(g_tx_len == 0U);
    assert(mqtt_session_drops() == 0U);

    /* The broker may have received the event before the link lost its PUBACK. */
    g_on_error(gp_tcp_arg, ERR_RST);
    assert(!mqtt_session_connected());
    poll_at(1034U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(1036U);
    assert(g_tx_len >= sizeof(heartbeat) + 10U);
    g_tx_len -= sizeof(heartbeat) + 10U;
    (void)memmove(g_tx, &g_tx[sizeof(heartbeat) + 10U], g_tx_len);
    uint16_t const first = expect_retained_event(1U);
    acknowledge(first);
    poll_at(1038U);
    uint16_t const second = expect_retained_event(2U);
    acknowledge(first);
    /* A stale acknowledgement cannot allow another record to overtake 2. */
    offer_event(3U);
    poll_at(1040U);
    assert(g_tx_len == 0U);
    acknowledge(second);
    poll_at(1042U);
    acknowledge(expect_retained_event(3U));
    poll_at(1044U);
    assert(g_tx_len == 0U && mqtt_session_drops() == 0U);
    assert(mqtt_session_publish_retries() == 2U);
}

static uint16_t expect_status (uint8_t sequence)
{
    assert(g_tx_len >= 2U && g_tx_len == 2U + g_tx[1]);
    assert(g_tx_len == 12U + TELEMETRY_STATUS_BUNDLE_WIRE_SIZE);
    assert(g_tx[0] == 0x32U && g_tx[9] == TELEMETRY_TOPIC_STATUS);
    assert(g_tx[12] == sequence);
    g_tx_len = 0U;
    return (uint16_t)(((uint16_t)g_tx[10] << 8U) | g_tx[11]);
}

static void test_latest_status (void)
{
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};
    size_t const heartbeat_size = 10U + sizeof(heartbeat);
    size_t const status_size = 12U + TELEMETRY_STATUS_BUNDLE_WIRE_SIZE;

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    assert(g_tx_len == heartbeat_size);
    g_tx_len = 0U;

    g_block_write = true;
    offer_status(1U);
    poll_at(4U);
    assert(g_tx_len == 0U);
    /* A TCP ACK for the earlier heartbeat can arrive while status 1 is
     * still waiting in the client's output ring. */
    assert(g_on_sent(gp_tcp_arg, &g_pcb, (u16_t)heartbeat_size) == ERR_OK);
    for (uint8_t seq = 2U; seq <= 10U; seq++)
    {
        offer_status(seq);
        poll_at(4U + seq);
    }
    assert(g_tx_len == 0U);
    g_block_write = false;
    assert(g_on_poll(gp_tcp_arg, &g_pcb) == ERR_OK);
    uint16_t const first = expect_status(1U);

    assert(g_on_sent(gp_tcp_arg, &g_pcb, (u16_t)status_size) == ERR_OK);
    poll_at(16U);
    assert(g_tx_len == 0U);
    acknowledge(first);
    poll_at(18U);
    acknowledge(expect_status(10U));
    poll_at(20U);
    assert(g_tx_len == 0U && telemetry_net_queue_status_replacements() == 8U);
    assert(mqtt_session_drops() == 0U && mqtt_session_publish_retries() == 0U);
}

static void test_latest_status_timeout (void)
{
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    assert(g_tx_len == 10U + sizeof(heartbeat));
    assert(g_on_sent(gp_tcp_arg, &g_pcb, (u16_t)g_tx_len) == ERR_OK);
    g_tx_len = 0U;
    g_block_write = true;
    offer_status(1U);
    poll_at(4U);
    offer_status(2U);
    poll_at(6U);
    assert(g_tx_len == 0U);

    /* A timed-out request may still have unsent bytes. Resetting the client
     * must discard these bytes before it publishes any later status. */
    poll_at(4003U);
    assert(mqtt_session_connected() && g_closed == 0U);
    poll_at(4004U);
    assert(!mqtt_session_connected() && g_closed == 1U && g_tx_len == 0U);
    offer_status(3U);
    g_block_write = false;
    poll_at(g_now + 1000U);
    assert(g_attempts == 2U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(g_now + 2U);
    size_t const heartbeat_size = 10U + sizeof(heartbeat);
    assert(g_tx_len > heartbeat_size && g_tx[9] == TELEMETRY_TOPIC_HEARTBEAT);
    g_tx_len -= heartbeat_size;
    (void)memmove(g_tx, &g_tx[heartbeat_size], g_tx_len);
    acknowledge(expect_status(3U));
    poll_at(g_now + 2U);
    assert(g_tx_len == 0U && mqtt_session_drops() == 0U);
}

static void put_u32 (uint8_t * out, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; i++)
    {
        out[i] = (uint8_t)(value >> (8U * (3U - i)));
    }
}

static void receipt (uint32_t boot_id, uint32_t record_id)
{
    uint8_t packet[12U + TELEMETRY_BARCODE_RECEIPT_WIRE_SIZE] = {
        0x32U, 22U, 0U, 6U, 't', 'e', 's', 't', '/', 'a', 0U, 8U,
    };
    static const uint8_t ack[] = {0x40U, 2U, 0U, 8U};

    put_u32(&packet[12U], CAR_ID);
    put_u32(&packet[16U], boot_id);
    put_u32(&packet[20U], record_id);
    receive(packet, sizeof(packet));
    expect_tx(ack, sizeof(ack));
}

static void receipt_suback (uint16_t packet_id, bool b_ok)
{
    const uint8_t ack[] = {0x90U, 3U, (uint8_t)(packet_id >> 8U),
                           (uint8_t)packet_id, b_ok ? 1U : 0x80U};
    receive(ack, sizeof(ack));
}

static uint16_t expect_receipt_subscribe (void)
{
    static const uint8_t topic[] = {0U, 6U, 't', 'e', 's', 't', '/', 'a', 1U};

    assert(g_tx_len == 4U + sizeof(topic) && g_tx[0] == 0x82U && g_tx[1] == 11U);
    assert(memcmp(&g_tx[4], topic, sizeof(topic)) == 0);
    g_tx_len = 0U;
    return (uint16_t)(((uint16_t)g_tx[2] << 8U) | g_tx[3]);
}

static uint16_t read_batch (uint8_t * payload)
{
    assert(g_tx_len == 12U + TELEMETRY_BARCODE_BATCH_WIRE_SIZE);
    assert(g_tx[0] == 0x32U && g_tx[9] == TELEMETRY_TOPIC_BARCODE_SAMPLE);
    memcpy(payload, &g_tx[12], TELEMETRY_BARCODE_BATCH_WIRE_SIZE);
    g_tx_len = 0U;
    return (uint16_t)(((uint16_t)g_tx[10] << 8U) | g_tx[11]);
}

static void test_barcode_receipt (void)
{
    const uint32_t boot = 0x12345678U;
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};
    uint8_t original[TELEMETRY_BARCODE_BATCH_WIRE_SIZE];
    uint8_t replay[TELEMETRY_BARCODE_BATCH_WIRE_SIZE];

    mqtt_session_set_boot_id(boot);
    for (uint32_t id = 1U; id <= 4U; id++)
    {
        const telemetry_barcode_sample_t sample = {.edge_us = id * 100U};
        assert(telemetry_try_emit_barcode_sample(&sample) == E_OK);
    }
    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    assert(g_tx_len == 10U + sizeof(heartbeat));
    g_tx_len = 0U;
    receipt_suback(2U, true);
    poll_at(4U);
    acknowledge(read_batch(original));
    assert(original[TELEMETRY_BARCODE_BATCH_OFF_SAMPLE_COUNT] == 3U);

    receipt(boot - 1U, 3U);
    poll_at(1003U);
    assert(g_tx_len == 0U);
    /* A broker PUBACK alone leaves a durably stored batch eligible for retry. */
    poll_at(1004U);
    acknowledge(read_batch(replay));
    assert(memcmp(replay, original, sizeof(replay)) == 0);
    g_on_error(gp_tcp_arg, ERR_RST);
    poll_at(2004U);
    connack();
    suback(true);
    g_tx_len = 0U;
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2006U);
    g_tx_len = 0U;
    receipt_suback(2U, true);
    poll_at(2008U);
    uint16_t const first = read_batch(replay);
    assert(memcmp(replay, original, sizeof(replay)) == 0);
    receipt(boot, 3U);
    poll_at(2010U);
    assert(g_tx_len == 0U);
    acknowledge(first);
    poll_at(2012U);
    acknowledge(read_batch(replay));
    assert(replay[TELEMETRY_BARCODE_BATCH_OFF_SAMPLE_COUNT] == 1U);
    assert(replay[TELEMETRY_BARCODE_BATCH_OFF_SAMPLES + 3U] == 4U);
    receipt(boot, 3U);
    poll_at(3012U);
    acknowledge(read_batch(replay));
    receipt(boot, 4U);
    poll_at(4012U);
    assert(g_tx_len == 0U && mqtt_session_drops() == 0U);
}

static void test_delivery_ownership_after_reset (void)
{
    const uint32_t boot = 0x12345678U;
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};
    size_t const event_at = 10U + sizeof(heartbeat);
    size_t const status_at = event_at + 12U + TELEMETRY_EVENT_WIRE_SIZE;
    uint8_t batch[TELEMETRY_BARCODE_BATCH_WIRE_SIZE];

    mqtt_session_set_boot_id(boot);
    for (uint32_t id = 1U; id <= 4U; id++)
    {
        const telemetry_barcode_sample_t sample = {.edge_us = id * 100U};
        assert(telemetry_try_emit_barcode_sample(&sample) == E_OK);
    }
    poll_at(0U);
    connack();
    g_tx_len = 0U;
    suback(true);
    uint16_t const subscribed = expect_receipt_subscribe();
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    assert(g_tx_len == event_at);
    g_tx_len = 0U;

    offer_event(1U);
    poll_at(4U);
    (void)expect_retained_event(1U);
    offer_status(1U);
    poll_at(6U);
    (void)expect_status(1U);
    receipt_suback(subscribed, true);
    poll_at(8U);
    (void)read_batch(batch);
    assert(batch[TELEMETRY_BARCODE_BATCH_OFF_SAMPLE_COUNT] == 3U);

    /* The archive receipt arrives before the broker's PUBACK for the batch. */
    receipt(boot, 3U);
    offer_event(2U);
    offer_status(2U);
    offer_status(3U);
    poll_at(10U);
    assert(g_tx_len == 0U);
    g_on_error(gp_tcp_arg, ERR_RST);

    poll_at(1010U);
    connack();
    g_tx_len = 0U;
    suback(true);
    uint16_t const resubscribed = expect_receipt_subscribe();
    poll_at(1012U);
    assert(g_tx_len == 0U);
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(1014U);

    /* The unacknowledged event is retried ahead of the latest pending status. */
    assert(g_tx_len == status_at + 12U + TELEMETRY_STATUS_BUNDLE_WIRE_SIZE);
    assert(g_tx[9U] == TELEMETRY_TOPIC_HEARTBEAT);
    assert(g_tx[event_at] == 0x32U && g_tx[event_at + 9U] == TELEMETRY_TOPIC_EVENT);
    assert(g_tx[event_at + 12U + TELEMETRY_EVENT_OFF_EVENT_ID + 1U] == 1U);
    assert(g_tx[status_at] == 0x32U && g_tx[status_at + 9U] == TELEMETRY_TOPIC_STATUS);
    assert(g_tx[status_at + 12U] == 3U);
    uint16_t const event_packet = (uint16_t)(((uint16_t)g_tx[event_at + 10U] << 8U) |
                                            g_tx[event_at + 11U]);
    uint16_t const status_packet = (uint16_t)(((uint16_t)g_tx[status_at + 10U] << 8U) |
                                             g_tx[status_at + 11U]);
    g_tx_len = 0U;
    acknowledge(status_packet);
    poll_at(1016U);
    assert(g_tx_len == 0U);
    acknowledge(event_packet);
    poll_at(1018U);
    acknowledge(expect_retained_event(2U));

    receipt_suback(resubscribed, true);
    poll_at(1020U);
    acknowledge(read_batch(batch));
    assert(batch[TELEMETRY_BARCODE_BATCH_OFF_SAMPLE_COUNT] == 1U);
    assert(batch[TELEMETRY_BARCODE_BATCH_OFF_SAMPLES + 3U] == 4U);
    receipt(boot, 4U);
    poll_at(2020U);
    assert(g_tx_len == 0U && mqtt_session_drops() == 0U);
}

static void test_barcode_subscription_retry (void)
{
    uint32_t const timeout_ms = MQTT_REQ_TIMEOUT * 1000U;
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};
    const telemetry_barcode_sample_t sample = {.edge_us = 100U};
    uint8_t batch[TELEMETRY_BARCODE_BATCH_WIRE_SIZE];

    mqtt_session_set_boot_id(0x12345678U);
    assert(telemetry_try_emit_barcode_sample(&sample) == E_OK);
    poll_at(0U);
    connack();
    g_tx_len = 0U;
    suback(true);
    uint16_t const lost = expect_receipt_subscribe();
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    assert(g_tx_len == sizeof(heartbeat) + 10U);
    assert(g_on_sent(gp_tcp_arg, &g_pcb, (u16_t)g_tx_len) == ERR_OK);
    g_tx_len = 0U;
    offer_event(7U);
    poll_at(3U);
    acknowledge(expect_retained_event(7U));
    offer_status(1U);
    poll_at(4U);
    acknowledge(expect_status(1U));

    for (unsigned second = 1U; second <= MQTT_REQ_TIMEOUT; second++)
    {
        sys_timeout_handler tick = g_timer;

        g_timer = NULL;
        g_now = second * 1000U;
        assert(tick != NULL);
        tick(gp_timer_arg);
        mqtt_session_poll();
        assert(g_tx_len == 0U && mqtt_session_connected());
    }
    poll_at(timeout_ms + 999U);
    assert(g_tx_len == 0U);
    poll_at(timeout_ms + 1000U);
    uint16_t const denied = expect_receipt_subscribe();
    receipt_suback(lost, true);
    poll_at(timeout_ms + 1002U);
    assert(g_tx_len == 0U);
    receipt_suback(denied, false);

    offer_event(8U);
    poll_at(timeout_ms + 1010U);
    acknowledge(expect_retained_event(8U));
    poll_at(timeout_ms + 2001U);
    assert(g_tx_len == 0U);
    poll_at(timeout_ms + 2002U);
    receipt_suback(expect_receipt_subscribe(), true);
    poll_at(timeout_ms + 2004U);
    acknowledge(read_batch(batch));
    assert(batch[TELEMETRY_BARCODE_BATCH_OFF_SAMPLE_COUNT] == 1U);
    assert(mqtt_session_connected() && g_attempts == 1U && g_closed == 0U);
}

static void test_retry (void)
{
    uint32_t now = UINT32_MAX - 500U;

    for (unsigned i = 0U; i < 7U; i++)
    {
        poll_at(now);
        assert(g_attempts == i + 1U);
        if (i == 0U)
        {
            const uint8_t refused[] = {0x20U, 2U, 0U, 5U};
            assert(ERR_OK == g_on_connected(gp_tcp_arg, &g_pcb, ERR_OK));
            receive(refused, sizeof(refused));
        }
        else
        {
            connack();
            suback(false);
        }
        poll_at(now);
        assert(g_closed == i + 1U && !mqtt_session_connected());
        g_tx_len = 0U;
        now += 1000U;
        poll_at(now - 1U);
        assert(g_attempts == i + 1U);
    }
    poll_at(now);
    connack();
    suback(true);
    assert(mqtt_session_connected());
    g_lease = WIFI_LEASE_PENDING;
    poll_at(now);
    assert(!mqtt_session_connected());
    poll_at(now + 1000U);
    assert(g_attempts == 8U);
    g_lease = WIFI_LEASE_READY;
    poll_at(now + 1000U);
    assert(g_attempts == 9U);
}

static void test_dns (void)
{
    g_async_dns = true;
    poll_at(0U);
    assert(g_resolve != NULL && g_attempts == 0U);
    g_lease = WIFI_LEASE_PENDING;
    poll_at(1U);
    g_lease = WIFI_LEASE_READY;
    poll_at(1200U);
    assert(g_attempts == 0U);
    g_resolve("localhost", &g_addr, gp_dns_arg);
    g_resolve = NULL;
    assert(g_attempts == 0U);
    poll_at(1201U);
    assert(g_resolve != NULL);
    g_resolve("localhost", &g_addr, gp_dns_arg);
    assert(g_attempts == 1U);
}

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    (void)task;
    (void)mod;
    (void)p_fmt;
    if (LOG_SEV_WARN == sev)
    {
        g_warnings++;
    }
}

static void test_dns_unavailable (void)
{
    g_dns_fail = true;
    poll_at(0U);
    assert(g_attempts == 0U && g_warnings == 1U);
    poll_at(1000U);
    poll_at(2000U);
    assert(g_attempts == 0U && g_warnings == 1U);
    g_dns_fail = false;
    poll_at(4000U);
    assert(g_attempts == 1U);
}

static void test_timeout (void)
{
    poll_at(0U);
    poll_at(3999U);
    assert(g_closed == 0U && g_attempts == 1U);
    poll_at(4000U);
    assert(g_closed == 1U);
    poll_at(5000U);
    assert(g_attempts == 2U);
    poll_at(8999U);
    connack();
    assert(g_timer != NULL);
    poll_at(8999U);
    assert(g_closed == 1U && !mqtt_session_connected());
    poll_at(9000U);
    assert(g_closed == 2U && g_timer == NULL && !mqtt_session_connected());
}

static void test_connect_write_retry (void)
{
    static const uint8_t accepted[] = {0x20U, 2U, 0U, 0U};

    poll_at(0U);
    poll_at(500U);
    assert(g_attempts == 1U && g_closed == 0U && g_timer == NULL);
    g_block_write = true;
    assert(ERR_OK == g_on_connected(gp_tcp_arg, &g_pcb, ERR_OK) && g_tx_len == 0U);
    g_block_write = false;
    assert(g_on_poll(gp_tcp_arg, &g_pcb) == ERR_OK);

    /* TCP is up and has room again. The pinned SDK cannot send residual
     * CONNECT bytes from its TCP poll until it has received CONNACK. */
    poll_at(1499U);
    assert(g_closed == 0U);
    sys_timeout_handler tick = g_timer;
    assert(tick != NULL);
    g_timer = NULL;
    g_now = 1500U;
    tick(gp_timer_arg);
    mqtt_session_poll();
    assert(g_closed == 1U && g_timer == NULL && g_tx_len == 0U);

    poll_at(2499U);
    assert(g_attempts == 1U);
    poll_at(2500U);
    assert(g_attempts == 2U);
    assert(ERR_OK == g_on_connected(gp_tcp_arg, &g_pcb, ERR_OK));
    assert(g_tx_len > 0U && g_tx[0] == 0x10U);

    /* A sent CONNECT can still wait for CONNACK through a timer tick. */
    tick = g_timer;
    assert(tick != NULL);
    g_timer = NULL;
    g_now = 3500U;
    tick(gp_timer_arg);
    mqtt_session_poll();
    assert(g_closed == 1U && !mqtt_session_connected());
    receive(accepted, sizeof(accepted));
    suback(true);
    assert(g_attempts == 2U && mqtt_session_connected());
}

static void test_keepalive (void)
{
    static const uint8_t ping[] = {0xC0U, 0U};

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    for (unsigned second = 1U; second <= 46U; second++)
    {
        sys_timeout_handler tick = g_timer;

        g_timer = NULL;
        g_now = second * 1000U;
        assert(tick != NULL);
        tick(gp_timer_arg);
        if (second == 30U)
        {
            assert(g_on_poll(gp_tcp_arg, &g_pcb) == ERR_OK);
            expect_tx(ping, sizeof(ping));
        }
        assert(mqtt_session_connected() == (second < 46U));
    }
    assert(g_timer == NULL && g_closed == 1U);
    poll_at(46999U);
    assert(g_attempts == 1U);
    poll_at(47000U);
    assert(g_attempts == 2U);
}

static void test_waits_for_car_id (void)
{
    poll_at(0U);
    poll_at(1000U);
    assert(g_resolve == NULL && g_attempts == 0U);
    mqtt_session_set_car_id(CAR_ID);
    poll_at(1001U);
    assert(g_attempts == 1U);
}

int main (int argc, char ** argv)
{
    assert(argc == 2);
    IP_ADDR4(&g_addr, 127U, 0U, 0U, 1U);
    poll_at(0U);
    assert(g_attempts == 0U);
    assert(mqtt_session_init() == E_OK);
    assert(telemetry_net_queue_create() == E_OK);
    mqtt_session_start(1);
    if (strcmp(argv[1], "no-car-id") != 0)
    {
        mqtt_session_set_car_id(CAR_ID);
    }

    if (strcmp(argv[1], "wire") == 0)
    {
        test_wire();
    }
    else if (strcmp(argv[1], "command-before-suback") == 0)
    {
        test_command_before_suback();
    }
    else if (strcmp(argv[1], "command-subscription-recovery") == 0)
    {
        test_command_subscription_recovery();
    }
    else if (strcmp(argv[1], "backpressure") == 0)
    {
        test_command_backpressure();
    }
    else if (strcmp(argv[1], "puback-space") == 0)
    {
        test_puback_space();
    }
    else if (strcmp(argv[1], "output-headroom") == 0)
    {
        test_output_headroom();
    }
    else if (strcmp(argv[1], "retained-events") == 0)
    {
        test_retained_events();
    }
    else if (strcmp(argv[1], "latest-status") == 0)
    {
        test_latest_status();
    }
    else if (strcmp(argv[1], "latest-status-timeout") == 0)
    {
        test_latest_status_timeout();
    }
    else if (strcmp(argv[1], "barcode-receipt") == 0)
    {
        test_barcode_receipt();
    }
    else if (strcmp(argv[1], "delivery-ownership-after-reset") == 0)
    {
        test_delivery_ownership_after_reset();
    }
    else if (strcmp(argv[1], "barcode-subscription-retry") == 0)
    {
        test_barcode_subscription_retry();
    }
    else if (strcmp(argv[1], "keepalive") == 0)
    {
        test_keepalive();
    }
    else if (strcmp(argv[1], "retry") == 0)
    {
        test_retry();
    }
    else if (strcmp(argv[1], "no-car-id") == 0)
    {
        test_waits_for_car_id();
    }
    else if (strcmp(argv[1], "dns") == 0)
    {
        test_dns();
    }
    else if (strcmp(argv[1], "dns-unavailable") == 0)
    {
        test_dns_unavailable();
    }
    else if (strcmp(argv[1], "connect-write-retry") == 0)
    {
        test_connect_write_retry();
    }
    else
    {
        assert(strcmp(argv[1], "timeout") == 0);
        test_timeout();
    }
    return 0;
}
