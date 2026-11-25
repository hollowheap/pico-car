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
#include "telemetry/net_queue.h"
#include "telemetry/wifi_link.h"
#include "tk/mbf_fake.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static uint32_t g_now;
static wifi_lease_t g_lease = WIFI_LEASE_READY;
static struct tcp_pcb g_pcb;
const ip_addr_t ip_addr_any = IPADDR4_INIT(0U);
static ip_addr_t g_addr;
static tcp_connected_fn g_on_connected;
static tcp_recv_fn g_on_recv;
static tcp_err_fn g_on_error;
static tcp_poll_fn g_on_poll;
static void * gp_tcp_arg;
static sys_timeout_handler g_timer;
static void * gp_timer_arg;
static dns_found_callback g_resolve;
static void * gp_dns_arg;
static bool g_async_dns;
static unsigned g_attempts;
static unsigned g_closed;
static unsigned g_data_writes;
static uint8_t g_tx[1024];
static size_t g_tx_len;
static bool g_next_frame;
static bool g_profile_frame;
static bool g_block_write;
static uint32_t g_fault;
static unsigned g_waiting_sends;

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
    if (timeout == TMO_FEVR)
    {
        uint8_t payload[TELEMETRY_CMD_WIRE_SIZE];
        uint16_t received = 0U;

        /* Simulate core 1 taking the oldest message while the radio waits.
         * No PUBACK may be sent before the new command has a queue slot. */
        assert(g_tx_len == 0U && mbf_fake_free(mbfid) == 0);
        assert(mqtt_session_command(payload, &received));
        assert(received == TELEMETRY_CMD_WIRE_SIZE && payload[4] == TELEMETRY_CMD_STOP);
        g_waiting_sends++;
    }
    return __real_tk_snd_mbf(mbfid, p_msg, len, timeout);
}

void system_fault_assign (fault_owner_t owner, uint32_t mask, uint32_t active)
{
    assert(owner == FAULT_OWNER_LINK && mask == FAULT_MQTT_NACK_BIT);
    g_fault = active;
}

INT telemetry_net_queue_pop (uint8_t * p_frame, TMO timeout)
{
    assert(timeout == TMO_POL);
    if (!g_next_frame)
    {
        return E_TMOUT;
    }
    g_next_frame = false;
    if (g_profile_frame)
    {
        p_frame[0] = TELEMETRY_TOPIC_PROFILE;
        (void)memset(&p_frame[1], 0x5A, TELEMETRY_PROFILE_WIRE_SIZE);
        return 1 + TELEMETRY_PROFILE_WIRE_SIZE;
    }
    p_frame[0] = TELEMETRY_TOPIC_EVENT;
    p_frame[1] = 0xA5U;
    p_frame[2] = 0x5AU;
    return 3;
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
void tcp_sent (struct tcp_pcb * p_pcb, tcp_sent_fn sent) { (void)p_pcb; (void)sent; }
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
    assert(p_pcb == &g_pcb && flags == TCP_WRITE_FLAG_COPY && g_tx_len + len <= sizeof(g_tx));
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

static void test_wire (void)
{
    static const uint8_t connect[] = {
        0x10U, 16U, 0U, 4U, 'M', 'Q', 'T', 'T', 4U, 2U, 0U, 30U, 0U, 4U, 't', 'e', 's', 't',
    };
    static const uint8_t subscribe[] = {0x82U, 11U, 0U, 1U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 1U};
    static const uint8_t heartbeat[] = {
        0x30U, 18U, 0U, 6U, 't', 'e', 's', 't', '/', 'h', 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,
    };
    static const uint8_t event[] = {0x30U, 10U, 0U, 6U, 't', 'e', 's', 't', '/', 'e', 0xA5U, 0x5AU};
    static const uint8_t command_start[] = {0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U, 0U, 9U};
    static const uint8_t command_end[] = {0x11U, 0x22U, 6U, 0U, 0U, 0U, 1U};
    static const uint8_t command[] = {0U, 9U, 0x11U, 0x22U, 6U, 0U, 0U, 0U, 1U};
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    static const uint8_t wrong_topic[] = {0x30U, 8U, 0U, 6U, 't', 'e', 's', 't', '/', 'x'};
    static const uint8_t empty[] = {0x30U, 8U, 0U, 6U, 't', 'e', 's', 't', '/', 'c'};
    uint8_t payload[TELEMETRY_CMD_WIRE_SIZE];
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
    g_next_frame = true;
    poll_at(2U);
    assert(g_next_frame && !mqtt_session_connected());
    suback(true);
    assert(mqtt_session_connected());
    poll_at(4U);
    assert(g_next_frame);
    assert(mqtt_session_heartbeat(&heartbeat[10]));
    poll_at(6U);
    assert(g_tx_len == sizeof(heartbeat) + sizeof(event));
    assert(memcmp(g_tx, heartbeat, sizeof(heartbeat)) == 0);
    assert(memcmp(&g_tx[sizeof(heartbeat)], event, sizeof(event)) == 0);
    assert(g_fault == 0U);
    g_tx_len = 0U;

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
        g_next_frame = true;
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

static void test_command_backpressure (void)
{
    uint8_t command[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U,
        0U, 9U, 0x11U, 0x22U, TELEMETRY_CMD_STOP, 0U, 0U, 0U, 1U,
    };
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    uint8_t payload[TELEMETRY_CMD_WIRE_SIZE];
    uint16_t len;
    unsigned queued = 0U;

    poll_at(0U);
    connack();
    suback(true);
    g_tx_len = 0U;
    while (mbf_fake_free(1) > 0)
    {
        receive(command, sizeof(command));
        expect_tx(puback, sizeof(puback));
        queued++;
    }
    assert(queued >= 8U && g_waiting_sends == 0U);
    command[16] = TELEMETRY_CMD_ESTOP;
    receive(command, sizeof(command));
    expect_tx(puback, sizeof(puback));
    assert(g_waiting_sends == 1U && mqtt_session_drops() == 0U);
    for (unsigned i = 1U; i < queued; i++)
    {
        assert(mqtt_session_command(payload, &len));
        assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[4] == TELEMETRY_CMD_STOP);
    }
    assert(mqtt_session_command(payload, &len));
    assert(len == TELEMETRY_CMD_WIRE_SIZE && payload[4] == TELEMETRY_CMD_ESTOP);
    assert(!mqtt_session_command(payload, &len));
}

static void test_puback_space (void)
{
    static const uint8_t estop[] = {
        0x32U, 19U, 0U, 6U, 't', 'e', 's', 't', '/', 'c', 0U, 4U,
        0U, 9U, 0x11U, 0x22U, TELEMETRY_CMD_ESTOP, 0U, 0U, 0U, 1U,
    };
    static const uint8_t puback[] = {0x40U, 2U, 0U, 4U};
    uint8_t payload[TELEMETRY_CMD_WIRE_SIZE];
    uint16_t len;

    poll_at(0U);
    connack();
    suback(true);
    const uint8_t heartbeat[TELEMETRY_HEARTBEAT_WIRE_SIZE] = {0};
    assert(mqtt_session_heartbeat(heartbeat));
    poll_at(2U);
    g_tx_len = 0U;
    g_block_write = true;
    g_profile_frame = true;
    for (unsigned i = 0U; i < MQTT_REQ_MAX_IN_FLIGHT; i++)
    {
        g_next_frame = true;
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

static void test_retry (void)
{
    static const uint32_t delays[] = {1000U, 2000U, 4000U, 8000U, 16000U, 30000U, 30000U};
    uint32_t now = UINT32_MAX - 500U;

    for (unsigned i = 0U; i < sizeof(delays) / sizeof(delays[0]); i++)
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
        now += delays[i];
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

static void test_timeout (void)
{
    poll_at(0U);
    poll_at(9999U);
    assert(g_closed == 0U && g_attempts == 1U);
    poll_at(10000U);
    assert(g_closed == 1U);
    poll_at(11000U);
    assert(g_attempts == 2U);
    connack();
    assert(g_timer != NULL);
    poll_at(20999U);
    assert(g_closed == 1U && !mqtt_session_connected());
    poll_at(21000U);
    assert(g_closed == 2U && g_timer == NULL && !mqtt_session_connected());
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

int main (int argc, char ** argv)
{
    assert(argc == 2);
    IP_ADDR4(&g_addr, 127U, 0U, 0U, 1U);
    poll_at(0U);
    assert(g_attempts == 0U);
    assert(mqtt_session_init() == E_OK);
    mqtt_session_start(1);

    if (strcmp(argv[1], "wire") == 0)
    {
        test_wire();
    }
    else if (strcmp(argv[1], "backpressure") == 0)
    {
        test_command_backpressure();
    }
    else if (strcmp(argv[1], "puback-space") == 0)
    {
        test_puback_space();
    }
    else if (strcmp(argv[1], "keepalive") == 0)
    {
        test_keepalive();
    }
    else if (strcmp(argv[1], "retry") == 0)
    {
        test_retry();
    }
    else if (strcmp(argv[1], "dns") == 0)
    {
        test_dns();
    }
    else
    {
        assert(strcmp(argv[1], "timeout") == 0);
        test_timeout();
    }
    return 0;
}
