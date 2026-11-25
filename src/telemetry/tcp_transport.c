#include <string.h>

#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

#include "cyw43_utk.h"
#include "telemetry/tcp_transport.h"

#define TCP_TRANSPORT_RETRY_MS          1000u
#define TCP_TRANSPORT_CONNECT_TIMEOUT_MS 10000u

#if ((TCP_TRANSPORT_RX_CAPACITY & (TCP_TRANSPORT_RX_CAPACITY - 1u)) != 0u)
#error "TCP_TRANSPORT_RX_CAPACITY must be a power of two"
#endif
#if ((TCP_TRANSPORT_TX_CAPACITY & (TCP_TRANSPORT_TX_CAPACITY - 1u)) != 0u)
#error "TCP_TRANSPORT_TX_CAPACITY must be a power of two"
#endif
#if TCP_TRANSPORT_TX_CAPACITY != TCP_TRANSPORT_RX_CAPACITY
#error "TCP_TRANSPORT_TX_CAPACITY must match TCP_TRANSPORT_RX_CAPACITY"
#endif

typedef struct
{
    uint8_t           data[TCP_TRANSPORT_RX_CAPACITY];
    volatile uint16_t head;
    volatile uint16_t tail;
} byte_ring_t;

static byte_ring_t             s_rx;
static byte_ring_t             s_tx;
static struct tcp_pcb *        s_pcb;
static ip_addr_t               s_resolved;
static char                    s_host[TCP_TRANSPORT_HOST_MAX];
static uint16_t                s_port;
static volatile bool           s_desired;
static volatile bool           s_resolving;
static volatile tcp_transport_state_t s_state;
static uint32_t                s_retry_at_ms;
static uint32_t                s_connect_started_ms;

static void
memory_barrier (void)
{
    __asm__ volatile ("dmb" ::: "memory");
}

static uint16_t
ring_used (const byte_ring_t * ring)
{
    return (uint16_t) (ring->head - ring->tail);
}

static void
ring_reset (byte_ring_t * ring)
{
    ring->tail = ring->head;
}

static void
ring_push (byte_ring_t * ring, const uint8_t * data, uint16_t len)
{
    uint16_t index = (uint16_t) (ring->head & (TCP_TRANSPORT_RX_CAPACITY - 1u));
    uint16_t first = (uint16_t) (TCP_TRANSPORT_RX_CAPACITY - index);

    if (first > len)
    {
        first = len;
    }
    (void) memcpy(&ring->data[index], data, first);
    if (len > first)
    {
        (void) memcpy(&ring->data[0], &data[first], (size_t) (len - first));
    }
    memory_barrier();
    ring->head = (uint16_t) (ring->head + len);
}

static uint16_t
ring_pop (byte_ring_t * ring, uint8_t * data, uint16_t len)
{
    uint16_t used = ring_used(ring);
    uint16_t index;
    uint16_t first;

    if (len > used)
    {
        len = used;
    }
    index = (uint16_t) (ring->tail & (TCP_TRANSPORT_RX_CAPACITY - 1u));
    first = (uint16_t) (TCP_TRANSPORT_RX_CAPACITY - index);
    if (first > len)
    {
        first = len;
    }
    (void) memcpy(data, &ring->data[index], first);
    if (len > first)
    {
        (void) memcpy(&data[first], &ring->data[0], (size_t) (len - first));
    }
    memory_barrier();
    ring->tail = (uint16_t) (ring->tail + len);
    return len;
}

static void
detach_callbacks (struct tcp_pcb * pcb)
{
    tcp_arg(pcb, 0);
    tcp_recv(pcb, 0);
    tcp_err(pcb, 0);
    tcp_sent(pcb, 0);
}

static void
fail_attempt (uint32_t now_ms)
{
    s_resolving = false;
    s_state = TCP_TRANSPORT_FAILED;
    s_retry_at_ms = now_ms + TCP_TRANSPORT_RETRY_MS;
    ring_reset(&s_tx);
}

static void
close_abort (void)
{
    struct tcp_pcb * pcb = s_pcb;

    if (pcb == 0)
    {
        return;
    }
    s_pcb = 0;
    detach_callbacks(pcb);
    tcp_abort(pcb);
    ring_reset(&s_tx);
}

static void
close_graceful (void)
{
    struct tcp_pcb * pcb = s_pcb;

    if (pcb == 0)
    {
        return;
    }
    s_pcb = 0;
    detach_callbacks(pcb);
    if (tcp_close(pcb) != ERR_OK)
    {
        tcp_abort(pcb);
    }
    ring_reset(&s_tx);
}

static void drain_tx (void);

static err_t
on_connected (void * arg, struct tcp_pcb * pcb, err_t err)
{
    (void) arg;

    if (err != ERR_OK)
    {
        s_pcb = 0;
        fail_attempt((uint32_t) sys_now());
        detach_callbacks(pcb);
        tcp_abort(pcb);
        return ERR_OK;
    }
    tcp_nagle_disable(pcb);
    s_state = TCP_TRANSPORT_CONNECTED;
    drain_tx();
    return ERR_OK;
}

static err_t
on_recv (void * arg, struct tcp_pcb * pcb, struct pbuf * p, err_t err)
{
    uint16_t copied = 0u;

    (void) arg;

    if (p == 0)
    {
        close_graceful();
        fail_attempt((uint32_t) sys_now());
        return ERR_OK;
    }
    if (err != ERR_OK)
    {
        (void) pbuf_free(p);
        close_abort();
        fail_attempt((uint32_t) sys_now());
        return ERR_ABRT;
    }
    if (ring_used(&s_rx) + p->tot_len > TCP_TRANSPORT_RX_CAPACITY)
    {
        /* Leave the pbuf with lwIP and retry after the application drains. */
        return ERR_MEM;
    }
    while (copied < p->tot_len)
    {
        uint16_t index = (uint16_t) (s_rx.head &
                                     (TCP_TRANSPORT_RX_CAPACITY - 1u));
        uint16_t chunk = (uint16_t) (TCP_TRANSPORT_RX_CAPACITY - index);

        if (chunk > (uint16_t) (p->tot_len - copied))
        {
            chunk = (uint16_t) (p->tot_len - copied);
        }
        (void) pbuf_copy_partial(p, &s_rx.data[index], chunk, copied);
        memory_barrier();
        s_rx.head = (uint16_t) (s_rx.head + chunk);
        copied = (uint16_t) (copied + chunk);
    }
    tcp_recved(pcb, p->tot_len);
    (void) pbuf_free(p);
    return ERR_OK;
}

static void
on_error (void * arg, err_t err)
{
    (void) arg;
    (void) err;

    /* lwIP has already freed the pcb when this callback runs. */
    s_pcb = 0;
    fail_attempt((uint32_t) sys_now());
}

static void
start_connect (void)
{
    err_t err;

    if (netif_default == 0)
    {
        fail_attempt((uint32_t) sys_now());
        return;
    }
    s_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (s_pcb == 0)
    {
        fail_attempt((uint32_t) sys_now());
        return;
    }
    tcp_bind_netif(s_pcb, netif_default);
    tcp_arg(s_pcb, 0);
    tcp_recv(s_pcb, on_recv);
    tcp_err(s_pcb, on_error);

    err = tcp_connect(s_pcb, &s_resolved, s_port, on_connected);
    if (err != ERR_OK)
    {
        detach_callbacks(s_pcb);
        tcp_abort(s_pcb);
        s_pcb = 0;
        fail_attempt((uint32_t) sys_now());
        return;
    }
    s_state = TCP_TRANSPORT_CONNECTING;
    s_connect_started_ms = (uint32_t) sys_now();
}

static void
on_resolved (const char * name, const ip_addr_t * address, void * arg)
{
    (void) name;
    (void) arg;

    s_resolving = false;
    if (!s_desired)
    {
        return;
    }
    if (address == 0)
    {
        fail_attempt((uint32_t) sys_now());
        return;
    }
    s_resolved = *address;
    start_connect();
}

static void
begin_attempt (uint32_t now_ms)
{
    ip_addr_t address;
    err_t err;

    if (ipaddr_aton(s_host, &address))
    {
        s_resolved = address;
        start_connect();
        return;
    }
    s_state = TCP_TRANSPORT_RESOLVING;
    err = dns_gethostbyname(s_host, &address, on_resolved, 0);
    if (err == ERR_OK)
    {
        s_resolved = address;
        start_connect();
    }
    else if (err == ERR_INPROGRESS)
    {
        s_resolving = true;
    }
    else
    {
        fail_attempt(now_ms);
    }
}

static void
drain_tx (void)
{
    bool wrote = false;

    if ((s_pcb == 0) || (s_state != TCP_TRANSPORT_CONNECTED))
    {
        return;
    }
    while (ring_used(&s_tx) > 0u)
    {
        uint16_t index = (uint16_t) (s_tx.tail &
                                     (TCP_TRANSPORT_TX_CAPACITY - 1u));
        uint16_t chunk = (uint16_t) (TCP_TRANSPORT_TX_CAPACITY - index);
        uint16_t sndbuf = tcp_sndbuf(s_pcb);

        if (sndbuf == 0u)
        {
            break;
        }
        if (chunk > ring_used(&s_tx))
        {
            chunk = ring_used(&s_tx);
        }
        if (chunk > sndbuf)
        {
            chunk = sndbuf;
        }
        if (tcp_write(s_pcb, &s_tx.data[index], chunk,
                      TCP_WRITE_FLAG_COPY) != ERR_OK)
        {
            break;
        }
        memory_barrier();
        s_tx.tail = (uint16_t) (s_tx.tail + chunk);
        wrote = true;
    }
    if (wrote)
    {
        (void) tcp_output(s_pcb);
    }
}

bool
tcp_transport_open (const char * host, uint16_t port)
{
    size_t len;

    if ((host == 0) || (port == 0u) || s_desired)
    {
        return false;
    }
    len = strlen(host);
    if ((len == 0u) || (len >= sizeof(s_host)))
    {
        return false;
    }
    (void) memcpy(s_host, host, len + 1u);
    s_port = port;
    ring_reset(&s_rx);
    ring_reset(&s_tx);
    s_retry_at_ms = 0u;
    s_resolving = false;
    s_state = TCP_TRANSPORT_WAITING;
    memory_barrier();
    s_desired = true;
    return true;
}

void
tcp_transport_close (void)
{
    s_desired = false;
    s_resolving = false;
    s_state = TCP_TRANSPORT_CLOSED;
}

tcp_transport_state_t
tcp_transport_state (void)
{
    return s_state;
}

uint32_t
tcp_transport_now_ms (void)
{
    return (uint32_t) sys_now();
}

int
tcp_transport_send (void * context, const uint8_t * data, uint16_t len)
{
    (void) context;

    if ((data == 0) || (len == 0u) || !s_desired)
    {
        return 0;
    }
    if ((uint16_t) (TCP_TRANSPORT_TX_CAPACITY - ring_used(&s_tx)) < len)
    {
        return 0;
    }
    ring_push(&s_tx, data, len);
    return (int) len;
}

int
tcp_transport_recv (void * context, uint8_t * data, uint16_t len)
{
    (void) context;

    if ((data == 0) || (len == 0u))
    {
        return -1;
    }
    if ((s_state == TCP_TRANSPORT_FAILED) ||
        (s_state == TCP_TRANSPORT_CLOSED))
    {
        ring_reset(&s_rx);
        return -1;
    }
    return (int) ring_pop(&s_rx, data, len);
}

void
tcp_transport_poll (void)
{
    T_CYW43_UTK_STATUS status;
    uint32_t now_ms;

    if (!s_desired)
    {
        close_abort();
        if (s_state != TCP_TRANSPORT_CLOSED)
        {
            s_state = TCP_TRANSPORT_IDLE;
        }
        return;
    }

    cyw43_utk_get_status(&status);
    if ((status.link_up == 0u) || (status.dhcp_complete == 0u) ||
        (status.dhcp_result != 0))
    {
        close_abort();
        s_resolving = false;
        s_state = TCP_TRANSPORT_WAITING;
        return;
    }

    now_ms = (uint32_t) sys_now();
    if ((s_pcb != 0) && (s_state == TCP_TRANSPORT_CONNECTING) &&
        ((uint32_t) (now_ms - s_connect_started_ms) >=
         TCP_TRANSPORT_CONNECT_TIMEOUT_MS))
    {
        close_abort();
        fail_attempt(now_ms);
        return;
    }

    if ((s_pcb == 0) && !s_resolving &&
        ((int32_t) (now_ms - s_retry_at_ms) >= 0))
    {
        begin_attempt(now_ms);
    }
    drain_tx();
}

/* Weak hook defined by the kernel fork; the lwIP owner task calls it. */
void
cyw43_utk_app_poll (void)
{
    tcp_transport_poll();
}
