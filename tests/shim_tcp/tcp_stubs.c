#include <stdarg.h>
#include <string.h>

#include "cyw43_utk.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "platform/log.h"
#include "tcp_stubs.h"

tcp_stubs_t g_stubs;

static struct netif g_netif;
struct netif * netif_default = &g_netif;

void
tcp_stubs_reset (void)
{
    (void) memset(&g_stubs, 0, sizeof(g_stubs));
    g_stubs.link_up = 1u;
    g_stubs.dhcp_complete = 1u;
}

struct tcp_pcb *
tcp_new_ip_type (u8_t type)
{
    (void) type;
    (void) memset(&g_stubs.pcb, 0, sizeof(g_stubs.pcb));
    g_stubs.pcb.snd_buf = 1024u;
    return &g_stubs.pcb;
}

void
tcp_bind_netif (struct tcp_pcb * pcb, const struct netif * netif)
{
    (void) pcb;
    (void) netif;
}

void
tcp_arg (struct tcp_pcb * pcb, void * arg)
{
    (void) pcb;
    (void) arg;
}

void
tcp_recv (struct tcp_pcb * pcb, tcp_recv_fn recv)
{
    (void) pcb;
    g_stubs.on_recv = recv;
}

void
tcp_err (struct tcp_pcb * pcb, tcp_err_fn err)
{
    (void) pcb;
    g_stubs.on_error = err;
}

void
tcp_sent (struct tcp_pcb * pcb, tcp_sent_fn sent)
{
    (void) pcb;
    (void) sent;
}

err_t
tcp_connect (struct tcp_pcb * pcb, const ip_addr_t * addr, u16_t port,
             tcp_connected_fn connected)
{
    (void) pcb;
    (void) addr;
    (void) port;
    g_stubs.on_connected = connected;
    return g_stubs.connect_result;
}

void
tcp_abort (struct tcp_pcb * pcb)
{
    (void) pcb;
    g_stubs.aborts++;
    if (g_stubs.abort_hook != 0)
    {
        g_stubs.abort_hook();
    }
    if (g_stubs.on_error != 0)
    {
        g_stubs.on_error (0, ERR_ABRT);
    }
}

err_t
tcp_close (struct tcp_pcb * pcb)
{
    (void) pcb;
    g_stubs.closes++;
    return g_stubs.close_result;
}

err_t
tcp_write (struct tcp_pcb * pcb, const void * data, u16_t len, u8_t flags)
{
    (void) pcb;
    (void) data;
    (void) flags;
    if (g_stubs.write_result == ERR_OK)
    {
        g_stubs.writes++;
        g_stubs.written_bytes += len;
    }
    return g_stubs.write_result;
}

err_t
tcp_output (struct tcp_pcb * pcb)
{
    (void) pcb;
    return ERR_OK;
}

void
tcp_recved (struct tcp_pcb * pcb, u16_t len)
{
    (void) pcb;
    (void) len;
}

u8_t
pbuf_free (struct pbuf * p)
{
    (void) p;
    return 1u;
}

u16_t
pbuf_copy_partial (const struct pbuf * p, void * dataptr, u16_t len,
                   u16_t offset)
{
    (void) p;
    (void) dataptr;
    (void) len;
    (void) offset;
    return 0u;
}

err_t
dns_gethostbyname (const char * hostname, ip_addr_t * addr,
                   dns_found_callback found, void * arg)
{
    (void) hostname;
    (void) addr;
    (void) found;
    (void) arg;
    return ERR_ARG;
}

int
ip4addr_aton (const char * cp, ip4_addr_t * addr)
{
    (void) cp;
    addr->addr = 1u;
    return 1;
}

u32_t
sys_now (void)
{
    g_stubs.sys_now_calls++;
    if (g_stubs.sys_now_hook != 0)
    {
        g_stubs.sys_now_hook();
    }
    return g_stubs.now_ms;
}

void
cyw43_utk_get_status (T_CYW43_UTK_STATUS * status)
{
    (void) memset(status, 0, sizeof(*status));
    status->link_up = g_stubs.link_up;
    status->dhcp_complete = g_stubs.dhcp_complete;
    status->dhcp_result = g_stubs.dhcp_result;
}

void
log_write (log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt,
           ...)
{
    (void) sev;
    (void) task;
    (void) mod;
    (void) p_fmt;
    g_stubs.log_calls++;
    if (g_stubs.log_hook != 0)
    {
        g_stubs.log_hook();
    }
}
