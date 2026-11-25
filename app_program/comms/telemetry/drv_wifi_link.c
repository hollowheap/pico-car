#include <stdbool.h>
#include <stdint.h>
#include <tk/smp.h>

#include "cyw43.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip_utk_ipv4.h"

#include "mqtt_session.h"
#include "comms/telemetry/drv_wifi_link.h"

#if TM_WIFI_JOIN
#include "wifi_credentials.h"
#endif

#define WIFI_LINK_TICK_MS      2u
#define WIFI_LEASE_TIMEOUT_MS  30000u
#define WIFI_JOIN_RETRY_MS     5000u
#define WIFI_JOIN_TIMEOUT_MS   30000u

#if LWIP_NUM_SYS_TIMEOUT_INTERNAL + 2U > MEMP_NUM_SYS_TIMEOUT
#error "lwIP needs timeout slots for the radio and MQTT timers"
#endif

/* The timer runs inside lwIP's sys_check_timeouts() in the radio owner task.
 * Its state has that one writer; other tasks only read the published
 * words. */
static cyw43_t *             gp_cyw43;
static volatile uint32_t     g_lease;
static volatile int32_t      g_rssi = INT32_MIN;
static bool                  g_lease_started;
static bool                  g_adopted;
static uint32_t              g_lease_since_ms;
static ip4_addr_t            g_adopted_addr;
static ip4_addr_t            g_adopted_mask;
static ip4_addr_t            g_adopted_gw;

#if TM_WIFI_JOIN
static const uint8_t         g_ssid[] = UTK_WIFI_SSID;
static const uint8_t         g_password[] = UTK_WIFI_PASSWORD;
static volatile uint32_t     g_join;
static bool                  g_scan_idle;
static bool                  g_started;
static bool                  g_attempting;
static bool                  g_was_up;
static uint32_t              g_attempt_ms;
static uint32_t              g_retry_ms;
#endif

static void
publish_lease (wifi_lease_t lease)
{
    mp_memory_barrir();
    g_lease = (uint32_t) lease;
}

static void
lease_reset (void)
{
    g_lease_started = false;
    g_adopted = false;
    g_rssi = INT32_MIN;
    publish_lease(WIFI_LEASE_PENDING);
}

static bool
lease_usable (struct netif * netif)
{
    const ip_addr_t * dns = dns_getserver(0u);

    return (dhcp_supplied_address(netif) != 0u) &&
           !ip4_addr_isany_val(*netif_ip4_addr(netif)) &&
           !ip4_addr_isany_val(*netif_ip4_netmask(netif)) &&
           !ip4_addr_isany_val(*netif_ip4_gw(netif)) &&
           (dns != 0) && !ip4_addr_isany_val(*ip_2_ip4(dns));
}

static void
lease_update (uint32_t now_ms)
{
    struct netif * netif = &gp_cyw43->netif[CYW43_ITF_STA];

    if (!netif_is_link_up(netif))
    {
        lease_reset();
        return;
    }
    if (!g_lease_started)
    {
        int32_t rssi = INT32_MIN;

        g_lease_started = true;
        g_lease_since_ms = now_ms;
        /* Upstream records RSSI for its initial join only. Sample each
         * association here, including joins initiated by this module. */
        if (cyw43_wifi_get_rssi(gp_cyw43, &rssi) == 0)
        {
            g_rssi = rssi;
        }
    }
    if (lease_usable(netif))
    {
        if (!g_adopted ||
            !ip4_addr_cmp(&g_adopted_addr, netif_ip4_addr(netif)) ||
            !ip4_addr_cmp(&g_adopted_mask, netif_ip4_netmask(netif)) ||
            !ip4_addr_cmp(&g_adopted_gw, netif_ip4_gw(netif)))
        {
            ip4_addr_copy(g_adopted_addr, *netif_ip4_addr(netif));
            ip4_addr_copy(g_adopted_mask, *netif_ip4_netmask(netif));
            ip4_addr_copy(g_adopted_gw, *netif_ip4_gw(netif));
            (void) lwip_utk_ipv4_adopt_netif(netif);
            g_adopted = true;
        }
        publish_lease(WIFI_LEASE_READY);
        return;
    }
    if (g_adopted)
    {
        g_adopted = false;
        g_lease_since_ms = now_ms;
        publish_lease(WIFI_LEASE_PENDING);
    }
    else if (((uint32_t) (now_ms - g_lease_since_ms) >= WIFI_LEASE_TIMEOUT_MS) &&
             (g_lease == (uint32_t) WIFI_LEASE_PENDING))
    {
        publish_lease(WIFI_LEASE_TIMEOUT);
    }
}

#if TM_WIFI_JOIN
static void
publish_join (wifi_join_t join)
{
    mp_memory_barrir();
    g_join = (uint32_t) join;
}

static void
join_failed (uint32_t now_ms)
{
    g_attempting = false;
    g_retry_ms = now_ms + WIFI_JOIN_RETRY_MS;
    publish_join(WIFI_JOIN_FAILED);
}

/* The kernel's radio task submits the first join when the scan ends and never
 * retries it. This follows that attempt and, after a failure or a lost
 * association, submits the next one itself. */
static void
join_update (uint32_t now_ms)
{
    struct netif * netif = &gp_cyw43->netif[CYW43_ITF_STA];
    int            link = 0;

    if (cyw43_wifi_scan_active(gp_cyw43))
    {
        g_scan_idle = false;
        return;
    }
    if (!g_scan_idle)
    {
        /* The kernel submits its join later in the poll that saw the scan
         * end; wait one tick so that attempt is not mistaken for none. */
        g_scan_idle = true;
        return;
    }
    if (!g_started)
    {
        g_started = true;
        g_attempting = true;
        g_attempt_ms = now_ms;
        publish_join(WIFI_JOIN_ACTIVE);
    }
    if (netif_is_link_up(netif))
    {
        /* A late link-up, or the chip's own reassociation, can arrive during
         * the backoff; a new join would interrupt it. */
        g_was_up = true;
        g_attempting = false;
        publish_join(WIFI_JOIN_IDLE);
        return;
    }
    if (g_was_up)
    {
        g_was_up = false;
        join_failed(now_ms);
        return;
    }
    link = cyw43_wifi_link_status(gp_cyw43, CYW43_ITF_STA);
    if (g_attempting)
    {
        /* A request the driver refused leaves the state down; an accepted
         * one reports a join in progress until it succeeds or fails. */
        if ((link <= CYW43_LINK_DOWN) ||
            ((uint32_t) (now_ms - g_attempt_ms) >= WIFI_JOIN_TIMEOUT_MS))
        {
            join_failed(now_ms);
        }
    }
    else if ((int32_t) (now_ms - g_retry_ms) >= 0)
    {
        g_attempting = true;
        g_attempt_ms = now_ms;
        publish_join(WIFI_JOIN_ACTIVE);
        if (cyw43_wifi_join(gp_cyw43, sizeof(g_ssid) - 1u, g_ssid,
                            sizeof(g_password) - 1u, g_password,
                            UTK_WIFI_AUTH, 0, CYW43_CHANNEL_NONE) != 0)
        {
            join_failed(sys_now());
        }
    }
    else
    {
        /* Backoff. */
    }
}
#endif

static void
tick (void * arg)
{
    (void) arg;
    sys_timeout(WIFI_LINK_TICK_MS, tick, 0);
    lease_update(sys_now());
#if TM_WIFI_JOIN
    join_update(sys_now());
#endif
    mqtt_session_poll();
}

void
wifi_link_netif_init (void * cyw43, int itf)
{
    if ((itf == CYW43_ITF_STA) && (gp_cyw43 == 0))
    {
        gp_cyw43 = (cyw43_t *) cyw43;
        lease_reset();
#if TM_WIFI_JOIN
        publish_join(WIFI_JOIN_IDLE);
#endif
        sys_timeout(WIFI_LINK_TICK_MS, tick, 0);
    }
}

wifi_lease_t
wifi_link_lease (void)
{
    return (wifi_lease_t) g_lease;
}

int32_t
wifi_link_rssi (void)
{
    return g_rssi;
}

wifi_join_t
wifi_link_join (void)
{
#if TM_WIFI_JOIN
    return (wifi_join_t) g_join;
#else
    return WIFI_JOIN_IDLE;
#endif
}
