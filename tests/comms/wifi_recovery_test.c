/* Run the radio owner task with the link timer through startup, join failures,
 * a later link loss and a renumbered lease. */
#include <assert.h>
#include <setjmp.h>
#include <string.h>

#include "cyw43.h"
#include "cyw43_utk.h"
#include "lwip/dns.h"
#include "lwip/timeouts.h"
#include "lwip_utk_ipv4.h"
#include "comms/telemetry/drv_wifi_link.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

cyw43_t cyw43_state;
volatile uint32_t cyw43_utk_poll_requested;
struct netif *netif_default = &cyw43_state.netif[CYW43_ITF_STA];
static ip_addr_t dns;
static uint64_t now_us, attempt_us, failure_us, lost_us;
static unsigned attempts, adoptions, adoptions_at_recovery, attempts_before_loss, mqtt_polls;
static bool supplied, recovered, dropped, saw_timeout, in_driver, renumbered;
static sys_timeout_handler timer_handler;
static void *timer_arg;
static uint32_t timer_due_ms;
static int link;
static jmp_buf done;
static const char *scenario;

#define RETRY_US 5000000ULL
#define RETRY_SLACK_US 10000ULL
#define JOIN_TIMEOUT_US 30000000ULL

static bool is_case(const char *name) { return strcmp(scenario, name) == 0; }
uint64_t time_us_64(void) { return now_us; }
void mp_memory_barrir(void) {}
int32_t tk_get_prc(void) { return 1; }
void cyw43_thread_enter(void) { assert(!in_driver); in_driver = true; }
void cyw43_thread_exit(void) { assert(in_driver); in_driver = false; }
void lwip_init(void) {}
u32_t sys_now(void) { return (u32_t)(now_us / 1000U); }

void sys_timeout(u32_t msecs, sys_timeout_handler handler, void *arg)
{
    assert(in_driver && timer_handler == NULL);
    timer_handler = handler;
    timer_arg = arg;
    timer_due_ms = sys_now() + msecs;
}

void sys_check_timeouts(void)
{
    sys_timeout_handler handler = timer_handler;

    assert(in_driver);
    if (handler != NULL && (int32_t)(sys_now() - timer_due_ms) >= 0) {
        timer_handler = NULL;
        handler(timer_arg);
    }
}

void mqtt_session_poll(void)
{
    assert(in_driver);
    mqtt_polls++;
}
void cyw43_init(cyw43_t *self) { (void)self; }
int cyw43_wifi_get_mac(cyw43_t *s, int i, uint8_t mac[6]) { (void)s; (void)i; (void)mac; return 0; }
int cyw43_wifi_get_bssid(cyw43_t *s, uint8_t b[6]) { (void)s; (void)b; return 0; }
int cyw43_wifi_get_rssi(cyw43_t *s, int32_t *r)
{
    (void)s;
    assert(in_driver && netif_is_link_up(netif_default));
    *r = dropped ? -64 : -50;
    return 0;
}
int cyw43_wifi_link_status(cyw43_t *s, int i) { (void)s; (void)i; return link; }
const ip_addr_t *dns_getserver(u8_t i) { (void)i; return &dns; }
u8_t dhcp_supplied_address(const struct netif *n) { (void)n; return supplied; }
void lwip_utk_ipv4_poll(struct netif *n) { (void)n; }
void lwip_utk_ipv4_get_status(T_LWIP_UTK_IPV4_STATUS *s) { *s = (T_LWIP_UTK_IPV4_STATUS){0}; }
int32_t lwip_utk_ipv4_adopt_netif(struct netif *n) { (void)n; adoptions++; return 0; }

void cyw43_wifi_set_up(cyw43_t *self, int itf, bool up, uint32_t country)
{
    (void)country;
    assert(up && itf == CYW43_ITF_STA);
    self->itf_state = 1U << itf;
    wifi_link_netif_init(self, itf);
}

int cyw43_wifi_scan(cyw43_t *s, cyw43_wifi_scan_options_t *opts, void *env,
                    int (*cb)(void *, const cyw43_ev_scan_result_t *))
{
    (void)opts; (void)env; (void)cb;
    s->wifi_scan_state = 1;
    return 0;
}

int cyw43_wifi_join(cyw43_t *s, size_t len, const uint8_t *ssid, size_t key_len,
                    const uint8_t *key, uint32_t auth, const uint8_t *bssid, uint32_t channel)
{
    (void)key; (void)auth; (void)bssid; (void)channel;
    assert(len == 7 && memcmp(ssid, "test-ap", len) == 0 && key_len == 0);
    assert(!netif_is_link_up(netif_default));
    assert(!cyw43_wifi_scan_active(s));
    if (attempts != 0) {
        /* Space retries from the failure, even when the attempt itself took 30 s. */
        uint64_t elapsed = now_us - (dropped ? lost_us : failure_us);
        assert(elapsed >= RETRY_US && elapsed <= RETRY_US + RETRY_SLACK_US);
        assert(link != CYW43_LINK_JOIN || is_case("timeout"));
    }
    attempts++;
    attempt_us = now_us;
    if (attempts == 2 && is_case("request")) {
        now_us += 250000U;
        failure_us = now_us;
        return -CYW43_EIO;
    }
    if (attempts == 1 && is_case("request")) {
        failure_us = now_us;
        return -CYW43_EIO;
    }
    link = CYW43_LINK_JOIN; /* A rejected request leaves the SDK's link state unchanged. */
    return 0;
}

static void poll_driver(void)
{
    assert(in_driver);
    cyw43_utk_poll_requested = 1U;
    if (now_us >= 100000U) cyw43_state.wifi_scan_state = 0;
    if (!attempts) return;

    if (!dropped && attempts <= 2 && !recovered) {
        if (is_case("late-link") && failure_us && now_us >= failure_us + 500000U) {
            netif_default->flags |= NETIF_FLAG_LINK_UP;
            supplied = true;
        } else if (is_case("timeout")) {
            if (failure_us < attempt_us && now_us >= attempt_us + JOIN_TIMEOUT_US) {
                failure_us = attempt_us + JOIN_TIMEOUT_US;
            }
        } else if (now_us - attempt_us >= 100000U && link == CYW43_LINK_JOIN) {
            link = CYW43_LINK_FAIL;
            failure_us = now_us;
        }
    } else if (dropped && attempts == attempts_before_loss) {
        netif_default->flags &= (u8_t)~NETIF_FLAG_LINK_UP;
        link = CYW43_LINK_DOWN;
        /* lwIP can retain the old lease while the physical link is down. */
    } else if (now_us - attempt_us >= 100000U) {
        netif_default->flags |= NETIF_FLAG_LINK_UP;
        supplied = !dropped || now_us - attempt_us > 32000000U;
    }
}
void (*cyw43_poll)(void) = poll_driver;

int32_t tk_dly_tsk(uint32_t delay)
{
    T_CYW43_UTK_STATUS st;
    bool leased = wifi_link_lease() == WIFI_LEASE_READY;

    assert(!in_driver);
    cyw43_utk_get_status(&st);
    assert(st.status_retries == 0U);
    assert(st.poll_count == mqtt_polls + 1U);
    if (leased) {
        assert(supplied && netif_is_link_up(netif_default) && st.link_up);
        assert(wifi_link_rssi() == (dropped ? -64 : -50));
        if (!recovered) {
            assert(attempts == (is_case("late-link") ? 1U : 3U));
            recovered = true;
            adoptions_at_recovery = adoptions;
            assert(adoptions_at_recovery >= 1U);
            attempts_before_loss = attempts;
            lost_us = now_us + 10000000U;
        } else if (renumbered) {
            assert(adoptions == adoptions_at_recovery + 1U && attempts == attempts_before_loss);
            longjmp(done, 1);
        } else if (dropped) {
            assert(attempts == attempts_before_loss + 1U && saw_timeout);
            assert(adoptions == adoptions_at_recovery + 1U);
            longjmp(done, 1);
        }
    }
    if (recovered && !dropped && !renumbered && now_us >= lost_us) {
        if (is_case("renumber")) {
            IP4_ADDR(&netif_default->ip_addr, 192, 0, 2, 9);
            renumbered = true;
        } else {
            dropped = true;
        }
    }
    if (dropped && !netif_is_link_up(netif_default)) {
        assert(!leased && "old lease survived link loss");
        assert(wifi_link_rssi() == INT32_MIN);
        assert(now_us - lost_us < 15000000U);
    }
    if (dropped && wifi_link_lease() == WIFI_LEASE_TIMEOUT) saw_timeout = true;
    assert(now_us < 180000000U && "radio never recovered");
    now_us += delay * 1000U;
    return 0;
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    IP4_ADDR(&netif_default->ip_addr, 192, 0, 2, 8);
    IP4_ADDR(&netif_default->netmask, 255, 255, 255, 0);
    IP4_ADDR(&netif_default->gw, 192, 0, 2, 1);
    dns = netif_default->gw;
    if (setjmp(done) == 0) cyw43_utk_task(0, NULL);
    return 0;
}
