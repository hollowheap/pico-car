/* Exercise the radio owner task through startup, join failures and a later link loss. */
#include <assert.h>
#include <setjmp.h>
#include <string.h>

#include "cyw43.h"
#include "cyw43_utk.h"
#include "lwip/dns.h"
#include "lwip_utk_ipv4.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

cyw43_t cyw43_state;
volatile uint32_t cyw43_utk_poll_requested;
struct netif *netif_default = &cyw43_state.netif[CYW43_ITF_STA];
static ip_addr_t dns;
static uint64_t now_us, attempt_us, failure_us, lost_us;
static unsigned attempts, adoptions, attempts_before_loss, app_polls, timeout_checks;
static bool supplied, recovered, dropped, saw_timeout, in_driver;
static int link;
static jmp_buf done;
static const char *scenario;

#define RETRY_US 5000000ULL
#define POLL_US 2000ULL

static bool is_case(const char *name) { return strcmp(scenario, name) == 0; }
uint64_t time_us_64(void) { return now_us; }
void mp_memory_barrir(void) {}
int32_t tk_get_prc(void) { return 1; }
void cyw43_thread_enter(void) { assert(!in_driver); in_driver = true; }
void cyw43_thread_exit(void) { assert(in_driver); in_driver = false; }
void lwip_init(void) {}
void sys_check_timeouts(void) { assert(in_driver); timeout_checks++; }
void cyw43_init(cyw43_t *self) { (void)self; }
int cyw43_wifi_get_mac(cyw43_t *s, int i, uint8_t mac[6]) { (void)s; (void)i; (void)mac; return 0; }
int cyw43_wifi_get_bssid(cyw43_t *s, uint8_t b[6]) { (void)s; (void)b; return 0; }
int cyw43_wifi_get_rssi(cyw43_t *s, int32_t *r) { (void)s; *r = -50; return 0; }
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
        assert(elapsed >= RETRY_US && elapsed <= RETRY_US + POLL_US);
    }
    attempts++;
    attempt_us = now_us;
    link = CYW43_LINK_JOIN; /* The SDK also returns JOIN while still associating. */
    return (attempts == 1 && is_case("request")) ? -CYW43_EIO : 0;
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
        } else if (!is_case("timeout") && now_us - attempt_us >= 100000U) {
            link = CYW43_LINK_FAIL;
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

void cyw43_utk_app_poll(void)
{
    T_CYW43_UTK_STATUS st;

    assert(in_driver);
    app_polls++;
    assert(timeout_checks == app_polls);
    cyw43_utk_get_status(&st);
    assert(st.status_retries == 0U);
    assert(st.poll_count == app_polls);
    if (!cyw43_wifi_scan_active(&cyw43_state)) {
        assert(st.scan_complete == 1U);
    }
    if (st.link_up) {
        assert(st.join_complete == 1U && st.join_result == 0);
    }
    assert(cyw43_utk_poll_requested == 0U);
    cyw43_utk_poll_requested = 1U;
}

int32_t tk_dly_tsk(uint32_t delay)
{
    T_CYW43_UTK_STATUS st;
    assert(!in_driver);
    assert(app_polls > 0U);
    assert(cyw43_utk_poll_requested == 1U);
    cyw43_utk_get_status(&st);
    assert(st.status_retries == 0U);
    assert(st.poll_count == app_polls);
    if (st.join_complete && st.join_result != 0 && !st.link_up && !dropped) {
        if (failure_us < attempt_us) {
            failure_us = now_us;
            if (is_case("request") && attempts == 1U) {
                assert(st.join_request_result == -CYW43_EIO && st.join_result == -CYW43_EIO);
            } else if (is_case("timeout")) {
                assert(st.join_result == -CYW43_ETIMEDOUT && st.join_elapsed_ms == 30000U);
            } else {
                assert(st.join_result == CYW43_LINK_FAIL);
            }
        }
        assert(now_us - failure_us < 15000000U && "failed Wi-Fi join never retried");
    }
    if (st.link_up && st.dhcp_complete && st.dhcp_result == 0) {
        assert(supplied && st.join_complete && st.join_result == 0);
        if (!recovered) {
            assert(attempts == (is_case("late-link") ? 1U : 3U));
            assert(adoptions == 1U);
            recovered = true;
            attempts_before_loss = attempts;
            lost_us = now_us + 10000000U;
        } else if (dropped) {
            assert(attempts == attempts_before_loss + 1U && adoptions == 2U && saw_timeout);
            longjmp(done, 1);
        }
    }
    if (recovered && !dropped && now_us >= lost_us) dropped = true;
    if (dropped && !st.link_up) {
        assert(!st.dhcp_complete && "old lease survived link loss");
        assert(now_us - lost_us < 15000000U);
    }
    if (dropped && st.link_up && st.dhcp_complete && st.dhcp_result != 0) saw_timeout = true;
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
