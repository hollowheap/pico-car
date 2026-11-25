#ifndef TELEMETRY_WIFI_LINK_H
#define TELEMETRY_WIFI_LINK_H

#include <stdint.h>

#include "platform/static_assert.h"

typedef enum
{
    WIFI_LEASE_PENDING = 0,
    WIFI_LEASE_READY,
    WIFI_LEASE_TIMEOUT
} wifi_lease_t;

STATIC_ASSERT (sizeof(wifi_lease_t) <= 4U, "wifi_lease_t size");

typedef enum
{
    WIFI_JOIN_IDLE = 0,
    WIFI_JOIN_ACTIVE,
    WIFI_JOIN_FAILED
} wifi_join_t;

STATIC_ASSERT (sizeof(wifi_join_t) <= 4U, "wifi_join_t size");

/* Arms the repeating radio timer. The CYW43 driver calls it from the radio
 * owner task while it brings the station interface up, which is the only
 * context where lwIP timeouts may be registered. The declaration is weak so
 * the driver builds without this library. */
void wifi_link_netif_init (void * cyw43, int itf) __attribute__((weak));

/* DHCP lease state, kept by the radio timer. Any task may read it. */
wifi_lease_t wifi_link_lease (void);

/* Join state while the link is down, kept by the radio timer. Any task may
 * read it. */
wifi_join_t wifi_link_join (void);

/* Signal for the current association, or INT32_MIN if unavailable. */
int32_t wifi_link_rssi (void);

#endif /* TELEMETRY_WIFI_LINK_H */
