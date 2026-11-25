/** @file drv_wifi.h
 * @brief Hardware diagnostics and boot ID for the connection task.
 */
#ifndef TELEMETRY_DRV_WIFI_H
#define TELEMETRY_DRV_WIFI_H

#include <stdint.h>

#include "platform/static_assert.h"

#define DRV_WIFI_MAC_BYTES 6U

typedef struct
{
    int32_t  init_result;
    int32_t  join_result;
    uint32_t join_started;
    uint32_t join_complete;
    uint32_t link_up;
    uint8_t  netif_mac[DRV_WIFI_MAC_BYTES];
} drv_wifi_status_t;

STATIC_ASSERT (sizeof(drv_wifi_status_t) == 28U, "Wi-Fi status size");

uint16_t drv_wifi_boot_run_id (void);
void drv_wifi_read_status (drv_wifi_status_t * p_status);

#endif /* TELEMETRY_DRV_WIFI_H */

/*** end of file ***/
