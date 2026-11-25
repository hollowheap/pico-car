/** @file drv_wifi.c
 * @brief Hardware diagnostics and boot ID for the connection task.
 */
#define PROHIBIT_DEF_SIZE_T
#include <string.h>
#include <tk/tkernel.h>

#include "cyw43_utk.h"

#include "drv_wifi.h"

#define CONNECTION_ROSC_RANDOMBIT (0x40060000U + 0x1CU) /* ROSC RANDOMBIT */
#define CONNECTION_RUN_ID_BITS    (16U)

/* Only the connection task reads the kernel's full radio snapshot. Keep it
 * in static storage so those diagnostics do not use the task stack. */
static T_CYW43_UTK_STATUS g_radio_status;

uint16_t drv_wifi_boot_run_id (void)
{
    uint32_t id = 0U;

    /* The kernel leaves the ring oscillator running from reset. */
    for (uint32_t bit = 0U; bit < CONNECTION_RUN_ID_BITS; bit++)
    {
        id = (id << 1U) | (in_w(CONNECTION_ROSC_RANDOMBIT) & 1U);
    }
    return (uint16_t)id;
}

void drv_wifi_read_status (drv_wifi_status_t * p_status)
{
    cyw43_utk_get_status(&g_radio_status);
    p_status->init_result   = g_radio_status.init_result;
    p_status->join_result   = g_radio_status.join_result;
    p_status->join_started  = g_radio_status.join_started;
    p_status->join_complete = g_radio_status.join_complete;
    p_status->link_up       = g_radio_status.link_up;
    (void)memcpy(p_status->netif_mac, g_radio_status.netif_mac,
                 sizeof(p_status->netif_mac));
}

/*** end of file ***/
