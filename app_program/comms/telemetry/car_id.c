/** @file car_id.c
 * @brief Car fingerprint taken from the radio MAC, and the MQTT client ID.
 */
#include "comms/telemetry/car_id.h"

#define CAR_ID_ID_BYTES 4U
#define CAR_ID_BYTE_BITS 8U
#define CAR_ID_NIBBLE_BITS 4U
#define CAR_ID_NIBBLE_MASK 0xFU
#define CAR_ID_DECIMAL_LETTERS 10U

uint32_t car_id_from_mac (const uint8_t mac[CAR_ID_MAC_BYTES])
{
    uint32_t id = 0U;

    for (uint32_t i = CAR_ID_MAC_BYTES - CAR_ID_ID_BYTES; i < CAR_ID_MAC_BYTES;
         i++)
    {
        id = (id << CAR_ID_BYTE_BITS) | mac[i];
    }
    return id;
}

uint32_t car_id_client_id (char * p_dst, uint32_t size, const char * p_base,
                           uint32_t id)
{
    uint32_t len = 0U;

    while (p_base[len] != '\0')
    {
        len++;
    }
    if ((len + 1U + CAR_ID_HEX_DIGITS + 1U) > size)
    {
        return 0U;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        p_dst[i] = p_base[i];
    }
    p_dst[len] = '-';
    for (uint32_t digit = 0U; digit < CAR_ID_HEX_DIGITS; digit++)
    {
        uint32_t const shift =
            (uint32_t)(CAR_ID_HEX_DIGITS - 1U - digit) * CAR_ID_NIBBLE_BITS;
        uint32_t const nibble = (id >> shift) & CAR_ID_NIBBLE_MASK;

        p_dst[len + 1U + digit] =
            (char)((nibble < CAR_ID_DECIMAL_LETTERS) ? ('0' + nibble)
                                                      : ('a' + nibble - CAR_ID_DECIMAL_LETTERS));
    }
    p_dst[len + 1U + CAR_ID_HEX_DIGITS] = '\0';
    return len + 1U + CAR_ID_HEX_DIGITS;
}

/*** end of file ***/
