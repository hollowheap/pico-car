/** @file car_id.h
 * @brief Car fingerprint taken from the radio MAC, and the MQTT client ID.
 */
#ifndef TELEMETRY_CAR_ID_H
#define TELEMETRY_CAR_ID_H

#include <stdint.h>

#define CAR_ID_MAC_BYTES 6U
#define CAR_ID_HEX_DIGITS 8U

/* The last four MAC bytes, big-endian. The first two bytes are the same on
 * every Pico W. Zero means the radio has no MAC yet. */
uint32_t car_id_from_mac (const uint8_t mac[CAR_ID_MAC_BYTES]);

/* Write "<base>-<id as eight lowercase hex digits>" and a terminator. The
 * result is the text length, or 0 when it does not fit in size bytes. */
uint32_t car_id_client_id (char * p_dst, uint32_t size, const char * p_base,
                           uint32_t id);

#endif /* TELEMETRY_CAR_ID_H */

/*** end of file ***/
