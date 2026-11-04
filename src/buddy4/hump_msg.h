/** @file hump_msg.h
 *
 * @brief Packed 32-bit Buddy 4 hump/terrain message (b4_packed_hump_t).
 *
 * Implements the minimum-bit requirement from Tribunal 1 for the messages
 * Buddy 4 sends to Navigation (mbf_nav_evt_id) and Telemetry (mbf_telem_id),
 * and for the binary MQTT payload. Uses explicit masks and shifts on an
 * unsigned 32-bit word instead of C bit-fields, whose bit order is
 * implementation-defined (BARR-C:2018 rule 5.5b). No bitwise operator touches
 * a signed value (rule 5.3): pitch is stored as an offset code.
 *
 * Bit layout, bit 0 = least significant:
 *
 *   bits  0-1   state        hump_state_t (FLAT, CLIMB, CREST, DESCEND)
 *   bits  2-4   motion       b4_motion_t, 8 values
 *   bits  5-7   faults       B4_FAULT_* bits (same positions as in the
 *                            shared 16-bit fault word, bits 5 to 7)
 *   bits  8-14  pitch        offset code = half-degrees + 64, 0..127,
 *                            i.e. -32.0 to +31.5 degrees in 0.5 degree steps
 *   bits 15-21  hump_peak    peak of the current or just-finished hump,
 *                            0..126 mm; 127 = 127 mm or more (overflow)
 *   bits 22-28  run_peak     highest valid complete hump this run,
 *                            0..126 mm; 127 = 127 mm or more (overflow)
 *   bit  29     hump_active  a hump is in progress
 *   bit  30     finished     this message ends a hump (EVT_HUMP_END)
 *   bit  31     height_valid hump_peak is a trustworthy measurement
 *
 * Reading rules for receivers:
 *   - motion == B4_MOTION_UNKNOWN means the IMU is not calibrated yet. The
 *     first message with any other motion value is EVT_IMU_CAL_DONE.
 *   - hump_active = 1, finished = 0: EVT_HUMP_START or an update.
 *   - finished = 1 (hump_active = 0): EVT_HUMP_END. Use hump_peak only if
 *     height_valid = 1. Zero fault bits alone do not make a height valid.
 *   - hump_active = 1 with finished = 1 is never sent and fails to unpack.
 *
 * In memory the word is a native uint32_t. Use b4_hump_to_bytes() and
 * b4_hump_from_bytes() for MQTT, which fix the byte order (little-endian).
 */

#ifndef HUMP_MSG_H
#define HUMP_MSG_H

#include <stdbool.h>
#include <stdint.h>

#include "hump_detect.h"
#include "imu_motion.h"

typedef uint32_t b4_packed_hump_t;

/* b4_motion_t (3 bits) is defined in imu_motion.h. */

/* Fault bits within the 3-bit message field. */
#define B4_FAULT_I2C        (0x1U)  /* I2C NACK/timeout after 3 retries.  */
#define B4_FAULT_STALE      (0x2U)  /* Buddy 2 motion data stale/invalid. */
#define B4_FAULT_TILT       (0x4U)  /* Pitch outside the packed range.    */
#define B4_FAULT_ALL        (0x7U)

/* Position of the same three bits in the shared 16-bit system fault word. */
#define B4_SHARED_FAULT_SHIFT (5U)
#define B4_SHARED_FAULT_MASK  ((uint16_t)(B4_FAULT_ALL << B4_SHARED_FAULT_SHIFT))

/* Field ranges. */
#define B4_PITCH_MIN_DEG    (-32.0f)
#define B4_PITCH_MAX_DEG    (31.5f)
#define B4_HEIGHT_MAX_MM    (126U)  /* Largest in-range height.            */
#define B4_HEIGHT_OVERFLOW  (127U)  /* Code meaning 127 mm or more.        */

/* Message size in a micro T-Kernel message buffer: 4-byte header plus the
 * payload rounded up to 4 bytes (HEADERSZ + ROUNDSZ(msgsz) in
 * kernel/tkernel/messagebuf.c). */
#define B4_MBF_MSG_COST     (4U + 4U)
#define B4_MBF_DEPTH        (4U)
#define B4_MBF_BUFSZ        (B4_MBF_DEPTH * B4_MBF_MSG_COST)   /* 32 bytes */

/* Unpacked fields, in engineering units. */
typedef struct
{
    hump_state_t state;
    b4_motion_t  motion;
    uint8_t      faults;         /* B4_FAULT_* bits.                       */
    float        pitch_deg;
    float        hump_peak_m;    /* Must be >= 0.                          */
    float        run_peak_m;     /* Must be >= 0.                          */
    bool         hump_active;
    bool         finished;
    bool         height_valid;
} b4_hump_fields_t;

/* Highest valid, complete hump seen in the run. */
typedef struct
{
    float run_peak_m;
    uint16_t valid_humps;
    uint16_t rejected_humps;     /* Finished but invalid or incomplete.    */
} b4_run_t;

/* Pack fields into a word. Values are rounded to the nearest step.
 * Out-of-range pitch is saturated and sets B4_FAULT_TILT. A peak above
 * 126 mm is sent as the overflow code; for hump_peak this also clears
 * height_valid. Returns false and leaves *p_word unchanged for a null
 * pointer, an out-of-range enum or fault value, a non-finite value, a
 * negative height, or active and finished both set. */
bool b4_hump_pack (const b4_hump_fields_t * p_fields,
                   b4_packed_hump_t * p_word);

/* Unpack a word. Overflow codes decode to 0.127 m. Returns false and leaves
 * *p_fields unchanged for a null pointer or a word with active and finished
 * both set. */
bool b4_hump_unpack (b4_packed_hump_t word, b4_hump_fields_t * p_fields);

/* Little-endian byte order for MQTT and other byte streams. */
void b4_hump_to_bytes (b4_packed_hump_t word, uint8_t p_bytes[4]);
b4_packed_hump_t b4_hump_from_bytes (const uint8_t p_bytes[4]);

/* Run peak tracking. Call b4_run_update with every hump_detect output; it
 * only acts on HUMP_EVENT_FINISHED. A hump counts only if it finished
 * complete and with a valid height. */
void b4_run_init (b4_run_t * p_run);
void b4_run_update (b4_run_t * p_run, const hump_output_t * p_out);

/* Build message fields from one hump_detect output. motion and i2c_fault
 * come from the caller. height_valid is set only when the hump height is
 * valid and, on FINISHED, the hump was complete. Call b4_run_update with the
 * same output first, so a FINISHED message already includes this hump in
 * run_peak. */
bool b4_hump_fields_from_output (const hump_output_t * p_out,
                                 const b4_run_t * p_run,
                                 b4_motion_t motion,
                                 bool i2c_fault,
                                 b4_hump_fields_t * p_fields);

#endif /* HUMP_MSG_H */

/*** end of file ***/
