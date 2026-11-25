/** @file hump_msg.h
 *
 * @brief Packed 32-bit Buddy 4 hump/terrain message (packed_hump_t).
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
 *   bits  2-4   motion       motion_class_t, 8 values
 *   bits  5-7   faults       PHYSICS_FAULT_* bits (same positions as in the
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
 *   - motion == MOTION_CLASS_UNKNOWN means the IMU is not calibrated yet. The
 *     first message with any other motion value is EVT_IMU_CAL_DONE.
 *   - hump_active = 1, finished = 0: EVT_HUMP_START or an update.
 *   - finished = 1 (hump_active = 0): EVT_HUMP_END. Use hump_peak only if
 *     height_valid = 1. Zero fault bits alone do not make a height valid.
 *   - hump_active = 1 with finished = 1 is never sent and fails to unpack.
 *
 * In memory the word is a native uint32_t. Use hump_to_bytes() and
 * hump_from_bytes() for MQTT, which fix the byte order (little-endian).
 */

#ifndef HUMP_MSG_H
#define HUMP_MSG_H

#include <stdbool.h>
#include <stdint.h>

#include "hump_detect.h"
#include "imu_motion.h"

typedef uint32_t packed_hump_t;

#define HUMP_WORD_BYTES (4U)   /* Bytes in the packed word.                */

/* motion_class_t (3 bits) is defined in imu_motion.h. */

/* Fault bits within the 3-bit message field. */
#define PHYSICS_FAULT_I2C        (0x1U)  /* I2C NACK/timeout after 3 retries.  */
#define PHYSICS_FAULT_STALE      (0x2U)  /* motion data stale/invalid. */
#define PHYSICS_FAULT_TILT       (0x4U)  /* Pitch outside the packed range.    */
#define PHYSICS_FAULT_ALL        (0x7U)

/* Position of the same three bits in the shared 16-bit system fault word. */
#define PHYSICS_SHARED_FAULT_SHIFT (5U)
#define PHYSICS_SHARED_FAULT_MASK  ((uint16_t)(PHYSICS_FAULT_ALL << PHYSICS_SHARED_FAULT_SHIFT))

/* Field ranges. */
#define HUMP_PITCH_MIN_DEG    (-32.0f)
#define HUMP_PITCH_MAX_DEG    (31.5f)
#define HUMP_HEIGHT_MAX_MM    (126U)  /* Largest in-range height.            */
#define HUMP_HEIGHT_OVERFLOW  (127U)  /* Code meaning 127 mm or more.        */

/* Message size in a micro T-Kernel message buffer: 4-byte header plus the
 * payload rounded up to 4 bytes (HEADERSZ + ROUNDSZ(msgsz) in
 * kernel/tkernel/messagebuf.c). */
#define PHYSICS_MBF_MSG_COST     (4U + 4U)
#define PHYSICS_MBF_DEPTH        (4U)
#define PHYSICS_MBF_BUFSZ        (PHYSICS_MBF_DEPTH * PHYSICS_MBF_MSG_COST)   /* 32 bytes */

/* Unpacked fields, in engineering units. */
typedef struct
{
    hump_state_t state;
    motion_class_t  motion;
    uint8_t      faults;         /* PHYSICS_FAULT_* bits.                       */
    float        pitch_deg;
    float        hump_peak_m;    /* Must be >= 0.                          */
    float        run_peak_m;     /* Must be >= 0.                          */
    bool         hump_active;
    bool         finished;
    bool         height_valid;
} hump_fields_t;

/* Highest valid, complete hump seen in the run. */
typedef struct
{
    float run_peak_m;
    uint16_t valid_humps;
    uint16_t rejected_humps;     /* Finished but invalid or incomplete.    */
} hump_run_t;

/* Pack fields into a word. Values are rounded to the nearest step.
 * Out-of-range pitch is saturated and sets PHYSICS_FAULT_TILT. A peak above
 * 126 mm is sent as the overflow code; for hump_peak this also clears
 * height_valid. Returns false and leaves *p_word unchanged for a null
 * pointer, an out-of-range enum or fault value, a non-finite value, a
 * negative height, or active and finished both set. */
bool hump_pack (const hump_fields_t * p_fields,
                   packed_hump_t * p_word);

/* Unpack a word. Overflow codes decode to 0.127 m. Returns false and leaves
 * *p_fields unchanged for a null pointer or a word with active and finished
 * both set. */
bool hump_unpack (packed_hump_t word, hump_fields_t * p_fields);

/* Little-endian byte order for MQTT and other byte streams. */
void hump_to_bytes (packed_hump_t word, uint8_t p_bytes[HUMP_WORD_BYTES]);
packed_hump_t hump_from_bytes (const uint8_t p_bytes[HUMP_WORD_BYTES]);

/* Run peak tracking. Call hump_run_update with every hump_detect output; it
 * only acts on HUMP_EVENT_FINISHED. A hump counts only if it finished
 * complete and with a valid height. */
void hump_run_init (hump_run_t * p_run);
void hump_run_update (hump_run_t * p_run, const hump_output_t * p_out);

/* Build message fields from one hump_detect output. motion and i2c_fault
 * come from the caller. height_valid is set only when the hump height is
 * valid and, on FINISHED, the hump was complete. Call hump_run_update with the
 * same output first, so a FINISHED message already includes this hump in
 * run_peak. */
bool hump_fields_from_output (const hump_output_t * p_out,
                                 const hump_run_t * p_run,
                                 motion_class_t motion,
                                 bool i2c_fault,
                                 hump_fields_t * p_fields);

#endif /* HUMP_MSG_H */

/*** end of file ***/
