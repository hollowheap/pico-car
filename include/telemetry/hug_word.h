/** @file hug_word.h
 * @brief The hug snapshot for the obstacle frame, packed into two words.
 *
 * The line task writes the two words together under one spinlock; the
 * telemetry task reads both under the same lock, so the fields of a frame
 * always belong to one pilot tick. The line task adds a 2-bit publish count
 * (seq) to word 0 after packing, so a reader can tell a new sample from a
 * repeat. A zero word 0 means "never published".
 *
 * Word 0: width 12 | breadth 12 | state 4 | side 1 | seq 2 | valid 1.
 * Word 1: face 12 | servo 6 (whole degrees + 30) | heading 12 (deci-degrees + 1800) | spare 2.
 *
 * Pure C99, no kernel. Only static inline functions and macros.
 */

#ifndef TELEMETRY_HUG_WORD_H
#define TELEMETRY_HUG_WORD_H

#include <stdbool.h>
#include <stdint.h>

#include "pilot/hug.h"
#include "telemetry/wire_schema.h"

#define HUG_WORD_COUNT      (2U)

#define HUG_WORD_WIDTH_SH   (0U)
#define HUG_WORD_WIDTH_MK   (0xFFFU)
#define HUG_WORD_BREADTH_SH (12U)
#define HUG_WORD_BREADTH_MK (0xFFFU)
#define HUG_WORD_STATE_SH   (24U)
#define HUG_WORD_STATE_MK   (0xFU)
#define HUG_WORD_SIDE_SH    (28U)
#define HUG_WORD_SIDE_MK    (0x1U)
#define HUG_WORD_SEQ_SH     (29U)
#define HUG_WORD_SEQ_MK     (0x3U)
#define HUG_WORD_VALID_SH   (31U)
#define HUG_WORD_VALID_MK   (0x1U)
#define HUG_WORD_FACE_SH    (0U)
#define HUG_WORD_FACE_MK    (0xFFFU)
#define HUG_WORD_SERVO_SH   (12U)
#define HUG_WORD_SERVO_MK   (0x3FU)
#define HUG_WORD_HEAD_SH    (18U)
#define HUG_WORD_HEAD_MK    (0xFFFU)

/* Heading field: deci-degrees plus this offset, so -1800..1800 fits 12 bits. */
#define HUG_WORD_HEAD_OFFSET (1800)

/* Servo field: whole degrees plus this offset, as in the ping word. */
#define HUG_WORD_SERVO_OFFSET (30)
#define HUG_WORD_DECI_PER_DEG (10)
#define HUG_WORD_SERVO_MAX    (60)

/* The internal codes of hug.c that the wire state is made from. They must
 * stay equal to the ST_*, THEN_* and PU_* codes there; a host test runs the
 * hug and checks that every wire state shows up. */
#define HUG_WORD_ST_IDLE     (0U)
#define HUG_WORD_ST_SCAN     (1U)
#define HUG_WORD_ST_DRIVE    (3U)
#define HUG_WORD_THEN_SCAN   (0U)
#define HUG_WORD_THEN_BACK   (1U)
#define HUG_WORD_THEN_LOOK   (5U)
#define HUG_WORD_THEN_TURN   (6U)
#define HUG_WORD_THEN_EXIT   (7U)
#define HUG_WORD_THEN_RELEASE (8U)
#define HUG_WORD_THEN_FLANK_LOOK (13U)
#define HUG_WORD_PU_CONTACT  (0U)
#define HUG_WORD_PU_INNER    (2U)

static inline uint32_t hug_word_get (uint32_t word, uint32_t shift, uint32_t mask)
{
    return ((word >> shift) & mask);
}

static inline uint32_t hug_word_put (uint32_t value, uint32_t shift, uint32_t mask)
{
    return ((value & mask) << shift);
}

static inline uint32_t hug_word_clamp (int32_t value, uint32_t mask)
{
    uint32_t result = 0U;

    if (value > 0)
    {
        result = ((uint32_t)value > mask) ? mask : (uint32_t)value;
    }

    return (result);
}

/*!
 * @brief The TELEMETRY_HUG_STATE_* value of a hug.
 *
 * @param[in] p_hug Hug state.
 *
 * @return The wire state; IDLE while the hug does not own the wheels.
 */
static inline uint8_t hug_word_state (hug_t const * p_hug)
{
    uint8_t state = (uint8_t)TELEMETRY_HUG_STATE_IDLE;

    if (HUG_WORD_ST_IDLE != p_hug->state)
    {
        if (HUG_WORD_ST_SCAN == p_hug->state)
        {
            state = (uint8_t)TELEMETRY_HUG_STATE_LOOK;
            if (HUG_WORD_PU_CONTACT == p_hug->purpose)
            {
                state = (uint8_t)TELEMETRY_HUG_STATE_CONTACT;
            }
            else if (HUG_WORD_PU_INNER == p_hug->purpose)
            {
                state = (uint8_t)TELEMETRY_HUG_STATE_INNER;
            }
            else
            {
                /* A look or a finer look. */
            }
        }
        else if (HUG_WORD_THEN_SCAN == p_hug->then)
        {
            /* A pivot to a scan, or the back-off drive before the contact look. */
            state = (uint8_t)TELEMETRY_HUG_STATE_LOOK;
            if (HUG_WORD_ST_DRIVE == p_hug->state)
            {
                state = (uint8_t)TELEMETRY_HUG_STATE_BACKOFF;
            }
            else if (HUG_WORD_PU_CONTACT == p_hug->purpose)
            {
                state = (uint8_t)TELEMETRY_HUG_STATE_CONTACT;
            }
            else
            {
                /* The pivot to a look. */
            }
        }
        else if (HUG_WORD_THEN_BACK == p_hug->then)
        {
            state = (uint8_t)TELEMETRY_HUG_STATE_BACKOFF;
        }
        else if ((HUG_WORD_THEN_LOOK == p_hug->then) || (HUG_WORD_THEN_FLANK_LOOK == p_hug->then))
        {
            state = (uint8_t)TELEMETRY_HUG_STATE_LEG;
        }
        else if (HUG_WORD_THEN_TURN == p_hug->then)
        {
            state = (uint8_t)TELEMETRY_HUG_STATE_TURN;
        }
        else if ((HUG_WORD_THEN_EXIT == p_hug->then) || (HUG_WORD_THEN_RELEASE == p_hug->then))
        {
            state = (uint8_t)TELEMETRY_HUG_STATE_EXIT;
        }
        else
        {
            /* Approach, align and the pivot before a leg. */
            state = (uint8_t)TELEMETRY_HUG_STATE_ALIGN;
        }
    }

    return (state);
}

/*!
 * @brief Pack the hug and the servo angle of the latest ping.
 *
 * Ranges: width, breadth and face saturate to 0..4095 mm (a negative face is
 * 0). The servo goes in whole degrees, cut toward zero (157 deci-degrees is
 * sent as 150), within -30..+30 degrees. The heading fits -1800..1800
 * deci-degrees.
 *
 * @param[out] p_word     Two words.
 * @param[in]  p_hug      Hug state.
 * @param[in]  servo_deci Servo angle of the latest ping, deci-degrees.
 * @param[in]  heading_deci Body heading from the hit, deci-degrees (hug_heading_deci).
 */
static inline void hug_word_pack (uint32_t * p_word, hug_t const * p_hug, int32_t servo_deci,
                                  int32_t heading_deci)
{
    int32_t const deg   = (servo_deci / HUG_WORD_DECI_PER_DEG) + HUG_WORD_SERVO_OFFSET;
    uint32_t const side = (p_hug->side > 0) ? (uint32_t)TELEMETRY_HUG_SIDE_LEFT
                                            : (uint32_t)TELEMETRY_HUG_SIDE_RIGHT;
    bool const b_idle   = (HUG_WORD_ST_IDLE == p_hug->state);
    int32_t const face  = b_idle ? 0 : p_hug->seen_mm;
    int32_t const head  = b_idle ? 0 : heading_deci;

    p_word[0] = hug_word_put(hug_word_clamp(p_hug->width_mm, HUG_WORD_WIDTH_MK), HUG_WORD_WIDTH_SH,
                             HUG_WORD_WIDTH_MK) |
                hug_word_put(hug_word_clamp(p_hug->breadth_mm, HUG_WORD_BREADTH_MK),
                             HUG_WORD_BREADTH_SH, HUG_WORD_BREADTH_MK) |
                hug_word_put(hug_word_state(p_hug), HUG_WORD_STATE_SH, HUG_WORD_STATE_MK) |
                hug_word_put(side, HUG_WORD_SIDE_SH, HUG_WORD_SIDE_MK) |
                hug_word_put(1U, HUG_WORD_VALID_SH, HUG_WORD_VALID_MK);
    p_word[1] = hug_word_put(hug_word_clamp(face, HUG_WORD_FACE_MK), HUG_WORD_FACE_SH,
                             HUG_WORD_FACE_MK) |
                hug_word_put(hug_word_clamp(head + HUG_WORD_HEAD_OFFSET, HUG_WORD_HEAD_MK),
                             HUG_WORD_HEAD_SH, HUG_WORD_HEAD_MK) |
                hug_word_put(hug_word_clamp(deg, HUG_WORD_SERVO_MAX), HUG_WORD_SERVO_SH,
                             HUG_WORD_SERVO_MK);
}

/*!
 * @brief Tell whether the line task has published a snapshot.
 */
static inline bool hug_word_is_valid (uint32_t const * p_word)
{
    return (0U != hug_word_get(p_word[0], HUG_WORD_VALID_SH, HUG_WORD_VALID_MK));
}

/*!
 * @brief Servo angle of the snapshot, deci-degrees (whole degrees only).
 */
static inline int16_t hug_word_servo_deci (uint32_t const * p_word)
{
    int32_t const deg = (int32_t)hug_word_get(p_word[1], HUG_WORD_SERVO_SH, HUG_WORD_SERVO_MK);

    /* Cast is safe: the angle is within +-300 deci-degrees. */
    return ((int16_t)((deg - HUG_WORD_SERVO_OFFSET) * HUG_WORD_DECI_PER_DEG));
}

/*!
 * @brief Body heading of the snapshot from the hit, deci-degrees, left positive.
 */
static inline int16_t hug_word_heading_deci (uint32_t const * p_word)
{
    int32_t const head = (int32_t)hug_word_get(p_word[1], HUG_WORD_HEAD_SH, HUG_WORD_HEAD_MK);

    /* Cast is safe: the heading is within +-1800 deci-degrees. */
    return ((int16_t)(head - HUG_WORD_HEAD_OFFSET));
}

#endif /* TELEMETRY_HUG_WORD_H */

/*** end of file ***/
