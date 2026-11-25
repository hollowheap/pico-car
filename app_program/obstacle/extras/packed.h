/** @file packed.h
 *
 * @brief Buddy 5 codes and packed-word layouts. Every shared or stored
 *        field is packed with explicit masks and shifts (no C bit-fields
 *        across task/ISR boundaries). See the bit table in
 *        testing/README.md.
 *
 * This header has no kernel includes.
 */

#ifndef EXTRAS_PACK_H
#define EXTRAS_PACK_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

/* ------------------------------------------------------------------ *
 * Codes.
 * ------------------------------------------------------------------ */

/* Echo class, 2 bits. A missing echo is never OPEN. */
#define EXTRAS_CLASS_VALID   (0U) /* rise + fall, 116..23500 us           */
#define EXTRAS_CLASS_OPEN    (1U) /* rise seen, pulse beyond 4 m           */
#define EXTRAS_CLASS_UNKNOWN (2U) /* no rise, glitch or lost edge          */
#define EXTRAS_CLASS_STUCK   (3U) /* ECHO high before the trigger          */

/* Statechart, 3 bits. */
#define EXTRAS_STATE_STOPPED   (0U)
#define EXTRAS_STATE_MONITOR   (1U)
#define EXTRAS_STATE_WAIT_STOP (2U)
#define EXTRAS_STATE_SCANNING  (3U)
#define EXTRAS_STATE_PLAN      (4U)
#define EXTRAS_STATE_BYPASS    (5U) /* escape: bend away, servo ahead      */
#define EXTRAS_STATE_REACQUIRE (6U) /* bypass side scans, find the line    */

/* Why Buddy 5 is STOPPED, 4 bits. */
#define EXTRAS_CAUSE_NONE          (0U)
#define EXTRAS_CAUSE_BOOT          (1U)
#define EXTRAS_CAUSE_EXTERNAL      (2U) /* STOP command                     */
#define EXTRAS_CAUSE_NO_ECHO       (3U)
#define EXTRAS_CAUSE_ECHO_STUCK    (4U)
#define EXTRAS_CAUSE_SCAN_TIMEOUT  (5U)
#define EXTRAS_CAUSE_NO_SAFE_PATH  (6U)
#define EXTRAS_CAUSE_COLLISION     (7U)
#define EXTRAS_CAUSE_HANDOVER      (8U) /* no SCAN_REQ in time              */
#define EXTRAS_CAUSE_PLAN_TIMEOUT  (9U) /* no ACCEPT/REJECT in time         */
#define EXTRAS_CAUSE_PLAN_REJECTED (10U)
#define EXTRAS_CAUSE_BYPASS_LIMIT  (11U)
#define EXTRAS_CAUSE_LINE_TIMEOUT  (12U)
#define EXTRAS_CAUSE_INIT_FAIL     (13U)

/* Fault byte: Buddy 5's task is its only writer. */
#define EXTRAS_FAULT_NO_ECHO      (1U << 0U)
#define EXTRAS_FAULT_ECHO_STUCK   (1U << 1U)
#define EXTRAS_FAULT_SCAN_TIMEOUT (1U << 2U)
#define EXTRAS_FAULT_NO_SAFE_PATH (1U << 3U)
#define EXTRAS_FAULT_COLLISION    (1U << 4U)
#define EXTRAS_FAULT_HANDOVER     (1U << 5U)
#define EXTRAS_FAULT_BYPASS_LIMIT (1U << 6U)
#define EXTRAS_FAULT_LINE_TIMEOUT (1U << 7U)

/* Commands from Navigation, 3 bits. */
#define EXTRAS_CMD_NONE       (0U)
#define EXTRAS_CMD_SCAN_REQ   (1U) /* car confirmed stopped, please scan   */
#define EXTRAS_CMD_ACCEPT     (2U)
#define EXTRAS_CMD_REJECT     (3U)
#define EXTRAS_CMD_LINE_FOUND (4U)
#define EXTRAS_CMD_STOP       (5U)
#define EXTRAS_CMD_RESET      (6U)
#define EXTRAS_CMD_START      (7U)

/* Events to Navigation, 2 bits. */
#define EXTRAS_EVT_OBSTACLE    (0U)
#define EXTRAS_EVT_SCAN_RESULT (1U)
#define EXTRAS_EVT_FAULT       (2U)
#define EXTRAS_EVT_STEER       (3U)

/* Steering, 2 bits. Same order as Buddy 2's motion_drive_action_t. */
#define EXTRAS_STEER_STRAIGHT (0U)
#define EXTRAS_STEER_LEFT     (1U)
#define EXTRAS_STEER_RIGHT    (2U)
#define EXTRAS_STEER_HOLD     (3U)

/* Scan result status, 2 bits, and bypass side, 1 bit. */
#define EXTRAS_RESULT_OK           (0U)
#define EXTRAS_RESULT_CLEAR        (1U) /* nothing within 600 mm, all real */
#define EXTRAS_RESULT_NO_SAFE_PATH (2U)
#define EXTRAS_RESULT_FAILED       (3U)
#define EXTRAS_SIDE_LEFT           (0U) /* pass the obstacle on its left   */
#define EXTRAS_SIDE_RIGHT          (1U)

/* ------------------------------------------------------------------ *
 * Layouts, as (shift, mask) pairs.
 * ------------------------------------------------------------------ */

/* Echo mailbox (u32, ISR writes it whole): width 16 | RISE | DONE. */
#define EXTRAS_MBX_WIDTH_MASK (0x0000FFFFUL)
#define EXTRAS_MBX_RISE       (1UL << 16U)
#define EXTRAS_MBX_DONE       (1UL << 17U)

/* Scan slot (u16): dist 12 | class 2 | measured 1. */
#define EXTRAS_SLOT_DIST_SH  (0U)
#define EXTRAS_SLOT_DIST_MK  (0x0FFFU)
#define EXTRAS_SLOT_CLASS_SH (12U)
#define EXTRAS_SLOT_CLASS_MK (0x3U)
#define EXTRAS_SLOT_MEAS_SH  (14U)
#define EXTRAS_SLOT_MEAS_MK  (0x1U)

/* Status (u32, one store): state 3 | cause 4 | class 2 | dist 12 | id 5 |
 * seq 6. */
#define EXTRAS_ST_STATE_SH (0U)
#define EXTRAS_ST_STATE_MK (0x7U)
#define EXTRAS_ST_CAUSE_SH (3U)
#define EXTRAS_ST_CAUSE_MK (0xFU)
#define EXTRAS_ST_CLASS_SH (7U)
#define EXTRAS_ST_CLASS_MK (0x3U)
#define EXTRAS_ST_DIST_SH  (9U)
#define EXTRAS_ST_DIST_MK  (0xFFFU)
#define EXTRAS_ST_ID_SH    (21U)
#define EXTRAS_ST_ID_MK    (0x1FU)
#define EXTRAS_ST_SEQ_SH   (26U)
#define EXTRAS_ST_SEQ_MK   (0x3FU)

/* Ping tag (u32, one store): seq 6 | servo angle + EXTRAS_TAG_LOOK_BIAS 11.
 * seq matches the status word of the same ping. */
#define EXTRAS_TAG_SEQ_SH    (0U)
#define EXTRAS_TAG_SEQ_MK    (0x3FU)
#define EXTRAS_TAG_LOOK_SH   (6U)
#define EXTRAS_TAG_LOOK_MK   (0x7FFU)
#define EXTRAS_TAG_LOOK_BIAS (1024)

/* Command (u8): type 3 | id 5. */
#define EXTRAS_CMD_TYPE_SH (0U)
#define EXTRAS_CMD_TYPE_MK (0x7U)
#define EXTRAS_CMD_ID_SH   (3U)
#define EXTRAS_CMD_ID_MK   (0x1FU)

/* Event (u32): type 2 | id 5 | payload 25. */
#define EXTRAS_EV_TYPE_SH (0U)
#define EXTRAS_EV_TYPE_MK (0x3U)
#define EXTRAS_EV_ID_SH   (2U)
#define EXTRAS_EV_ID_MK   (0x1FU)
/* OBSTACLE payload: dist 12. */
#define EXTRAS_EV_DIST_SH (7U)
#define EXTRAS_EV_DIST_MK (0xFFFU)
/* SCAN_RESULT payload: status 2 | side 1 | width 10 | start angle 6. */
#define EXTRAS_EV_STATUS_SH (7U)
#define EXTRAS_EV_STATUS_MK (0x3U)
#define EXTRAS_EV_SIDE_SH   (9U)
#define EXTRAS_EV_SIDE_MK   (0x1U)
#define EXTRAS_EV_WIDTH_SH  (10U)
#define EXTRAS_EV_WIDTH_MK  (0x3FFU)
#define EXTRAS_EV_ANGLE_SH  (20U)
#define EXTRAS_EV_ANGLE_MK  (0x3FU)
/* FAULT payload: cause 4 | fault byte 8. */
#define EXTRAS_EV_CAUSE_SH  (7U)
#define EXTRAS_EV_CAUSE_MK  (0xFU)
#define EXTRAS_EV_FAULTS_SH (11U)
#define EXTRAS_EV_FAULTS_MK (0xFFU)
/* STEER payload: action 2 | seq 6. */
#define EXTRAS_EV_STEER_SH (7U)
#define EXTRAS_EV_STEER_MK (0x3U)
#define EXTRAS_EV_SSEQ_SH  (9U)
#define EXTRAS_EV_SSEQ_MK  (0x3FU)

/* Task-private counters (u16): unknown 3 | stuck 2 | hits 3 |
 * collision 2 | clear 3. */
#define EXTRAS_CNT_UNK_SH   (0U)
#define EXTRAS_CNT_UNK_MK   (0x7U)
#define EXTRAS_CNT_STUCK_SH (3U)
#define EXTRAS_CNT_STUCK_MK (0x3U)
#define EXTRAS_CNT_HIT_SH   (5U)
#define EXTRAS_CNT_HIT_MK   (0x7U)
#define EXTRAS_CNT_COLL_SH  (8U)
#define EXTRAS_CNT_COLL_MK  (0x3U)
#define EXTRAS_CNT_CLEAR_SH (10U)
#define EXTRAS_CNT_CLEAR_MK (0x7U)

/* ------------------------------------------------------------------ *
 * Helpers. The value is masked before it is placed, so a value that is
 * too big can never spill into its neighbouring field.
 * ------------------------------------------------------------------ */

/*!
 * @brief Read one field of a packed word.
 *
 * @param[in] word  The packed word.
 * @param[in] shift Position of the field's lowest bit.
 * @param[in] mask  Mask of the field, already shifted down to bit 0.
 *
 * @return The field value.
 */
static inline uint32_t extras_pack_get (uint32_t word, uint32_t shift,
                                    uint32_t mask)
{
    return ((word >> shift) & mask);
}

/*!
 * @brief Replace one field of a packed word.
 *
 * @param[in] word  The packed word.
 * @param[in] shift Position of the field's lowest bit.
 * @param[in] mask  Mask of the field, already shifted down to bit 0.
 * @param[in] value New field value; any bits outside the mask are dropped.
 *
 * @return The new packed word.
 */
static inline uint32_t extras_pack_put (uint32_t word, uint32_t shift,
                                    uint32_t mask, uint32_t value)
{
    return ((word & ~(mask << shift)) | ((value & mask) << shift));
}

/*!
 * @brief Pack one measured scan slot.
 *
 * @param[in] dist_mm Distance; values above 4095 are saturated to 4095.
 * @param[in] cls     Echo class (EXTRAS_CLASS_*).
 *
 * @return The packed 16-bit slot.
 */
static inline uint16_t extras_pack_slot (uint32_t dist_mm, uint32_t cls)
{
    uint32_t       word = 0U;
    uint32_t const dist =
        (dist_mm > EXTRAS_SLOT_DIST_MK) ? EXTRAS_SLOT_DIST_MK : dist_mm;

    word = extras_pack_put(word, EXTRAS_SLOT_DIST_SH, EXTRAS_SLOT_DIST_MK, dist);
    word = extras_pack_put(word, EXTRAS_SLOT_CLASS_SH, EXTRAS_SLOT_CLASS_MK, cls);
    word = extras_pack_put(word, EXTRAS_SLOT_MEAS_SH, EXTRAS_SLOT_MEAS_MK, 1U);

    /* Cast is safe: the three fields use bits 0-14 only. */
    return ((uint16_t)word);
}

/*!
 * @brief Read the distance of a packed slot.
 *
 * @param[in] slot The packed slot.
 *
 * @return Distance in mm (0..4095).
 */
static inline uint16_t extras_pack_slot_dist (uint16_t slot)
{
    /* Cast is safe: the field is 12 bits wide. */
    return ((uint16_t)extras_pack_get(slot, EXTRAS_SLOT_DIST_SH, EXTRAS_SLOT_DIST_MK));
}

/*!
 * @brief Read the echo class of a packed slot.
 *
 * @param[in] slot The packed slot.
 *
 * @return Echo class (EXTRAS_CLASS_*).
 */
static inline uint8_t extras_pack_slot_class (uint16_t slot)
{
    /* Cast is safe: the field is 2 bits wide. */
    return ((uint8_t)extras_pack_get(slot, EXTRAS_SLOT_CLASS_SH, EXTRAS_SLOT_CLASS_MK));
}

/*!
 * @brief Tell whether a slot has been measured in this scan.
 *
 * @param[in] slot The packed slot.
 *
 * @return True if the slot was measured.
 */
static inline bool extras_pack_slot_is_measured (uint16_t slot)
{
    return (0U != extras_pack_get(slot, EXTRAS_SLOT_MEAS_SH, EXTRAS_SLOT_MEAS_MK));
}

/*!
 * @brief Build a command byte for obstacle_extras_post_cmd().
 *
 * @param[in] type   Command type (EXTRAS_CMD_*).
 * @param[in] req_id Request id (0..31).
 *
 * @return The packed command byte.
 */
static inline uint8_t extras_pack_cmd (uint32_t type, uint32_t req_id)
{
    uint32_t word = 0U;

    word = extras_pack_put(word, EXTRAS_CMD_TYPE_SH, EXTRAS_CMD_TYPE_MK, type);
    word = extras_pack_put(word, EXTRAS_CMD_ID_SH, EXTRAS_CMD_ID_MK, req_id);

    /* Cast is safe: the two fields use bits 0-7 only. */
    return ((uint8_t)word);
}

#endif /* EXTRAS_PACK_H */

/*** end of file ***/
