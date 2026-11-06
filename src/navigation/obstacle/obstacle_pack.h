/** @file obstacle_pack.h
 *
 * @brief Buddy 5 codes and packed-word layouts. Every shared or stored
 *        field is packed with explicit masks and shifts (no C bit-fields
 *        across task/ISR boundaries). See the bit table in
 *        docs/obstacle.md.
 *
 * This header has no kernel includes.
 */

#ifndef OBSTACLE_PACK_H
#define OBSTACLE_PACK_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "navigation/obstacle_cmd.h"

/* ------------------------------------------------------------------ *
 * Codes.
 * ------------------------------------------------------------------ */

/* Echo class, 2 bits. A missing echo is never OPEN. */
#define OBSTACLE_CLASS_VALID   (0U) /* rise + fall, 116..23500 us           */
#define OBSTACLE_CLASS_OPEN    (1U) /* rise seen, pulse beyond 4 m           */
#define OBSTACLE_CLASS_UNKNOWN (2U) /* no rise, glitch or lost edge          */
#define OBSTACLE_CLASS_STUCK   (3U) /* ECHO high before the trigger          */

/* Statechart, 3 bits. */
#define OBSTACLE_STATE_STOPPED   (0U)
#define OBSTACLE_STATE_MONITOR   (1U)
#define OBSTACLE_STATE_WAIT_STOP (2U)
#define OBSTACLE_STATE_SCANNING  (3U)
#define OBSTACLE_STATE_PLAN      (4U)
#define OBSTACLE_STATE_BYPASS    (5U) /* escape: bend away, servo ahead      */
#define OBSTACLE_STATE_REACQUIRE (6U) /* hug with servo at 45 deg, find line */

/* Why Buddy 5 is STOPPED, 4 bits. */
#define OBSTACLE_CAUSE_NONE          (0U)
#define OBSTACLE_CAUSE_BOOT          (1U)
#define OBSTACLE_CAUSE_EXTERNAL      (2U) /* STOP command                     */
#define OBSTACLE_CAUSE_NO_ECHO       (3U)
#define OBSTACLE_CAUSE_ECHO_STUCK    (4U)
#define OBSTACLE_CAUSE_SCAN_TIMEOUT  (5U)
#define OBSTACLE_CAUSE_NO_SAFE_PATH  (6U)
#define OBSTACLE_CAUSE_COLLISION     (7U)
#define OBSTACLE_CAUSE_HANDOVER      (8U) /* no SCAN_REQ in time              */
#define OBSTACLE_CAUSE_PLAN_TIMEOUT  (9U) /* no ACCEPT/REJECT in time         */
#define OBSTACLE_CAUSE_PLAN_REJECTED (10U)
#define OBSTACLE_CAUSE_BYPASS_LIMIT  (11U)
#define OBSTACLE_CAUSE_LINE_TIMEOUT  (12U)
#define OBSTACLE_CAUSE_INIT_FAIL     (13U)

/* Fault byte: Buddy 5's task is its only writer. */
#define OBSTACLE_FAULT_NO_ECHO      (1U << 0U)
#define OBSTACLE_FAULT_ECHO_STUCK   (1U << 1U)
#define OBSTACLE_FAULT_SCAN_TIMEOUT (1U << 2U)
#define OBSTACLE_FAULT_NO_SAFE_PATH (1U << 3U)
#define OBSTACLE_FAULT_COLLISION    (1U << 4U)
#define OBSTACLE_FAULT_HANDOVER     (1U << 5U)
#define OBSTACLE_FAULT_BYPASS_LIMIT (1U << 6U)
#define OBSTACLE_FAULT_LINE_TIMEOUT (1U << 7U)

/* Commands from Navigation, 3 bits. */
#define OBSTACLE_CMD_NONE       (0U)
#define OBSTACLE_CMD_SCAN_REQ   (1U) /* car confirmed stopped, please scan   */
#define OBSTACLE_CMD_ACCEPT     (2U)
#define OBSTACLE_CMD_REJECT     (3U)
#define OBSTACLE_CMD_LINE_FOUND (4U)

/* Events to Navigation, 2 bits. */
#define OBSTACLE_EVT_OBSTACLE    (0U)
#define OBSTACLE_EVT_SCAN_RESULT (1U)
#define OBSTACLE_EVT_FAULT       (2U)
#define OBSTACLE_EVT_STEER       (3U)

/* Steering, 2 bits. Same order as Buddy 2's motion_drive_action_t. */
#define OBSTACLE_STEER_STRAIGHT (0U)
#define OBSTACLE_STEER_LEFT     (1U)
#define OBSTACLE_STEER_RIGHT    (2U)
#define OBSTACLE_STEER_HOLD     (3U)

/* Scan result status, 2 bits, and bypass side, 1 bit. */
#define OBSTACLE_RESULT_OK           (0U)
#define OBSTACLE_RESULT_CLEAR        (1U) /* nothing within 600 mm, all real */
#define OBSTACLE_RESULT_NO_SAFE_PATH (2U)
#define OBSTACLE_RESULT_FAILED       (3U)
#define OBSTACLE_SIDE_LEFT           (0U) /* pass the obstacle on its left   */
#define OBSTACLE_SIDE_RIGHT          (1U)

/* ------------------------------------------------------------------ *
 * Layouts, as (shift, mask) pairs.
 * ------------------------------------------------------------------ */

/* Echo mailbox (u32, ISR writes it whole): width 16 | RISE | DONE. */
#define OBSTACLE_MBX_WIDTH_MASK (0x0000FFFFUL)
#define OBSTACLE_MBX_RISE       (1UL << 16U)
#define OBSTACLE_MBX_DONE       (1UL << 17U)

/* Scan slot (u16): dist 12 | class 2 | measured 1. */
#define OBSTACLE_SLOT_DIST_SH  (0U)
#define OBSTACLE_SLOT_DIST_MK  (0x0FFFU)
#define OBSTACLE_SLOT_CLASS_SH (12U)
#define OBSTACLE_SLOT_CLASS_MK (0x3U)
#define OBSTACLE_SLOT_MEAS_SH  (14U)
#define OBSTACLE_SLOT_MEAS_MK  (0x1U)

/* Status (u32, one store): state 3 | cause 4 | class 2 | dist 12 | id 5 |
 * seq 6. */
#define OBSTACLE_ST_STATE_SH (0U)
#define OBSTACLE_ST_STATE_MK (0x7U)
#define OBSTACLE_ST_CAUSE_SH (3U)
#define OBSTACLE_ST_CAUSE_MK (0xFU)
#define OBSTACLE_ST_CLASS_SH (7U)
#define OBSTACLE_ST_CLASS_MK (0x3U)
#define OBSTACLE_ST_DIST_SH  (9U)
#define OBSTACLE_ST_DIST_MK  (0xFFFU)
#define OBSTACLE_ST_ID_SH    (21U)
#define OBSTACLE_ST_ID_MK    (0x1FU)
#define OBSTACLE_ST_SEQ_SH   (26U)
#define OBSTACLE_ST_SEQ_MK   (0x3FU)

/* Command (u8): type 3 | id 5. */
#define OBSTACLE_CMD_TYPE_SH (0U)
#define OBSTACLE_CMD_TYPE_MK (0x7U)
#define OBSTACLE_CMD_ID_SH   (3U)
#define OBSTACLE_CMD_ID_MK   (0x1FU)

/* Event (u32): type 2 | id 5 | payload 25. */
#define OBSTACLE_EV_TYPE_SH (0U)
#define OBSTACLE_EV_TYPE_MK (0x3U)
#define OBSTACLE_EV_ID_SH   (2U)
#define OBSTACLE_EV_ID_MK   (0x1FU)
/* OBSTACLE payload: dist 12. */
#define OBSTACLE_EV_DIST_SH (7U)
#define OBSTACLE_EV_DIST_MK (0xFFFU)
/* SCAN_RESULT payload: status 2 | side 1 | width 10 | start angle 6. */
#define OBSTACLE_EV_STATUS_SH (7U)
#define OBSTACLE_EV_STATUS_MK (0x3U)
#define OBSTACLE_EV_SIDE_SH   (9U)
#define OBSTACLE_EV_SIDE_MK   (0x1U)
#define OBSTACLE_EV_WIDTH_SH  (10U)
#define OBSTACLE_EV_WIDTH_MK  (0x3FFU)
#define OBSTACLE_EV_ANGLE_SH  (20U)
#define OBSTACLE_EV_ANGLE_MK  (0x3FU)
/* FAULT payload: cause 4 | fault byte 8. */
#define OBSTACLE_EV_CAUSE_SH  (7U)
#define OBSTACLE_EV_CAUSE_MK  (0xFU)
#define OBSTACLE_EV_FAULTS_SH (11U)
#define OBSTACLE_EV_FAULTS_MK (0xFFU)
/* STEER payload: action 2 | seq 6. */
#define OBSTACLE_EV_STEER_SH (7U)
#define OBSTACLE_EV_STEER_MK (0x3U)
#define OBSTACLE_EV_SSEQ_SH  (9U)
#define OBSTACLE_EV_SSEQ_MK  (0x3FU)

/* Task-private counters (u16): unknown 3 | stuck 2 | hits 3 |
 * collision 2 | clear 3. */
#define OBSTACLE_CNT_UNK_SH   (0U)
#define OBSTACLE_CNT_UNK_MK   (0x7U)
#define OBSTACLE_CNT_STUCK_SH (3U)
#define OBSTACLE_CNT_STUCK_MK (0x3U)
#define OBSTACLE_CNT_HIT_SH   (5U)
#define OBSTACLE_CNT_HIT_MK   (0x7U)
#define OBSTACLE_CNT_COLL_SH  (8U)
#define OBSTACLE_CNT_COLL_MK  (0x3U)
#define OBSTACLE_CNT_CLEAR_SH (10U)
#define OBSTACLE_CNT_CLEAR_MK (0x7U)

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
static inline uint32_t obstacle_pack_get (uint32_t word, uint32_t shift,
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
static inline uint32_t obstacle_pack_put (uint32_t word, uint32_t shift,
                                    uint32_t mask, uint32_t value)
{
    return ((word & ~(mask << shift)) | ((value & mask) << shift));
}

/*!
 * @brief Pack one measured scan slot.
 *
 * @param[in] dist_mm Distance; values above 4095 are saturated to 4095.
 * @param[in] cls     Echo class (OBSTACLE_CLASS_*).
 *
 * @return The packed 16-bit slot.
 */
static inline uint16_t obstacle_pack_slot (uint32_t dist_mm, uint32_t cls)
{
    uint32_t       word = 0U;
    uint32_t const dist =
        (dist_mm > OBSTACLE_SLOT_DIST_MK) ? OBSTACLE_SLOT_DIST_MK : dist_mm;

    word = obstacle_pack_put(word, OBSTACLE_SLOT_DIST_SH, OBSTACLE_SLOT_DIST_MK, dist);
    word = obstacle_pack_put(word, OBSTACLE_SLOT_CLASS_SH, OBSTACLE_SLOT_CLASS_MK, cls);
    word = obstacle_pack_put(word, OBSTACLE_SLOT_MEAS_SH, OBSTACLE_SLOT_MEAS_MK, 1U);

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
static inline uint16_t obstacle_pack_slot_dist (uint16_t slot)
{
    /* Cast is safe: the field is 12 bits wide. */
    return ((uint16_t)obstacle_pack_get(slot, OBSTACLE_SLOT_DIST_SH, OBSTACLE_SLOT_DIST_MK));
}

/*!
 * @brief Read the echo class of a packed slot.
 *
 * @param[in] slot The packed slot.
 *
 * @return Echo class (OBSTACLE_CLASS_*).
 */
static inline uint8_t obstacle_pack_slot_class (uint16_t slot)
{
    /* Cast is safe: the field is 2 bits wide. */
    return ((uint8_t)obstacle_pack_get(slot, OBSTACLE_SLOT_CLASS_SH, OBSTACLE_SLOT_CLASS_MK));
}

/*!
 * @brief Tell whether a slot has been measured in this scan.
 *
 * @param[in] slot The packed slot.
 *
 * @return True if the slot was measured.
 */
static inline bool obstacle_pack_slot_is_measured (uint16_t slot)
{
    return (0U != obstacle_pack_get(slot, OBSTACLE_SLOT_MEAS_SH, OBSTACLE_SLOT_MEAS_MK));
}

/*!
 * @brief Build a command byte for obstacle_post_cmd().
 *
 * @param[in] type   Command type (OBSTACLE_CMD_*).
 * @param[in] req_id Request id (0..31).
 *
 * @return The packed command byte.
 */
static inline uint8_t obstacle_pack_cmd (uint32_t type, uint32_t req_id)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_CMD_TYPE_SH, OBSTACLE_CMD_TYPE_MK, type);
    word = obstacle_pack_put(word, OBSTACLE_CMD_ID_SH, OBSTACLE_CMD_ID_MK, req_id);

    /* Cast is safe: the two fields use bits 0-7 only. */
    return ((uint8_t)word);
}

#endif /* OBSTACLE_PACK_H */

/*** end of file ***/

