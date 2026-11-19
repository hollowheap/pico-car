/** @file obstacle_ping.h
 *
 * @brief Layout of the ping word that obstacle_ping_read() returns and of the
 *        request word that obstacle_ping_request() stands for, with the
 *        decoders a reader needs. Kernel-free, so the line task and host
 *        tests can include it. The private pack header builds the words.
 */

#ifndef NAVIGATION_OBSTACLE_PING_H
#define NAVIGATION_OBSTACLE_PING_H

#include <stdbool.h>
#include <stdint.h>

#include "pilot/look.h"

/* Echo class, 3 bits. A missing echo is never OPEN, and no class means
 * "unknown": each class has its own reaction. */
#define OBSTACLE_CLASS_VALID     (0U) /* rise + fall, 116..23500 us         */
#define OBSTACLE_CLASS_OPEN      (1U) /* rise seen, pulse beyond 4 m        */
#define OBSTACLE_CLASS_TOO_CLOSE (2U) /* clean echo, 30..116 us: an object  */
#define OBSTACLE_CLASS_NO_ECHO   (3U) /* nothing on ECHO within the wait    */
#define OBSTACLE_CLASS_GLITCH    (4U) /* one edge missing, or a tiny pulse  */
#define OBSTACLE_CLASS_STUCK     (5U) /* ECHO high before the trigger       */

/* Ping (u32, one store by the scan task): seq 6 | class 3 | dist 12 |
 * servo 6. The servo field is the servo angle in whole degrees plus 30. */
#define OBSTACLE_PING_SEQ_SH   (0U)
#define OBSTACLE_PING_SEQ_MK   (0x3FU)
#define OBSTACLE_PING_CLASS_SH (6U)
#define OBSTACLE_PING_CLASS_MK (0x7U)
#define OBSTACLE_PING_DIST_SH  (9U)
#define OBSTACLE_PING_DIST_MK  (0xFFFU)
#define OBSTACLE_PING_SERVO_SH (21U)
#define OBSTACLE_PING_SERVO_MK (0x3FU)

/* Bit 27 marks a ping taken for a request; a free-running ping leaves it 0.
 * Bits 28 to 31 stay free. Every reader masks its own field. */
#define OBSTACLE_PING_REPLY_SH (27U)
#define OBSTACLE_PING_REPLY_MK (0x1U)

/* Servo field offset: whole degrees plus 30 (the servo limit). */
#define OBSTACLE_PING_SERVO_OFFSET (30)

/* Deci-degrees per degree, and the range a TOO_CLOSE echo reads as. The
 * private obstacle config holds the same values and checks them at build
 * time. */
#define OBSTACLE_PING_DECI_PER_DEG  (10)
#define OBSTACLE_PING_TOO_CLOSE_MM  (20U)

/* Ping request (u32, written whole under the spinlock): servo 16 (signed
 * deci-degrees) | seq 6 | pending 1. A zero word is "no request". */
#define OBSTACLE_REQ_SERVO_SH   (0U)
#define OBSTACLE_REQ_SERVO_MK   (0xFFFFU)
#define OBSTACLE_REQ_SEQ_SH     (16U)
#define OBSTACLE_REQ_SEQ_MK     (0x3FU)
#define OBSTACLE_REQ_PENDING_SH (22U)
#define OBSTACLE_REQ_PENDING_MK (0x1U)

/*!
 * @brief Servo angle of a ping word.
 *
 * @param[in] word Packed ping word.
 *
 * @return Angle in deci-degrees (whole degrees only).
 */
static inline int32_t obstacle_pack_ping_servo (uint32_t word)
{
    int32_t const deg = (int32_t)((word >> OBSTACLE_PING_SERVO_SH) & OBSTACLE_PING_SERVO_MK);

    return ((deg - OBSTACLE_PING_SERVO_OFFSET) * OBSTACLE_PING_DECI_PER_DEG);
}

/*!
 * @brief Pack one ping request.
 *
 * @param[in] servo_deci Wanted servo angle, deci-degrees (the scan task clamps it).
 * @param[in] seq        Request sequence (wraps at 64).
 *
 * @return The request word, with the pending flag set (never zero).
 */
static inline uint32_t obstacle_pack_ping_req (int32_t servo_deci, uint32_t seq)
{
    /* Cast is safe: only the low 16 bits are kept, as two's complement. */
    uint32_t const angle = (uint32_t)(uint16_t)servo_deci;

    return (((angle & OBSTACLE_REQ_SERVO_MK) << OBSTACLE_REQ_SERVO_SH) |
            ((seq & OBSTACLE_REQ_SEQ_MK) << OBSTACLE_REQ_SEQ_SH) |
            (OBSTACLE_REQ_PENDING_MK << OBSTACLE_REQ_PENDING_SH));
}

/*!
 * @brief Tell whether a ping word was taken for a request.
 *
 * @param[in] word Packed ping word.
 *
 * @return True if the reply flag is set.
 */
static inline bool obstacle_pack_ping_is_reply (uint32_t word)
{
    return (0U != ((word >> OBSTACLE_PING_REPLY_SH) & OBSTACLE_PING_REPLY_MK));
}

/*!
 * @brief Echo class of a ping word as the pilot reads it (LOOK_ECHO_*).
 *
 * NO_ECHO, GLITCH, STUCK and any unknown code have no reading: they give
 * LOOK_ECHO_NONE and never LOOK_ECHO_OPEN.
 *
 * @param[in] word Packed ping word.
 *
 * @return LOOK_ECHO_VALID, LOOK_ECHO_OPEN, LOOK_ECHO_TOO_CLOSE or LOOK_ECHO_NONE.
 */
static inline uint8_t obstacle_pack_ping_echo (uint32_t word)
{
    uint32_t const cls  = (word >> OBSTACLE_PING_CLASS_SH) & OBSTACLE_PING_CLASS_MK;
    uint8_t        echo = LOOK_ECHO_NONE;

    if (OBSTACLE_CLASS_VALID == cls)
    {
        echo = LOOK_ECHO_VALID;
    }
    else if (OBSTACLE_CLASS_OPEN == cls)
    {
        echo = LOOK_ECHO_OPEN;
    }
    else if (OBSTACLE_CLASS_TOO_CLOSE == cls)
    {
        echo = LOOK_ECHO_TOO_CLOSE;
    }
    else
    {
        /* No reading. */
    }

    return (echo);
}

/*!
 * @brief Distance of a ping word: the range for VALID, the minimum range for
 *        TOO_CLOSE (a hit, as the statechart reads it), 0 otherwise.
 *
 * @param[in] word Packed ping word.
 *
 * @return Distance in mm.
 */
static inline uint16_t obstacle_pack_ping_mm (uint32_t word)
{
    uint32_t const cls = (word >> OBSTACLE_PING_CLASS_SH) & OBSTACLE_PING_CLASS_MK;
    uint16_t       mm  = 0U;

    if (OBSTACLE_CLASS_VALID == cls)
    {
        /* Cast is safe: the field is 12 bits wide. */
        mm = (uint16_t)((word >> OBSTACLE_PING_DIST_SH) & OBSTACLE_PING_DIST_MK);
    }
    else if (OBSTACLE_CLASS_TOO_CLOSE == cls)
    {
        mm = (uint16_t)OBSTACLE_PING_TOO_CLOSE_MM;
    }
    else
    {
        /* No range. */
    }

    return (mm);
}

/*!
 * @brief Sequence of a ping word.
 *
 * @param[in] word Packed ping word.
 *
 * @return Sequence, 0..63.
 */
static inline uint8_t obstacle_pack_ping_seq (uint32_t word)
{
    /* Cast is safe: the field is 6 bits wide. */
    return ((uint8_t)((word >> OBSTACLE_PING_SEQ_SH) & OBSTACLE_PING_SEQ_MK));
}

#endif /* NAVIGATION_OBSTACLE_PING_H */

/*** end of file ***/
