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

/* Physical travel from straight ahead, in either direction. */
#define OBSTACLE_SERVO_LIMIT_DECI (200)

/* Echo class, 3 bits. A missing echo is never OPEN, and no class means
 * "unknown": each class has its own reaction. */
#define OBSTACLE_CLASS_VALID     (0U) /* rise + fall, 116..23500 us         */
#define OBSTACLE_CLASS_OPEN      (1U) /* rise seen, pulse beyond 4 m        */
#define OBSTACLE_CLASS_TOO_CLOSE (2U) /* clean echo, 30..116 us: an object  */
#define OBSTACLE_CLASS_NO_ECHO   (3U) /* nothing on ECHO within the wait    */
#define OBSTACLE_CLASS_GLITCH    (4U) /* one edge missing, or a tiny pulse  */
#define OBSTACLE_CLASS_STUCK     (5U) /* ECHO high before the trigger       */

/* Normalized result for a reader when the raw class gives no usable ray. */
#define OBSTACLE_ECHO_UNUSABLE   (3U)

/* Two VALID samples from the same point agree within this range. */
#define OBSTACLE_AGREE_MM          (15U)

/* Range plausibility defaults for two VALID readings. After this much
 * straight travel, their range change should agree with the wheel travel. */
#define OBSTACLE_CONSIST_TRAVEL_MM (100U)
#define OBSTACLE_CONSIST_RANGE_MM  (600U)
#define OBSTACLE_CONSIST_TOL_MM    (40U)

/* Ping (u32, one store by the scan task): seq 6 | class 3 | dist 12 |
 * servo 7. The servo field is the servo angle in whole degrees plus 64. */
#define OBSTACLE_PING_SEQ_SH   (0U)
#define OBSTACLE_PING_SEQ_MK   (0x3FU)
#define OBSTACLE_PING_CLASS_SH (6U)
#define OBSTACLE_PING_CLASS_MK (0x7U)
#define OBSTACLE_PING_DIST_SH  (9U)
#define OBSTACLE_PING_DIST_MK  (0xFFFU)
#define OBSTACLE_PING_SERVO_SH (21U)
#define OBSTACLE_PING_SERVO_MK (0x7FU)

/* Bit 28 marks a ping taken for a request; a free-running ping leaves it 0. */
#define OBSTACLE_PING_REPLY_SH (28U)
#define OBSTACLE_PING_REPLY_MK (0x1U)

/* This is an internal word. The wire frame carries signed deci-degrees. */
#define OBSTACLE_PING_SERVO_OFFSET (64)

/* Deci-degrees per degree, and the range a TOO_CLOSE echo reads as. The
 * private obstacle config holds the same values and checks them at build
 * time. */
#define OBSTACLE_PING_DECI_PER_DEG  (10)
#define OBSTACLE_PING_MAX_QUANTIZATION_DECI (OBSTACLE_PING_DECI_PER_DEG - 1)
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

/* Match a decoded reply to the requested sequence and bearing. A zero
 * tolerance requires an exact bearing. The caller checks age and chassis pose. */
static inline bool obstacle_ping_reply_matches (bool b_reply, uint8_t seq, int16_t servo_deci,
                                                uint8_t requested_seq, int32_t requested_deci,
                                                uint8_t max_error_deci)
{
    return b_reply && (seq == requested_seq) &&
           (requested_deci >= (int32_t)servo_deci - max_error_deci) &&
           (requested_deci <= (int32_t)servo_deci + max_error_deci);
}

/*!
 * @brief Usable sonar observation in a published ping word.
 *
 * @param[in] word Packed ping word.
 *
 * @return VALID, OPEN or TOO_CLOSE. A failed or unknown echo yields OBSTACLE_ECHO_UNUSABLE.
 */
static inline uint8_t obstacle_pack_ping_echo (uint32_t word)
{
    uint32_t const cls = (word >> OBSTACLE_PING_CLASS_SH) & OBSTACLE_PING_CLASS_MK;
    return (uint8_t)((cls <= OBSTACLE_CLASS_TOO_CLOSE) ? cls : OBSTACLE_ECHO_UNUSABLE);
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

/* Store a ray at the nearest profile bearing. A tie or a bearing outside
 * the profile is unresolved. Return 1..9 for a stored ray in the stationary
 * servo fan (-20..+20 degrees), zero otherwise. */
uint8_t obstacle_profile_record_ray (uint16_t * p_slots, int32_t angle_deci,
                                     uint16_t range_mm, uint8_t echo);

#endif /* NAVIGATION_OBSTACLE_PING_H */

/*** end of file ***/
