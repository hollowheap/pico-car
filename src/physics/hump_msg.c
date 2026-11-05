/** @file hump_msg.c
 *
 * @brief Packed 32-bit Buddy 4 hump/terrain message (b4_packed_hump_t).
 *
 * See hump_msg.h for the bit layout and the reading rules.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hump_detect.h"
#include "imu_motion.h"
#include "hump_msg.h"

#define STATE_SHIFT     (0U)
#define STATE_MASK      (0x3U)
#define MOTION_SHIFT    (2U)
#define MOTION_MASK     (0x7U)
#define FAULT_SHIFT     (5U)
#define FAULT_MASK      (0x7U)
#define PITCH_SHIFT     (8U)
#define PITCH_MASK      (0x7FU)
#define HPEAK_SHIFT     (15U)
#define HPEAK_MASK      (0x7FU)
#define RPEAK_SHIFT     (22U)
#define RPEAK_MASK      (0x7FU)
#define ACTIVE_BIT      (0x20000000UL)
#define FINISHED_BIT    (0x40000000UL)
#define VALID_BIT       (0x80000000UL)

#define PITCH_OFFSET    (64.0f)   /* Code for 0 degrees.                   */
#define PITCH_CODE_MAX  (127.0f)
#define HALF_DEG_PER_DEG (2.0f)

/* Round a height in metres to a 7-bit code. Sets *p_overflow when the value
 * is above B4_HEIGHT_MAX_MM after rounding. Caller has checked >= 0. */
static uint32_t height_code (float height_m, bool * p_overflow)
{
    float mm = floorf ((height_m * 1000.0f) + 0.5f);
    uint32_t code;

    if (mm > (float)B4_HEIGHT_MAX_MM)
    {
        code = B4_HEIGHT_OVERFLOW;
        *p_overflow = true;
    }
    else
    {
        code = (uint32_t)mm;
        *p_overflow = false;
    }

    return code;
}

/* Round a pitch to a 7-bit offset code. Sets *p_clipped when the value is
 * outside B4_PITCH_MIN_DEG .. B4_PITCH_MAX_DEG after rounding. */
static uint32_t pitch_code (float pitch_deg, bool * p_clipped)
{
    float code = floorf ((pitch_deg * HALF_DEG_PER_DEG) + 0.5f) + PITCH_OFFSET;

    *p_clipped = false;
    if (code < 0.0f)
    {
        code = 0.0f;
        *p_clipped = true;
    }
    else if (code > PITCH_CODE_MAX)
    {
        code = PITCH_CODE_MAX;
        *p_clipped = true;
    }
    else
    {
        /* In range. */
    }

    return (uint32_t)code;
}

bool b4_hump_pack (const b4_hump_fields_t * p_fields,
                   b4_packed_hump_t * p_word)
{
    uint32_t word;
    uint32_t faults;
    bool clipped;
    bool hump_over;
    bool run_over;
    bool valid;

    if ((NULL == p_fields) || (NULL == p_word))
    {
        return false;
    }

    if (((uint32_t)p_fields->state > (uint32_t)HUMP_STATE_DESCEND)
        || ((uint32_t)p_fields->motion >= B4_MOTION_COUNT)
        || (((uint32_t)p_fields->faults & ~FAULT_MASK) != 0U)
        || !isfinite (p_fields->pitch_deg)
        || !isfinite (p_fields->hump_peak_m)
        || !isfinite (p_fields->run_peak_m)
        || (p_fields->hump_peak_m < 0.0f) || (p_fields->run_peak_m < 0.0f)
        || (p_fields->hump_active && p_fields->finished))
    {
        return false;
    }

    faults = (uint32_t)p_fields->faults;
    valid = p_fields->height_valid;

    word = ((uint32_t)p_fields->state & STATE_MASK) << STATE_SHIFT;
    word |= ((uint32_t)p_fields->motion & MOTION_MASK) << MOTION_SHIFT;

    word |= pitch_code (p_fields->pitch_deg, &clipped) << PITCH_SHIFT;
    if (clipped)
    {
        faults |= B4_FAULT_TILT;
    }

    word |= height_code (p_fields->hump_peak_m, &hump_over) << HPEAK_SHIFT;
    if (hump_over)
    {
        /* Never report a saturated height as a valid measurement. */
        valid = false;
    }

    /* run_peak saturates without a validity bit of its own: code 127 means
     * "at least 127 mm" and receivers must treat it as out of range. */
    word |= height_code (p_fields->run_peak_m, &run_over) << RPEAK_SHIFT;

    word |= (faults & FAULT_MASK) << FAULT_SHIFT;

    if (p_fields->hump_active)
    {
        word |= ACTIVE_BIT;
    }
    if (p_fields->finished)
    {
        word |= FINISHED_BIT;
    }
    if (valid)
    {
        word |= VALID_BIT;
    }

    *p_word = word;
    return true;
}

bool b4_hump_unpack (b4_packed_hump_t word, b4_hump_fields_t * p_fields)
{
    uint32_t pitch;

    if ((NULL == p_fields)
        || (((word & ACTIVE_BIT) != 0U) && ((word & FINISHED_BIT) != 0U)))
    {
        return false;
    }

    p_fields->state = (hump_state_t)((word >> STATE_SHIFT) & STATE_MASK);
    p_fields->motion = (b4_motion_t)((word >> MOTION_SHIFT) & MOTION_MASK);
    p_fields->faults = (uint8_t)((word >> FAULT_SHIFT) & FAULT_MASK);
    pitch = (word >> PITCH_SHIFT) & PITCH_MASK;
    p_fields->pitch_deg = ((float)pitch - PITCH_OFFSET) / HALF_DEG_PER_DEG;
    p_fields->hump_peak_m = (float)((word >> HPEAK_SHIFT) & HPEAK_MASK)
                            / 1000.0f;
    p_fields->run_peak_m = (float)((word >> RPEAK_SHIFT) & RPEAK_MASK)
                           / 1000.0f;
    p_fields->hump_active = ((word & ACTIVE_BIT) != 0U);
    p_fields->finished = ((word & FINISHED_BIT) != 0U);
    p_fields->height_valid = ((word & VALID_BIT) != 0U);

    return true;
}

void b4_hump_to_bytes (b4_packed_hump_t word, uint8_t p_bytes[4])
{
    if (NULL != p_bytes)
    {
        p_bytes[0] = (uint8_t)(word & 0xFFU);
        p_bytes[1] = (uint8_t)((word >> 8U) & 0xFFU);
        p_bytes[2] = (uint8_t)((word >> 16U) & 0xFFU);
        p_bytes[3] = (uint8_t)((word >> 24U) & 0xFFU);
    }
}

b4_packed_hump_t b4_hump_from_bytes (const uint8_t p_bytes[4])
{
    b4_packed_hump_t word = 0U;

    if (NULL != p_bytes)
    {
        word = (uint32_t)p_bytes[0]
               | ((uint32_t)p_bytes[1] << 8U)
               | ((uint32_t)p_bytes[2] << 16U)
               | ((uint32_t)p_bytes[3] << 24U);
    }

    return word;
}

void b4_run_init (b4_run_t * p_run)
{
    if (NULL != p_run)
    {
        p_run->run_peak_m = 0.0f;
        p_run->valid_humps = 0U;
        p_run->rejected_humps = 0U;
    }
}

void b4_run_update (b4_run_t * p_run, const hump_output_t * p_out)
{
    if ((NULL == p_run) || (NULL == p_out)
        || (HUMP_EVENT_FINISHED != p_out->event))
    {
        return;
    }

    if (p_out->height_valid && p_out->complete && isfinite (p_out->peak_m))
    {
        if (p_run->valid_humps < UINT16_MAX)
        {
            p_run->valid_humps++;
        }
        if (p_out->peak_m > p_run->run_peak_m)
        {
            p_run->run_peak_m = p_out->peak_m;
        }
    }
    else if (p_run->rejected_humps < UINT16_MAX)
    {
        p_run->rejected_humps++;
    }
    else
    {
        /* Counter saturated. */
    }
}

bool b4_hump_fields_from_output (const hump_output_t * p_out,
                                 const b4_run_t * p_run,
                                 b4_motion_t motion,
                                 bool i2c_fault,
                                 b4_hump_fields_t * p_fields)
{
    bool finished;
    uint8_t faults = 0U;

    if ((NULL == p_out) || (NULL == p_run) || (NULL == p_fields))
    {
        return false;
    }

    finished = (HUMP_EVENT_FINISHED == p_out->event);

    if (i2c_fault)
    {
        faults |= (uint8_t)B4_FAULT_I2C;
    }
    if (p_out->motion_stale)
    {
        faults |= (uint8_t)B4_FAULT_STALE;
    }

    /* hump_detect returns to FLAT on the FINISHED call, so state and the
     * active flag both describe the car after this sample. */
    p_fields->state = p_out->state;
    p_fields->motion = p_out->calibrated ? motion : B4_MOTION_UNKNOWN;
    p_fields->faults = faults;
    p_fields->pitch_deg = p_out->pitch_deg;
    p_fields->hump_peak_m = (p_out->peak_m > 0.0f) ? p_out->peak_m : 0.0f;
    p_fields->run_peak_m = p_run->run_peak_m;
    p_fields->hump_active = (HUMP_STATE_FLAT != p_out->state);
    p_fields->finished = finished;
    p_fields->height_valid = p_out->height_valid
                             && (!finished || p_out->complete);

    return true;
}

/*** end of file ***/
