/** @file guidance.h
 * @brief Decoded sensor intents; Buddy3 owns the IR polarity/truth table.
 */
#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "controller.h"
#include "motion/types.h"

/* Produces signed wheel targets with the ORIGINAL sensor timestamp/lease.
 * Straight always gives identical targets. Left slows left, right slows right.
 * No wheel geometry/distance input. Failure leaves output unchanged.
 * Limits are per-wheel motor-service limits, not a guessed operating speed. */
bool guidance_plan (const guidance_request_t * p_request,
                    const uint32_t limits_um_s[ENCODER_WHEELS],
                    uint32_t max_ttl_us, uint32_t now_us,
                    motion_command_t * p_command);

#endif /* GUIDANCE_H */
