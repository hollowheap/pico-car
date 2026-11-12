/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	faults.h
 *	Shared 16-bit system fault register (D110, S46).
 *
 *	Core 0 and Core 1 (and tasks and ISRs on either core) may assert or clear
 *	bits concurrently.  Every operation takes the kernel's SMP atomic path,
 *	which runs on SIO Spinlock 1, so no bit update is lost and no reader sees
 *	a torn value.
 *
 *	There are two words.  The level word shows what is wrong now: a setter
 *	asserts a bit and clears it when the condition ends.  The sticky word
 *	records every bit asserted since the last system_fault_sticky_clear(), so
 *	a fault that came and went is still visible.  Bit positions match the
 *	fault_bits list in protocol/telemetry.yaml.
 */

#ifndef PLATFORM_FAULTS_H
#define PLATFORM_FAULTS_H

#include <stdint.h>

/* Fault bits.  Each argument to the accessors below is one of these masks. */
#define FAULT_WIFI_DISCONN_BIT    (1U << 0U)
#define FAULT_MQTT_NACK_BIT       (1U << 1U)
#define FAULT_MOTOR_STALL_BIT     (1U << 2U)
#define FAULT_ENCODER_DEAD_BIT    (1U << 3U)
#define FAULT_LINE_LOST_BIT       (1U << 4U)
#define FAULT_IMU_NACK_BIT        (1U << 5U)
#define FAULT_STALE_SPEED_BIT     (1U << 6U)
#define FAULT_INVALID_TILT_BIT    (1U << 7U)
#define FAULT_SONAR_NO_ECHO_BIT   (1U << 8U)
#define FAULT_ESTOP_LATCHED_BIT   (1U << 9U)
#define FAULT_ECHO_LINE_BIT       (1U << 10U)
#define FAULT_LIFTED_BIT          (1U << 11U)
#define FAULT_OVERTURNED_BIT      (1U << 12U)
#define FAULT_OBSTACLE_STOPPED_BIT (1U << 13U)
#define FAULT_TASK_STALLED_BIT    (1U << 14U)
#define FAULT_ABNORMAL_RESET_BIT  (1U << 15U)

/* Every defined fault bit.  Bits outside this mask are ignored by the accessors. */
#define FAULT_ALL_MASK            (0xFFFFU)

/* Set the bits in fault_bit in the level word and the sticky word.  Bits
   already set stay set. */
void system_fault_set(uint16_t fault_bit);

/* Clear the bits in fault_bit in the level word.  Bits already clear stay
   clear.  The sticky word keeps them. */
void system_fault_clear(uint16_t fault_bit);

/* Report the bits an owner controls: set the bits of mask that are also in
   active, and clear the other bits of mask.  Bits outside mask are not touched,
   so each owner passes only its own bits. */
void system_fault_assign(uint16_t mask, uint16_t active);

/* Return the current 16-bit fault mask (the level word). */
uint16_t system_fault_get(void);

/* Return every bit asserted since the last system_fault_sticky_clear(). */
uint16_t system_fault_sticky_get(void);

/* Clear the bits in fault_bit in the sticky word. */
void system_fault_sticky_clear(uint16_t fault_bit);

#endif /* PLATFORM_FAULTS_H */
