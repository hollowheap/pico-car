/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	faults.h
 *	Shared 16-bit system fault register.
 *
 *	Core 0 and Core 1 (and tasks and ISRs on either core) may assert or clear
 *	bits concurrently.  Every operation takes the kernel's SMP atomic path,
 *	which runs on SIO Spinlock 1, so no bit update is lost and no reader sees
 *	a torn value.
 */

#ifndef PLATFORM_FAULTS_H
#define PLATFORM_FAULTS_H

#include <stdint.h>

/* Fault bits.  Each argument to the accessors below is one of these masks. */
#define FAULT_WIFI_DISCONN_BIT    (1U << 0)
#define FAULT_MQTT_NACK_BIT       (1U << 1)
#define FAULT_MOTOR_STALL_BIT     (1U << 2)
#define FAULT_ENCODER_DEAD_BIT    (1U << 3)
#define FAULT_LINE_LOST_BIT       (1U << 4)
#define FAULT_IMU_NACK_BIT        (1U << 5)
#define FAULT_STALE_SPEED_BIT     (1U << 6)
#define FAULT_INVALID_TILT_BIT    (1U << 7)
#define FAULT_SONAR_NO_ECHO_BIT   (1U << 8)
#define FAULT_ESTOP_LATCHED_BIT  (1U << 9)

/* Every defined fault bit.  Bits outside this mask are ignored by the accessors. */
#define FAULT_ALL_MASK            (0x03FFU)

/* Set the bits in fault_bit.  Bits already set stay set. */
void system_fault_set(uint16_t fault_bit);

/* Clear the bits in fault_bit.  Bits already clear stay clear. */
void system_fault_clear(uint16_t fault_bit);

/* Return the current 16-bit fault mask. */
uint16_t system_fault_get(void);

#endif /* PLATFORM_FAULTS_H */
