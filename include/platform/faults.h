/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	faults.h
 *	Shared system fault register.
 *
 *	Core 0 and Core 1 (and tasks and ISRs on either core) may assert or clear
 *	bits concurrently.  Every operation takes the kernel's SMP atomic path,
 *	which runs on SIO Spinlock 1, so no bit update is lost and no reader sees
 *	a torn value.
 *
 *	There are two words.  The level word shows what is wrong now: the owner
 *	asserts a bit and clears it when the condition ends.  The sticky word
 *	records every bit asserted since the last system_fault_sticky_clear(), so
 *	a fault that came and went is still visible.  Bit positions match the
 *	fault_bits list in protocol/telemetry.yaml.
 *
 *	The words are 32 bits wide; 16 bits are defined, and the telemetry wire
 *	carries 16 bits until a seventeenth bit is defined.  Each bit has one
 *	owner, and only that owner can change it.
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

/* The task that reports each bit.  The masks are in faults.c. */
typedef enum
{
    FAULT_OWNER_LINK = 0,   /* Wi-Fi down, MQTT publish refused        */
    FAULT_OWNER_MOTION,     /* stall, dead encoder, emergency latch    */
    FAULT_OWNER_LINE,       /* line lost                               */
    FAULT_OWNER_PHYSICS,    /* IMU, stale speed, tilt, lift, overturn  */
    FAULT_OWNER_OBSTACLE,   /* no echo, echo line, obstacle stop       */
    FAULT_OWNER_SUPERVISOR, /* task stalled, abnormal reset            */
    FAULT_OWNER_COUNT
} fault_owner_t;

/* Return the bits owner may change; 0 for an unknown owner. */
uint32_t system_fault_owner_mask(fault_owner_t owner);

/* Report the bits an owner controls: set the bits of mask that are also in
   active, and clear the other bits of mask.  A set bit also goes into the
   sticky word.  Bits outside mask, and bits that owner does not own, are not
   touched. */
void system_fault_assign(fault_owner_t owner, uint32_t mask, uint32_t active);

/* Return the current fault mask (the level word). */
uint32_t system_fault_get(void);

/* Return every bit asserted since the last system_fault_sticky_clear(). */
uint32_t system_fault_sticky_get(void);

/* Clear the bits in fault_bit in the sticky word.  A bit still set in the
   level word stays: it is asserted now. */
void system_fault_sticky_clear(uint32_t fault_bit);

#endif /* PLATFORM_FAULTS_H */
