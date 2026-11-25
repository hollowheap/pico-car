/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	chassis.h
 *	Measured geometry of the car, one copy for every subsystem.
 */

#ifndef PLATFORM_CHASSIS_H
#define PLATFORM_CHASSIS_H

/* [Measured] Wheel centre to wheel centre. */
#define CHASSIS_WHEEL_TRACK_MM   (107)

/* [Measured] Wheel diameter 65 mm: pi x 65 mm of travel per wheel turn. */
#define CHASSIS_WHEEL_TRAVEL_UM  (204204U)

/* [Measured] Wheel axle to caster contact, along the car. The IMU sits 80 mm
 * above and 15 mm behind the axle; that offset is small enough to ignore. */
#define CHASSIS_AXLE_TO_CASTER_MM (120)

/* Encoder edges per wheel turn, both channels, both edges. */
#define CHASSIS_EDGES_PER_REV    (2530U)

#endif /* PLATFORM_CHASSIS_H */
