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

/* Effective track for spins on the spot. A 90-degree spin by the wheel
 * track turned about 120 degrees on the car, so 75 % of it. Rolling turns
 * keep the wheel track. Observed once, not measured. */
#define CHASSIS_PIVOT_TRACK_UM   ((CHASSIS_WHEEL_TRACK_MM * 1000 * 75) / 100)

/* [Measured] Outside tire face to outside tire face. */
#define CHASSIS_OUTER_WIDTH_MM   (126U)
/* The outer left barcode corner is 45 mm ahead of the drive axle and
 * 44 mm beyond the outside tire face (107 mm from the centreline). */
#define CHASSIS_BARCODE_FORWARD_MM (45)
#define CHASSIS_RIGHT_REACH_MM ((CHASSIS_OUTER_WIDTH_MM + 1U) / 2U)
#define CHASSIS_LEFT_REACH_MM  (CHASSIS_RIGHT_REACH_MM + 44U)

/* Owner's front outline measured along the chassis from the drive axle.
 * Widths are full widths. The optical and acoustic origins have their own offsets. */
#define CHASSIS_NOSE_FORWARD_MM  (110)
#define CHASSIS_NOSE_WIDTH_MM    (90)
#define CHASSIS_NOSE_LEFT_MM     (65)
/* [Measured] The front-most point is 133 mm ahead of the axle. */
#define CHASSIS_IR_EXTENSION_MM  (23)
#define CHASSIS_IR_WIDTH_MM      (45)
#define CHASSIS_FRONT_TIP_MM     (CHASSIS_NOSE_FORWARD_MM + CHASSIS_IR_EXTENSION_MM)

/* Axle-centred envelope of the nominal 200 × 126 mm body (sensors excluded). */
#define CHASSIS_BODY_RADIUS_MM   (119)
/* Nominal: the body envelope above puts the tail 100 mm behind the axle. */
#define CHASSIS_TAIL_MM          (100)

/* [Measured] Line sensors: 25 mm between the sensor midpoints and 123.5 mm
 * ahead of the drive axle, rounded up. */
#define CHASSIS_LINE_SENSOR_SPACING_MM (25)
#define CHASSIS_LINE_SENSOR_LEAD_MM    (124)

/* [Measured] Sonar servo shaft 68.5 mm ahead of the axle, rounded up. */
#define CHASSIS_SONAR_FORWARD_MM (69)

/* At 300 mm range the near edge dropped out at 50 mm left and 65 mm
 * right. The narrower observed half-angle is 9.5 degrees. */
#define CHASSIS_SONAR_BEAM_HALF_DECI (95)
/* Round the wider observed side (65 mm at 300 mm, 12.2 degrees) outward
 * when deciding whether a return could lie inside a turn. */
#define CHASSIS_SONAR_OUTER_HALF_DECI (130)

/* [Measured] Wheel diameter 65 mm: pi x 65 mm of travel per wheel turn. */
#define CHASSIS_WHEEL_TRAVEL_UM  (204204U)

/* [Measured] Wheel axle to caster contact, along the car. */
#define CHASSIS_AXLE_TO_CASTER_MM (120)

/* [Measured] IMU position from the axle. The pipeline ignores this small
 * offset. */
#define CHASSIS_IMU_ABOVE_AXLE_MM  (80)
#define CHASSIS_IMU_BEHIND_AXLE_MM (15)

/* Encoder edges per wheel turn, both channels, both edges. */
#define CHASSIS_EDGES_PER_REV    (2530U)

#endif /* PLATFORM_CHASSIS_H */
