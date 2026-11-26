/** @file config.h
 *
 * @brief Buddy 5 settings: pins, timings, tunables, calibration placeholders
 *        and compile-time checks that every packed field fits its range.
 *
 * Every number states where it came from:
 *   [Spec]     the Buddy 5 brief.
 *   [DR x]     Week 6 design review, section x.
 *   [Tribunal] Week 8 SE01 tribunal decisions.
 *   [Chat]     agreed while designing Buddy 5.
 *   [Default]  a starting value that is to be tuned on the car.
 *   FILL       a value that still has to be measured and replaced.
 *
 * Units are mm, ms, us and deci-degrees (0.1 degree, positive = left).
 * This header has no kernel includes.
 */

#ifndef EXTRAS_CFG_H
#define EXTRAS_CFG_H

#include <stdint.h>

#include "platform/chassis.h"

/* C99 has no _Static_assert, so a negative array size stops the build when
 * a condition is false. NOTE: This macro cannot be a function because it
 * must act at compile time (BARR-C 6.3.a). */
#define EXTRAS_CT_ASSERT(name, cond)                                               \
    typedef uint8_t extras_ct_##name##_t[(cond) ? 1 : -1]

/* ------------------------------------------------------------------ *
 * Pins on the Cytron Robo Pico [DR 2].
 * ------------------------------------------------------------------ */

/* Grove 1, GPIO output to the sensor trigger. */
#define EXTRAS_TRIG_PIN (0U)

/* Grove 1, GPIO input from the sensor echo, both-edge interrupt. */
#define EXTRAS_ECHO_PIN (1U)

/* Servo port 1, PWM slice 6 channel A. */
#define EXTRAS_SERVO_PIN (12U)

/* ------------------------------------------------------------------ *
 * Ping timing [Spec] [HC-SR04 datasheet].
 * ------------------------------------------------------------------ */

/* At least 60 ms between triggers, so old echoes die out. */
#define EXTRAS_PING_PERIOD_MS (60U)

/* The task waits at most this long for the end of an echo. */
#define EXTRAS_ECHO_WAIT_MS (40U)

/* The trigger pulse must be at least 10 us long. */
#define EXTRAS_TRIG_PULSE_US (12U)

/* Echo widths from 116 us (about 20 mm) to 23500 us (about 4 m) are
 * VALID readings. */
#define EXTRAS_VALID_MIN_US (116U)
#define EXTRAS_VALID_MAX_US (23500U)

/* Distance in mm is width_us * 343 / 2000 (speed of sound 343 m/s). */
#define EXTRAS_SOUND_NUM (343U)
#define EXTRAS_SOUND_DEN (2000U)

/* The 1 MHz TIMER must advance at least 25 ticks across a 50 us wait. */
#define EXTRAS_TIMER_CHECK_US        (50U)
#define EXTRAS_TIMER_CHECK_MIN_TICKS (25U)

/* ------------------------------------------------------------------ *
 * Faults [Spec].
 * ------------------------------------------------------------------ */

/* This many UNKNOWN pings in a row raise NO_ECHO. */
#define EXTRAS_NO_ECHO_COUNT (5U)

/* This many STUCK pings in a row raise ECHO_STUCK. */
#define EXTRAS_STUCK_COUNT (3U)

/* The whole scan must finish within 8 s. */
#define EXTRAS_SCAN_TIMEOUT_MS (8000U)

/* ------------------------------------------------------------------ *
 * Monitor ahead and the collision guard.
 * ------------------------------------------------------------------ */

/* [Chat] Detection distance, lowered from 250 mm (revised bypass, "decrease
 * detection distance"). The car ends about 15 mm closer after two pings and
 * braking, so B is about 205 mm. NOTE: After the first leg (B / 2) the
 * axle would be B / 2 + 69 mm from the obstacle face, but the first leg is
 * cut short to keep EXTRAS_SWING_MM + EXTRAS_SPIN_CLEAR_MM, and the car backs up
 * when it stopped closer than that (extras_plan_ahead_mm()). The Week 6 tribunal
 * sheet quotes a 200 mm standoff (team document, not a course rule). */
#define EXTRAS_DETECT_MM (220U)

/* [Spec] Consecutive VALID hits needed to report an obstacle. */
#define EXTRAS_DETECT_HITS (2U)

/* ------------------------------------------------------------------ *
 * Scan grid [Spec].
 * ------------------------------------------------------------------ */

/* 25 slots from -48 to +48 degrees in 4 degree steps; slot 12 is ahead. */
#define EXTRAS_SLOT_COUNT      (25U)
#define EXTRAS_SLOT_CENTRE     (12U)
#define EXTRAS_SLOT_STEP_DECI  (40)
#define EXTRAS_SLOT_FIRST_DECI (-480)

/* The coarse scan measures every third slot. */
#define EXTRAS_COARSE_STRIDE (3U)

/* [Spec 3c] The region is the closest VALID return within 600 mm plus
 * neighbouring coarse slots within closest + 250 mm. */
#define EXTRAS_REGION_MAX_MM   (600U)
#define EXTRAS_REGION_EXTRA_MM (250U)

/* [Chat] Flat face square to the car. A flat face straight ahead echoes
 * straight back so strongly that the sensor still hears it when the servo
 * points far off to the side, so every slot reads about the same distance
 * and the obstacle looks wider than the whole sweep (seen on the car as
 * NO_SAFE_PATH for a box and a wall faced square-on).
 * [Chat] Trigonometry check (the user's idea): the closest coarse reading
 * B is the adjacent side, taken where the servo points most directly at
 * the face (t0: the path slot nearest straight ahead reading within
 * EXTRAS_AGREE_MM of B, since on a square face every slot reads about B and
 * ping noise must not move t0). At servo angle t a real surface must read
 * at least the hypotenuse B / cos(d), d = the angle from the nearer of t0
 * and straight ahead. A reading r is "too short to be
 * real" when
 *   (r + EXTRAS_SONAR_FACE_MM) x cos(d - EXTRAS_BEAM_HALF_DECI)
 *       < B + EXTRAS_SONAR_FACE_MM
 * (EXTRAS_SONAR_FACE_MM: the sensor face sits in front of the servo shaft;
 * EXTRAS_BEAM_HALF_DECI: the sensor hears anything within its beam, so a real
 * corner near the beam's edge may read a little short). Readings with d
 * up to EXTRAS_ECHO_BEAM_DECI always count.
 *   - Flat: every coarse slot is VALID and both end slots (-60 and +60
 *     degrees) are too short to be real.
 *   - Only in a flat scan, every slot too short to be real is a side echo
 *     and does not count as the obstacle (extras_scan_is_side_echo()).
 * NOTE: A wall faced square-on then looks like a box too; the bypass stops
 * at the collision guard. A box wider than about 300 mm faced square-on
 * looks narrower than it is (its real side echoes arrive after the one
 * from straight ahead). */
#define EXTRAS_ECHO_BEAM_DECI (150)

/* [Chat] After this many NO_SAFE_PATH results in a row, Navigation stops
 * retrying and halts until the car is reset (extras_nav.c). */
#define EXTRAS_NO_PATH_TRIES (2U)

/* [Spec 3e] Two pings agree when within 15 mm; four pings without
 * agreement make the slot UNKNOWN. */
#define EXTRAS_AGREE_MM  (15U)
#define EXTRAS_MAX_PINGS (4U)

/* [Chat][Default] The car's path: +-24 degrees of straight ahead. CLEAR
 * needs a real reading in every slot of it, and the scan looks for the
 * closest reading here before it looks at the whole sweep. */
#define EXTRAS_PATH_HALF_DECI (240)

/* ------------------------------------------------------------------ *
 * Servo (SG90 on servo port 1). The datasheet gives both "1-2 ms for
 * +-90 degrees" and "500-2400 us for 180 degrees". The narrow scale
 * below cannot reach the end stops. Calibrate with checklist step H2.
 * ------------------------------------------------------------------ */

/* FILL: pulse width that points the sensor straight ahead. */
#define EXTRAS_SERVO_CENTRE_US (1450U)

/* FILL: microseconds per 10 degrees (a safe 5.6 us per degree guess). */
#define EXTRAS_SERVO_US_PER_10DEG (56U)

/* FILL: +1 if a longer pulse turns the sensor left, -1 otherwise. */
#define EXTRAS_SERVO_DIR (1)

/* [Spec] The scan covers +-48 degrees. */
#define EXTRAS_SERVO_LIMIT_DECI (480)

/* Never command a pulse outside the datasheet range. */
#define EXTRAS_SERVO_MIN_US (500U)
#define EXTRAS_SERVO_MAX_US (2400U)

/* [Spec] Settle 3 ms per degree plus 20 ms. The datasheet speed is
 * 1.7-2.0 ms per degree at 4.8 V; the margin covers LiPo sag, and the
 * 20 ms covers one PWM frame. FILL: power source (LiPo, powerbank or
 * VIN). Below 4.0 V the SG90 is out of spec, so re-check on a low
 * battery. */
#define EXTRAS_SETTLE_MS_PER_DEG (3U)
#define EXTRAS_SETTLE_BASE_MS    (20U)

/* [Chat] In states where the car may be driving, the servo waits this long
 * before it moves, so that Navigation (20 ms loop) and the motion service
 * (10 ms) have stopped the wheels: a servo swing while a motor drives
 * disturbs the wheel encoders (seen on the car as a FEEDBACK fault). */
#define EXTRAS_SERVO_PREDELAY_MS (60U)

/* PWM: 125 MHz / 125 = 1 MHz count and TOP 19999 give 50 Hz. CHx_DIV is
 * an 8.4 fixed-point value. */
#define EXTRAS_PWM_DIV_INT   (125U)
#define EXTRAS_PWM_DIV_SHIFT (4U)
#define EXTRAS_PWM_TOP       (19999U)
#define EXTRAS_PWM_SLICE     (6U)

/* ------------------------------------------------------------------ *
 * Handshake and bypass limits [Default].
 * ------------------------------------------------------------------ */

/* After OBSTACLE, the SCAN_REQ must arrive within 1 s. */
#define EXTRAS_WAIT_STOP_MS (1000U)

/* After SCAN_RESULT, ACCEPT or REJECT must arrive within 1 s. */
#define EXTRAS_PLAN_ACCEPT_MS (1000U)

/* The bend-away phase may last 10 s. */
#define EXTRAS_ESCAPE_MAX_MS (10000U)

/* The rest of the bypass (REACQUIRE: out, side scans, back to the line,
 * the 360 degree map) may last 180 s or 6000 mm (net forward travel of the
 * wheels) before the line must be found. [Chat] Each side scan loop
 * (rotation, sweep, rotation back, move on) takes about 8 s; the number of
 * loops itself is not limited. */
#define EXTRAS_BYPASS_MAX_MS (180000U)
#define EXTRAS_BYPASS_MAX_MM (6000U)

/* [Chat] The escape ends after this many clear forward pings in a row
 * (two, so that one noisy reading cannot end the turn early). Navigation
 * keeps rotating until the car is 90 degrees off the line anyway
 * (extras_nav.c). */
#define EXTRAS_ESCAPE_CLEAR_PINGS (2U)

/* ------------------------------------------------------------------ *
 * Car geometry. FILL: measure with a ruler and replace the values.
 * ------------------------------------------------------------------ */

/* [Measured] Widest point of the car is 126 mm (outside of wheel to
 * outside of wheel), so half is 63 mm. */
#define EXTRAS_CAR_HALF_W_MM (63U)

/* [Measured] An IR sensor sticks out EXTRAS_IR_PROTRUDE_MM on the car's left,
 * so the car reaches EXTRAS_HALF_W_LEFT_MM to the left of its centre line and
 * EXTRAS_HALF_W_RIGHT_MM to the right. The bypass clearances use these. */
#define EXTRAS_IR_PROTRUDE_MM  (45U)
#define EXTRAS_HALF_W_LEFT_MM  (EXTRAS_CAR_HALF_W_MM + EXTRAS_IR_PROTRUDE_MM)
#define EXTRAS_HALF_W_RIGHT_MM (EXTRAS_CAR_HALF_W_MM)

/* [Measured] Wheel diameter 65 mm. NOTE: The motion service already
 * uses 204 204 um per wheel turn (pi x 65 mm, main.c), so the odometer
 * needs no change. */

#define EXTRAS_WHEEL_TRACK_MM ((uint32_t)CHASSIS_WHEEL_TRACK_MM)

#define EXTRAS_NAV_TRACK_UM ((uint32_t)CHASSIS_PIVOT_TRACK_UM)

/* Servo shaft to the tail of the car (used by the planner's corridor
 * check). */
#define EXTRAS_SERVO_TO_REAR_MM ((uint32_t)(CHASSIS_TAIL_MM + CHASSIS_SONAR_FORWARD_MM))

/* Distances forward from the wheel axle along the car: to the servo shaft,
 * to the IR line sensors and to the front-most point of the car. */
#define EXTRAS_AXLE_TO_SONAR_MM ((uint32_t)CHASSIS_SONAR_FORWARD_MM)
#define EXTRAS_AXLE_TO_IR_MM    ((uint32_t)CHASSIS_LINE_SENSOR_LEAD_MM)
#define EXTRAS_AXLE_TO_FRONT_MM ((uint32_t)CHASSIS_FRONT_TIP_MM)

/* How far the car reaches from the middle of its axle when it rotates on
 * the spot: the left protrusion taken at the front of the car,
 * sqrt(133^2 + 108^2) = 171.3 mm, rounded up (checked below). FILL: if the
 * protrusion sits only at the IR sensors (124 mm ahead), this is 165. */
#define EXTRAS_SWING_MM (172U)

/* [Chat] Largest sideways offset a bypass may use. The course has no side
 * boundary, so this is not a wall: it only rejects absurd plans (for
 * example from a bad echo). The planner always picks the side that needs
 * the smaller offset, so real bypasses stay as close to the line as the
 * obstacle allows. */
#define EXTRAS_FREE_FLOOR_MM (500U)

/* [Chat] Bypass speed. [Measured] At the 25 % duty cap a wheel only
 * reaches about 142 mm/s, so 140 mm/s is the fastest speed the speed
 * control can really hold (the pivots and the heading hold rely on it).
 * Also used for the odometer estimate when no distance
 * reading is available. NOTE: motion_service_set_speed() rejects
 * (E_PAR) any target above max_target_um_s in main.c, so the car would not
 * move or its lease would expire; that limit must be at least
 * EXTRAS_NAV_SPEED_UM_S. */
#define EXTRAS_BYPASS_SPEED_MM_S (140U)

/* [Default] Extra sideways clearance. */
#define EXTRAS_SIDE_MARGIN_MM (50U)

/* [Chat] Assumed obstacle depth along the line: the deepest obstacle the
 * car must clear when the sonar cannot see its side (slanted, round or
 * soft faces). Raised from the design review's 180 mm to 220 mm so that a
 * 150 mm box placed corner-first (150 x sqrt 2 = 212 mm) is covered. Only
 * the planner's corridor check uses it (the bypass measures the depth with
 * its side scans). */
#define EXTRAS_DEPTH_ASSUMED_MM (220U)

/* [Measured] Half of the sensor cone. A power bank 300 mm ahead dropped
 * out with its near edge 50 mm left and 65 mm right of the centre line:
 * atan(50 / 300) = 9.5 and atan(65 / 300) = 12.2 degrees. The smaller
 * value is used, because a narrower cone gives the car more room. */
#define EXTRAS_BEAM_HALF_DECI (95)

/* ------------------------------------------------------------------ *
 * Phase 2 navigation glue (extras_nav.c) [Default]: how the line task drives
 * the motion service while Buddy 5 is in charge.
 * ------------------------------------------------------------------ */

/* Forward speed during the bypass, in micrometres per second. */
#define EXTRAS_NAV_SPEED_UM_S (EXTRAS_BYPASS_SPEED_MM_S * 1000U)

/* [Measured] The encoder plugs on the car are crossed relative to
 * src/motion/motion_pins.h (team decision, kept): the motion service's
 * wheel 0 reads the right encoder and wheel 1 the left one. Flight log:
 * both wheel totals fell while the car drove forwards, and the wheels ran
 * at about 190 mm/s when asked for 40. 1 = extras_nav.c undoes the swap for
 * its own odometry; 0 = the plugs match motion_pins.h (swapped back on the
 * car, way A). */
#define EXTRAS_ENCODERS_CROSSED (0)

/* [Chat] Every 90 degree change of heading is a rotation (see
 * EXTRAS_NAV_SPIN_UM_S): it slows for the last EXTRAS_NAV_TURN_SLOW_DECI. The
 * targets are absolute headings from the original line (90 away, 0, 90
 * towards = facing the line, 0), so errors do not add up from rotation to
 * rotation. 573 is deci-degrees per radian. */
/* [Default] At the 140 mm/s spin (about 200 degrees/s) the wheels need
 * roughly 10 degrees to slow down, so the slow zone is twice that.
 * Not measured on the car. */
#define EXTRAS_NAV_TURN_SLOW_DECI (200)
#define EXTRAS_NAV_DECI_PER_RAD   (573)

/* [Chat] Straight legs hold their heading: more than EXTRAS_NAV_HOLD_DEAD_DECI
 * off, the wheel on the side the car drifts towards runs
 * EXTRAS_NAV_TRIM_UM_S slower (a gentle correction, about 7 degrees/s). */
#define EXTRAS_NAV_HOLD_DEAD_DECI (20)
#define EXTRAS_NAV_TRIM_UM_S      (EXTRAS_NAV_SPEED_UM_S / 10U)

/* [Chat] Clearances the bypass keeps from the obstacle: EXTRAS_SPIN_CLEAR_MM
 * between the obstacle and the furthest reach of a rotation
 * (EXTRAS_SWING_MM), and EXTRAS_SENSOR_CLEAR_MM (the collision guard plus a
 * margin) between the obstacle and the sonar while the car rotates beside
 * it. */
#define EXTRAS_SPIN_CLEAR_MM   (20U)
#define EXTRAS_SENSOR_CLEAR_MM (100U)

/* [Chat] Revised bypass, first leg: the car drives on B / 2 towards the
 * obstacle before its first rotation, but keeps the axle at least
 * EXTRAS_SWING_MM + EXTRAS_SPIN_CLEAR_MM from the obstacle face; if it already
 * stopped closer, it backs up at EXTRAS_NAV_BACKUP_UM_S instead
 * (extras_plan_ahead_mm()). */
#define EXTRAS_NAV_BACKUP_UM_S (EXTRAS_NAV_SPEED_UM_S / 2U)

/* [Chat] Revised bypass, move out. A = B x tan(cleared angle), from the
 * first scan (B = closest reading, cleared angle = the first 5 degree step
 * past the obstacle edge on the chosen side, raw, no beam correction),
 * capped at EXTRAS_ROOM_MAX_MM so that one wide echo from a flat face cannot
 * send the car far off. The car then moves out E = A + EXTRAS_OUT_SIDE_MM,
 * and after rotating back drives on B + EXTRAS_LEVEL_BUFFER_MM, which brings
 * it level with the obstacle's near face. EXTRAS_OUT_SIDE_MM is the larger
 * of:
 *   - EXTRAS_SWING_MM + EXTRAS_SPIN_CLEAR_MM (192): the rotation to face the
 *     obstacle beside it clears it, left protrusion included;
 *   - EXTRAS_AXLE_TO_SONAR_MM + EXTRAS_SENSOR_CLEAR_MM (169): the sonar stays out
 *     of the collision guard's range while the car rotates.
 * NOTE: Both hold only while A is not below the obstacle's true half
 * width (the raw cleared angle normally makes A larger). */
#define EXTRAS_ROOM_MAX_MM   (200U)
#define EXTRAS_OUT_SWING_MM  (EXTRAS_SWING_MM + EXTRAS_SPIN_CLEAR_MM)
#define EXTRAS_OUT_SENSOR_MM (EXTRAS_AXLE_TO_SONAR_MM + EXTRAS_SENSOR_CLEAR_MM)
#define EXTRAS_OUT_SIDE_MM                                                         \
    ((EXTRAS_OUT_SWING_MM > EXTRAS_OUT_SENSOR_MM) ? EXTRAS_OUT_SWING_MM : EXTRAS_OUT_SENSOR_MM)
#define EXTRAS_LEVEL_BUFFER_MM (10U)

/* [Chat] Every side scan sweeps -EXTRAS_SIDE_PAN_DECI..+EXTRAS_SIDE_PAN_DECI in 4
 * degree steps, the same -48..+48 degree range and EXTRAS_SLOT_* angles as the
 * first scan, EXTRAS_SWEEP_PINGS ping at each angle (the nearest VALID reading
 * counts; the beam is about 19 degrees wide, so every spot is seen at
 * about four neighbouring angles). Only pings the Buddy 5 task took at the
 * current angle count (ping tag, extras_comms_set_ping_tag()), so none is
 * thrown away while the servo moves on, and the task skips its
 * EXTRAS_SERVO_PREDELAY_MS for every move after the first of a sweep (the car
 * already stands still). The car's lane to the line is its half width plus
 * EXTRAS_LANE_BUFFER_MM on each side (C = 138 mm on its left, with the IR
 * protrusion, and 93 mm on its right) and runs EXTRAS_LANE_EXTRA_MM past the
 * line (room for the last turn). A reading inside the lane means the way
 * to the line is blocked. While the car stands still for the sweep, the
 * collision guard is off: a close reading is "in the way", not a
 * collision. */
#define EXTRAS_SIDE_PAN_DECI  (480)
#define EXTRAS_SWEEP_PINGS    (1U)
#define EXTRAS_LANE_BUFFER_MM (30U)
#define EXTRAS_LANE_LEFT_MM   (EXTRAS_HALF_W_LEFT_MM + EXTRAS_LANE_BUFFER_MM)
#define EXTRAS_LANE_RIGHT_MM  (EXTRAS_HALF_W_RIGHT_MM + EXTRAS_LANE_BUFFER_MM)
#define EXTRAS_LANE_EXTRA_MM  (100U)

/* [Chat] Blocked: the car moves on (D + H + EXTRAS_MOVE_BUFFER_MM) /
 * EXTRAS_MOVE_DIVISOR along the line, H = the car's half width on the side
 * that faces the obstacle while it drives on (EXTRAS_HALF_W_LEFT_MM or
 * EXTRAS_HALF_W_RIGHT_MM), D = d x tan(forward cleared angle),
 * d = the closest reading of the sweep. With no cleared angle on the
 * forward side (the obstacle still reaches EXTRAS_SIDE_PAN_DECI),
 * EXTRAS_SIDE_ASSUMED_DECI is used. The car keeps scanning until the way is
 * clear (no limit on the number of scans). */
#define EXTRAS_MOVE_BUFFER_MM    (20U)
#define EXTRAS_MOVE_DIVISOR      (2U)
#define EXTRAS_SIDE_ASSUMED_DECI (300)

/* [Chat] Rotations spin the car on the middle of its axle: both wheels at the
 * same speed in opposite directions, EXTRAS_NAV_SPIN_UM_S, slowing to
 * EXTRAS_NAV_SPIN_SLOW_UM_S for the last EXTRAS_NAV_TURN_SLOW_DECI, ending within
 * EXTRAS_NAV_SPIN_TOL_DECI. A spin that overshoots is corrected the other way after
 * the reverse pause.
 * [Default] Spins use the fastest held wheel speed. The slow phase keeps the
 * earlier third of it, so the stopping accuracy stays as before. Neither
 * speed nor the resulting overshoot was measured on the car. */
#define EXTRAS_NAV_SPIN_UM_S      (EXTRAS_NAV_SPEED_UM_S)
#define EXTRAS_NAV_SPIN_SLOW_UM_S (EXTRAS_NAV_SPEED_UM_S / 3U)
#define EXTRAS_NAV_SPIN_TOL_DECI  (15)

/* [Chat] Chicken-head rotation scan (TRACK_OUT, TRACK_BACK; extras_nav.c),
 * between a clear SWEEP and TO_LINE. The car rotates EXTRAS_TRACK_PAN_DECI
 * towards the obstacle's side at a constant EXTRAS_TRACK_SPIN_UM_S (about 75
 * degrees/s, so about 4.5 degrees between pings) and back,
 * while the servo turns the opposite way (re-aimed every
 * EXTRAS_TRACK_PERIOD_MS when it has changed by at least
 * EXTRAS_TRACK_MIN_STEP_DECI), so the beam keeps one direction on the floor
 * (straight at the line on the way out, tilted EXTRAS_TRACK_TILT_DECI towards
 * the obstacle's side on the way back). Every echo becomes a floor point
 * (extras_plan_track_point(); the sensor face is EXTRAS_SONAR_FACE_MM in front of
 * the servo shaft) and is checked against the lane and the swing circle
 * (EXTRAS_SWING_MM). More than
 * EXTRAS_TRACK_MAX_UNKNOWN UNKNOWN or STUCK pings repeat the scan, at most
 * EXTRAS_TRACK_RETRIES times, after which it counts as blocked. A blocked scan
 * moves the car on at least EXTRAS_TRACK_MIN_MOVE_MM.
 * FILL: EXTRAS_SONAR_FACE_MM and EXTRAS_TRACK_TILT_DECI are defaults until
 * measured (120 is fine for the tilt if echoes survive at least 15 degrees
 * off square). */
#define EXTRAS_SONAR_FACE_MM       (15U)
#define EXTRAS_TRACK_PERIOD_MS     (20U)
#define EXTRAS_TRACK_PAN_DECI      (480)
#define EXTRAS_TRACK_TILT_DECI     (100)
#define EXTRAS_TRACK_MIN_STEP_DECI (10)
#define EXTRAS_TRACK_SPIN_UM_S     (EXTRAS_NAV_SPEED_UM_S / 2U)
#define EXTRAS_TRACK_MAX_UNKNOWN   (3U)
#define EXTRAS_TRACK_RETRIES       (1U)
#define EXTRAS_TRACK_MIN_MOVE_MM   (30U)

/* [Chat] Virtual bumper: a wheel told to run at EXTRAS_BUMP_MIN_UM_S or more
 * that measures under EXTRAS_BUMP_PERCENT of it for EXTRAS_BUMP_MS has hit
 * something. Not checked for EXTRAS_BUMP_SETTLE_MS after any speed change or
 * hold, while the wheel speeds up. The car then stops and stays stopped
 * until it is reset (no automatic retry). */
#define EXTRAS_BUMP_MIN_UM_S  (70000)
#define EXTRAS_BUMP_PERCENT   (40)
#define EXTRAS_BUMP_MS        (150U)
#define EXTRAS_BUMP_SETTLE_MS (300U)

/* [Measured] Black tape width 17.5 mm, rounded up. [Chat] The drive to the
 * line runs at EXTRAS_NAV_SPEED_UM_S while the IR sensors are more than
 * EXTRAS_LINE_SLOW_MM short of the original line by odometry, then slowly
 * (EXTRAS_NAV_LINE_UM_S); EXTRAS_LINE_SLOW_MM leaves 40 mm to slow down before the
 * window in which a line can count. An IR sensor must see black for
 * EXTRAS_LINE_MIN_MM of travel (gaps between floor mats are thinner), and the
 * sensor must be no more than EXTRAS_LINE_WINDOW_MM short of the original line
 * by odometry, for the line to count. Once the sensor is EXTRAS_LINE_GIVE_UP_MM
 * past it without a line, the car stops. */
#define EXTRAS_TAPE_W_MM       (18U)
#define EXTRAS_LINE_MIN_MM     (8U)
#define EXTRAS_LINE_WINDOW_MM  (100U)
#define EXTRAS_LINE_GIVE_UP_MM (600U)
#define EXTRAS_NAV_LINE_UM_S   (EXTRAS_NAV_SPEED_UM_S / 2U)
#define EXTRAS_LINE_SLOW_MM    (EXTRAS_LINE_WINDOW_MM + 40U)

/* [Chat] Once the line counts, the car drives on EXTRAS_NAV_CENTRE_MM at
 * EXTRAS_NAV_SPEED_UM_S (the user's value; the IR sensors are EXTRAS_AXLE_TO_IR_MM
 * ahead of the axle), so
 * the middle of the axle sits on the line. It then rotates in the
 * remembered direction (the same way as its first rotation), servo
 * straight ahead, until an IR sensor finds the line, at most
 * EXTRAS_MAP_TURN_DECI, and line following resumes from there. */
#define EXTRAS_NAV_CENTRE_MM (127U)
#define EXTRAS_MAP_TURN_DECI (3600)

/* [Chat] The servo has settled once a ping tagged with the new angle has
 * arrived (extras_comms_set_ping_tag()), or, where the statechart picks the
 * angle itself, once this many pings have arrived since it was told to
 * move (the first may have been taken before it moved). */
#define EXTRAS_NAV_SERVO_PINGS (2U)

/* [Chat] Before any wheel changes direction (end of the escape spin), both
 * wheels stop for this long. While both are at 0 the motion service is
 * idle and does not check the encoders, which rattle while a geared motor
 * brakes (seen on the car as invalid steps and a FEEDBACK fault).
 * [Chat] Shortened from 300 ms to speed up the bypass. NOTE: Watch the
 * log for MOTION fault=FEEDBACK after a rotation; if it returns, go back
 * to 300. */
#define EXTRAS_NAV_REVERSE_PAUSE_MS (150U)

/* [Chat] Longest wait for the servo to settle after it is moved while the
 * car could drive (entering REACQUIRE or MONITOR). The car stands still
 * meanwhile; normally the wait ends at the next ping (about 0.2 s). */
#define EXTRAS_NAV_SERVO_WAIT_MS (600U)

/* Lease of every speed command. It must cover several 20 ms loop passes
 * and stay within the motion service limit (200 ms in main.c). */
#define EXTRAS_NAV_TTL_US (100000U)

/* After OBSTACLE the car is held this long before SCAN_REQ is sent, so
 * that it has stopped before the servo starts to sweep. */
#define EXTRAS_NAV_BRAKE_MS (300U)

/* After a fault, RESET and START are retried this often. */
#define EXTRAS_NAV_RETRY_MS (3000U)

/* While looking for the line, LINE_FOUND is sent at most this often. */
#define EXTRAS_NAV_LINE_REPEAT_MS (100U)

/* ------------------------------------------------------------------ *
 * Flight recorder (extras_log.c) [Default].
 * ------------------------------------------------------------------ */

/* Records kept in RAM (16 bytes each; must be a power of two). */
#define EXTRAS_LOG_RECORDS (1024U)

/* Scan profiles kept in RAM. */
#define EXTRAS_LOG_PROFILES (8U)

/* A profile belongs to the RESULT record logged within this time. */
#define EXTRAS_LOG_PROFILE_MATCH_MS (50U)

/* Trace period while Buddy 5 drives the car (BYPASS, REACQUIRE), and
 * otherwise. */
#define EXTRAS_LOG_TRACE_MS      (100U)
#define EXTRAS_LOG_IDLE_TRACE_MS (500U)

/* Dump pacing: lines per line-task pass, and the most bytes left in the
 * 4 KB USB ring before another line is printed. */
#define EXTRAS_LOG_LINES_PER_PASS (4U)
#define EXTRAS_LOG_PENDING_MAX    (1024U)

/* ------------------------------------------------------------------ *
 * Kernel resources [Chat].
 * ------------------------------------------------------------------ */

/* Below motion control (5) and above line following (9). */
#define EXTRAS_TASK_PRI (6)

/* The team uses 4096 bytes per task under SMP. */
#define EXTRAS_TASK_STACK (4096U)

/* Lengths of the command queue and of the event outbox. */
#define EXTRAS_CMD_RING_LEN (4U)
#define EXTRAS_EVT_RING_LEN (4U)

/* ------------------------------------------------------------------ *
 * Field widths (Change Requirement 1).
 * ------------------------------------------------------------------ */

/* 12-bit distance: 0..4095 mm. */
#define EXTRAS_DIST_BITS (12U)
#define EXTRAS_DIST_MAX  ((1U << EXTRAS_DIST_BITS) - 1U)

/* 10-bit width: 0..1023 mm, where 1023 means "1023 mm or wider". */
#define EXTRAS_WIDTH_BITS (10U)
#define EXTRAS_WIDTH_MAX  ((1U << EXTRAS_WIDTH_BITS) - 1U)

/* 6-bit start angle: 0..60 degrees. */
#define EXTRAS_ANGLE_BITS (6U)
#define EXTRAS_ANGLE_MAX  (60U)

/* 5-bit request id: 0..31. */
#define EXTRAS_ID_BITS (5U)
#define EXTRAS_ID_MAX  ((1U << EXTRAS_ID_BITS) - 1U)

/* Other field widths. */
#define EXTRAS_SLOT_BITS      (5U)
#define EXTRAS_STATE_COUNT    (7U)
#define EXTRAS_STATE_BITS     (3U)
#define EXTRAS_CAUSE_COUNT    (14U)
#define EXTRAS_CAUSE_BITS     (4U)
#define EXTRAS_SEQ_BITS       (6U)
#define EXTRAS_MBX_WIDTH_BITS (16U)

/* Largest VALID distance: 23500 us * 343 / 2000 = 4030 mm. */
#define EXTRAS_VALID_MAX_MM ((EXTRAS_VALID_MAX_US * EXTRAS_SOUND_NUM) / EXTRAS_SOUND_DEN)

/* Counter limits for the packed counter word (3, 2, 3, 2 and 3 bits). */
#define EXTRAS_CNT3_MAX (7U)
#define EXTRAS_CNT2_MAX (3U)

/* Size of the scan profile in bytes. */
#define EXTRAS_PROFILE_BYTES (50U)

EXTRAS_CT_ASSERT(dist_fits, EXTRAS_VALID_MAX_MM <= EXTRAS_DIST_MAX);
EXTRAS_CT_ASSERT(region_fits,
             (EXTRAS_REGION_MAX_MM + EXTRAS_REGION_EXTRA_MM) <= EXTRAS_DIST_MAX);
EXTRAS_CT_ASSERT(mbx_width_fits,
             (EXTRAS_ECHO_WAIT_MS * 1000U) <= ((1UL << EXTRAS_MBX_WIDTH_BITS) - 1UL));
EXTRAS_CT_ASSERT(slots_fit, EXTRAS_SLOT_COUNT <= (1U << EXTRAS_SLOT_BITS));
EXTRAS_CT_ASSERT(states_fit, EXTRAS_STATE_COUNT <= (1U << EXTRAS_STATE_BITS));
EXTRAS_CT_ASSERT(causes_fit, EXTRAS_CAUSE_COUNT <= (1U << EXTRAS_CAUSE_BITS));
EXTRAS_CT_ASSERT(angle_fits, EXTRAS_ANGLE_MAX < (1U << EXTRAS_ANGLE_BITS));
EXTRAS_CT_ASSERT(profile_is_50_bytes,
             EXTRAS_PROFILE_BYTES == (sizeof(uint16_t) * EXTRAS_SLOT_COUNT));
EXTRAS_CT_ASSERT(scan_points_ahead,
             0 == (EXTRAS_SLOT_FIRST_DECI + (int32_t)EXTRAS_SLOT_CENTRE * EXTRAS_SLOT_STEP_DECI));
EXTRAS_CT_ASSERT(coarse_ends_on_edge,
             0U == ((EXTRAS_SLOT_COUNT - 1U) % EXTRAS_COARSE_STRIDE));
EXTRAS_CT_ASSERT(floor_fits, EXTRAS_FREE_FLOOR_MM <= EXTRAS_WIDTH_MAX);
EXTRAS_CT_ASSERT(ping_after_wait, EXTRAS_ECHO_WAIT_MS < EXTRAS_PING_PERIOD_MS);
EXTRAS_CT_ASSERT(unknown_cnt_fits, EXTRAS_NO_ECHO_COUNT <= EXTRAS_CNT3_MAX);
EXTRAS_CT_ASSERT(stuck_cnt_fits, EXTRAS_STUCK_COUNT <= EXTRAS_CNT2_MAX);
EXTRAS_CT_ASSERT(hit_cnt_fits, EXTRAS_DETECT_HITS <= EXTRAS_CNT3_MAX);
EXTRAS_CT_ASSERT(clear_cnt_fits, EXTRAS_ESCAPE_CLEAR_PINGS <= EXTRAS_CNT3_MAX);
EXTRAS_CT_ASSERT(nav_brake_in_time, EXTRAS_NAV_BRAKE_MS < EXTRAS_WAIT_STOP_MS);
EXTRAS_CT_ASSERT(nav_speed_in_range, EXTRAS_NAV_SPEED_UM_S <= 200000U);
EXTRAS_CT_ASSERT(sensors_in_front, (EXTRAS_AXLE_TO_SONAR_MM < EXTRAS_AXLE_TO_FRONT_MM) &&
                                   (EXTRAS_AXLE_TO_IR_MM < EXTRAS_AXLE_TO_FRONT_MM));
EXTRAS_CT_ASSERT(sweep_pings_fit, (EXTRAS_SWEEP_PINGS > 0U) && (EXTRAS_SWEEP_PINGS < 8U));
EXTRAS_CT_ASSERT(side_assumed_in_range,
             (EXTRAS_SIDE_ASSUMED_DECI > 0) &&
                 (EXTRAS_SIDE_ASSUMED_DECI < EXTRAS_SIDE_PAN_DECI));
EXTRAS_CT_ASSERT(spin_in_range, (EXTRAS_NAV_SPIN_SLOW_UM_S <= EXTRAS_NAV_SPIN_UM_S) &&
                                (EXTRAS_NAV_SPIN_UM_S <= EXTRAS_NAV_SPEED_UM_S) &&
                                (EXTRAS_NAV_SPIN_TOL_DECI > 0));
EXTRAS_CT_ASSERT(line_window_order, EXTRAS_LINE_WINDOW_MM < EXTRAS_LINE_GIVE_UP_MM);
EXTRAS_CT_ASSERT(line_min_order,
             (EXTRAS_LINE_MIN_MM > 0U) && (EXTRAS_LINE_MIN_MM <= (EXTRAS_TAPE_W_MM / 2U)));
EXTRAS_CT_ASSERT(centre_past_ir, EXTRAS_NAV_CENTRE_MM >= EXTRAS_AXLE_TO_IR_MM);
EXTRAS_CT_ASSERT(swing_covers_corners,
             (EXTRAS_SWING_MM * EXTRAS_SWING_MM) >=
                 ((EXTRAS_AXLE_TO_FRONT_MM * EXTRAS_AXLE_TO_FRONT_MM) +
                  (EXTRAS_HALF_W_LEFT_MM * EXTRAS_HALF_W_LEFT_MM)));
EXTRAS_CT_ASSERT(left_is_wider, EXTRAS_HALF_W_LEFT_MM >= EXTRAS_HALF_W_RIGHT_MM);
EXTRAS_CT_ASSERT(track_pan_in_range, EXTRAS_TRACK_PAN_DECI <= EXTRAS_SERVO_LIMIT_DECI);
EXTRAS_CT_ASSERT(track_tilt_below_pan, EXTRAS_TRACK_TILT_DECI < EXTRAS_TRACK_PAN_DECI);
EXTRAS_CT_ASSERT(track_step_positive, EXTRAS_TRACK_MIN_STEP_DECI > 0);
EXTRAS_CT_ASSERT(pan_in_range, (EXTRAS_SIDE_PAN_DECI > 0) &&
                               (EXTRAS_SIDE_PAN_DECI <= EXTRAS_SERVO_LIMIT_DECI) &&
                               (0 == (EXTRAS_SIDE_PAN_DECI % EXTRAS_SLOT_STEP_DECI)));
EXTRAS_CT_ASSERT(move_divisor_ok, EXTRAS_MOVE_DIVISOR > 0U);
EXTRAS_CT_ASSERT(bump_in_range,
             (EXTRAS_BUMP_PERCENT > 0) && (EXTRAS_BUMP_PERCENT < 100) &&
                 (EXTRAS_BUMP_MIN_UM_S <= (int32_t)EXTRAS_NAV_SPEED_UM_S));
EXTRAS_CT_ASSERT(servo_pings_fit,
             (EXTRAS_NAV_SERVO_PINGS > 0U) && (EXTRAS_NAV_SERVO_PINGS < 32U));
EXTRAS_CT_ASSERT(nav_ttl_in_range, EXTRAS_NAV_TTL_US <= 200000U);
EXTRAS_CT_ASSERT(log_pow2, 0U == (EXTRAS_LOG_RECORDS & (EXTRAS_LOG_RECORDS - 1U)));
EXTRAS_CT_ASSERT(log_fits_16bit, EXTRAS_LOG_RECORDS <= 65536U);

#endif /* EXTRAS_CFG_H */

/*** end of file ***/
