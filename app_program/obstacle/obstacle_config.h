/** @file obstacle_config.h
 *
 * @brief Obstacle settings: pins, timings, tunables, car geometry and
 *        compile-time checks that every packed field fits its range.
 *
 * Every number states where it came from:
 *   [Spec]     the project brief.
 *   [Measured] measured on the car.
 *   [Chat]     agreed with the user while designing.
 *   [Default]  a starting value that is to be tuned on the car.
 *   FILL       a value that still has to be measured and replaced.
 *
 * Units are mm, ms, us, um/s and deci-degrees (0.1 degree, positive =
 * left). This header has no kernel includes.
 */

#ifndef OBSTACLE_CFG_H
#define OBSTACLE_CFG_H

#include <stdint.h>

#include "obstacle/obstacle_ping.h"
#include "scan_limits.h"

/* C99 has no _Static_assert, so a negative array size stops the build when
 * a condition is false. NOTE: This macro cannot be a function because it
 * must act at compile time (BARR-C 6.3.a). */
#define OBSTACLE_CT_ASSERT(name, cond)                                               \
    typedef uint8_t obstacle_ct_##name##_t[(cond) ? 1 : -1]

/* ------------------------------------------------------------------ *
 * Pins on the Cytron Robo Pico [Spec].
 * ------------------------------------------------------------------ */

/* Grove 1, GPIO output to the sensor trigger. */
#define OBSTACLE_TRIG_PIN (0U)

/* Grove 1, GPIO input from the sensor echo, both-edge interrupt. */
#define OBSTACLE_ECHO_PIN (1U)

/* Servo port 1, PWM slice 6 channel A. */
#define OBSTACLE_SERVO_PIN (12U)

/* ------------------------------------------------------------------ *
 * Ping timing [Spec] [HC-SR04 datasheet].
 * ------------------------------------------------------------------ */

/* Let the transducer finish the preceding shot before retriggering. */
#define OBSTACLE_PING_PERIOD_MS (60U)

/* An echo must have begun by this deadline to get the rest of its slot. */
#define OBSTACLE_ECHO_WAIT_MS (40U)
/* The ISR timestamps edges in microseconds. */
#define OBSTACLE_ECHO_WAIT_US  (OBSTACLE_ECHO_WAIT_MS * 1000U)
#define OBSTACLE_PING_PERIOD_US (OBSTACLE_PING_PERIOD_MS * 1000U)

/* The trigger pulse must be at least 10 us long. */
#define OBSTACLE_TRIG_PULSE_US (12U)

/* Echo widths from 116 us (about 20 mm) to 23500 us (about 4 m) are
 * VALID readings. A clean echo from OBSTACLE_GLITCH_MAX_US up to 116 us is
 * TOO_CLOSE: an object inside the blind zone, which counts as a hit at
 * OBSTACLE_MIN_RANGE_MM. An echo shorter than OBSTACLE_GLITCH_MAX_US is too
 * short to come from a reflection and is a GLITCH. [Default] The glitch limit
 * is a placeholder: tune it on the car. */
#define OBSTACLE_VALID_MIN_US  (116U)
#define OBSTACLE_VALID_MAX_US  (23500U)
#define OBSTACLE_GLITCH_MAX_US (30U)
#define OBSTACLE_MIN_RANGE_MM  (20U)

/* Distance in mm is width_us * 343 / 2000 (speed of sound 343 m/s). */
#define OBSTACLE_SOUND_NUM (343U)
#define OBSTACLE_SOUND_DEN (2000U)

/* The 1 MHz TIMER must advance at least 25 ticks across a 50 us wait. */
#define OBSTACLE_TIMER_CHECK_US        (50U)
#define OBSTACLE_TIMER_CHECK_MIN_TICKS (25U)

/* ------------------------------------------------------------------ *
 * Faults [Spec].
 * ------------------------------------------------------------------ */

/* [Default] This many NO_ECHO pings in a row raise NO_ECHO. */
#define OBSTACLE_NO_ECHO_COUNT (3U)

/* [Default] This many STUCK pings in a row raise ECHO_STUCK. */
#define OBSTACLE_STUCK_COUNT (3U)

/* [Default] This many GLITCH pings in a row, or this many failed pings of
 * any mix (NO_ECHO, GLITCH, STUCK) in a row, raise ECHO_NOISE. */
#define OBSTACLE_GLITCH_COUNT (5U)
#define OBSTACLE_FAIL_COUNT   (5U)

/* A stationary step (scan, side check, servo move) that makes no progress
 * for this long is a hung task, not a slow step: stop the car. */
#define OBSTACLE_STEP_STALL_MS (3000U)
#define OBSTACLE_SCAN_STALL_MS OBSTACLE_STEP_STALL_MS

/* ------------------------------------------------------------------ *
 * Monitor ahead and the collision guard.
 * ------------------------------------------------------------------ */

/* [Default] The car must stop about 200 mm away; 50 mm more
 * leaves room for two pings and braking. */
#define OBSTACLE_DETECT_MM (250U)

/* [Spec] A ping is a hit when it is VALID within OBSTACLE_DETECT_MM, or
 * TOO_CLOSE. This many hits among the last OBSTACLE_HIT_WINDOW pings report
 * an obstacle. Two of three tolerates one missed ping and one outlier. */
#define OBSTACLE_DETECT_HITS (2U)
#define OBSTACLE_HIT_WINDOW  (3U)

/* [Default] A forward reading closer than this stops the car at once. */
#define OBSTACLE_COLLISION_MM (80U)

/* [Default] Close readings (VALID within OBSTACLE_COLLISION_MM, or TOO_CLOSE)
 * needed among the last OBSTACLE_HIT_WINDOW pings for the emergency stop. */
#define OBSTACLE_COLLISION_HITS (2U)

/* [Default] While the car drives, the sonar faces forward and every VALID
 * reading under this distance stops the car and starts a new scan. */
#define OBSTACLE_FWD_GUARD_MM (300U)

/* ------------------------------------------------------------------ *
 * Servo (SG90 on servo port 1). The datasheet gives both "1-2 ms for
 * +-90 degrees" and "500-2400 us for 180 degrees". Calibrate with
 * checklist step H2.
 * ------------------------------------------------------------------ */

/* FILL: nominal +1 degree mechanical correction for the observed boot aim. */
#define OBSTACLE_SERVO_CENTRE_US (1494U)

/* FILL: microseconds per 10 degrees (a safe 5.6 us per degree guess). */
#define OBSTACLE_SERVO_US_PER_10DEG (56U)

/* +1 if a longer pulse turns the sensor to the car's left, -1 otherwise.
 * -1 on the car: with +1 a scan that turned the body right (as the wheel
 * odometry and the line sensors count it) was seen to turn the car the
 * other way from where the sonar looked, so the plan took the wrong side.
 * The servo must turn the same way as the wheels count a turn. */
#define OBSTACLE_SERVO_DIR (-1)

/* Never command a pulse outside the datasheet range. */
#define OBSTACLE_SERVO_MIN_US (500U)
#define OBSTACLE_SERVO_MAX_US (2400U)

/* [Spec] Settle 3 ms per degree plus 20 ms. The datasheet speed is
 * 1.7-2.0 ms per degree at 4.8 V; the margin covers LiPo sag, and the
 * 20 ms covers one PWM frame. Below 4.0 V the SG90 is out of spec. */
#define OBSTACLE_SETTLE_MS_PER_DEG (3U)
#define OBSTACLE_SETTLE_BASE_MS    (20U)
#define OBSTACLE_SERVO_PREDELAY_MS (60U)

/* PWM: 125 MHz / 125 = 1 MHz count and TOP 19999 give 50 Hz. CHx_DIV is
 * an 8.4 fixed-point value. */
#define OBSTACLE_PWM_DIV_INT   (125U)
#define OBSTACLE_PWM_DIV_SHIFT (4U)
#define OBSTACLE_PWM_TOP       (19999U)
#define OBSTACLE_PWM_SLICE     (6U)

/* ------------------------------------------------------------------ *
 * Scan [Chat]. The profile has one slot per 5 degrees of bearing from the
 * car's heading when the scan started (body heading + servo angle):
 * -90..+90 degrees, 37 slots, slot 18 straight ahead.
 * ------------------------------------------------------------------ */

#define OBSTACLE_SLOT_COUNT      (37U)
#define OBSTACLE_SLOT_CENTRE     (18U)
#define OBSTACLE_SLOT_STEP_DECI  (50)
#define OBSTACLE_SLOT_FIRST_DECI (-900)

/* Coarse sweep: every 5 degrees over the centre servo window.
 * The fine walk uses the same 5 degree grid. */
#define OBSTACLE_COARSE_STEP_DECI (50)

/* Shift by one servo window at a time, with the last heading placing the
 * servo's outer edge on the last profile slot. */
#define OBSTACLE_SCAN_PIVOT_DECI     (2 * OBSTACLE_SERVO_LIMIT_DECI)
#define OBSTACLE_SCAN_PIVOT_MAX_DECI (-OBSTACLE_SLOT_FIRST_DECI - OBSTACLE_SERVO_LIMIT_DECI)

/* [Spec 3c] The region is the closest VALID return within 600 mm plus
 * neighbouring readings within closest + 250 mm. */
#define OBSTACLE_REGION_MAX_MM   (600U)
#define OBSTACLE_REGION_EXTRA_MM (250U)

/* [Default] Plausibility checks that raise warning flags and never stop the
 * car (sonar-fault-detection.md). P1: two VALID pings of one slot that differ by more than
 * OBSTACLE_NOISE_MM while the car and servo stand still, in OBSTACLE_NOISE_PINGS of the last
 * OBSTACLE_NOISE_WINDOW pings, raise echo_noisy. P5: more than OBSTACLE_SCAN_NOISY_PCT percent
 * of the slots in a finished scan needed 3 or 4 pings, which raises scan_noisy. */
#define OBSTACLE_NOISE_MM        (40U)
#define OBSTACLE_NOISE_PINGS     (3U)
#define OBSTACLE_NOISE_WINDOW    (8U)
#define OBSTACLE_SCAN_NOISY_PCT  (30U)
#define OBSTACLE_PCT_BASE        (100U)

/* [Default] P3: of the first OBSTACLE_BEAM_CHECK_PINGS pings after boot, this many
 * hits mean something is in the beam (a bracket, a wire, the floor): beam_blocked.
 * Nothing is expected within the detect distance at the start line. */
#define OBSTACLE_BEAM_CHECK_PINGS (8U)
#define OBSTACLE_BEAM_CHECK_HITS  (6U)

/* [Default] P4: read the TRIG pad back during the pulse. If it does not read high, the
 * ping counts as NO_ECHO and the 40 ms wait is skipped. Set 0 if the readback does not
 * work on the car (checklist H7e). */
#define OBSTACLE_TRIG_READBACK (1U)
#define OBSTACLE_RETRY_PINGS     (3U) /* a slot that needed this many pings was retried */

/* [Spec 3e] Four pings without agreement make the slot a GLITCH (unresolved). */
#define OBSTACLE_MAX_PINGS (4U)

/* [Chat][Default] The car's path: +-30 degrees of straight ahead. CLEAR
 * needs no reading in it and the scan looks for the closest reading here
 * before it looks at the whole profile. */
#define OBSTACLE_PATH_HALF_DECI (300)

/* [Default] An edge is trusted only when its bearing, after the beam
 * correction, is inside this angle. A flat face seen further off its
 * normal returns no echo (the simulation uses 35 degrees), so an apparent
 * edge beyond this angle may be the end of the echo and not the end of the
 * obstacle: the side is then treated as a wall. */
#define OBSTACLE_EDGE_TRUST_DECI (300)

/* [Measured] Half of the sensor cone. */
#define OBSTACLE_BEAM_HALF_DECI CHASSIS_SONAR_BEAM_HALF_DECI

/* ------------------------------------------------------------------ *
 * Car geometry. All distances are from the middle of the wheel axle.
 * ------------------------------------------------------------------ */

/* FILL: servo shaft to the front bumper. */
#define OBSTACLE_SERVO_TO_FRONT_MM (30U)

/* [Default] Sonar offset ahead of the wheel axle. */
#define OBSTACLE_SONAR_FWD_MM ((uint32_t)CHASSIS_SONAR_FORWARD_MM)

/* Front bumper and tail, from the axle. */
#define OBSTACLE_CAR_FRONT_MM (OBSTACLE_SONAR_FWD_MM + OBSTACLE_SERVO_TO_FRONT_MM)
#define OBSTACLE_CAR_REAR_MM  ((uint32_t)CHASSIS_TAIL_MM)

/* Radius of the circle the car sweeps when it spins on the spot: the
 * farthest corner from the axle middle, sqrt(100^2 + 63^2) = 118.2,
 * rounded up. */
#define OBSTACLE_PIVOT_RADIUS_MM ((uint32_t)CHASSIS_BODY_RADIUS_MM)

/* ------------------------------------------------------------------ *
 * Planning and the bypass [Chat] [Default].
 * ------------------------------------------------------------------ */

/* [Default] Extra sideways clearance. */
#define OBSTACLE_SIDE_MARGIN_MM (50U)

/* [Default] Gap kept between the axle line and the side of a box: the
 * pivot circle (the car turns toward the box for every side check) plus
 * the margin plus 30 mm. 190 left a 280 mm box 43 mm of clearance in the
 * simulation; 200 keeps every width at or above the 50 mm margin. */
#define OBSTACLE_BOX_GAP_MM (200U)

/* [Default] Angles the planner tries for the pivot away, smallest first. */
#define OBSTACLE_OUT_ANGLE_A_DECI (450)
#define OBSTACLE_OUT_ANGLE_B_DECI (600)
#define OBSTACLE_OUT_ANGLE_C_DECI (750)
#define OBSTACLE_OUT_ANGLE_D_DECI (900)

/* [Chat] Largest sideways offset a bypass may use. The course has no side
 * boundary, so this only rejects absurd plans (for example from a bad
 * echo). */
#define OBSTACLE_FREE_FLOOR_MM (500U)

/* [Default] Assumed obstacle depth, used to size the corridor check. */
#define OBSTACLE_DEPTH_ASSUMED_MM (180U)

/* [Chat] Bypass speed. [Measured] 160 mm/s was asked for at first, but at
 * the 25 % duty cap the fast wheel only reached about 142 mm/s. 140 mm/s is
 * within reach. NOTE: motion_set_speed() rejects (E_PAR) any target above
 * max_target_um_s in main.c, which must be at least this value. */
#define OBSTACLE_BYPASS_SPEED_MM_S (140U)

/* [Chat] A wall is planned when no trusted edge is found on either side:
 * the car pivots left to this angle from its heading and follows the wall
 * on its right. */
#define OBSTACLE_WALL_PIVOT_DECI (900)

/* [Default] Limits. All of them stop the car (cause BYPASS_LIMIT or
 * LINE_TIMEOUT) instead of letting it drive blind.
 *   HUG_MAX_MS        face following, from the start of REACQUIRE (the
 *                     distance budget does not end a wall; the pilot
 *                     enforces HUG_RUN_MAX_MS, derived in hug.h);
 *   BOX_ALONG_MAX_MM  distance along a box before it is called too deep;
 *   ESCAPE_CLEARS     clear forward pings that end the escape turn;
 *   ESCAPE_STEP_*     once the planned angle is reached and the beam still
 *                     sees the obstacle, the turn target moves on by
 *                     STEP_DECI every STEP_MS, so the car keeps turning
 *                     away instead of standing still;
 *   ESCAPE_TURN_MAX_DECI the furthest the escape turns from the line. */
#define OBSTACLE_HUG_MAX_MS       (50000U)
#define OBSTACLE_BOX_ALONG_MAX_MM (2500U)
#define OBSTACLE_HUG_EXTRA_MM     (100U)
#define OBSTACLE_HUG_MIN_MM       (200U)
#define OBSTACLE_HUG_MAX_MM       (5000U)
#define OBSTACLE_WAIT_STOP_MS     (500U)
#define OBSTACLE_PLAN_ACCEPT_MS   (200U)
#define OBSTACLE_ESCAPE_MAX_MS    (6000U)
#define OBSTACLE_ESCAPE_CLEARS    (2U)
#define OBSTACLE_ESCAPE_STEP_DECI (150)
#define OBSTACLE_ESCAPE_STEP_MS   (500U)
#define OBSTACLE_ESCAPE_TURN_MAX_DECI (1200)
#define OBSTACLE_LINE_GRACE_MS    (250U)

/* [Default] Bug2 face follow (follow.c). Tune on the car.
 *   ECHO_INCIDENCE_MAX  assumed echo limit off the face normal;
 *   FOLLOW_TOWARD       glance heading into the face: 90 - 45 - 15 (servo)
 *                       = 30 degrees off the normal;
 *   FOLLOW_HOLD         axle to face on a leg; a glance arc and the 80 mm
 *                       collision guard must both fit inside it;
 *   FOLLOW_LEG          leg length between two glances;
 *   FOLLOW_ARC          wheel speed difference of a glance arc (radius
 *                       about 7490 / ARC mm);
 *   FOLLOW_LOOK         speed once the glance heading is reached;
 *   FOLLOW_SEEN_MAX     the longest echo that counts as the face;
 *   FOLLOW_MISS_MAX     glance pings in a row without the face: a corner;
 *   FOLLOW_INNER        echo during a leg that means a face ahead;
 *   FOLLOW_SAME_FACE    an echo point further than this off the face line,
 *                       on the car's side, is a face across the path; a
 *                       30 degree slant moves about 100 mm per glance;
 *   FOLLOW_CORNER_PAST  axle past the corner reference before the wrap, so
 *                       the next face starts at the hold distance;
 *   FOLLOW_TRIM_*       leg heading correction per mm of gap error, cap.
 *                       One glance (arc in, arc back) closes about
 *                       0.59 x arc radius on the face; a leg at the cap
 *                       must win it back (120 mm x sin 30 = 60 mm);
 *   FOLLOW_SLANT_*      face point spacing for a new direction, the
 *                       largest change one glance may measure, and the
 *                       divisor that blends it in (noise on a flat face);
 *   FOLLOW_LEAVE_AHEAD  tape counts only this far past the hit point;
 *   FOLLOW_LINE_LEAD    creep onto the tape before the last turn;
 *   FOLLOW_LINE_WINDOW  the tape counts only this close to the original
 *                       line (x of the scan frame); odometry drifts about
 *                       200 mm over a bypass with 1 % wheel error. The IMU
 *                       has no gyroscope, so the heading stays on the
 *                       encoders. */
#define OBSTACLE_ECHO_INCIDENCE_MAX_DECI (300)
#define OBSTACLE_FOLLOW_TOWARD_DECI      (450)
/* The face follow looks this far to the side of the car's front. It stays at
 * 15 degrees, where its geometry was tuned, when the servo can turn further. */
#define OBSTACLE_FOLLOW_LOOK_DECI        (150)
#define OBSTACLE_FOLLOW_HOLD_MM          (280)
#define OBSTACLE_FOLLOW_LEG_MM           (120)
#define OBSTACLE_FOLLOW_ARC_MM_S         (100)
#define OBSTACLE_FOLLOW_LOOK_MM_S        (60)
#define OBSTACLE_FOLLOW_SEEN_MAX_MM      (500U)
#define OBSTACLE_FOLLOW_MISS_MAX         (3U)
#define OBSTACLE_FOLLOW_INNER_MM         (250U)
#define OBSTACLE_FOLLOW_SAME_FACE_MM     (150)
#define OBSTACLE_FOLLOW_CORNER_PAST_MM   (OBSTACLE_FOLLOW_HOLD_MM)
#define OBSTACLE_FOLLOW_TRIM_PER_MM      (5)
#define OBSTACLE_FOLLOW_TRIM_MAX_DECI    (300)
#define OBSTACLE_FOLLOW_SLANT_MIN_MM     (150)
#define OBSTACLE_FOLLOW_SLANT_GAIN_DIV   (2)
#define OBSTACLE_FOLLOW_SLANT_MAX_DECI   (450)
#define OBSTACLE_FOLLOW_LEAVE_AHEAD_MM   (150)
#define OBSTACLE_FOLLOW_LINE_LEAD_MM     (70)
#define OBSTACLE_FOLLOW_LINE_WINDOW_MM   (250)

/* [Default] Bug2 hard stops and the echo-loss retry.
 *   FOLLOW_BACKUP       reverse this far before a deeper look;
 *   FOLLOW_DEEPER       the deeper look turns this much further into the
 *                       face (15 degrees off the normal);
 *   FOLLOW_TRIES_MAX    echo-loss recoveries on one face before giving up;
 *   FOLLOW_HIT_RADIUS   back this close to the hit point after
 *                       FOLLOW_CIRCLE_MIN of travel: the obstacle cannot be
 *                       passed;
 *   FOLLOW_NO_PROGRESS  travel with no gain along the original line;
 *   BUMP_*              virtual bumper (from Buddy 5, B5_BUMP_*): a wheel
 *                       told BUMP_MIN or more that measures under
 *                       BUMP_PERCENT for BUMP_MS, not within BUMP_SETTLE
 *                       of a command change of more than BUMP_CHANGE. The
 *                       back-up and the look creep run under BUMP_MIN, so
 *                       only the 60 mm back-up goes unwatched. On a leg the
 *                       car is guarded by the servo-side beam, the
 *                       collision window on non-turning pings and the
 *                       bumper; there is no forward rescan. */
#define OBSTACLE_FOLLOW_BACKUP_MM        (60)
#define OBSTACLE_FOLLOW_DEEPER_DECI      (150)
#define OBSTACLE_FOLLOW_TRIES_MAX        (3U)
#define OBSTACLE_FOLLOW_HIT_RADIUS_MM    (150)
#define OBSTACLE_FOLLOW_CIRCLE_MIN_MM    (800U)
#define OBSTACLE_FOLLOW_NO_PROGRESS_MM   (2500U)
#define OBSTACLE_BUMP_MIN_MM_S           (70)
#define OBSTACLE_BUMP_PERCENT            (40)
#define OBSTACLE_BUMP_MS                 (150U)
#define OBSTACLE_BUMP_SETTLE_MS          (300U)
#define OBSTACLE_BUMP_CHANGE_MM_S        (20)

/* ------------------------------------------------------------------ *
 * Driving [Default]: how the brain turns decisions into wheel speeds.
 * ------------------------------------------------------------------ */

/* Spin on the spot: both wheels run at up to this speed, one backwards.
 * The speed falls with the remaining angle down to the minimum, and the
 * pivot ends inside the tolerance. On the car a spin scrubs both tyres: at a
 * 35 mm/s minimum the 30 degree scan pivot stalled near 15 degrees, so the
 * scan never finished. The minimum is 90 mm/s. The tolerance stays 2
 * degrees: the wall follow also turns to it, and a wider band changes how
 * the follow judges progress. */
#define OBSTACLE_PIVOT_SPEED_MM_S (140U)
#define OBSTACLE_PIVOT_MIN_MM_S   (90U)
#define OBSTACLE_PIVOT_RAMP_DECI  (450)
#define OBSTACLE_PIVOT_TOL_DECI   (20)

/* Heading hold while driving straight: wheel speed difference per
 * deci-degree of error (um/s), capped. */
#define OBSTACLE_HEAD_GAIN_UM_S_PER_DECI (400)
#define OBSTACLE_HEAD_MAX_DIFF_MM_S      (40U)

/* The car counts as stopped after the wheels moved less than this many
 * um per ms (sum of both wheels) for this long. Also the pause before
 * any wheel changes direction. */
#define OBSTACLE_STILL_UM_PER_MS (15U)
#define OBSTACLE_STOP_SETTLE_MS  (250U)

/* Lease of every speed command. It must cover several 10 ms loop passes
 * and stay within the motion service limit (200 ms in main.c). */
#define OBSTACLE_NAV_TTL_US (100000U)

/* Hold brake time before sending SCAN_REQ. */
#define OBSTACLE_NAV_BRAKE_MS (100U)

/* Settle wait time for servo in navigation. */
#define OBSTACLE_NAV_SERVO_WAIT_MS (200U)

/* Repeat period for sending LINE_FOUND / TURNED commands. */
#define OBSTACLE_NAV_LINE_REPEAT_MS (50U)

/* Navigation speed in um/s. */
#define OBSTACLE_NAV_SPEED_UM_S ((uint32_t)OBSTACLE_BYPASS_SPEED_MM_S * 1000U)

/* [Chat] Before any wheel changes direction, both wheels stop for this
 * long (the geared motor rattles otherwise and the encoder reports invalid
 * steps; seen on the car as a FEEDBACK fault). */
#define OBSTACLE_NAV_REVERSE_PAUSE_MS (300U)

/* [Measured] The encoder plugs on the car are crossed relative to
 * app_program/motion/motion_pins.h (team decision, kept): the motion service's
 * wheel 0 reads the right encoder and wheel 1 the left one. 1 =
 * obstacle_nav.c undoes the swap for its own odometry; 0 = the plugs match
 * motion_pins.h (swapped back on the car, way A). */
#define OBSTACLE_ENCODERS_CROSSED (0)

/* Deci-degrees per degree. */
#define OBSTACLE_DECI_PER_DEG     (10)

/* ------------------------------------------------------------------ *
 * Flight recorder (flight_log.c) [Default].
 * ------------------------------------------------------------------ */

/* Records kept in RAM (16 bytes each; must be a power of two). */
#define OBSTACLE_LOG_RECORDS (512U) /* 8 KiB: about 50 s of bypass at one trace per 100 ms */

/* Scan profiles kept in RAM. */
#define OBSTACLE_LOG_PROFILES (8U)

/* A profile belongs to the RESULT record logged within this time. */
#define OBSTACLE_LOG_PROFILE_MATCH_MS (50U)

/* Trace period while the car drives itself, and otherwise. */
#define OBSTACLE_LOG_TRACE_MS      (100U)
#define OBSTACLE_LOG_IDLE_TRACE_MS (500U)

/* Dump pacing: lines per line-task pass, and the most bytes left in the
 * 4 KB USB ring before another line is printed. */
#define OBSTACLE_LOG_LINES_PER_PASS (4U)
#define OBSTACLE_LOG_PENDING_MAX    (1024U)

/* ------------------------------------------------------------------ *
 * Kernel resources [Chat].
 * ------------------------------------------------------------------ */

/* Below motion control (5) and above line following (9). */
#define OBSTACLE_TASK_PRI (6)

/* The team uses 4096 bytes per task under SMP. */
#define OBSTACLE_TASK_STACK (4096U)

/* Length of the command queue and event ring. */
#define OBSTACLE_CMD_RING_LEN (4U)
#define OBSTACLE_EVT_RING_LEN (4U)

/* ------------------------------------------------------------------ *
 * Field widths.
 * ------------------------------------------------------------------ */

/* 12-bit distance: 0..4095 mm. */
#define OBSTACLE_DIST_BITS (12U)
#define OBSTACLE_DIST_MAX  ((1U << OBSTACLE_DIST_BITS) - 1U)

/* 5-bit request id: 0..31. */
#define OBSTACLE_ID_BITS (5U)
#define OBSTACLE_ID_MAX  ((1U << OBSTACLE_ID_BITS) - 1U)

/* Other field widths. */
#define OBSTACLE_SLOT_BITS      (6U)
#define OBSTACLE_STATE_COUNT    (7U)
#define OBSTACLE_STATE_BITS     (3U)
#define OBSTACLE_CAUSE_COUNT    (16U)
#define OBSTACLE_CAUSE_BITS     (4U)
#define OBSTACLE_SEQ_BITS       (5U)
#define OBSTACLE_ECHO_WIDTH_BITS (16U)
#define OBSTACLE_CLASS_BITS     (3U)
#define OBSTACLE_CLASS_COUNT    (6U)

/* Largest VALID distance: 23500 us * 343 / 2000 = 4030 mm. */
#define OBSTACLE_VALID_MAX_MM ((OBSTACLE_VALID_MAX_US * OBSTACLE_SOUND_NUM) / OBSTACLE_SOUND_DEN)

/* Value used for a reading that is not a distance (OPEN, NO_ECHO, GLITCH, STUCK):
 * farther than any echo (12-bit distance maximum). */
#define OBSTACLE_SIDE_FAR_MM (4095U)

/* Size of the scan profile in bytes. */
#define OBSTACLE_PROFILE_BYTES (74U)

OBSTACLE_CT_ASSERT(dist_fits, OBSTACLE_VALID_MAX_MM <= OBSTACLE_DIST_MAX);
OBSTACLE_CT_ASSERT(region_fits,
             (OBSTACLE_REGION_MAX_MM + OBSTACLE_REGION_EXTRA_MM) <= OBSTACLE_DIST_MAX);
OBSTACLE_CT_ASSERT(echo_width_fits,
             OBSTACLE_PING_PERIOD_US < ((1UL << OBSTACLE_ECHO_WIDTH_BITS) - 1UL));
OBSTACLE_CT_ASSERT(slots_fit, OBSTACLE_SLOT_COUNT <= (1U << OBSTACLE_SLOT_BITS));
OBSTACLE_CT_ASSERT(states_fit, OBSTACLE_STATE_COUNT <= (1U << OBSTACLE_STATE_BITS));
OBSTACLE_CT_ASSERT(causes_fit, OBSTACLE_CAUSE_COUNT <= (1U << OBSTACLE_CAUSE_BITS));
OBSTACLE_CT_ASSERT(classes_fit, OBSTACLE_CLASS_COUNT <= (1U << OBSTACLE_CLASS_BITS));
OBSTACLE_CT_ASSERT(window_holds_hits,
             (OBSTACLE_DETECT_HITS <= OBSTACLE_HIT_WINDOW) &&
             (OBSTACLE_COLLISION_HITS <= OBSTACLE_HIT_WINDOW) && (OBSTACLE_HIT_WINDOW <= 8U));
OBSTACLE_CT_ASSERT(noise_window_is_a_byte, OBSTACLE_NOISE_WINDOW == 8U);
OBSTACLE_CT_ASSERT(glitch_below_valid, OBSTACLE_GLITCH_MAX_US < OBSTACLE_VALID_MIN_US);
OBSTACLE_CT_ASSERT(profile_bytes,
             OBSTACLE_PROFILE_BYTES == (sizeof(uint16_t) * OBSTACLE_SLOT_COUNT));
OBSTACLE_CT_ASSERT(slot_grid_symmetric,
             (OBSTACLE_SLOT_FIRST_DECI + ((int32_t)OBSTACLE_SLOT_CENTRE * OBSTACLE_SLOT_STEP_DECI)) == 0);
OBSTACLE_CT_ASSERT(slot_grid_ends,
             (OBSTACLE_SLOT_FIRST_DECI +
              (((int32_t)OBSTACLE_SLOT_COUNT - 1) * OBSTACLE_SLOT_STEP_DECI)) == -OBSTACLE_SLOT_FIRST_DECI);
OBSTACLE_CT_ASSERT(servo_limit_20, OBSTACLE_SERVO_LIMIT_DECI == 200);
OBSTACLE_CT_ASSERT(coarse_on_grid, 0 == (OBSTACLE_COARSE_STEP_DECI % OBSTACLE_SLOT_STEP_DECI));
OBSTACLE_CT_ASSERT(coarse_divides_range,
             0 == (OBSTACLE_SERVO_LIMIT_DECI % OBSTACLE_COARSE_STEP_DECI));
OBSTACLE_CT_ASSERT(ping_after_wait, OBSTACLE_ECHO_WAIT_MS < OBSTACLE_PING_PERIOD_MS);
OBSTACLE_CT_ASSERT(fwd_guard_above_detect, OBSTACLE_FWD_GUARD_MM > OBSTACLE_DETECT_MM);
OBSTACLE_CT_ASSERT(fwd_guard_above_collision, OBSTACLE_FWD_GUARD_MM > OBSTACLE_COLLISION_MM);
OBSTACLE_CT_ASSERT(footprint_covers_wheel_track, CHASSIS_OUTER_WIDTH_MM >= CHASSIS_WHEEL_TRACK_MM);
OBSTACLE_CT_ASSERT(pivot_radius_covers_car,
             (OBSTACLE_PIVOT_RADIUS_MM * OBSTACLE_PIVOT_RADIUS_MM) >=
                 ((OBSTACLE_CAR_FRONT_MM * OBSTACLE_CAR_FRONT_MM) +
                  (OBSTACLE_CAR_HALF_W_MM * OBSTACLE_CAR_HALF_W_MM)));
OBSTACLE_CT_ASSERT(pivot_radius_covers_tail,
             (OBSTACLE_PIVOT_RADIUS_MM * OBSTACLE_PIVOT_RADIUS_MM) >=
                 ((OBSTACLE_CAR_REAR_MM * OBSTACLE_CAR_REAR_MM) +
                  (OBSTACLE_CAR_HALF_W_MM * OBSTACLE_CAR_HALF_W_MM)));
OBSTACLE_CT_ASSERT(box_gap_clears_pivot,
             OBSTACLE_BOX_GAP_MM >= (OBSTACLE_PIVOT_RADIUS_MM + OBSTACLE_SIDE_MARGIN_MM));
OBSTACLE_CT_ASSERT(free_floor_fits_gap,
             OBSTACLE_FREE_FLOOR_MM > (OBSTACLE_BOX_GAP_MM + OBSTACLE_SIDE_MARGIN_MM));
OBSTACLE_CT_ASSERT(out_angles_ordered,
             (OBSTACLE_OUT_ANGLE_A_DECI < OBSTACLE_OUT_ANGLE_B_DECI) &&
                 (OBSTACLE_OUT_ANGLE_B_DECI < OBSTACLE_OUT_ANGLE_C_DECI) &&
                 (OBSTACLE_OUT_ANGLE_C_DECI < OBSTACLE_OUT_ANGLE_D_DECI) &&
                 (OBSTACLE_OUT_ANGLE_D_DECI <= 900));
OBSTACLE_CT_ASSERT(escape_clears_fit, OBSTACLE_ESCAPE_CLEARS >= 2U);
OBSTACLE_CT_ASSERT(pivot_speeds, (OBSTACLE_PIVOT_MIN_MM_S < OBSTACLE_PIVOT_SPEED_MM_S) &&
                                     (OBSTACLE_PIVOT_SPEED_MM_S <= 300U));
OBSTACLE_CT_ASSERT(pivot_tol, (OBSTACLE_PIVOT_TOL_DECI > 0) &&
                                  (OBSTACLE_PIVOT_TOL_DECI < OBSTACLE_PIVOT_RAMP_DECI));
OBSTACLE_CT_ASSERT(bypass_speed_in_range, OBSTACLE_BYPASS_SPEED_MM_S <= 300U);
OBSTACLE_CT_ASSERT(side_far_fits, OBSTACLE_SIDE_FAR_MM == OBSTACLE_DIST_MAX);
OBSTACLE_CT_ASSERT(hug_limit_order, OBSTACLE_HUG_MIN_MM < OBSTACLE_HUG_MAX_MM);
OBSTACLE_CT_ASSERT(nav_ttl_in_range, OBSTACLE_NAV_TTL_US <= 200000U);
OBSTACLE_CT_ASSERT(settle_covers_reverse_pause,
             OBSTACLE_STOP_SETTLE_MS <= OBSTACLE_NAV_REVERSE_PAUSE_MS);
OBSTACLE_CT_ASSERT(log_pow2, 0U == (OBSTACLE_LOG_RECORDS & (OBSTACLE_LOG_RECORDS - 1U)));
OBSTACLE_CT_ASSERT(log_fits_16bit, OBSTACLE_LOG_RECORDS <= 65536U);
OBSTACLE_CT_ASSERT(follow_toward_incidence,
             (900 - OBSTACLE_FOLLOW_TOWARD_DECI - OBSTACLE_FOLLOW_LOOK_DECI) <=
                 OBSTACLE_ECHO_INCIDENCE_MAX_DECI);
OBSTACLE_CT_ASSERT(follow_hold_clears_pivot,
             OBSTACLE_FOLLOW_HOLD_MM >= (int)(OBSTACLE_PIVOT_RADIUS_MM + OBSTACLE_SIDE_MARGIN_MM));
OBSTACLE_CT_ASSERT(follow_corner_clears_pivot,
             OBSTACLE_FOLLOW_CORNER_PAST_MM >= (int)(OBSTACLE_PIVOT_RADIUS_MM + OBSTACLE_SIDE_MARGIN_MM));
OBSTACLE_CT_ASSERT(follow_misses_fit_2_bits, OBSTACLE_FOLLOW_MISS_MAX <= 3U);
OBSTACLE_CT_ASSERT(follow_tries_fit_2_bits, OBSTACLE_FOLLOW_TRIES_MAX <= 3U);
OBSTACLE_CT_ASSERT(follow_deeper_incidence,
             (900 - OBSTACLE_FOLLOW_TOWARD_DECI - OBSTACLE_FOLLOW_DEEPER_DECI - OBSTACLE_FOLLOW_LOOK_DECI) >= 0);
OBSTACLE_CT_ASSERT(follow_look_in_servo_range, OBSTACLE_FOLLOW_LOOK_DECI <= OBSTACLE_SERVO_LIMIT_DECI);
OBSTACLE_CT_ASSERT(follow_backup_clears_pivot,
             (OBSTACLE_FOLLOW_HOLD_MM - OBSTACLE_FOLLOW_BACKUP_MM) >= 0);
OBSTACLE_CT_ASSERT(bump_below_bypass_speed, OBSTACLE_BUMP_MIN_MM_S < (int)OBSTACLE_BYPASS_SPEED_MM_S);
/* spins drive both wheels at the full bypass speed, not half of it. */
OBSTACLE_CT_ASSERT(spin_at_bypass_speed, OBSTACLE_PIVOT_SPEED_MM_S == OBSTACLE_BYPASS_SPEED_MM_S);
OBSTACLE_CT_ASSERT(follow_arc_below_speed, OBSTACLE_FOLLOW_ARC_MM_S < (int)OBSTACLE_BYPASS_SPEED_MM_S);

#endif /* OBSTACLE_CFG_H */

/*** end of file ***/
