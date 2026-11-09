/** @file obstacle_config.h
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

#ifndef OBSTACLE_CFG_H
#define OBSTACLE_CFG_H

#include <stdint.h>

/* C99 has no _Static_assert, so a negative array size stops the build when
 * a condition is false. NOTE: This macro cannot be a function because it
 * must act at compile time (BARR-C 6.3.a). */
#define OBSTACLE_CT_ASSERT(name, cond)                                               \
    typedef uint8_t obstacle_ct_##name##_t[(cond) ? 1 : -1]

/* ------------------------------------------------------------------ *
 * Pins on the Cytron Robo Pico [DR 2].
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

/* At least 60 ms between triggers, so old echoes die out. */
#define OBSTACLE_PING_PERIOD_MS (60U)

/* The task waits at most this long for the end of an echo. */
#define OBSTACLE_ECHO_WAIT_MS (40U)

/* The trigger pulse must be at least 10 us long. */
#define OBSTACLE_TRIG_PULSE_US (12U)

/* Echo widths from 116 us (about 20 mm) to 23500 us (about 4 m) are
 * VALID readings. */
#define OBSTACLE_VALID_MIN_US (116U)
#define OBSTACLE_VALID_MAX_US (23500U)

/* Distance in mm is width_us * 343 / 2000 (speed of sound 343 m/s). */
#define OBSTACLE_SOUND_NUM (343U)
#define OBSTACLE_SOUND_DEN (2000U)

/* The 1 MHz TIMER must advance at least 25 ticks across a 50 us wait. */
#define OBSTACLE_TIMER_CHECK_US        (50U)
#define OBSTACLE_TIMER_CHECK_MIN_TICKS (25U)

/* ------------------------------------------------------------------ *
 * Faults [Spec].
 * ------------------------------------------------------------------ */

/* This many UNKNOWN pings in a row raise NO_ECHO. */
#define OBSTACLE_NO_ECHO_COUNT (5U)

/* This many STUCK pings in a row raise ECHO_STUCK. */
#define OBSTACLE_STUCK_COUNT (3U)

/* The scan is aborted when no slot settles for this long. One slot needs
 * at most four pings (about 400 ms with the 60 ms ping period and the
 * servo settle), so 3 s of silence is a hung task, not a slow scan. A
 * fixed total limit was removed because a long fine walk can legitimately
 * need more than 8 s. */
#define OBSTACLE_SCAN_STALL_MS (3000U)

/* ------------------------------------------------------------------ *
 * Monitor ahead and the collision guard.
 * ------------------------------------------------------------------ */

/* [Default] The Tribunal stops the car about 200 mm away; 50 mm more
 * leaves room for two pings and braking. */
#define OBSTACLE_DETECT_MM (250U)

/* [Spec] Consecutive VALID hits needed to report an obstacle. */
#define OBSTACLE_DETECT_HITS (2U)

/* [Default] Readings closer than this call the emergency stop. */
#define OBSTACLE_COLLISION_MM (80U)

/* [Default] Consecutive close readings needed for the emergency stop. */
#define OBSTACLE_COLLISION_HITS (2U)

/* ------------------------------------------------------------------ *
 * Scan grid [Spec].
 * ------------------------------------------------------------------ */

/* 25 slots from -60 to +60 degrees in 5 degree steps; slot 12 is ahead. */
#define OBSTACLE_SLOT_COUNT      (25U)
#define OBSTACLE_SLOT_CENTRE     (12U)
#define OBSTACLE_SLOT_STEP_DECI  (50)
#define OBSTACLE_SLOT_FIRST_DECI (-600)

/* The coarse scan measures every third slot. */
#define OBSTACLE_COARSE_STRIDE (3U)

/* [Spec 3c] The region is the closest VALID return within 600 mm plus
 * neighbouring coarse slots within closest + 250 mm. */
#define OBSTACLE_REGION_MAX_MM   (600U)
#define OBSTACLE_REGION_EXTRA_MM (250U)

/* [Spec 3e] Two pings agree when within 15 mm; four pings without
 * agreement make the slot UNKNOWN. */
#define OBSTACLE_AGREE_MM  (15U)
#define OBSTACLE_MAX_PINGS (4U)

/* [Chat][Default] The car's path: +-30 degrees of straight ahead. CLEAR
 * needs a real reading in every slot of it, and the scan looks for the
 * closest reading here before it looks at the whole sweep. */
#define OBSTACLE_PATH_HALF_DECI (300)

/* ------------------------------------------------------------------ *
 * Servo (SG90 on servo port 1). The datasheet gives both "1-2 ms for
 * +-90 degrees" and "500-2400 us for 180 degrees". The narrow scale
 * below cannot reach the end stops. Calibrate with checklist step H2.
 * ------------------------------------------------------------------ */

/* FILL: pulse width that points the sensor straight ahead. */
#define OBSTACLE_SERVO_CENTRE_US (1500U)

/* FILL: microseconds per 10 degrees (a safe 5.6 us per degree guess). */
#define OBSTACLE_SERVO_US_PER_10DEG (56U)

/* FILL: +1 if a longer pulse turns the sensor left, -1 otherwise. */
#define OBSTACLE_SERVO_DIR (1)

/* [Spec] The scan covers +-60 degrees. */
#define OBSTACLE_SERVO_LIMIT_DECI (600)

/* Never command a pulse outside the datasheet range. */
#define OBSTACLE_SERVO_MIN_US (500U)
#define OBSTACLE_SERVO_MAX_US (2400U)

/* [Spec] Settle 3 ms per degree plus 20 ms. The datasheet speed is
 * 1.7-2.0 ms per degree at 4.8 V; the margin covers LiPo sag, and the
 * 20 ms covers one PWM frame. FILL: power source (LiPo, powerbank or
 * VIN). Below 4.0 V the SG90 is out of spec, so re-check on a low
 * battery. */
#define OBSTACLE_SETTLE_MS_PER_DEG (3U)
#define OBSTACLE_SETTLE_BASE_MS    (20U)

/* [Chat] In states where the car may be driving, the servo waits this long
 * before it moves, so that Navigation (20 ms loop) and the motion service
 * (10 ms) have stopped the wheels: a servo swing while a motor drives
 * disturbs the wheel encoders (seen on the car as a FEEDBACK fault). */
#define OBSTACLE_SERVO_PREDELAY_MS (60U)

/* PWM: 125 MHz / 125 = 1 MHz count and TOP 19999 give 50 Hz. CHx_DIV is
 * an 8.4 fixed-point value. */
#define OBSTACLE_PWM_DIV_INT   (125U)
#define OBSTACLE_PWM_DIV_SHIFT (4U)
#define OBSTACLE_PWM_TOP       (19999U)
#define OBSTACLE_PWM_SLICE     (6U)

/* ------------------------------------------------------------------ *
 * Handshake and bypass limits [Default].
 * ------------------------------------------------------------------ */

/* After OBSTACLE, the SCAN_REQ must arrive within 1 s. */
#define OBSTACLE_WAIT_STOP_MS (1000U)

/* After SCAN_RESULT, ACCEPT or REJECT must arrive within 1 s. */
#define OBSTACLE_PLAN_ACCEPT_MS (1000U)

/* The bend-away phase may last 10 s. */
#define OBSTACLE_ESCAPE_MAX_MS (10000U)

/* The rest of the bypass (out, round, back) may last 60 s. The distance
 * budget is worked out from the plan by obstacle_plan_hug_limit_mm(): a
 * fixed 2500 mm cut the U-turn short for rocks wider than about 0.7 m.
 * The plan value is clamped to these bounds. */
#define OBSTACLE_HUG_MAX_MS (60000U)
#define OBSTACLE_HUG_MIN_MM (2500U)
#define OBSTACLE_HUG_MAX_MM (8000U)
#define OBSTACLE_HUG_EXTRA_MM (500U)

/* [Chat] The escape ends after this many clear forward pings in a row
 * (two, so that one noisy reading cannot end the turn early; about 1
 * degree of extra turn). Navigation also keeps turning until the planned
 * start angle is reached (obstacle_nav.c). */
#define OBSTACLE_ESCAPE_CLEAR_PINGS (2U)

/* [Chat] Guard band on the look-ray (obstacle_nav.c HUG loop). The
 * proportional controller below holds OBSTACLE_HUG_TARGET_MM; NEAR is
 * only the hard override that bends the car away on one close ping.
 * [Default] 250 mm: in the closed-loop simulation 150 mm reacted too late
 * (the car curved into 150 and 300 mm deep boxes). The old 350 mm "far"
 * limit belonged to the STEER-word law, which D95 removed.
 * [Chat] Look-ray angle: 60 degrees (the servo limit), raised from 45 for
 * the steeper HUG curve: looking further sideways, the sensor sees the
 * obstacle's side sooner. In the simulation the 13/20 curve hit the box
 * in half the cases at 45 degrees and passed every case at 60. */
#define OBSTACLE_HUG_NEAR_MM    (250U)
#define OBSTACLE_HUG_SERVO_DECI (600)

/* [Default] Clearance the HUG loop holds on the look-ray. The inner wheel
 * speed falls from straight (1000) by up to OBSTACLE_HUG_TURN_PERMILLE (to 200) as the
 * error grows past the deadband, so the car curves smoothly instead of
 * flipping between two thresholds. */
#define OBSTACLE_HUG_TARGET_MM   (300U)
#define OBSTACLE_HUG_DEADBAND_MM (20U)
#define OBSTACLE_HUG_SPAN_MM     (80U)
#define OBSTACLE_HUG_TURN_PERMILLE (800U)

/* ------------------------------------------------------------------ *
 * D95: look ahead while the car moves, and follow a wall in parallel.
 * Every number here is a default chosen so the car works without tuning
 * (the user cannot tune on the car); each one is pinned by a host test.
 * ------------------------------------------------------------------ */

/* [Chat] While the car moves round an obstacle (REACQUIRE), every fourth
 * ping aims the servo straight ahead (the other three at the hug ray). A
 * VALID forward reading under OBSTACLE_FWD_GUARD_MM stops the car and
 * starts a new stationary scan. One look cycle is about 0.65 s: three side
 * pings (60 ms apart) plus two servo moves of about 260 ms each (60 ms
 * pre-delay and 200 ms settle for 60 degrees), about 90 mm of travel at
 * 140 mm/s. The number must be a power of two so the 6-bit ping counter
 * wraps on a whole cycle. */
#define OBSTACLE_LOOK_AHEAD_EVERY (4U)
#define OBSTACLE_FWD_GUARD_MM     (300U)

/* [Default] Value used for an OPEN, UNKNOWN or STUCK side reading in the
 * filter: farther than any echo (12-bit distance maximum). */
#define OBSTACLE_SIDE_FAR_MM (4095U)

/* [Chat] Wall mode (the plan came from the left-wall fallback): pivot left
 * to +90 degrees from the original line, then follow the wall on the right
 * with the heading kept inside +60..+120 degrees. */
#define OBSTACLE_WALL_PIVOT_DECI    (900)
#define OBSTACLE_WALL_HEAD_MIN_DECI (600)
#define OBSTACLE_WALL_HEAD_MAX_DECI (1200)

/* [Default] Wall mode steers by heading. The heading loop runs every 20 ms
 * from the wheel odometry, so it is not delayed by the slow side readings
 * (the servo spends about 0.4 s of every 0.65 s looking ahead). No turn
 * inside +-1 degree of the wanted heading, a full turn
 * (OBSTACLE_HUG_TURN_PERMILLE) at +-15 degrees or more. */
#define OBSTACLE_WALL_HEAD_DEAD_DECI (10)
#define OBSTACLE_WALL_HEAD_SPAN_DECI (150)

/* [Default] The distance to the wall sets the wanted heading: +90 degrees
 * inside the deadband around OBSTACLE_HUG_TARGET_MM; 0.2 degree per mm of
 * error beyond it, leaning away when too close and leaning in when a little
 * too far, at most OBSTACLE_HUG_SPAN_MM (80 mm, so 16 degrees). OPEN,
 * UNKNOWN, STUCK and readings beyond target + deadband + span want exactly
 * +90 degrees: a far or missing echo never turns the car into the wall.
 * Without the lean in, a heading error of 1 degree (2.4 mm/s) would walk
 * the car 140 mm away from the wall in a minute. */
#define OBSTACLE_WALL_LEAN_DECI_PER_MM (2)

/* [Default] Wall mode heading trim. The dead-reckoned heading cannot see a
 * wheel-size or encoder scale error: at 1 % the true heading drifts 0.75
 * degree per second, 45 degrees in a minute, and the car would leave the
 * wall (or run into it) although the heading loop looks perfect. So the
 * wall distance trims the heading: every side ping that reads a wall
 * within the lean range adds OBSTACLE_WALL_TRIM_CDECI_PER_MM hundredths of
 * a deci-degree per mm of distance error (too far raises the trim, too
 * close lowers it), and the trim is added to the heading before the law
 * and the caps see it. With about 4.6 side pings per second this holds a
 * 2 % scale error at 30 mm from the target and stays well damped with the
 * 0.5 s reading delay (host simulation). The trim stops at
 * +-OBSTACLE_WALL_TRIM_MAX_DECI so a wrong echo cannot wind it up. */
#define OBSTACLE_WALL_TRIM_CDECI_PER_MM (11)
#define OBSTACLE_WALL_TRIM_MAX_DECI     (900)

/* [Default] The wall ray (servo at 60 degrees) hits the wall at a slant.
 * The planner turns its reading into the distance to the wall with the
 * cosine of the ray angle off the wall normal, less the beam half angle
 * (the nearest echo in the cone). A ray more than 80 degrees off the
 * normal is almost parallel to the wall and its reading is not trusted. */
#define OBSTACLE_WALL_RAY_MAX_DECI (800)

/* [Chat] Box mode heading caps, measured from the original line, positive
 * away from the obstacle: a turn away stops at +60 degrees (so a long
 * obstacle cannot turn the car round) and a turn towards stops at -60
 * degrees (so a far reading cannot spin the car back into the obstacle). */
#define OBSTACLE_HUG_AWAY_MAX_DECI   (600)
#define OBSTACLE_HUG_TOWARD_MAX_DECI (600)

/* [Default] Box mode: the out-leg ends with a turn back towards the
 * obstacle side until the car is within 5 degrees of parallel to the
 * original line, and only then does HUG start. */
#define OBSTACLE_ALIGN_DONE_DECI (50)

/* [Default] The escape pivot of a box plan is held at 90 degrees: if the
 * forward beam is still not clear then, the car stands still and the 10 s
 * escape limit stops it (a wall plan uses OBSTACLE_WALL_HEAD_MAX_DECI). */
#define OBSTACLE_PIVOT_MAX_DECI (900)

/* ------------------------------------------------------------------ *
 * Car geometry. FILL: measure with a ruler and replace the values.
 * ------------------------------------------------------------------ */

/* [Measured] Widest point of the car is 126 mm (outside of wheel to
 * outside of wheel), so half is 63 mm. */
#define OBSTACLE_CAR_HALF_W_MM (63U)

/* [Measured] Wheel diameter 65 mm. NOTE: The motion service already
 * uses 204 204 um per wheel turn (pi x 65 mm, main.c), so the odometer
 * needs no change. */

/* [Measured] Wheel centre to wheel centre: 107 mm. Used to measure how
 * far the car has turned (obstacle_nav.c); a value that is too small makes the
 * car stop turning a little early. */
#define OBSTACLE_WHEEL_TRACK_MM (107U)

/* FILL: servo shaft to the front bumper, and to the tail of the car. */
#define OBSTACLE_SERVO_TO_FRONT_MM (30U)
#define OBSTACLE_SERVO_TO_REAR_MM  (170U)

/* [Chat] Largest sideways offset a bypass may use. The course has no side
 * boundary, so this is not a wall: it only rejects absurd plans (for
 * example from a bad echo). The planner always picks the side that needs
 * the smaller offset, so real bypasses stay as close to the line as the
 * obstacle allows. */
#define OBSTACLE_FREE_FLOOR_MM (500U)

/* [Chat] Bypass speed. [Measured] 160 mm/s was asked for at first, but at
 * the 25 % duty cap the fast wheel only reached about 142 mm/s while the
 * slow wheel made its 120, so the HUG curve was much wider than planned
 * (about 0.83 instead of 3/4). 140 mm/s is within reach, so the 3/4 curve
 * is really driven. Also used for the odometer estimate when no distance
 * reading is available. NOTE: motion_service_set_speed() rejects
 * (E_PAR) any target above max_target_um_s in main.c, so the car would not
 * move or its lease would expire; that limit must be at least
 * OBSTACLE_NAV_SPEED_UM_S. */
#define OBSTACLE_BYPASS_SPEED_MM_S (140U)

/* [Default] Extra sideways clearance. */
#define OBSTACLE_SIDE_MARGIN_MM (50U)

/* [Tribunal] Assumed obstacle depth. */
#define OBSTACLE_DEPTH_ASSUMED_MM (180U)

/* [Measured] Half of the sensor cone. A power bank 300 mm ahead dropped
 * out with its near edge 50 mm left and 65 mm right of the centre line:
 * atan(50 / 300) = 9.5 and atan(65 / 300) = 12.2 degrees. The smaller
 * value is used, because a narrower cone gives the car more room. */
#define OBSTACLE_BEAM_HALF_DECI (95)

/* ------------------------------------------------------------------ *
 * Phase 2 navigation glue (obstacle_nav.c) [Default]: how the line task drives
 * the motion service while Buddy 5 is in charge.
 * ------------------------------------------------------------------ */

/* Forward speed during the bypass, in micrometres per second. */
#define OBSTACLE_NAV_SPEED_UM_S (OBSTACLE_BYPASS_SPEED_MM_S * 1000U)

/* [Measured] The encoder plugs on the car are crossed relative to
 * src/motion/motion_pins.h (team decision, kept): the motion service's
 * wheel 0 reads the right encoder and wheel 1 the left one. Flight log:
 * both wheel totals fell while the car drove forwards, and the wheels ran
 * at about 190 mm/s when asked for 40. 1 = obstacle_nav.c undoes the swap for
 * its own odometry; 0 = the plugs match motion_pins.h (swapped back on the
 * car, way A). */
#define OBSTACLE_ENCODERS_CROSSED (0)

/* [Chat] Escape turn: pivot, the inside wheel stopped and the outside
 * wheel forwards at OBSTACLE_NAV_SPEED_UM_S (kept as it was with crossed
 * encoders: no wheel ever reverses). */
#define OBSTACLE_NAV_ESCAPE_INSIDE_UM_S (0)

/* [Chat] HUG: the wheel next to the obstacle (or, while the guard turns
 * the car away, the wheel on the other side) runs at INSIDE and the other
 * at OUTSIDE, so the car curves. [Chat] INSIDE is 1/5 of OUTSIDE (28 /
 * 140 mm/s): a near-pivot that keeps moving, so the car hugs the obstacle
 * and the look-ray keeps finding it (team decision; in the simulation
 * curves tighter than 13/20 hit the box in about half the cases). NOTE:
 * Needs working speed control (encoders not crossed); with crossed
 * encoders any driven wheel runs at the duty cap, so INSIDE had to be 0
 * (pivots). FILL: check on the car that the slow wheel keeps turning; a
 * wheel with no encoder edges for 2 s latches the motion service's
 * NO_EDGES fault. */
#define OBSTACLE_NAV_HUG_INSIDE_UM_S  (OBSTACLE_NAV_SPEED_UM_S / 5U)
#define OBSTACLE_NAV_HUG_OUTSIDE_UM_S (OBSTACLE_NAV_SPEED_UM_S)

/* [Chat] The escape pivot goes on until the forward beam is clear AND the
 * car has turned the planned start angle (wheel odometry and
 * OBSTACLE_WHEEL_TRACK_MM), whichever comes later. 573 is deci-degrees per
 * radian. */
#define OBSTACLE_NAV_DECI_PER_RAD (573)

/* HUG anti-windup: the heading caps are OBSTACLE_HUG_AWAY_MAX_DECI and
 * OBSTACLE_HUG_TOWARD_MAX_DECI above. The proportional controller holds
 * OBSTACLE_HUG_TARGET_MM; one ping under OBSTACLE_HUG_NEAR_MM still forces
 * the car away (within the cap). */

/* [Chat] Before any wheel changes direction (end of the escape spin), both
 * wheels stop for this long. While both are at 0 the motion service is
 * idle and does not check the encoders, which rattle while a geared motor
 * brakes (seen on the car as invalid steps and a FEEDBACK fault). */
#define OBSTACLE_NAV_REVERSE_PAUSE_MS (300U)

/* [Chat] Longest wait for the servo to settle after it is moved while the
 * car could drive (entering REACQUIRE or MONITOR). The car stands still
 * meanwhile; normally the wait ends at the next ping (about 0.2 s). */
#define OBSTACLE_NAV_SERVO_WAIT_MS (600U)

/* Lease of every speed command. It must cover several 20 ms loop passes
 * and stay within the motion service limit (200 ms in main.c). */
#define OBSTACLE_NAV_TTL_US (100000U)

/* After OBSTACLE the car is held this long before SCAN_REQ is sent, so
 * that it has stopped before the servo starts to sweep. */
#define OBSTACLE_NAV_BRAKE_MS (300U)

/* After a fault, RESET and START are retried this often. */
#define OBSTACLE_NAV_RETRY_MS (3000U)

/* While looking for the line, LINE_FOUND is sent at most this often. */
#define OBSTACLE_NAV_LINE_REPEAT_MS (100U)

/* [Default] TURNED and LINE_FOUND share one opcode. Navigation repeats
 * TURNED every OBSTACLE_NAV_LINE_REPEAT_MS while it sees BYPASS, so a copy
 * can reach the statechart just after it entered REACQUIRE. A LINE_FOUND in
 * the first OBSTACLE_LINE_GRACE_MS of REACQUIRE is therefore ignored (it is
 * sent again after OBSTACLE_NAV_LINE_REPEAT_MS if the line is really there).
 * Two repeat periods plus one scan-task pass. */
#define OBSTACLE_LINE_GRACE_MS (250U)

/* ------------------------------------------------------------------ *
 * Flight recorder (flight_log.c) [Default].
 * ------------------------------------------------------------------ */

/* Records kept in RAM (16 bytes each; must be a power of two). */
#define OBSTACLE_LOG_RECORDS (1024U)

/* Scan profiles kept in RAM. */
#define OBSTACLE_LOG_PROFILES (8U)

/* A profile belongs to the RESULT record logged within this time. */
#define OBSTACLE_LOG_PROFILE_MATCH_MS (50U)

/* Trace period while Buddy 5 drives the car (BYPASS, REACQUIRE), and
 * otherwise. */
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

/* Lengths of the command queue and of the event outbox. */
#define OBSTACLE_CMD_RING_LEN (4U)
#define OBSTACLE_EVT_RING_LEN (4U)

/* ------------------------------------------------------------------ *
 * Field widths (Change Requirement 1).
 * ------------------------------------------------------------------ */

/* 12-bit distance: 0..4095 mm. */
#define OBSTACLE_DIST_BITS (12U)
#define OBSTACLE_DIST_MAX  ((1U << OBSTACLE_DIST_BITS) - 1U)

/* 10-bit width: 0..1023 mm, where 1023 means "1023 mm or wider". */
#define OBSTACLE_WIDTH_BITS (10U)
#define OBSTACLE_WIDTH_MAX  ((1U << OBSTACLE_WIDTH_BITS) - 1U)

/* 6-bit start angle: 0..60 degrees. */
#define OBSTACLE_ANGLE_BITS (6U)
#define OBSTACLE_ANGLE_MAX  (60U)

/* 5-bit request id: 0..31. */
#define OBSTACLE_ID_BITS (5U)
#define OBSTACLE_ID_MAX  ((1U << OBSTACLE_ID_BITS) - 1U)

/* Other field widths. */
#define OBSTACLE_SLOT_BITS      (5U)
#define OBSTACLE_STATE_COUNT    (7U)
#define OBSTACLE_STATE_BITS     (3U)
#define OBSTACLE_CAUSE_COUNT    (14U)
#define OBSTACLE_CAUSE_BITS     (4U)
#define OBSTACLE_SEQ_BITS       (6U)
#define OBSTACLE_MBX_WIDTH_BITS (16U)

/* Largest VALID distance: 23500 us * 343 / 2000 = 4030 mm. */
#define OBSTACLE_VALID_MAX_MM ((OBSTACLE_VALID_MAX_US * OBSTACLE_SOUND_NUM) / OBSTACLE_SOUND_DEN)

/* Counter limits for the packed counter word (3, 2, 3, 2 and 3 bits). */
#define OBSTACLE_CNT3_MAX (7U)
#define OBSTACLE_CNT2_MAX (3U)

/* Size of the scan profile in bytes. */
#define OBSTACLE_PROFILE_BYTES (50U)

OBSTACLE_CT_ASSERT(dist_fits, OBSTACLE_VALID_MAX_MM <= OBSTACLE_DIST_MAX);
OBSTACLE_CT_ASSERT(region_fits,
             (OBSTACLE_REGION_MAX_MM + OBSTACLE_REGION_EXTRA_MM) <= OBSTACLE_DIST_MAX);
OBSTACLE_CT_ASSERT(mbx_width_fits,
             (OBSTACLE_ECHO_WAIT_MS * 1000U) <= ((1UL << OBSTACLE_MBX_WIDTH_BITS) - 1UL));
OBSTACLE_CT_ASSERT(slots_fit, OBSTACLE_SLOT_COUNT <= (1U << OBSTACLE_SLOT_BITS));
OBSTACLE_CT_ASSERT(states_fit, OBSTACLE_STATE_COUNT <= (1U << OBSTACLE_STATE_BITS));
OBSTACLE_CT_ASSERT(causes_fit, OBSTACLE_CAUSE_COUNT <= (1U << OBSTACLE_CAUSE_BITS));
OBSTACLE_CT_ASSERT(angle_fits, OBSTACLE_ANGLE_MAX < (1U << OBSTACLE_ANGLE_BITS));
OBSTACLE_CT_ASSERT(profile_is_50_bytes,
             OBSTACLE_PROFILE_BYTES == (sizeof(uint16_t) * OBSTACLE_SLOT_COUNT));
OBSTACLE_CT_ASSERT(coarse_ends_on_edge,
             0U == ((OBSTACLE_SLOT_COUNT - 1U) % OBSTACLE_COARSE_STRIDE));
OBSTACLE_CT_ASSERT(floor_fits, OBSTACLE_FREE_FLOOR_MM <= OBSTACLE_WIDTH_MAX);
OBSTACLE_CT_ASSERT(ping_after_wait, OBSTACLE_ECHO_WAIT_MS < OBSTACLE_PING_PERIOD_MS);
OBSTACLE_CT_ASSERT(unknown_cnt_fits, OBSTACLE_NO_ECHO_COUNT <= OBSTACLE_CNT3_MAX);
OBSTACLE_CT_ASSERT(stuck_cnt_fits, OBSTACLE_STUCK_COUNT <= OBSTACLE_CNT2_MAX);
OBSTACLE_CT_ASSERT(hit_cnt_fits, OBSTACLE_DETECT_HITS <= OBSTACLE_CNT3_MAX);
OBSTACLE_CT_ASSERT(coll_cnt_fits, OBSTACLE_COLLISION_HITS <= OBSTACLE_CNT2_MAX);
OBSTACLE_CT_ASSERT(clear_cnt_fits, OBSTACLE_ESCAPE_CLEAR_PINGS <= OBSTACLE_CNT3_MAX);
OBSTACLE_CT_ASSERT(hug_band_order, OBSTACLE_COLLISION_MM < OBSTACLE_HUG_NEAR_MM);
OBSTACLE_CT_ASSERT(hug_target_above_near,
                   OBSTACLE_HUG_NEAR_MM <
                       (OBSTACLE_HUG_TARGET_MM - OBSTACLE_HUG_DEADBAND_MM));
OBSTACLE_CT_ASSERT(hug_deadband_fits,
                   (OBSTACLE_HUG_DEADBAND_MM > 0U) &&
                       (OBSTACLE_HUG_DEADBAND_MM < OBSTACLE_HUG_TARGET_MM));
OBSTACLE_CT_ASSERT(hug_span_fits, OBSTACLE_HUG_SPAN_MM > 0U);
OBSTACLE_CT_ASSERT(hug_turn_fits, (OBSTACLE_HUG_TURN_PERMILLE > 0U) &&
                                      (OBSTACLE_HUG_TURN_PERMILLE <= 1000U));
OBSTACLE_CT_ASSERT(hug_limit_order, OBSTACLE_HUG_MIN_MM < OBSTACLE_HUG_MAX_MM);
OBSTACLE_CT_ASSERT(nav_brake_in_time, OBSTACLE_NAV_BRAKE_MS < OBSTACLE_WAIT_STOP_MS);
OBSTACLE_CT_ASSERT(nav_escape_in_range, (OBSTACLE_NAV_ESCAPE_INSIDE_UM_S >= -100000) &&
                                      (OBSTACLE_NAV_ESCAPE_INSIDE_UM_S <= 0));
OBSTACLE_CT_ASSERT(nav_hug_turns, OBSTACLE_NAV_HUG_INSIDE_UM_S < OBSTACLE_NAV_HUG_OUTSIDE_UM_S);
OBSTACLE_CT_ASSERT(nav_hug_in_range, OBSTACLE_NAV_HUG_OUTSIDE_UM_S <= 200000U);
OBSTACLE_CT_ASSERT(hug_head_caps, (OBSTACLE_HUG_AWAY_MAX_DECI > 0) &&
                                      (OBSTACLE_HUG_AWAY_MAX_DECI <= 900) &&
                                      (OBSTACLE_HUG_TOWARD_MAX_DECI > 0) &&
                                      (OBSTACLE_HUG_TOWARD_MAX_DECI <= 900));
OBSTACLE_CT_ASSERT(look_ahead_pow2, (OBSTACLE_LOOK_AHEAD_EVERY >= 2U) &&
                                        (0U == (OBSTACLE_LOOK_AHEAD_EVERY &
                                                (OBSTACLE_LOOK_AHEAD_EVERY - 1U))) &&
                                        (0U == ((1U << OBSTACLE_SEQ_BITS) %
                                                OBSTACLE_LOOK_AHEAD_EVERY)));
OBSTACLE_CT_ASSERT(fwd_guard_above_detect, OBSTACLE_FWD_GUARD_MM > OBSTACLE_DETECT_MM);
OBSTACLE_CT_ASSERT(fwd_guard_above_collision, OBSTACLE_FWD_GUARD_MM > OBSTACLE_COLLISION_MM);
OBSTACLE_CT_ASSERT(wall_window_order, (OBSTACLE_WALL_HEAD_MIN_DECI < OBSTACLE_WALL_PIVOT_DECI) &&
                                          (OBSTACLE_WALL_PIVOT_DECI < OBSTACLE_WALL_HEAD_MAX_DECI) &&
                                          (OBSTACLE_WALL_HEAD_MAX_DECI <= 1800));
OBSTACLE_CT_ASSERT(wall_lean_inside_window,
                   ((OBSTACLE_HUG_SPAN_MM * OBSTACLE_WALL_LEAN_DECI_PER_MM) <
                    (OBSTACLE_WALL_PIVOT_DECI - OBSTACLE_WALL_HEAD_MIN_DECI)) &&
                       ((OBSTACLE_HUG_SPAN_MM * OBSTACLE_WALL_LEAN_DECI_PER_MM) <
                        (OBSTACLE_WALL_HEAD_MAX_DECI - OBSTACLE_WALL_PIVOT_DECI)));
OBSTACLE_CT_ASSERT(wall_head_band, (OBSTACLE_WALL_HEAD_DEAD_DECI > 0) &&
                                       (OBSTACLE_WALL_HEAD_DEAD_DECI < OBSTACLE_WALL_HEAD_SPAN_DECI));
OBSTACLE_CT_ASSERT(line_grace, OBSTACLE_LINE_GRACE_MS > OBSTACLE_NAV_LINE_REPEAT_MS);
OBSTACLE_CT_ASSERT(wall_trim, (OBSTACLE_WALL_TRIM_CDECI_PER_MM > 0) &&
                                  (OBSTACLE_WALL_TRIM_MAX_DECI > 0) &&
                                  (OBSTACLE_WALL_TRIM_MAX_DECI <= 1800));
OBSTACLE_CT_ASSERT(wall_ray_max, (OBSTACLE_WALL_RAY_MAX_DECI > 0) && (OBSTACLE_WALL_RAY_MAX_DECI < 900));
OBSTACLE_CT_ASSERT(align_done, (OBSTACLE_ALIGN_DONE_DECI >= 0) &&
                                   (OBSTACLE_ALIGN_DONE_DECI < OBSTACLE_HUG_AWAY_MAX_DECI));
OBSTACLE_CT_ASSERT(pivot_max, OBSTACLE_PIVOT_MAX_DECI > ((int32_t)OBSTACLE_ANGLE_MAX * 10));
OBSTACLE_CT_ASSERT(side_far_fits, OBSTACLE_SIDE_FAR_MM == OBSTACLE_DIST_MAX);
OBSTACLE_CT_ASSERT(nav_ttl_in_range, OBSTACLE_NAV_TTL_US <= 200000U);
OBSTACLE_CT_ASSERT(log_pow2, 0U == (OBSTACLE_LOG_RECORDS & (OBSTACLE_LOG_RECORDS - 1U)));
OBSTACLE_CT_ASSERT(log_fits_16bit, OBSTACLE_LOG_RECORDS <= 65536U);

#endif /* OBSTACLE_CFG_H */

/*** end of file ***/

