/** @file b5_cfg.h
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

#ifndef B5_CFG_H
#define B5_CFG_H

#include <stdint.h>

/* C99 has no _Static_assert, so a negative array size stops the build when
 * a condition is false. NOTE: This macro cannot be a function because it
 * must act at compile time (BARR-C 6.3.a). */
#define B5_CT_ASSERT(name, cond)                                               \
    typedef uint8_t b5_ct_##name##_t[(cond) ? 1 : -1]

/* ------------------------------------------------------------------ *
 * Pins on the Cytron Robo Pico [DR 2].
 * ------------------------------------------------------------------ */

/* Grove 1, GPIO output to the sensor trigger. */
#define B5_TRIG_PIN (0U)

/* Grove 1, GPIO input from the sensor echo, both-edge interrupt. */
#define B5_ECHO_PIN (1U)

/* Servo port 1, PWM slice 6 channel A. */
#define B5_SERVO_PIN (12U)

/* ------------------------------------------------------------------ *
 * Ping timing [Spec] [HC-SR04 datasheet].
 * ------------------------------------------------------------------ */

/* At least 60 ms between triggers, so old echoes die out. */
#define B5_PING_PERIOD_MS (60U)

/* The task waits at most this long for the end of an echo. */
#define B5_ECHO_WAIT_MS (40U)

/* The trigger pulse must be at least 10 us long. */
#define B5_TRIG_PULSE_US (12U)

/* Echo widths from 116 us (about 20 mm) to 23500 us (about 4 m) are
 * VALID readings. */
#define B5_VALID_MIN_US (116U)
#define B5_VALID_MAX_US (23500U)

/* Distance in mm is width_us * 343 / 2000 (speed of sound 343 m/s). */
#define B5_SOUND_NUM (343U)
#define B5_SOUND_DEN (2000U)

/* The 1 MHz TIMER must advance at least 25 ticks across a 50 us wait. */
#define B5_TIMER_CHECK_US        (50U)
#define B5_TIMER_CHECK_MIN_TICKS (25U)

/* ------------------------------------------------------------------ *
 * Faults [Spec].
 * ------------------------------------------------------------------ */

/* This many UNKNOWN pings in a row raise NO_ECHO. */
#define B5_NO_ECHO_COUNT (5U)

/* This many STUCK pings in a row raise ECHO_STUCK. */
#define B5_STUCK_COUNT (3U)

/* The whole scan must finish within 8 s. */
#define B5_SCAN_TIMEOUT_MS (8000U)

/* ------------------------------------------------------------------ *
 * Monitor ahead and the collision guard.
 * ------------------------------------------------------------------ */

/* [Default] The Tribunal stops the car about 200 mm away; 50 mm more
 * leaves room for two pings and braking. */
#define B5_DETECT_MM (250U)

/* [Spec] Consecutive VALID hits needed to report an obstacle. */
#define B5_DETECT_HITS (2U)

/* [Default] Readings closer than this call the emergency stop. */
#define B5_COLLISION_MM (80U)

/* [Default] Consecutive close readings needed for the emergency stop. */
#define B5_COLLISION_HITS (2U)

/* ------------------------------------------------------------------ *
 * Scan grid [Spec].
 * ------------------------------------------------------------------ */

/* 25 slots from -60 to +60 degrees in 5 degree steps; slot 12 is ahead. */
#define B5_SLOT_COUNT      (25U)
#define B5_SLOT_CENTRE     (12U)
#define B5_SLOT_STEP_DECI  (50)
#define B5_SLOT_FIRST_DECI (-600)

/* The coarse scan measures every third slot. */
#define B5_COARSE_STRIDE (3U)

/* [Spec 3c] The region is the closest VALID return within 600 mm plus
 * neighbouring coarse slots within closest + 250 mm. */
#define B5_REGION_MAX_MM   (600U)
#define B5_REGION_EXTRA_MM (250U)

/* [Spec 3e] Two pings agree when within 15 mm; four pings without
 * agreement make the slot UNKNOWN. */
#define B5_AGREE_MM  (15U)
#define B5_MAX_PINGS (4U)

/* [Chat][Default] The car's path: +-30 degrees of straight ahead. CLEAR
 * needs a real reading in every slot of it, and the scan looks for the
 * closest reading here before it looks at the whole sweep. */
#define B5_PATH_HALF_DECI (300)

/* ------------------------------------------------------------------ *
 * Servo (SG90 on servo port 1). The datasheet gives both "1-2 ms for
 * +-90 degrees" and "500-2400 us for 180 degrees". The narrow scale
 * below cannot reach the end stops. Calibrate with checklist step H2.
 * ------------------------------------------------------------------ */

/* FILL: pulse width that points the sensor straight ahead. */
#define B5_SERVO_CENTRE_US (1500U)

/* FILL: microseconds per 10 degrees (a safe 5.6 us per degree guess). */
#define B5_SERVO_US_PER_10DEG (56U)

/* FILL: +1 if a longer pulse turns the sensor left, -1 otherwise. */
#define B5_SERVO_DIR (1)

/* [Spec] The scan covers +-60 degrees. */
#define B5_SERVO_LIMIT_DECI (600)

/* Never command a pulse outside the datasheet range. */
#define B5_SERVO_MIN_US (500U)
#define B5_SERVO_MAX_US (2400U)

/* [Spec] Settle 3 ms per degree plus 20 ms. The datasheet speed is
 * 1.7-2.0 ms per degree at 4.8 V; the margin covers LiPo sag, and the
 * 20 ms covers one PWM frame. FILL: power source (LiPo, powerbank or
 * VIN). Below 4.0 V the SG90 is out of spec, so re-check on a low
 * battery. */
#define B5_SETTLE_MS_PER_DEG (3U)
#define B5_SETTLE_BASE_MS    (20U)

/* [Chat] In states where the car may be driving, the servo waits this long
 * before it moves, so that Navigation (20 ms loop) and the motion service
 * (10 ms) have stopped the wheels: a servo swing while a motor drives
 * disturbs the wheel encoders (seen on the car as a FEEDBACK fault). */
#define B5_SERVO_PREDELAY_MS (60U)

/* PWM: 125 MHz / 125 = 1 MHz count and TOP 19999 give 50 Hz. CHx_DIV is
 * an 8.4 fixed-point value. */
#define B5_PWM_DIV_INT   (125U)
#define B5_PWM_DIV_SHIFT (4U)
#define B5_PWM_TOP       (19999U)
#define B5_PWM_SLICE     (6U)

/* ------------------------------------------------------------------ *
 * Handshake and bypass limits [Default].
 * ------------------------------------------------------------------ */

/* After OBSTACLE, the SCAN_REQ must arrive within 1 s. */
#define B5_WAIT_STOP_MS (1000U)

/* After SCAN_RESULT, ACCEPT or REJECT must arrive within 1 s. */
#define B5_PLAN_ACCEPT_MS (1000U)

/* The bend-away phase may last 10 s. */
#define B5_ESCAPE_MAX_MS (10000U)

/* The rest of the bypass (out, along, back) may last 60 s or 2500 mm
 * before the line must be found. At 40-50 mm/s a bypass of a 10 cm
 * obstacle is about 1 m and 20-25 s. */
#define B5_HUG_MAX_MS (60000U)
#define B5_HUG_MAX_MM (2500U)

/* [Chat] The escape ends after this many clear forward pings in a row
 * (two, so that one noisy reading cannot end the turn early; about 1
 * degree of extra turn). Navigation also keeps turning until the planned
 * start angle is reached (b5_nav.c). */
#define B5_ESCAPE_CLEAR_PINGS (2U)

/* [Chat] Zig-zag band on the 45 degree look-ray: closer than NEAR bends
 * away, further than FAR (or OPEN) bends towards. 150 / 250 mm on the ray
 * is about 106 / 177 mm sideways from the sensor, leaving room for the
 * tail to swing when the car bends away. */
#define B5_HUG_NEAR_MM    (150U)
#define B5_HUG_FAR_MM     (250U)
#define B5_HUG_SERVO_DECI (450)

/* NOTE: Navigation (b5_nav.c) steers REACQUIRE from the wheel odometry
 * and uses only B5_HUG_NEAR_MM (bend away when closer); the STEER words
 * from the band above are published but not used for driving. */

/* ------------------------------------------------------------------ *
 * Car geometry. FILL: measure with a ruler and replace the values.
 * ------------------------------------------------------------------ */

/* [Measured] Widest point of the car is 126 mm (outside of wheel to
 * outside of wheel), so half is 63 mm. */
#define B5_CAR_HALF_W_MM (63U)

/* [Measured] Wheel diameter 65 mm. NOTE: The motion service already
 * uses 204 204 um per wheel turn (pi x 65 mm, main.c), so the odometer
 * needs no change. */

/* [Measured] Wheel centre to wheel centre: 107 mm. Used to measure how
 * far the car has turned (b5_nav.c); a value that is too small makes the
 * car stop turning a little early. */
#define B5_WHEEL_TRACK_MM (107U)

/* FILL: servo shaft to the front bumper, and to the tail of the car. */
#define B5_SERVO_TO_FRONT_MM (30U)
#define B5_SERVO_TO_REAR_MM  (170U)

/* [Chat] Largest sideways offset a bypass may use. The course has no side
 * boundary, so this is not a wall: it only rejects absurd plans (for
 * example from a bad echo). The planner always picks the side that needs
 * the smaller offset, so real bypasses stay as close to the line as the
 * obstacle allows. */
#define B5_FREE_FLOOR_MM (500U)

/* FILL: Buddy 2's bypass speed. It is used only when no distance reading
 * is available. */
#define B5_BYPASS_SPEED_MM_S (40U)

/* [Default] Extra sideways clearance. */
#define B5_SIDE_MARGIN_MM (50U)

/* [Tribunal] Assumed obstacle depth. */
#define B5_DEPTH_ASSUMED_MM (180U)

/* [Measured] Half of the sensor cone. A power bank 300 mm ahead dropped
 * out with its near edge 50 mm left and 65 mm right of the centre line:
 * atan(50 / 300) = 9.5 and atan(65 / 300) = 12.2 degrees. The smaller
 * value is used, because a narrower cone gives the car more room. */
#define B5_BEAM_HALF_DECI (95)

/* ------------------------------------------------------------------ *
 * Phase 2 navigation glue (b5_nav.c) [Default]: how the line task drives
 * the motion service while Buddy 5 is in charge.
 * ------------------------------------------------------------------ */

/* Forward speed during the bypass, in micrometres per second. It matches
 * the 40 mm/s used for the odometer estimate (main.c uses 40 841 um/s). */
#define B5_NAV_SPEED_UM_S (B5_BYPASS_SPEED_MM_S * 1000U)

/* [Chat] Escape turn: spin on the spot, the inside wheel backwards at this
 * speed and the outside wheel forwards at B5_NAV_SPEED_UM_S. NOTE: With the
 * inside wheel stopped instead (a pivot), the car scrubbed the held wheel
 * backwards (-31 and -56 mm/s seen on the car); the geared motor chattered,
 * the encoder reported invalid steps and the motion service latched its
 * FEEDBACK fault. A spin drives both wheels, so none is dragged. */
#define B5_NAV_ESCAPE_INSIDE_UM_S (-40000)

/* [Chat] Zig-zag (option A): the inside wheel keeps B5_NAV_SPEED_UM_S and
 * the outside wheel runs faster, so the car curves gently (about 250 mm
 * radius with a 100 mm track). NOTE: Slower inside speeds are avoided: the
 * motors only get up to 25 % duty, and a driven wheel with no encoder
 * edges for 2 s latches the motion service's NO_EDGES fault (reboot only).
 * If the motors cannot reach 60 mm/s, the outside wheel just runs at its
 * top speed, which still curves the car. FILL: check the speeds in the
 * flight log (wheel distances in the trace lines). */
#define B5_NAV_HUG_OUTSIDE_UM_S (60000U)

/* [Chat] The escape pivot goes on until the forward beam is clear AND the
 * car has turned the planned start angle (wheel odometry and
 * B5_WHEEL_TRACK_MM), whichever comes later. 573 is deci-degrees per
 * radian. */
#define B5_NAV_DECI_PER_RAD (573)

/* [Default] Zig-zag along the obstacle (ALONG phase): the car keeps its
 * sideways offset between (planned - IN) and (planned + OUT) mm, and its
 * heading within +-HEAD_MAX of the line. */
#define B5_NAV_BAND_IN_MM    (10U)
#define B5_NAV_BAND_OUT_MM   (30U)
#define B5_NAV_HEAD_MAX_DECI (300)

/* [Default] BACK phase: the car curves towards the line until it heads
 * this many deci-degrees across it, then drives straight to the line, so
 * it meets the line at a shallow angle and does not overshoot. */
#define B5_NAV_REJOIN_DECI (300)

/* [Chat] Before any wheel changes direction (end of the escape spin), both
 * wheels stop for this long. While both are at 0 the motion service is
 * idle and does not check the encoders, which rattle while a geared motor
 * brakes (seen on the car as invalid steps and a FEEDBACK fault). */
#define B5_NAV_REVERSE_PAUSE_MS (300U)

/* [Chat] Longest wait for the servo to settle after it is moved while the
 * car could drive (entering REACQUIRE or MONITOR). The car stands still
 * meanwhile; normally the wait ends at the next ping (about 0.2 s). */
#define B5_NAV_SERVO_WAIT_MS (600U)

/* Lease of every speed command. It must cover several 20 ms loop passes
 * and stay within the motion service limit (200 ms in main.c). */
#define B5_NAV_TTL_US (100000U)

/* After OBSTACLE the car is held this long before SCAN_REQ is sent, so
 * that it has stopped before the servo starts to sweep. */
#define B5_NAV_BRAKE_MS (300U)

/* After a fault, RESET and START are retried this often. */
#define B5_NAV_RETRY_MS (3000U)

/* While looking for the line, LINE_FOUND is sent at most this often. */
#define B5_NAV_LINE_REPEAT_MS (100U)

/* ------------------------------------------------------------------ *
 * Flight recorder (b5_log.c) [Default].
 * ------------------------------------------------------------------ */

/* Records kept in RAM (16 bytes each; must be a power of two). */
#define B5_LOG_RECORDS (1024U)

/* Scan profiles kept in RAM. */
#define B5_LOG_PROFILES (8U)

/* A profile belongs to the RESULT record logged within this time. */
#define B5_LOG_PROFILE_MATCH_MS (50U)

/* Trace period while Buddy 5 drives the car (BYPASS, REACQUIRE), and
 * otherwise. */
#define B5_LOG_TRACE_MS      (100U)
#define B5_LOG_IDLE_TRACE_MS (500U)

/* Dump pacing: lines per line-task pass, and the most bytes left in the
 * 4 KB USB ring before another line is printed. */
#define B5_LOG_LINES_PER_PASS (4U)
#define B5_LOG_PENDING_MAX    (1024U)

/* ------------------------------------------------------------------ *
 * Kernel resources [Chat].
 * ------------------------------------------------------------------ */

/* Below motion control (5) and above line following (9). */
#define B5_TASK_PRI (6)

/* The team uses 4096 bytes per task under SMP. */
#define B5_TASK_STACK (4096U)

/* Lengths of the command queue and of the event outbox. */
#define B5_CMD_RING_LEN (4U)
#define B5_EVT_RING_LEN (4U)

/* ------------------------------------------------------------------ *
 * Field widths (Change Requirement 1).
 * ------------------------------------------------------------------ */

/* 12-bit distance: 0..4095 mm. */
#define B5_DIST_BITS (12U)
#define B5_DIST_MAX  ((1U << B5_DIST_BITS) - 1U)

/* 10-bit width: 0..1023 mm, where 1023 means "1023 mm or wider". */
#define B5_WIDTH_BITS (10U)
#define B5_WIDTH_MAX  ((1U << B5_WIDTH_BITS) - 1U)

/* 6-bit start angle: 0..60 degrees. */
#define B5_ANGLE_BITS (6U)
#define B5_ANGLE_MAX  (60U)

/* 5-bit request id: 0..31. */
#define B5_ID_BITS (5U)
#define B5_ID_MAX  ((1U << B5_ID_BITS) - 1U)

/* Other field widths. */
#define B5_SLOT_BITS      (5U)
#define B5_STATE_COUNT    (7U)
#define B5_STATE_BITS     (3U)
#define B5_CAUSE_COUNT    (14U)
#define B5_CAUSE_BITS     (4U)
#define B5_SEQ_BITS       (6U)
#define B5_MBX_WIDTH_BITS (16U)

/* Largest VALID distance: 23500 us * 343 / 2000 = 4030 mm. */
#define B5_VALID_MAX_MM ((B5_VALID_MAX_US * B5_SOUND_NUM) / B5_SOUND_DEN)

/* Counter limits for the packed counter word (3, 2, 3, 2 and 3 bits). */
#define B5_CNT3_MAX (7U)
#define B5_CNT2_MAX (3U)

/* Size of the scan profile in bytes. */
#define B5_PROFILE_BYTES (50U)

B5_CT_ASSERT(dist_fits, B5_VALID_MAX_MM <= B5_DIST_MAX);
B5_CT_ASSERT(region_fits,
             (B5_REGION_MAX_MM + B5_REGION_EXTRA_MM) <= B5_DIST_MAX);
B5_CT_ASSERT(mbx_width_fits,
             (B5_ECHO_WAIT_MS * 1000U) <= ((1UL << B5_MBX_WIDTH_BITS) - 1UL));
B5_CT_ASSERT(slots_fit, B5_SLOT_COUNT <= (1U << B5_SLOT_BITS));
B5_CT_ASSERT(states_fit, B5_STATE_COUNT <= (1U << B5_STATE_BITS));
B5_CT_ASSERT(causes_fit, B5_CAUSE_COUNT <= (1U << B5_CAUSE_BITS));
B5_CT_ASSERT(angle_fits, B5_ANGLE_MAX < (1U << B5_ANGLE_BITS));
B5_CT_ASSERT(profile_is_50_bytes,
             B5_PROFILE_BYTES == (sizeof(uint16_t) * B5_SLOT_COUNT));
B5_CT_ASSERT(coarse_ends_on_edge,
             0U == ((B5_SLOT_COUNT - 1U) % B5_COARSE_STRIDE));
B5_CT_ASSERT(floor_fits, B5_FREE_FLOOR_MM <= B5_WIDTH_MAX);
B5_CT_ASSERT(ping_after_wait, B5_ECHO_WAIT_MS < B5_PING_PERIOD_MS);
B5_CT_ASSERT(unknown_cnt_fits, B5_NO_ECHO_COUNT <= B5_CNT3_MAX);
B5_CT_ASSERT(stuck_cnt_fits, B5_STUCK_COUNT <= B5_CNT2_MAX);
B5_CT_ASSERT(hit_cnt_fits, B5_DETECT_HITS <= B5_CNT3_MAX);
B5_CT_ASSERT(coll_cnt_fits, B5_COLLISION_HITS <= B5_CNT2_MAX);
B5_CT_ASSERT(clear_cnt_fits, B5_ESCAPE_CLEAR_PINGS <= B5_CNT3_MAX);
B5_CT_ASSERT(hug_band_order, B5_COLLISION_MM < B5_HUG_NEAR_MM);
B5_CT_ASSERT(hug_band_order2, B5_HUG_NEAR_MM < B5_HUG_FAR_MM);
B5_CT_ASSERT(nav_brake_in_time, B5_NAV_BRAKE_MS < B5_WAIT_STOP_MS);
B5_CT_ASSERT(nav_escape_in_range, (B5_NAV_ESCAPE_INSIDE_UM_S >= -100000) &&
                                      (B5_NAV_ESCAPE_INSIDE_UM_S <= 0));
B5_CT_ASSERT(nav_hug_faster, B5_NAV_HUG_OUTSIDE_UM_S > B5_NAV_SPEED_UM_S);
B5_CT_ASSERT(nav_hug_in_range, B5_NAV_HUG_OUTSIDE_UM_S <= 100000U);
B5_CT_ASSERT(nav_band_order, B5_NAV_BAND_IN_MM < B5_NAV_BAND_OUT_MM);
B5_CT_ASSERT(nav_ttl_in_range, B5_NAV_TTL_US <= 200000U);
B5_CT_ASSERT(log_pow2, 0U == (B5_LOG_RECORDS & (B5_LOG_RECORDS - 1U)));
B5_CT_ASSERT(log_fits_16bit, B5_LOG_RECORDS <= 65536U);

#endif /* B5_CFG_H */

/*** end of file ***/

