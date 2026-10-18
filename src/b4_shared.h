/** @file b4_shared.h
 *
 * @brief B4 Task 3: RTOS objects, numeric priorities, shared contracts.
 *
 * RTOS of record: micro T-Kernel 3.0 (IEEE 2050-2018) on the official
 * RP2040 BSP (tron-forum/mtk3_bsp, pico_rp2040 branch). The repo is
 * currently bare-metal (no kernel linked). This header freezes the
 * interface so B4 bring-up works polling-first and ports 1:1 to micro
 * T-Kernel without changing Navigation/Telemetry callers.
 *
 * Numeric priorities (micro T-Kernel: smaller value = higher
 * precedence, 1 is highest; valid range 1..TK_MAX_TSKPRI, BSP default
 * CNF_MAX_TSKPRI = 32):
 *   1 Movement control (10 ms, only motor writer, handles stop signal)
 *   2 Obstacle scan    (50 ms, collision avoidance)
 *   2 Navigation       (event, owns state machine)
 *   3 Line+barcode     (event + 10 ms poll, bar timing by encoder)
 *   4 Hump/B4          (20 ms, this module)
 *   5 Connection       (event, owns MQTT client, reconnect backoff)
 *   6 Telemetry        (1 s + event copies, drops under pressure)
 *
 * Rationale (prof feedback): Connection sits BELOW all sensing so WiFi
 * retries never delay IR sampling or distort bar widths. Hump at 4
 * sits above Connection/Telemetry but below motion-critical tasks.
 * The ordering is unchanged from the earlier design; only the numeric
 * scale is inverted because micro T-Kernel numbers 1 = highest.
 *
 * RTOS objects to create on micro T-Kernel cutover:
 *  - speed_snap_mtx: mutex (cre_mtx, TA_CEILING recommended) guarding
 *    B2 speed_snapshot_t. B2 writes with wai_mtx/rel_mtx; B4/Hump +
 *    Obstacle + Telemetry take copy with pol_mtx (0-block try-lock);
 *    on fail reuse last copy and count a drop. Never block control.
 *  - hump_event_mbf: message buffer (cre_mbf, bufsz = 4 x sizeof
 *    b4_hump_msg_t), B4 -> Navigation (snd_mbf/rcv_mbf). If full:
 *    psnd_mbf fails -> drop the NEW message + increment dropped
 *    counter (report), never block. Data queues (snd_dtq) carry
 *    single-word messages only and cannot hold b4_hump_msg_t; a
 *    data queue may instead carry an index into a static 4-slot ring.
 *  - hump_report_mbf: message buffer (cre_mbf, 4 x sizeof
 *    b4_hump_msg_t), B4 -> Telemetry copy. Same overflow rule.
 *    Navigation buffer is never consumed by Telemetry (design 4.1).
 *  - b4_stop_flag: event flag (cre_flg; set_flg from task portion,
 *    iwup_tsk/isig_sem from task-independent portion) to Movement.
 *    Latched, cleared only by local reset + cause clear.
 *  - Periodic rates: cyclic handlers (cre_cyc/call_cyc) for the 10 ms,
 *    20 ms, 50 ms and 1 s tasks, or dly_tsk loops inside tasks. Raise
 *    CNF_MAX_CYCID (BSP default 4) before creating a 5th handler.
 *  - No ISR in B4 v1: IMU is polled over I2C. If DRDY wired later, ISR
 *    does timestamp + isig_sem only (interrupt API prefix is 'i').
 */

#ifndef B4_SHARED_H
#define B4_SHARED_H

#include <stdbool.h>
#include <stdint.h>

/* micro T-Kernel task priorities: 1 = highest precedence. */
#define B4_PRIO_MOVEMENT   1u
#define B4_PRIO_OBSTACLE   2u
#define B4_PRIO_NAV        2u
#define B4_PRIO_LINE       3u
#define B4_PRIO_HUMP       4u
#define B4_PRIO_CONNECTION 5u
#define B4_PRIO_TELEMETRY  6u

#define B4_HUMP_Q_LEN      4u
#define B4_REPORT_Q_LEN    4u

/* Telemetry topic payload agreed with B1 (JSON over MQTT). */
typedef struct
{
    uint32_t run_id;
    uint32_t event_id;
    uint32_t timestamp_ms;
    float    peak_height_mm;
    float    peak_pitch_deg;
    float    run_peak_mm;
    uint8_t  state;
    uint8_t  motion;
    uint8_t  fault;
} b4_hump_msg_t;

/*!
 * @brief Pack hump result into telemetry message (no heap, no block).
 *
 * @param[in]  run_id       Current run ID from Navigation.
 * @param[in]  event_id     Monotonic event counter (B4-owned).
 * @param[in]  now_ms       Timestamp in ms.
 * @param[in]  peak_mm      Window or run peak in mm.
 * @param[in]  pitch_deg    Peak pitch in deg.
 * @param[in]  run_peak_mm  Highest hump this run.
 * @param[in]  state        hump_state_t value.
 * @param[in]  motion       motion_event_t value.
 * @param[in]  fault        hump_fault_t value.
 * @param[out] p_msg        Packed message.
 */
void b4_pack_msg (uint32_t run_id, uint32_t event_id, uint32_t now_ms,
                  float peak_mm, float pitch_deg, float run_peak_mm,
                  uint8_t state, uint8_t motion, uint8_t fault,
                  b4_hump_msg_t * p_msg);

#endif /* B4_SHARED_H */

/*** end of file ***/
