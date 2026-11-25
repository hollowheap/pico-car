/** @file flight_log.h
 *
 * @brief Buddy 5 flight recorder: a RAM log of what Buddy 5 and the car
 *        did, printed over USB when a serial monitor is opened.
 *
 * Use from the line task only (extras_nav.c): every function here touches the
 * log without locking, so one task must own it.
 *
 * How to read it after an untethered run: keep the car switched on, plug
 * in USB and open the serial monitor. The whole log is printed once, oldest
 * record first. Reopening the monitor prints it again.
 */

#ifndef EXTRAS_LOG_H
#define EXTRAS_LOG_H

#include <stdbool.h>
#include <stdint.h>

/* Record kinds. The fields each kind uses are listed in extras_log.c. */
#define EXTRAS_LOG_BOOT   (0U)
#define EXTRAS_LOG_STATE  (1U)
#define EXTRAS_LOG_OBST   (2U)
#define EXTRAS_LOG_FAULT  (3U)
#define EXTRAS_LOG_RESULT (4U)
#define EXTRAS_LOG_STEER  (5U)
#define EXTRAS_LOG_MOTION (6U)
#define EXTRAS_LOG_CMD    (7U)
#define EXTRAS_LOG_TRACE  (8U)
#define EXTRAS_LOG_TURNED (9U)
#define EXTRAS_LOG_PHASE  (10U)
#define EXTRAS_LOG_MFAULT (11U)
#define EXTRAS_LOG_SWEEP  (12U)
#define EXTRAS_LOG_PLAN   (13U)
#define EXTRAS_LOG_BUMP   (14U)
#define EXTRAS_LOG_LINE   (15U)
#define EXTRAS_LOG_HALT   (16U)
#define EXTRAS_LOG_MAP    (17U)
#define EXTRAS_LOG_TRACK  (18U)
#define EXTRAS_LOG_KINDS  (19U)

/* One record: 16 bytes. */
typedef struct
{
    uint32_t t_ms;
    uint8_t  kind;
    uint8_t  val_a;
    uint8_t  val_b;
    uint8_t  val_c;
    int16_t  val_d;
    int16_t  val_e;
    int32_t  val_f;
} extras_log_rec_t;

void     extras_log_add (extras_log_rec_t const * p_rec);
uint32_t extras_log_profile (uint8_t req_id, uint16_t const * p_profile);
void     extras_log_service (void);

#endif /* EXTRAS_LOG_H */

/*** end of file ***/
