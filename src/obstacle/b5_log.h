/** @file b5_log.h
 *
 * @brief Buddy 5 flight recorder: a RAM log of what Buddy 5 and the car
 *        did, printed over USB when a serial monitor is opened.
 *
 * Use from the line task only (b5_nav.c): every function here touches the
 * log without locking, so one task must own it.
 *
 * How to read it after an untethered run: keep the car switched on, plug
 * in USB and open the serial monitor. The whole log is printed once, oldest
 * record first. Reopening the monitor prints it again.
 */

#ifndef B5_LOG_H
#define B5_LOG_H

#include <stdbool.h>
#include <stdint.h>

/* Record kinds. The fields each kind uses are listed in b5_log.c. */
#define B5_LOG_BOOT   (0U)
#define B5_LOG_STATE  (1U)
#define B5_LOG_OBST   (2U)
#define B5_LOG_FAULT  (3U)
#define B5_LOG_RESULT (4U)
#define B5_LOG_STEER  (5U)
#define B5_LOG_MOTION (6U)
#define B5_LOG_CMD    (7U)
#define B5_LOG_TRACE  (8U)
#define B5_LOG_TURNED (9U)
#define B5_LOG_PHASE  (10U)
#define B5_LOG_MFAULT (11U)
#define B5_LOG_KINDS  (12U)

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
} b5_log_rec_t;

void     b5_log_add (b5_log_rec_t const * p_rec);
uint32_t b5_log_profile (uint8_t req_id, uint16_t const * p_profile);
void     b5_log_service (void);

#endif /* B5_LOG_H */

/*** end of file ***/

