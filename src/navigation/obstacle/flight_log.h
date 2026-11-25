/** @file flight_log.h
 *
 * @brief Buddy 5 flight recorder: a RAM log of what Buddy 5 and the car
 *        did, printed over USB when a serial monitor is opened.
 *
 * Use from the line task only (obstacle_nav.c): every function here touches the
 * log without locking, so one task must own it.
 *
 * How to read it after an untethered run: keep the car switched on, plug
 * in USB and open the serial monitor. The whole log is printed once, oldest
 * record first. Reopening the monitor prints it again.
 */

#ifndef OBSTACLE_LOG_H
#define OBSTACLE_LOG_H

#include <stdbool.h>
#include <stdint.h>

/* Record kinds. The fields each kind uses are listed in flight_log.c. */
#define OBSTACLE_LOG_BOOT   (0U)
#define OBSTACLE_LOG_STATE  (1U)
#define OBSTACLE_LOG_OBST   (2U)
#define OBSTACLE_LOG_FAULT  (3U)
#define OBSTACLE_LOG_RESULT (4U)
#define OBSTACLE_LOG_STEER  (5U)
#define OBSTACLE_LOG_MOTION (6U)
#define OBSTACLE_LOG_CMD    (7U)
#define OBSTACLE_LOG_TRACE  (8U)
#define OBSTACLE_LOG_TURNED (9U)
#define OBSTACLE_LOG_PHASE  (10U)
#define OBSTACLE_LOG_MFAULT (11U)
#define OBSTACLE_LOG_GUARD  (12U)
#define OBSTACLE_LOG_KINDS  (13U)

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
} flight_log_rec_t;

void     flight_log_add (flight_log_rec_t const * p_rec);
uint32_t flight_log_profile (uint8_t req_id, uint16_t const * p_profile);
void     flight_log_service (void);

#endif /* OBSTACLE_LOG_H */

/*** end of file ***/

