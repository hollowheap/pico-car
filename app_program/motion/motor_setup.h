/** @file motor_setup.h
 * @brief The motor service configuration of the car, shared by the firmware
 *        and the bench programs so that they cannot drift apart.
 */
#ifndef MOTION_MOTOR_SETUP_H
#define MOTION_MOTOR_SETUP_H

#include <stdbool.h>
#include <tk/typedef.h>

/** Cap on every wheel speed target the controller accepts (um/s). */
#define MOTOR_MAX_TARGET_UM_S (300000U)

/**
 * Configure the motion service with the encoder scale, the speed PID and the
 * time limits of the car. Call once from the startup task, after the GPIO
 * interrupt layer is up.
 *
 * @param[in] motion_tid  The task that runs motion_service_run().
 *
 * @return E_OK, or the error of motion_service_init().
 */
ER motion_setup (ID motion_tid);

/** Hold the motor bridge inputs low and register the task stack. Call first. */
void motion_init (void);

/** Create and start the motion task. Returns false when it could not. */
bool motion_start (void);

/** The ID of the motion task, or 0 before motion_start() created it. */
ID motion_task_id (void);

/**
 * Task body of the motion service. The task created by motion_start() calls
 * this; the task owns the motors until reboot. Never returns.
 */
void motion_service_run (void);

#endif /* MOTION_MOTOR_SETUP_H */

/*** end of file ***/
