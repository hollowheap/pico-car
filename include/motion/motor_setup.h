/** @file motor_setup.h
 * @brief The motor service configuration of the car, shared by the firmware
 *        and the bench programs so that they cannot drift apart.
 */
#ifndef MOTION_MOTOR_SETUP_H
#define MOTION_MOTOR_SETUP_H

#include <tk/tkernel.h>

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

#endif /* MOTION_MOTOR_SETUP_H */

/*** end of file ***/
