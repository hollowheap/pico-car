/** @file b4_log.c
 *
 * @brief CSV logger implementation (USB stdio, no heap).
 */

#include <stdio.h>

#include "b4_log.h"
#include "hump.h"
#include "imu_lsm303.h"

void b4_log_header (void)
{
    printf("ms,ax_g,ay_g,az_g,pitch_raw,pitch_filt,speed_mps,state,win_mm,"
           "run_peak_mm,motion,fault\n");
}

void b4_log_sample (unsigned long now_ms, const imu_accel_g_t * p_accel,
                    float pitch_raw, float pitch_filt,
                    const speed_snapshot_t * p_speed,
                    const hump_result_t * p_res)
{
    float speed = 0.0f;
    int   state = 0;
    float win   = 0.0f;
    float run   = 0.0f;
    int   mot   = 0;
    int   flt   = 0;

    if ((p_accel == NULL) || (p_speed == NULL) || (p_res == NULL))
    {
        return;
    }

    speed = p_speed->forward_mps;
    state = (int) p_res->state;
    win   = p_res->peak_height_mm;
    run   = p_res->run_peak_height_mm;
    mot   = (int) p_res->motion;
    flt   = (int) p_res->fault;

    printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%d,%.1f,%.1f,%d,%d\n", now_ms,
           (double) p_accel->x_g, (double) p_accel->y_g,
           (double) p_accel->z_g, (double) pitch_raw, (double) pitch_filt,
           (double) speed, state, (double) win, (double) run, mot, flt);
}

/*** end of file ***/
