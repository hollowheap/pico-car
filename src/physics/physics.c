/** @file physics.c
 *
 * @brief Physics facade: start the task and publish a telemetry-ready
 *        snapshot. The packed hump word stays private.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "motion_glue.h"
#include "physics_task.h"
#include "hump_msg.h"
#include "physics/physics.h"

#define MM_PER_M             (1000.0f)
#define ROUND_HALF           (0.5f)

ER physics_init (ID task_id)
{
    physics_task_config_t cfg;

    physics_car_config(&cfg);
    cfg.read_motion = motion_glue_read;

    return (physics_task_init(&cfg, task_id));
}

void physics_run (void)
{
    physics_task_run();
}

ER physics_snapshot (physics_status_t * p_status)
{
    physics_task_stats_t  stats;
    hump_fields_t fields;
    ER               result = E_PAR;

    if (NULL != p_status)
    {
        physics_task_get_stats(&stats);

        p_status->cycles   = stats.cycles;
        p_status->overruns = stats.overruns;
        p_status->ready    = (stats.cycles > 0U) &&
                             hump_unpack(stats.last_word, &fields);
        p_status->calibrated = stats.calibrated;

        if (p_status->ready)
        {
            p_status->state        = (uint8_t)fields.state;
            p_status->motion       = (uint8_t)fields.motion;
            p_status->faults       = fields.faults;
            p_status->pitch_cdeg   = (int16_t)(fields.pitch_deg * 100.0f);
            p_status->heading_ddeg = stats.heading_valid ? stats.heading_ddeg
                                                         : 0U;
            p_status->yaw_rate_cdps = stats.yaw_rate_cdps;
            p_status->a_fwd_mg      = stats.a_fwd_mg;
            p_status->roll_cdeg     = stats.roll_cdeg;
            p_status->az_mg         = stats.az_mg;
            p_status->amag_mg       = stats.amag_mg;
            p_status->attitude_state = stats.attitude_state;
            p_status->heading_valid = stats.heading_valid;
            p_status->hump_peak_mm =
                (uint16_t)((fields.hump_peak_m * MM_PER_M) + ROUND_HALF);
            p_status->run_peak_mm =
                (uint16_t)((fields.run_peak_m * MM_PER_M) + ROUND_HALF);
            p_status->hump_active  = fields.hump_active;
            p_status->finished     = fields.finished;
            p_status->height_valid = fields.height_valid;
        }
        else
        {
            p_status->state        = 0U;
            p_status->motion       = 0U;
            p_status->faults       = 0U;
            p_status->calibrated   = false;
            p_status->pitch_cdeg   = 0;
            p_status->heading_ddeg = 0U;
            p_status->yaw_rate_cdps = 0;
            p_status->a_fwd_mg      = 0;
            p_status->roll_cdeg     = 0;
            p_status->az_mg         = 0;
            p_status->amag_mg       = 0U;
            p_status->attitude_state = 0U;
            p_status->heading_valid = false;
            p_status->hump_peak_mm = 0U;
            p_status->run_peak_mm  = 0U;
            p_status->hump_active  = false;
            p_status->finished     = false;
            p_status->height_valid = false;
        }

        result = E_OK;
    }

    return (result);
}

void physics_request_new_run (void)
{
    physics_task_request_new_run();
}

void physics_set_wheels_free (bool b_free)
{
    physics_task_set_wheels_free(b_free);
}

void physics_request_clear_fault (void)
{
    physics_task_request_clear_fault();
}

/*** end of file ***/
