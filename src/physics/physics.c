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

ER physics_init (ID task_id)
{
    physics_task_config_t cfg;

    physics_task_default_config(&cfg);
    /* The GY-511 sits with X pointing aft on the car, so nose-up reads
     * negative. Flip the forward axis so the hump state machine and the
     * compass use the car's forward direction. The logged humps peak near
     * 5 deg above rest, so enter at 4 deg with a 2 deg exit band. */
    cfg.hump.fwd_sign = -1;
    cfg.hump.enter_deg = 4.0f;
    cfg.hump.exit_band_deg = 2.0f;
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
        p_status->dropped  = stats.nav_dropped + stats.telem_dropped;
        p_status->ready    = (stats.cycles > 0U) &&
                             hump_unpack(stats.last_word, &fields);

        if (p_status->ready)
        {
            p_status->state        = (uint8_t)fields.state;
            p_status->motion       = (uint8_t)fields.motion;
            p_status->faults       = fields.faults;
            p_status->pitch_cdeg   = (int16_t)(fields.pitch_deg * 100.0f);
            p_status->heading_deg  = stats.heading_valid ? stats.heading_deg
                                                         : 0U;
            p_status->heading_valid = stats.heading_valid;
            p_status->hump_peak_mm =
                (uint16_t)((fields.hump_peak_m * 1000.0f) + 0.5f);
            p_status->run_peak_mm =
                (uint16_t)((fields.run_peak_m * 1000.0f) + 0.5f);
            p_status->hump_active  = fields.hump_active;
            p_status->finished     = fields.finished;
            p_status->height_valid = fields.height_valid;
        }
        else
        {
            p_status->state        = 0U;
            p_status->motion       = 0U;
            p_status->faults       = 0U;
            p_status->pitch_cdeg   = 0;
            p_status->heading_deg  = 0U;
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

void physics_request_clear_fault (void)
{
    physics_task_request_clear_fault();
}

/*** end of file ***/
