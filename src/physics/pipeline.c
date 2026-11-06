/** @file pipeline.c
 *
 * @brief One 20 ms Buddy 4 cycle: sensor values in, packed messages out.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pipeline.h"
#include "hump_detect.h"
#include "hump_msg.h"
#include "imu_motion.h"

bool pipeline_init (pipeline_t * p_pipe,
                       const hump_config_t * p_hump_cfg,
                       const imu_motion_config_t * p_motion_cfg,
                       uint32_t telem_every)
{
    pipeline_t fresh;

    if ((NULL == p_pipe) || (NULL == p_hump_cfg) || (NULL == p_motion_cfg)
        || (telem_every < 1U))
    {
        return false;
    }

    (void)memset (&fresh, 0, sizeof (fresh));
    if (!hump_init (&fresh.hump, p_hump_cfg)
        || !imu_motion_init (&fresh.motion, p_motion_cfg, p_hump_cfg))
    {
        return false;
    }
    hump_run_init (&fresh.run);
    fresh.telem_every = telem_every;
    fresh.last_motion = MOTION_CLASS_UNKNOWN;

    *p_pipe = fresh;
    return true;
}

void pipeline_new_run (pipeline_t * p_pipe)
{
    if (NULL != p_pipe)
    {
        hump_recalibrate (&p_pipe->hump);
        hump_run_init (&p_pipe->run);
    }
}

/* Message for a cycle with no accelerometer data: the last message with the
 * I2C fault set, no event flags, and the height marked not valid. */
static packed_hump_t fault_word (const pipeline_t * p_pipe)
{
    hump_fields_t fields;
    packed_hump_t word = 0U;

    (void)memset (&fields, 0, sizeof (fields));
    if (p_pipe->have_last)
    {
        (void)hump_unpack (p_pipe->last_word, &fields);
    }
    fields.faults = (uint8_t)(fields.faults | PHYSICS_FAULT_I2C);
    fields.finished = false;
    fields.height_valid = false;
    (void)hump_pack (&fields, &word);

    return word;
}

bool pipeline_step (pipeline_t * p_pipe,
                       const float * p_acc_g,
                       const float * p_mag_g,
                       const motion_input_t * p_motion,
                       float dt_s,
                       bool i2c_fault,
                       pipeline_step_t * p_out)
{
    hump_sample_t sample;
    imu_motion_input_t in;
    hump_fields_t fields;
    packed_hump_t word = 0U;
    uint8_t faults;
    bool event = false;

    if ((NULL == p_pipe) || (NULL == p_motion) || (NULL == p_out)
        || !isfinite (dt_s) || (dt_s <= 0.0f))
    {
        return false;
    }

    (void)memset (p_out, 0, sizeof (*p_out));
    p_out->sensors_ok = false;

    if (NULL == p_acc_g)
    {
        p_pipe->carry_ds_m += p_motion->valid ? p_motion->ds_m : 0.0f;
        p_pipe->carry_dt_s += dt_s;
        p_pipe->carry_invalid = p_pipe->carry_invalid || !p_motion->valid
                                || !isfinite (p_motion->ds_m);
    }
    else
    {
        (void)memset (&sample, 0, sizeof (sample));
        sample.ax_g = p_acc_g[0];
        sample.ay_g = p_acc_g[1];
        sample.az_g = p_acc_g[2];
        sample.motion_valid = p_motion->valid && !p_pipe->carry_invalid;
        sample.motion_age_s = p_motion->age_s;
        sample.speed_mps = p_motion->speed_mps;
        sample.ds_m = p_motion->ds_m + p_pipe->carry_ds_m;
        sample.accel_mps2 = p_motion->accel_mps2;

        (void)memset (&in, 0, sizeof (in));
        in.dt_s = dt_s + p_pipe->carry_dt_s;
        in.wheels_valid = sample.motion_valid && p_motion->wheels_valid;
        p_pipe->carry_ds_m = 0.0f;
        p_pipe->carry_dt_s = 0.0f;
        p_pipe->carry_invalid = false;
        in.v_left_mps = p_motion->v_left_mps;
        in.v_right_mps = p_motion->v_right_mps;
        if (NULL != p_mag_g)
        {
            in.mag_new = true;
            in.mx_g = p_mag_g[0];
            in.my_g = p_mag_g[1];
            in.mz_g = p_mag_g[2];
        }

        /* Bad numbers from Buddy 2 make hump_update refuse the sample;
         * retry once with the motion data marked invalid. */
        if (!hump_update (&p_pipe->hump, &sample, &p_out->hump))
        {
            sample.motion_valid = false;
            in.wheels_valid = false;
            p_out->sensors_ok = hump_update (&p_pipe->hump, &sample,
                                             &p_out->hump);
        }
        else
        {
            p_out->sensors_ok = true;
        }

        if (p_out->sensors_ok
            && !imu_motion_update (&p_pipe->motion, &sample, &p_out->hump,
                                   &in, &p_out->motion))
        {
            /* Only non-finite magnetometer values get here. */
            in.mag_new = false;
            p_out->sensors_ok = imu_motion_update (&p_pipe->motion, &sample,
                                                   &p_out->hump, &in,
                                                   &p_out->motion);
        }
    }

    if (p_out->sensors_ok)
    {
        hump_run_update (&p_pipe->run, &p_out->hump);
        (void)hump_fields_from_output (&p_out->hump, &p_pipe->run,
                                          p_out->motion.motion, i2c_fault,
                                          &fields);
        if (!hump_pack (&fields, &word))
        {
            p_out->sensors_ok = false;
        }
        else
        {
            event = (HUMP_EVENT_NONE != p_out->hump.event)
                    || (p_out->motion.motion != p_pipe->last_motion);
            p_pipe->last_motion = p_out->motion.motion;
        }
    }

    if (!p_out->sensors_ok)
    {
        word = fault_word (p_pipe);
    }

    (void)hump_unpack (word, &fields);
    faults = fields.faults;
    if (!p_pipe->have_last || (faults != p_pipe->last_faults))
    {
        event = true;
    }

    p_pipe->cycle++;
    p_out->word = word;
    p_out->to_nav = event;
    p_out->to_telem = event || (0U == (p_pipe->cycle % p_pipe->telem_every));

    p_pipe->last_word = word;
    p_pipe->last_faults = faults;
    p_pipe->have_last = true;

    return true;
}

/*** end of file ***/
