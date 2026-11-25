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
#include "attitude.h"
#include "imu_motion.h"

#define RAD_TO_DEG (57.2957795f)
#define MM_TO_M    (0.001f)
/* Wheel velocity arrives in integer micrometres per second. */
#define WHEEL_SPEED_QUANTUM_MPS (0.000001f)
/* Guard against a scheduling gap even when the next bus read succeeds. */
#define ATTITUDE_EVIDENCE_GAP_S (0.050f)
/* Attempts per transaction after a failed cycle. A timed-out attempt costs
 * about 6 ms (the 5 ms poll deadline, the abort and the bus clear), so two
 * fit a 20 ms cycle with room for the rest of it. */
#define RETRY_ATTEMPTS (2U)

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
    {
        attitude_config_t att_cfg;

        attitude_default_config (&att_cfg);
        if (!attitude_init (&fresh.attitude, &att_cfg))
        {
            return false;
        }
        fresh.hump_pitch_max_deg =
            (asinf ((float)PIPELINE_HUMP_MAX_MM
                    / (float)CHASSIS_AXLE_TO_CASTER_MM) * RAD_TO_DEG)
            + PIPELINE_HUMP_PITCH_MARGIN_DEG;
        if (fresh.hump_pitch_max_deg >= att_cfg.tilt_envelope_deg)
        {
            return false;
        }
    }
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
        attitude_reset (&p_pipe->attitude);
        p_pipe->hump_travel_m = 0.0f;
        p_pipe->hump_rejected = false;
    }
}

void pipeline_read (const pipeline_t * p_pipe, lsm303_t * p_imu,
                    uint8_t attempts, bool b_mag_due,
                    lsm303_vec_t * p_acc, lsm303_vec_t * p_mag,
                    bool * p_acc_ok, bool * p_mag_ok)
{
    *p_acc_ok = false;
    *p_mag_ok = false;
    if (p_imu->ready)
    {
        uint8_t tries = attempts;

        if ((0U != p_pipe->failed_cycles) && (tries > RETRY_ATTEMPTS))
        {
            tries = RETRY_ATTEMPTS;
        }
        p_imu->max_attempts = tries;
        *p_acc_ok = (LSM303_OK == lsm303_read_accel (p_imu, p_acc));
        /* A bus that failed the accelerometer is not asked again. */
        if (b_mag_due && *p_acc_ok)
        {
            *p_mag_ok = (LSM303_OK == lsm303_read_mag (p_imu, p_mag));
        }
    }
}

bool pipeline_config_readback (const pipeline_t * p_pipe, lsm303_t * p_imu,
                               uint32_t cycle, uint32_t every,
                               uint8_t attempts)
{
    bool b_mismatch = false;

    if (p_imu->ready && (0U == p_pipe->failed_cycles)
        && (0U == (cycle % every)))
    {
        p_imu->max_attempts = attempts;
        b_mismatch = (LSM303_CONFIG_MISMATCH == lsm303_check_config (p_imu));
    }
    return b_mismatch;
}

bool pipeline_attitude_known (const pipeline_step_t * p_step)
{
    return (NULL != p_step) && p_step->sensors_ok && !p_step->attitude_unknown;
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

/* Message once the IMU is lost: the fault word with no hump and an unknown
 * motion class, so no stale lift or hump stands. */
static packed_hump_t lost_word (const pipeline_t * p_pipe)
{
    hump_fields_t fields;
    packed_hump_t word = fault_word (p_pipe);

    (void)hump_unpack (word, &fields);
    fields.state       = HUMP_STATE_FLAT;
    fields.motion      = MOTION_CLASS_UNKNOWN;
    fields.hump_active = false;
    (void)hump_pack (&fields, &word);

    return word;
}

/* Build the hump sample and the motion input for one cycle with good
 * accelerometer data, folding in the travel and time carried over from
 * failed cycles. */
static void prepare_inputs (pipeline_t * p_pipe, const float * p_acc_g,
                            const float * p_mag_g,
                            const motion_input_t * p_motion, float dt_s,
                            hump_sample_t * p_sample, imu_motion_input_t * p_in)
{
    (void)memset (p_sample, 0, sizeof (*p_sample));
    p_sample->ax_g = p_acc_g[HUMP_AXIS_X];
    p_sample->ay_g = p_acc_g[HUMP_AXIS_Y];
    p_sample->az_g = p_acc_g[HUMP_AXIS_Z];
    p_sample->motion_valid = p_motion->valid && !p_pipe->carry_invalid;
    p_sample->motion_age_s = p_motion->age_s;
    p_sample->speed_mps = p_motion->speed_mps;
    p_sample->ds_m = p_motion->ds_m + p_pipe->carry_ds_m;
    p_sample->accel_mps2 = p_motion->accel_mps2;

    (void)memset (p_in, 0, sizeof (*p_in));
    p_in->dt_s = dt_s + p_pipe->carry_dt_s;
    p_in->wheels_valid = p_sample->motion_valid && p_motion->wheels_valid;
    p_pipe->carry_ds_m = 0.0f;
    p_pipe->carry_dt_s = 0.0f;
    p_pipe->carry_invalid = false;
    p_in->v_left_mps = p_motion->v_left_mps;
    p_in->v_right_mps = p_motion->v_right_mps;
    if (NULL != p_mag_g)
    {
        p_in->mag_new = true;
        p_in->mx_g = p_mag_g[HUMP_AXIS_X];
        p_in->my_g = p_mag_g[HUMP_AXIS_Y];
        p_in->mz_g = p_mag_g[HUMP_AXIS_Z];
    }
}

static void run_attitude (pipeline_t * p_pipe, const hump_sample_t * p_sample,
                          const imu_motion_input_t * p_motion, pipeline_step_t * p_out);

static void judge_hump (pipeline_t * p_pipe, pipeline_step_t * p_out);

/* Run hump detection and motion classification on one prepared sample. */
static void run_detectors (pipeline_t * p_pipe, hump_sample_t * p_sample,
                           imu_motion_input_t * p_in, pipeline_step_t * p_out)
{
    /* Bad numbers from Buddy 2 make hump_update refuse the sample;
     * retry once with the motion data marked invalid. */
    if (!hump_update (&p_pipe->hump, p_sample, &p_out->hump))
    {
        p_sample->motion_valid = false;
        p_in->wheels_valid = false;
        p_out->sensors_ok = hump_update (&p_pipe->hump, p_sample,
                                         &p_out->hump);
    }
    else
    {
        p_out->sensors_ok = true;
    }

    if (p_out->sensors_ok
        && !imu_motion_update (&p_pipe->motion, p_sample, &p_out->hump,
                               p_in, &p_out->motion))
    {
        /* Only non-finite magnetometer values get here. */
        p_in->mag_new = false;
        p_out->sensors_ok = imu_motion_update (&p_pipe->motion, p_sample,
                                               &p_out->hump, p_in,
                                               &p_out->motion);
    }

    if (p_out->sensors_ok)
    {
        judge_hump (p_pipe, p_out);
        run_attitude (p_pipe, p_sample, p_in, p_out);
    }
}

/* A hump taller than the car is built for is not a hump to ride over: the
 * pitch limit comes from the axle to caster distance, the height limit from
 * the tallest hump plus the height error. The hump state is reported as FLAT
 * from the first sample past a limit until the detector ends the hump, so
 * the lift checks and the line follower treat the car as on level ground. */
static void judge_hump (pipeline_t * p_pipe, pipeline_step_t * p_out)
{
    hump_output_t * p_hump = &p_out->hump;
    float const height_max_m =
        (float)(PIPELINE_HUMP_MAX_MM + PIPELINE_HUMP_HEIGHT_BUDGET_MM)
        * MM_TO_M;
    float const pitch_deg = fmaxf (p_hump->peak_pitch_deg,
                                   fabsf (p_hump->pitch_deg));
    bool const b_in_hump = (HUMP_STATE_FLAT != p_hump->state);
    bool const b_ending = (HUMP_EVENT_FINISHED == p_hump->event);

    if (p_pipe->hump_rejected)
    {
        p_hump->event = HUMP_EVENT_NONE;
        p_hump->state = HUMP_STATE_FLAT;
        p_hump->height_valid = false;
        p_pipe->hump_rejected = b_in_hump;
    }
    else if ((b_in_hump || b_ending)
             && ((pitch_deg > p_pipe->hump_pitch_max_deg)
                 || (p_hump->peak_m > height_max_m)))
    {
        p_out->hump_rejected = true;
        p_pipe->hump_rejected = b_in_hump;
        p_hump->state = HUMP_STATE_FLAT;
        p_hump->event = HUMP_EVENT_FINISHED;
        p_hump->complete = false;
        p_hump->height_valid = false;
    }
    else
    {
        /* No hump, or a valid one. */
    }
}

/* Lift and overturn from the gravity direction. A lifted or overturned car
 * reports the LIFTED motion class. The upward cue is masked while a valid
 * hump is in progress and the car drives forward; the travel cap only catches
 * a hump state that never ends, such as wheels spinning on a lifted car. */
static void run_attitude (pipeline_t * p_pipe, const hump_sample_t * p_sample,
                          const imu_motion_input_t * p_motion, pipeline_step_t * p_out)
{
    attitude_input_t in;
    float const cap_m = (float)PIPELINE_HUMP_MASK_MM * MM_TO_M;
    /* Height estimation needs useful speed. The attitude mask only needs
     * fresh encoder evidence that the car is still travelling forward. */
    bool const forward_travel = !p_out->hump.motion_stale
                                && (p_sample->speed_mps > 0.0f)
                                && (p_sample->ds_m > 0.0f);

    if (HUMP_STATE_FLAT == p_out->hump.state)
    {
        p_pipe->hump_travel_m = 0.0f;
    }
    else if (forward_travel)
    {
        p_pipe->hump_travel_m += p_sample->ds_m;
    }
    else
    {
        /* Not driving forward: no travel to count. */
    }

    (void)memset (&in, 0, sizeof (in));
    in.ax_g = p_sample->ax_g;
    in.ay_g = p_sample->ay_g;
    in.az_g = p_sample->az_g;
    in.calibrated = hump_rest_gravity (&p_pipe->hump, in.g0);
    in.fwd[p_pipe->hump.cfg.fwd_axis] = (float)p_pipe->hump.cfg.fwd_sign;
    in.b_wheels_free = p_pipe->wheels_free;
    in.b_wheels_still = p_motion->wheels_valid && !p_out->hump.motion_stale &&
                        (fabsf (p_motion->v_left_mps) < WHEEL_SPEED_QUANTUM_MPS) &&
                        (fabsf (p_motion->v_right_mps) < WHEEL_SPEED_QUANTUM_MPS);
    in.b_on_hump = (HUMP_STATE_FLAT != p_out->hump.state)
                   && forward_travel && (p_pipe->hump_travel_m < cap_m);

    if (attitude_update (&p_pipe->attitude, &in, &p_out->attitude)
        && ((ATTITUDE_LIFTED == p_out->attitude.state)
            || (ATTITUDE_OVERTURNED == p_out->attitude.state)))
    {
        p_out->motion.motion = MOTION_CLASS_LIFTED;
    }
}

/* Pack the detector results into *p_word. Clears sensors_ok when the fields
 * do not pack; sets *p_event when a message must go to navigation. */
static void pack_result (pipeline_t * p_pipe, pipeline_step_t * p_out,
                         bool i2c_fault, packed_hump_t * p_word,
                         bool * p_event)
{
    hump_fields_t fields;

    hump_run_update (&p_pipe->run, &p_out->hump);
    (void)hump_fields_from_output (&p_out->hump, &p_pipe->run,
                                   p_out->motion.motion, i2c_fault, &fields);
    if (0U != (fields.faults & PHYSICS_FAULT_STALE))
    {
        p_pipe->stale_hold = (uint8_t)PIPELINE_STALE_HOLD_CYCLES;
    }
    else if (p_pipe->stale_hold > 0U)
    {
        p_pipe->stale_hold--;
        fields.faults = (uint8_t)(fields.faults | PHYSICS_FAULT_STALE);
    }
    else
    {
        /* Motion data is fresh and the hold has run out. */
    }
    if (!hump_pack (&fields, p_word))
    {
        p_out->sensors_ok = false;
    }
    else
    {
        *p_event = (HUMP_EVENT_NONE != p_out->hump.event)
                   || (p_out->motion.motion != p_pipe->last_motion);
        p_pipe->last_motion = p_out->motion.motion;
    }
}

static void handle_imu_lost (pipeline_t * p_pipe)
{
    hump_recalibrate (&p_pipe->hump);
    if ((ATTITUDE_LIFTED != p_pipe->attitude.state) &&
        (ATTITUDE_OVERTURNED != p_pipe->attitude.state))
    {
        attitude_reset (&p_pipe->attitude);
    }
    else
    {
        attitude_interrupt_evidence (&p_pipe->attitude);
    }
    p_pipe->hump_travel_m = 0.0f;
    p_pipe->hump_rejected = false;
    p_pipe->last_motion = MOTION_CLASS_UNKNOWN;
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
    uint8_t faults = 0U;
    bool event = false;

    if ((NULL == p_pipe) || (NULL == p_motion) || (NULL == p_out)
        || !isfinite (dt_s) || (dt_s <= 0.0f))
    {
        return false;
    }

    (void)memset (p_out, 0, sizeof (*p_out));
    bool const was_lost = (p_pipe->failed_cycles >= PIPELINE_LOST_CYCLES);

    if ((dt_s + p_pipe->carry_dt_s) > ATTITUDE_EVIDENCE_GAP_S)
    {
        attitude_interrupt_evidence (&p_pipe->attitude);
    }
    if ((NULL == p_acc_g) || !isfinite (p_acc_g[HUMP_AXIS_X]) ||
        !isfinite (p_acc_g[HUMP_AXIS_Y]) || !isfinite (p_acc_g[HUMP_AXIS_Z]))
    {
        p_pipe->carry_ds_m += p_motion->valid ? p_motion->ds_m : 0.0f;
        p_pipe->carry_dt_s += dt_s;
        p_pipe->carry_invalid = p_pipe->carry_invalid || !p_motion->valid
                                || !isfinite (p_motion->ds_m);
        if (p_pipe->failed_cycles < UINT16_MAX)
        {
            p_pipe->failed_cycles++;
        }
    }
    else
    {
        p_pipe->failed_cycles = 0U;
        prepare_inputs (p_pipe, p_acc_g, p_mag_g, p_motion, dt_s, &sample, &in);
        run_detectors (p_pipe, &sample, &in, p_out);
    }

    if (p_out->sensors_ok)
    {
        pack_result (p_pipe, p_out, i2c_fault, &word, &event);
    }
    else
    {
        /* Missing samples interrupt the set-down window even before IMU loss. */
        attitude_interrupt_evidence (&p_pipe->attitude);
        if (!p_pipe->hump.calibrated && (p_pipe->hump.cal_count > 0U))
        {
            hump_recalibrate (&p_pipe->hump);
        }
    }
    if (p_pipe->failed_cycles >= PIPELINE_LOST_CYCLES)
    {
        /* Reacquire the gravity reference at rest after a sustained outage. */
        p_out->imu_lost = true;
        if (PIPELINE_LOST_CYCLES == p_pipe->failed_cycles)
        {
            handle_imu_lost (p_pipe);
            event = true;
        }
        word = lost_word (p_pipe);
    }
    else if (!p_out->sensors_ok)
    {
        word = fault_word (p_pipe);
    }
    else
    {
        /* sensors_ok: the word is packed. */
    }
    p_out->imu_lost_started = !was_lost && p_out->imu_lost;
    p_out->imu_recovered = was_lost && !p_out->imu_lost;

    (void)hump_unpack (word, &fields);
    faults = fields.faults;
    if (!p_pipe->have_last || (faults != p_pipe->last_faults))
    {
        event = true;
    }

    p_pipe->cycle++;
    p_out->word = word;
    p_out->calibrated = p_pipe->hump.calibrated;
    p_out->attitude_unknown = !p_out->calibrated;
    p_out->to_nav = event;
    p_out->to_telem = event || (0U == (p_pipe->cycle % p_pipe->telem_every));

    p_pipe->last_word = word;
    p_pipe->last_faults = faults;
    p_pipe->have_last = true;

    return true;
}

/*** end of file ***/
