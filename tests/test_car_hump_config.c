#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "physics_task.h"
#include "track/line_control.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define DEG_TO_RAD (0.01745329252f)
#define SAMPLE_TIME_S (0.02f)
#define DRIVE_SPEED_MPS (0.15f)

static void init_at_rest (pipeline_t * p_pipe)
{
    physics_task_config_t cfg;
    motion_input_t const motion = {.valid = true};
    float const acc[] = {0.0f, 0.0f, 1.0f};
    pipeline_step_t out;
    unsigned i;

    physics_car_config (&cfg);
    assert (pipeline_init (p_pipe, &cfg.hump, &cfg.motion, cfg.telem_every));
    for (i = 0U; i < 60U; ++i)
    {
        assert (pipeline_step (p_pipe, acc, NULL, &motion, SAMPLE_TIME_S, false, &out));
    }
    assert (out.calibrated);
}

static bool sample_pitch (pipeline_t * p_pipe, float pitch_deg, unsigned count,
                          hump_event_t event, pipeline_step_t * p_out)
{
    float const angle = pitch_deg * DEG_TO_RAD;
    float const acc[] = {0.0f, -sinf (angle), cosf (angle)};
    motion_input_t const motion = {
        .valid = true,
        .speed_mps = DRIVE_SPEED_MPS,
        .ds_m = DRIVE_SPEED_MPS * SAMPLE_TIME_S,
        .wheels_valid = true,
        .v_left_mps = DRIVE_SPEED_MPS,
        .v_right_mps = DRIVE_SPEED_MPS,
    };
    bool seen_event = false;
    unsigned i;

    for (i = 0U; i < count; ++i)
    {
        assert (pipeline_step (p_pipe, acc, NULL, &motion, SAMPLE_TIME_S, false, p_out));
        assert (p_out->sensors_ok);
        seen_event = seen_event || (event == p_out->hump.event);
    }
    return seen_event;
}

static void start_descent (pipeline_t * p_pipe, pipeline_step_t * p_out)
{
    init_at_rest (p_pipe);
    assert (sample_pitch (p_pipe, 6.0f, 30U, HUMP_EVENT_STARTED, p_out));
    (void)sample_pitch (p_pipe, -6.0f, 30U, HUMP_EVENT_NONE, p_out);
    assert (HUMP_STATE_DESCEND == p_out->hump.state);
}

int main (int argc, char * argv[])
{
    physics_task_config_t cfg;
    pipeline_t pipe;
    pipeline_step_t out;

    if (2 != argc)
    {
        (void)fprintf (stderr, "usage: %s CASE\n", argv[0]);
        return 2;
    }

    if (0 == strcmp (argv[1], "shallow-climb"))
    {
        init_at_rest (&pipe);
        assert (sample_pitch (&pipe, 3.2f, 30U, HUMP_EVENT_STARTED, &out));
    }
    else if (0 == strcmp (argv[1], "below-enter-band"))
    {
        init_at_rest (&pipe);
        assert (!sample_pitch (&pipe, 2.8f, 30U, HUMP_EVENT_STARTED, &out));
        assert (HUMP_STATE_FLAT == out.hump.state);
    }
    else if (0 == strcmp (argv[1], "outside-exit-band"))
    {
        start_descent (&pipe, &out);
        assert (!sample_pitch (&pipe, -2.2f, 30U, HUMP_EVENT_FINISHED, &out));
        assert (HUMP_STATE_DESCEND == out.hump.state);
    }
    else if (0 == strcmp (argv[1], "configured-enter-band"))
    {
        physics_car_config (&cfg);
        assert (fabsf (cfg.hump.enter_deg - 3.0f) < 0.001f);
    }
    else if (0 == strcmp (argv[1], "inside-exit-band"))
    {
        start_descent (&pipe, &out);
        assert (sample_pitch (&pipe, -1.8f, 30U, HUMP_EVENT_FINISHED, &out));
    }
    else if (0 == strcmp (argv[1], "shallow-hump-message"))
    {
        hump_fields_t fields;

        init_at_rest (&pipe);
        (void)sample_pitch (&pipe, 3.6f, 30U, HUMP_EVENT_NONE, &out);
        assert (hump_unpack (out.word, &fields));
        assert (fields.hump_active);
    }
    else if (0 == strcmp (argv[1], "configured-exit-band"))
    {
        physics_car_config (&cfg);
        assert (fabsf (cfg.hump.exit_band_deg - 2.0f) < 0.001f);
    }
    else if (0 == strcmp (argv[1], "pitch-limit-before-height"))
    {
        init_at_rest (&pipe);
        assert (sample_pitch (&pipe, 8.0f, 15U, HUMP_EVENT_STARTED, &out));
        assert (sample_pitch (&pipe, 24.0f, 12U, HUMP_EVENT_FINISHED, &out));
        assert (HUMP_STATE_FLAT == out.hump.state);
    }
    else if (0 == strcmp (argv[1], "hump-message-extends-bridge"))
    {
        hump_fields_t fields;
        line_follow_t follower;
        line_targets_t wheels;

        init_at_rest (&pipe);
        assert (sample_pitch (&pipe, 8.0f, 15U, HUMP_EVENT_STARTED, &out));
        assert (hump_unpack (out.word, &fields));
        line_follow_init (&follower);
        line_follow_set_hump (&follower, fields.hump_active || (HUMP_STATE_FLAT != fields.state));
        (void)line_follow_update (&follower, true, true, 0, 0, 0U, &wheels);
        assert (LINE_FOLLOW_BRIDGING ==
                line_follow_update (&follower, false, false, 0, 300000, 10U, &wheels));
    }
    else
    {
        (void)fprintf (stderr, "unknown case %s\n", argv[1]);
        return 2;
    }

    (void)printf ("%s: passed\n", argv[1]);
    return 0;
}
