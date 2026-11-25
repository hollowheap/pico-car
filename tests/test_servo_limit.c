#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "obstacle_pack.h"
#include "pilot_sim.h"
#include "scan_algo.h"

#define LIMIT_DECI (200)

static bool angle_eq (char const * p_label, int32_t angle, int32_t expected)
{
    if (angle == expected)
    {
        return true;
    }
    (void)fprintf (stderr, "%s: commanded %ld deci-degrees, expected %ld\n",
                   p_label, (long)angle, (long)expected);
    return false;
}

static bool pack_reply_matches_clamped_angle (void)
{
    uint32_t const request = obstacle_pack_ping_req (-300, 12U);
    int32_t const target = obstacle_pack_servo_clamp (obstacle_pack_ping_req_servo (request));
    uint32_t const reply = obstacle_pack_ping_reply (obstacle_pack_ping_req_seq (request),
                                                     OBSTACLE_CLASS_VALID, 320U, target);

    return angle_eq ("ping response", obstacle_pack_ping_servo (reply), -LIMIT_DECI);
}

static bool legacy_scan_stays_in_window (void)
{
    scan_algo_ctx_t scan;
    bool seen_left = false;
    bool seen_right = false;
    unsigned sample;

    scan_algo_start (&scan);
    for (sample = 0U; sample < 100U && SCAN_ALGO_NO_SLOT != scan_algo_target (&scan); ++sample)
    {
        int16_t const angle = scan_algo_servo_deci (&scan);

        if (angle < -LIMIT_DECI || angle > LIMIT_DECI)
        {
            int32_t const edge = (angle < 0) ? -LIMIT_DECI : LIMIT_DECI;

            return angle_eq ("legacy scan", angle, edge);
        }
        seen_left = seen_left || (LIMIT_DECI == angle);
        seen_right = seen_right || (-LIMIT_DECI == angle);
        (void)scan_algo_feed (&scan, OBSTACLE_CLASS_OPEN, 0U);
    }
    if (!seen_left || !seen_right || SCAN_ALGO_NO_SLOT != scan_algo_target (&scan))
    {
        (void)fprintf (stderr, "legacy scan failed to finish after visiting both 20 degree edges\n");
        return false;
    }
    return true;
}

static bool hug_request_contract (bool check_consumer)
{
    static pilot_sim_t sim;
    static pilot_sim_seg_t const line[] = {{8000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    bool got_angled_request = false;
    unsigned step;

    pilot_sim_init (&sim, line, 1U);
    if (!pilot_sim_add_box (&sim, 650.0, 0.0, 200.0, 150.0, 0.0))
    {
        (void)fputs ("failed to place the obstacle in the simulation\n", stderr);
        return false;
    }
    for (step = 0U; step < 12000U; ++step)
    {
        int16_t angle;

        pilot_sim_step (&sim);
        if (0U == sim.cmd.b_ping_req)
        {
            continue;
        }
        angle = sim.cmd.ping_servo_deci;
        got_angled_request = got_angled_request || (0 != angle);
        if (check_consumer)
        {
            if (!angle_eq ("servo clamp changed pilot angle",
                           obstacle_pack_servo_clamp (angle), angle))
            {
                return false;
            }
        }
        else if (angle < -LIMIT_DECI || angle > LIMIT_DECI)
        {
            int32_t const edge = (angle < 0) ? -LIMIT_DECI : LIMIT_DECI;

            return angle_eq ("hug request exceeded servo window", angle, edge);
        }
    }
    if (!got_angled_request)
    {
        (void)fputs ("the simulated box did not trigger an angled hug request\n", stderr);
    }
    return got_angled_request;
}

int main (int argc, char * argv[])
{
    bool pass;

    if (2 != argc)
    {
        (void)fprintf (stderr, "usage: %s CASE\n", argv[0]);
        return 2;
    }
    if (0 == strcmp (argv[1], "positive-limit"))
    {
        pass = angle_eq ("positive limit", obstacle_pack_servo_clamp (220), LIMIT_DECI);
    }
    else if (0 == strcmp (argv[1], "negative-limit"))
    {
        pass = angle_eq ("negative limit", obstacle_pack_servo_clamp (-201), -LIMIT_DECI);
    }
    else if (0 == strcmp (argv[1], "clamped-reply"))
    {
        pass = pack_reply_matches_clamped_angle ();
    }
    else if (0 == strcmp (argv[1], "legacy-scan"))
    {
        pass = legacy_scan_stays_in_window ();
    }
    else if (0 == strcmp (argv[1], "hug-window"))
    {
        pass = hug_request_contract (false);
    }
    else if (0 == strcmp (argv[1], "hug-into-sonar"))
    {
        pass = hug_request_contract (true);
    }
    else
    {
        (void)fprintf (stderr, "unknown case %s\n", argv[1]);
        return 2;
    }
    if (pass)
    {
        (void)printf ("%s: passed\n", argv[1]);
    }
    return pass ? 0 : 1;
}
