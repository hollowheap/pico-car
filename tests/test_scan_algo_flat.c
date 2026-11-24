/** @file test_scan_algo_flat.c
 * @brief Known-answer tests for the flat-face rule of the Buddy 5 first
 *        scan (trigonometry check, scan_algo_is_side_echo()): scans from
 *        the car facing a box square-on, plus faces that are not flat.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "b5_pack.h"
#include "b5_plan.h"
#include "scan_algo.h"

/* Readings for the 25 slots (-60..+60 in 5 degree steps); 0 = OPEN. */
typedef struct
{
    uint16_t mm[B5_SLOT_COUNT];
} scene_t;

static unsigned int failures;

static void check (bool condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

/* Run one whole scan, answering every slot the engine asks for twice. */
static void run_scan (scene_t const * p_scene, scan_algo_ctx_t * p_ctx)
{
    unsigned int guard  = 0U;
    bool         b_done = false;

    scan_algo_start(p_ctx);

    while ((!b_done) && (guard < 100U))
    {
        uint8_t const  slot = scan_algo_target(p_ctx);
        uint16_t const mm   = p_scene->mm[slot % B5_SLOT_COUNT];
        uint8_t const  cls  = (0U == mm) ? B5_CLASS_OPEN : B5_CLASS_VALID;

        if (SCAN_ALGO_NO_SLOT == slot)
        {
            b_done = true;
        }
        else
        {
            (void)scan_algo_feed(p_ctx, cls, mm);
            b_done = scan_algo_feed(p_ctx, cls, mm);
        }

        guard++;
    }
}

/* Fill the in-between slots of a scene given at the coarse slots. */
static void fill_fine (scene_t * p_scene)
{
    unsigned int slot = 0U;

    for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
    {
        unsigned int const lo = (slot / 3U) * 3U;
        unsigned int const hi = (lo + 3U < B5_SLOT_COUNT) ? lo + 3U : lo;

        if ((slot % 3U) != 0U)
        {
            uint16_t const a = p_scene->mm[lo];
            uint16_t const b = p_scene->mm[hi];

            p_scene->mm[slot] =
                ((0U == a) || (0U == b))
                    ? 0U
                    : (uint16_t)(a + (((b - a) * (slot - lo)) / 3U));
        }
    }
}

/* Coarse readings at -60, -45, ... +60 (slots 0, 3, ... 24). */
static void set_coarse (scene_t * p_scene, uint16_t const coarse[9])
{
    unsigned int idx = 0U;

    for (idx = 0U; idx < 9U; idx++)
    {
        p_scene->mm[idx * 3U] = coarse[idx];
    }

    fill_fine(p_scene);
}

static void check_flat_log (uint16_t const coarse[9], const char * p_name)
{
    scene_t          scene = {{0U}};
    scan_algo_ctx_t  ctx;
    b5_plan_cfg_t    cfg;
    b5_plan_result_t res;

    set_coarse(&scene, coarse);
    run_scan(&scene, &ctx);
    b5_plan_cfg_default(&cfg);
    b5_plan_build(&ctx, &cfg, &res);

    check(ctx.b_flat, p_name);
    check(SCAN_ALGO_EDGE_BOUNDED == ctx.edge_state[SCAN_ALGO_LEFT], p_name);
    check(SCAN_ALGO_EDGE_BOUNDED == ctx.edge_state[SCAN_ALGO_RIGHT], p_name);
    check(B5_RESULT_NO_SAFE_PATH != res.status, p_name);
    (void)printf("  %s: edges %d..%d deg, status %u\n", p_name,
                 (int)scan_algo_slot_deci(ctx.edge_hit[SCAN_ALGO_RIGHT]) / 10,
                 (int)scan_algo_slot_deci(ctx.edge_hit[SCAN_ALGO_LEFT]) / 10,
                 (unsigned int)res.status);
}

int main (void)
{
    /* The three scans from the car log (box faced square-on). */
    static uint16_t const log1[9] = {189, 191, 191, 187, 188,
                                     196, 201, 205, 213};
    static uint16_t const log2[9] = {203, 191, 187, 187, 188,
                                     193, 193, 201, 212};
    static uint16_t const log3[9] = {202, 191, 191, 187, 188,
                                     196, 201, 209, 230};

    /* Car log 2026-11-24: a face a few degrees off square (failed the old
     * "within B + 50 mm" test by 6 mm and 1 mm). */
    static uint16_t const today1[9] = {204, 197, 192, 192, 202,
                                       198, 216, 223, 248};
    static uint16_t const today2[9] = {210, 202, 201, 193, 198,
                                       198, 212, 228, 244};

    /* One end long enough to be real: not flat. */
    static uint16_t const one_real[9] = {205, 197, 192, 192, 190,
                                         198, 216, 260, 330};

    /* A face turned about 10 degrees: one end goes quiet (not flat). */
    static uint16_t const slant[9] = {0, 0, 300, 220, 192, 190, 200, 232, 290};

    /* A 150 mm box with clean edges (not flat). */
    static uint16_t const box[9] = {0, 0, 0, 195, 188, 195, 0, 0, 0};

    scene_t         scene = {{0U}};
    scan_algo_ctx_t ctx;

    check_flat_log(log1, "log scan 1 is flat and bounded");
    check_flat_log(log2, "log scan 2 is flat and bounded");
    check_flat_log(log3, "log scan 3 is flat and bounded");
    check_flat_log(today1, "today scan 1 is flat and bounded");
    check_flat_log(today2, "today scan 2 is flat and bounded");

    set_coarse(&scene, one_real);
    run_scan(&scene, &ctx);
    check(!ctx.b_flat, "one end real is not flat");

    set_coarse(&scene, slant);
    run_scan(&scene, &ctx);
    check(!ctx.b_flat, "slanted face is not flat");
    check(!scan_algo_is_side_echo(&ctx, 18U), "no side echo when not flat");

    set_coarse(&scene, box);
    run_scan(&scene, &ctx);
    check(!ctx.b_flat, "box with open ends is not flat");
    check(SCAN_ALGO_EDGE_BOUNDED == ctx.edge_state[SCAN_ALGO_LEFT],
          "box left edge bounded");
    check(SCAN_ALGO_EDGE_BOUNDED == ctx.edge_state[SCAN_ALGO_RIGHT],
          "box right edge bounded");

    if (0U == failures)
    {
        (void)printf("test_scan_algo_flat: all passed\n");
    }

    return ((0U == failures) ? 0 : 1);
}
