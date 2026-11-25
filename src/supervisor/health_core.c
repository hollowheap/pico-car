/*
 *	health_core.c
 *	Liveness, progress and stuck-sensor checks. See health_core.h.
 */

#include "health_core.h"

void liveness_init (liveness_t * p_live, uint16_t required_mask)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < LIVE_TASKS; idx++)
    {
        p_live->miss[idx] = 0U;
    }
    p_live->required = required_mask;
    p_live->armed    = 0U;
    p_live->stalled  = 0U;
    p_live->checks   = 0U;
}

void liveness_check (liveness_t * p_live, uint16_t taken, uint16_t * p_new, uint16_t * p_back)
{
    uint32_t const before = p_live->stalled;
    uint32_t       stalled = p_live->stalled;
    uint32_t       armed  = (uint32_t) p_live->armed | (uint32_t) taken;
    uint32_t       idx    = 0U;

    if (p_live->checks < LIVENESS_BOOT_CHECKS)
    {
        p_live->checks++;
    }
    p_live->armed = (uint16_t) armed;

    for (idx = 0U; idx < LIVE_TASKS; idx++)
    {
        uint32_t const bit = 1UL << idx;

        if (0U == ((uint32_t) p_live->required & bit))
        {
            p_live->miss[idx] = 0U;
        }
        else if (0U != ((uint32_t) taken & bit))
        {
            p_live->miss[idx] = 0U;
            stalled           = stalled & ~bit;
        }
        else if ((0U != (armed & bit)) || (LIVENESS_BOOT_CHECKS == p_live->checks))
        {
            if (p_live->miss[idx] < UINT8_MAX)
            {
                p_live->miss[idx]++;
            }
            if (p_live->miss[idx] >= LIVENESS_MISS_LIMIT)
            {
                stalled = stalled | bit;
            }
        }
        else
        {
            /* Not started yet and still inside the boot allowance. */
        }
    }

    p_live->stalled = (uint16_t) stalled;
    *p_new          = (uint16_t) (stalled & ~before);
    *p_back         = (uint16_t) (before & ~stalled);
}

bool liveness_critical_stalled (const liveness_t * p_live)
{
    return (0U != (p_live->stalled & LIVENESS_CRITICAL_MASK));
}

void progress_init (progress_t * p_prog)
{
    p_prog->last_mm   = 0U;
    p_prog->still_ms  = 0U;
    p_prog->b_tripped = false;
}

bool progress_update (progress_t * p_prog, bool b_expected, uint32_t travel_mm, uint32_t dt_ms)
{
    uint32_t const moved = travel_mm - p_prog->last_mm;
    bool           b_new = false;

    if (!b_expected || (moved >= PROGRESS_MIN_MM))
    {
        p_prog->still_ms  = 0U;
        p_prog->b_tripped = false;
        p_prog->last_mm   = travel_mm;
    }
    else
    {
        p_prog->still_ms += dt_ms;
        if ((p_prog->still_ms >= PROGRESS_STALL_MS) && !p_prog->b_tripped)
        {
            p_prog->b_tripped = true;
            b_new             = true;
        }
    }

    return b_new;
}

void stuck_init (stuck_t * p_stuck)
{
    p_stuck->edges     = 0U;
    p_stuck->anchor_mm = 0U;
    p_stuck->b_tripped = false;
}

bool stuck_update (stuck_t * p_stuck, bool b_active, uint32_t edges, uint32_t travel_mm)
{
    bool b_new = false;

    if (!b_active || (edges != p_stuck->edges))
    {
        p_stuck->edges     = edges;
        p_stuck->anchor_mm = travel_mm;
        p_stuck->b_tripped = false;
    }
    else if (((travel_mm - p_stuck->anchor_mm) >= STUCK_TRAVEL_MM) && !p_stuck->b_tripped)
    {
        p_stuck->b_tripped = true;
        b_new              = true;
    }
    else
    {
        /* Not far enough yet. */
    }

    return b_new;
}

#define DRIVE_PERCENT_BASE (100)
#define DRIVE_RATIO_SCALE  (1000000)

static int32_t abs32 (int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t sign3 (int32_t value)
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

void drive_init (drive_t * p_drive)
{
    p_drive->agree_ms    = 0U;
    p_drive->disagree_ms = 0U;
    p_drive->window_ms   = 0U;
    p_drive->base[0]     = 0U;
    p_drive->base[1]     = 0U;
    p_drive->base_ms     = 0U;
    p_drive->free_run    = 0U;
    p_drive->busy_run    = 0U;
    p_drive->b_crossed   = false;
    p_drive->b_free      = false;
}

/* The faster-commanded wheel must turn faster. */
static void drive_vote (drive_t * p_drive, const drive_in_t * p_in, uint32_t dt_ms)
{
    int32_t const gap_t = abs32 (p_in->target_um_s[0]) - abs32 (p_in->target_um_s[1]);
    int32_t const gap_s = abs32 (p_in->speed_um_s[0]) - abs32 (p_in->speed_um_s[1]);
    uint32_t      idx   = 0U;

    if ((abs32 (gap_t) >= DRIVE_TARGET_GAP_UM_S) && (abs32 (gap_s) >= DRIVE_SPEED_GAP_UM_S))
    {
        if (sign3 (gap_t) == sign3 (gap_s))
        {
            p_drive->agree_ms = (uint16_t) (p_drive->agree_ms + dt_ms);
        }
        else
        {
            p_drive->disagree_ms = (uint16_t) (p_drive->disagree_ms + dt_ms);
        }
    }
    /* A wheel that turns against its command also disagrees. */
    for (idx = 0U; idx < DRIVE_WHEELS; idx++)
    {
        if ((abs32 (p_in->target_um_s[idx]) >= DRIVE_TARGET_MIN_UM_S) &&
            (abs32 (p_in->speed_um_s[idx]) >= DRIVE_SPEED_GAP_UM_S) &&
            (sign3 (p_in->target_um_s[idx]) != sign3 (p_in->speed_um_s[idx])))
        {
            p_drive->disagree_ms = (uint16_t) (p_drive->disagree_ms + dt_ms);
        }
    }
}

/* A wheel is on target when it is told to go and runs within the tolerance. */
static bool wheel_on_target (const drive_in_t * p_in, uint32_t idx)
{
    int32_t const target = p_in->target_um_s[idx];
    int32_t const err    = abs32 (p_in->speed_um_s[idx] - target);

    return (abs32 (target) >= DRIVE_TARGET_MIN_UM_S) && (sign3 (target) == sign3 (p_in->speed_um_s[idx])) &&
           ((err * DRIVE_PERCENT_BASE) <= (abs32 (target) * DRIVE_TRACK_PERCENT));
}

/* Duty permille per m/s. */
static uint32_t duty_ratio (const drive_in_t * p_in, uint32_t idx)
{
    uint32_t const duty  = (uint32_t) abs32 (p_in->duty_pm[idx]);
    uint32_t const speed = (uint32_t) abs32 (p_in->speed_um_s[idx]);

    return (duty * (uint32_t) DRIVE_RATIO_SCALE) / speed;
}

/* The busy count clears the cue after DRIVE_CLEAR_CHECKS checks that are not light. */
static void drive_busy (drive_t * p_drive)
{
    p_drive->free_run = 0U;
    if (p_drive->busy_run < UINT8_MAX)
    {
        p_drive->busy_run++;
    }
    if (p_drive->busy_run >= DRIVE_CLEAR_CHECKS)
    {
        p_drive->b_free = false;
    }
}

/* Move a baseline 1/16 of the way to a grounded sample. */
static uint32_t drive_learn (uint32_t base, uint32_t sample)
{
    int32_t const step = ((int32_t) sample - (int32_t) base) / DRIVE_BASE_DIVISOR;

    return (0U == base) ? sample : (uint32_t) ((int32_t) base + step);
}

static void drive_free (drive_t * p_drive, const drive_in_t * p_in, uint32_t dt_ms)
{
    bool b_both = p_in->b_judge && wheel_on_target (p_in, 0U) && wheel_on_target (p_in, 1U);

    if (!b_both)
    {
        drive_busy (p_drive);
    }
    else
    {
        uint32_t const left  = duty_ratio (p_in, 0U);
        uint32_t const right = duty_ratio (p_in, 1U);
        bool const     b_light = (p_drive->base_ms >= DRIVE_BASE_MS) &&
                             ((left * DRIVE_FREE_DIVISOR) < p_drive->base[0]) &&
                             ((right * DRIVE_FREE_DIVISOR) < p_drive->base[1]);

        if (b_light)
        {
            p_drive->busy_run = 0U;
            if (p_drive->free_run < UINT8_MAX)
            {
                p_drive->free_run++;
            }
            if (p_drive->free_run >= DRIVE_FREE_CHECKS)
            {
                p_drive->b_free = true;
            }
        }
        else
        {
            drive_busy (p_drive);
            if (!p_drive->b_free)
            {
                /* Only a grounded sample teaches the baseline. */
                p_drive->base_ms += dt_ms;
                p_drive->base[0] = drive_learn (p_drive->base[0], left);
                p_drive->base[1] = drive_learn (p_drive->base[1], right);
            }
        }
    }
}

bool drive_update (drive_t * p_drive, const drive_in_t * p_in, uint32_t dt_ms)
{
    bool b_new = false;

    if (p_in->b_judge)
    {
        drive_vote (p_drive, p_in, dt_ms);
        p_drive->window_ms = (uint16_t) (p_drive->window_ms + dt_ms);
    }
    if (p_drive->window_ms >= DRIVE_WINDOW_MS)
    {
        bool const b_bad = (p_drive->disagree_ms >= DRIVE_CROSS_MS) &&
                           (p_drive->disagree_ms > (DRIVE_CROSS_RATIO * (uint32_t) p_drive->agree_ms));

        if (b_bad && !p_drive->b_crossed)
        {
            b_new = true;
        }
        if (b_bad || (p_drive->agree_ms >= DRIVE_CROSS_MS))
        {
            p_drive->b_crossed = b_bad;
        }
        p_drive->agree_ms    = 0U;
        p_drive->disagree_ms = 0U;
        p_drive->window_ms   = 0U;
    }
    drive_free (p_drive, p_in, dt_ms);

    return b_new;
}

drive_probe_t drive_probe_verdict (int32_t driven_mm, int32_t other_mm)
{
    int32_t const driven = abs32 (driven_mm);
    int32_t const other  = abs32 (other_mm);
    drive_probe_t verdict = DRIVE_PROBE_UNCLEAR;

    if ((driven < (DRIVE_PROBE_MOVED_MM / DRIVE_PROBE_DEAD_DIV)) && (other < (DRIVE_PROBE_MOVED_MM / DRIVE_PROBE_DEAD_DIV)))
    {
        verdict = DRIVE_PROBE_DEAD;
    }
    else if ((driven >= DRIVE_PROBE_MOVED_MM) && (other < (driven / DRIVE_PROBE_RATIO)))
    {
        verdict = (driven_mm > 0) ? DRIVE_PROBE_OK : DRIVE_PROBE_REVERSED;
    }
    else if ((other >= DRIVE_PROBE_MOVED_MM) && (driven < (other / DRIVE_PROBE_RATIO)))
    {
        verdict = DRIVE_PROBE_CROSSED;
    }
    else
    {
        /* Both moved, or too little to tell. */
    }

    return verdict;
}
