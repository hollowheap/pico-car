/** @file line_filter.c
 * @brief Pure per-sensor debounce for the line sensors (S09).
 *
 * Runs inside the 5 ms alarm interrupt, so it does a few integer compares and
 * nothing else.
 */

#include <stddef.h>

#include "line_filter.h"

void line_debounce_init (line_debounce_t * p_filter, bool b_level)
{
    if (NULL != p_filter)
    {
        p_filter->count   = 0U;
        p_filter->b_level = b_level;
    }
}

bool line_debounce_update (line_debounce_t * p_filter, bool b_raw)
{
    bool b_level = false;

    if (NULL != p_filter)
    {
        if (b_raw == p_filter->b_level)
        {
            /* Agreement breaks any run of disagreeing samples. */
            p_filter->count = 0U;
        }
        else
        {
            p_filter->count++;
            if (p_filter->count >= LINE_DEBOUNCE_SAMPLES)
            {
                p_filter->b_level = b_raw;
                p_filter->count   = 0U;
            }
        }
        b_level = p_filter->b_level;
    }

    return b_level;
}

/*** end of file ***/
