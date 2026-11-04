/** @file line_filter.c
 * @brief 3-sample (15 ms at 5 ms) line debounce.
 */
#include <stddef.h>
#include "line_filter.h"

#define LINE_CONFIRM_SAMPLES 3U

static void update_one (bool sample, bool * p_confirmed, bool * p_candidate,
                        uint8_t * p_count)
{
    if (sample == *p_confirmed)
    {
        *p_candidate = sample;
        *p_count     = LINE_CONFIRM_SAMPLES;
    }
    else if (sample == *p_candidate)
    {
        if (*p_count < LINE_CONFIRM_SAMPLES)
        {
            (*p_count)++;
        }
        if (*p_count >= LINE_CONFIRM_SAMPLES)
        {
            *p_confirmed = sample;
        }
    }
    else
    {
        *p_candidate = sample;
        *p_count     = 1U;
    }
}

void line_filter_init (line_filter_t * p_filter, bool left, bool right)
{
    if (p_filter == NULL)
    {
        return;
    }
    p_filter->left           = left;
    p_filter->right          = right;
    p_filter->left_candidate = left;
    p_filter->right_candidate = right;
    p_filter->left_count     = LINE_CONFIRM_SAMPLES;
    p_filter->right_count    = LINE_CONFIRM_SAMPLES;
}

void line_filter_update (line_filter_t * p_filter, bool left, bool right)
{
    if (p_filter == NULL)
    {
        return;
    }
    update_one(left, &p_filter->left, &p_filter->left_candidate,
               &p_filter->left_count);
    update_one(right, &p_filter->right, &p_filter->right_candidate,
               &p_filter->right_count);
}

/*** end of file ***/
