#ifndef LINE_FILTER_H
#define LINE_FILTER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool     left;
    bool     right;
    bool     left_candidate;
    bool     right_candidate;
    uint8_t  left_count;
    uint8_t  right_count;
} line_filter_t;

void line_filter_init (line_filter_t * p_filter, bool left, bool right);
void line_filter_update (line_filter_t * p_filter, bool left, bool right);

#endif /* LINE_FILTER_H */
