/** @file encoder_quad.c
 * @brief Count valid adjacent quadrature transitions in either direction.
 */
#include <stddef.h>
#include "encoder_quad.h"

void encoder_quad_init (encoder_quad_t * p_quad, uint8_t states)
{
    unsigned int wheel;

    if (p_quad != NULL)
    {
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_quad->wheel[wheel].positive = 0U;
            p_quad->wheel[wheel].negative = 0U;
            p_quad->wheel[wheel].invalid = 0U;
        }
        p_quad->states = states & UINT8_C(15);
    }
}

bool encoder_quad_update (encoder_quad_t * p_quad, unsigned int wheel,
                          uint8_t state, bool ambiguous)
{
    static const int8_t steps[16] = {
         0,  1, -1,  2,
        -1,  0,  2,  1,
         1,  2,  0, -1,
         2, -1,  1,  0
    };
    unsigned int shift;
    uint8_t previous;
    int8_t step;

    if ((p_quad == NULL) || (wheel >= ENCODER_WHEELS) || (state > 3U))
    {
        return false;
    }
    shift = wheel * 2U;
    previous = (uint8_t)((p_quad->states >> shift) & 3U);
    step = steps[(previous * 4U) + state];
    p_quad->states = (uint8_t)((p_quad->states & ~(3U << shift)) |
                              ((unsigned int)state << shift));
    if (ambiguous || (step == 2))
    {
        p_quad->wheel[wheel].invalid++;
        return false;
    }
    if (step > 0)
    {
        p_quad->wheel[wheel].positive++;
    }
    else if (step < 0)
    {
        p_quad->wheel[wheel].negative++;
    }
    else
    {
        /* A repeated sample contains no new transition. */
    }
    return true;
}

/*** end of file ***/
