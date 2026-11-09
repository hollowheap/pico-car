/** @file quadrature.c
 * @brief Count valid adjacent quadrature transitions in either direction.
 */
#include <stddef.h>
#include "quadrature.h"

/* Two state bits per wheel, and the four states of one wheel's A/B pair. */
#define QUAD_BITS_PER_WHEEL 2U
#define QUAD_STATE_COUNT    4U
#define QUAD_STATE_MASK     3U
#define QUAD_MAX_STATE      3U

/* Valid bits of quadrature_t.states: both wheels, two bits each. */
#define QUAD_STATES_MASK 15U

/* Step table entry for a transition that skipped a state. */
#define QUAD_STEP_INVALID 2

void quadrature_init (quadrature_t * p_quad, uint8_t states)
{
    unsigned int wheel = 0U;

    if (p_quad != NULL)
    {
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_quad->wheel[wheel].positive = 0U;
            p_quad->wheel[wheel].negative = 0U;
            p_quad->wheel[wheel].invalid  = 0U;
        }
        p_quad->states = (uint8_t)((unsigned int)states & QUAD_STATES_MASK);
    }
}

bool quadrature_update (quadrature_t * p_quad, unsigned int wheel,
                          uint8_t state, bool ambiguous)
{
    static const int8_t steps[16] = {0, 1, -1, 2,  -1, 0,  2, 1,
                                     1, 2, 0,  -1, 2,  -1, 1, 0};
    unsigned int        shift    = 0U;
    uint8_t             previous = 0U;
    int8_t              step     = 0;

    if ((p_quad == NULL) || (wheel >= ENCODER_WHEELS) ||
        (state > QUAD_MAX_STATE))
    {
        return false;
    }
    shift    = wheel * QUAD_BITS_PER_WHEEL;
    previous = (uint8_t)(((unsigned int)p_quad->states >> shift) &
                         QUAD_STATE_MASK);
    step     = steps[(previous * QUAD_STATE_COUNT) + state];
    p_quad->states =
        (uint8_t)(((unsigned int)p_quad->states & ~(QUAD_STATE_MASK << shift)) |
                  ((unsigned int)state << shift));
    if (ambiguous || (step == QUAD_STEP_INVALID))
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
