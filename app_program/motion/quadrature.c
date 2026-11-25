/** @file quadrature.c
 * @brief Extend the quadrature state machine words into running counts.
 */
#include <stddef.h>
#include "quadrature.h"

/* Word layout of quadrature.pio. */
#define QUAD_NET_MASK   0x00FFFFFFU
#define QUAD_NET_SIGN   0x00800000U
#define QUAD_NET_SPAN   0x01000000U
#define QUAD_SKIP_SHIFT 24U
#define QUAD_SKIP_MASK  0xFFU

void quadrature_init (quadrature_t * p_quad, const uint32_t words[ENCODER_WHEELS])
{
    unsigned int wheel = 0U;

    if ((p_quad != NULL) && (words != NULL))
    {
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_quad->wheel[wheel].positive = 0U;
            p_quad->wheel[wheel].negative = 0U;
            p_quad->wheel[wheel].invalid  = 0U;
            p_quad->last[wheel]           = words[wheel];
        }
    }
}

bool quadrature_apply (quadrature_t * p_quad, unsigned int wheel, uint32_t word)
{
    uint32_t net  = 0U;
    uint32_t skip = 0U;

    if ((p_quad == NULL) || (wheel >= ENCODER_WHEELS))
    {
        return false;
    }
    net  = (word - p_quad->last[wheel]) & QUAD_NET_MASK;
    /* X counts down, so the skips are the old value minus the new one. */
    skip = ((p_quad->last[wheel] >> QUAD_SKIP_SHIFT) - (word >> QUAD_SKIP_SHIFT)) &
           QUAD_SKIP_MASK;
    p_quad->last[wheel] = word;

    if ((net & QUAD_NET_SIGN) != 0U)
    {
        p_quad->wheel[wheel].negative += QUAD_NET_SPAN - net;
    }
    else
    {
        p_quad->wheel[wheel].positive += net;
    }
    p_quad->wheel[wheel].invalid += skip;
    return true;
}

/*** end of file ***/
