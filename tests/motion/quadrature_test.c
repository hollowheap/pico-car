/** @file quadrature_test.c
 * @brief Host test for the quadrature counter: the PIO program, run on a
 *        small model of one state machine, and the word decoder.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "quadrature.h"
#include "quadrature.pio.h"

static unsigned g_fail;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            g_fail++;                                                        \
            (void)printf("FAIL line %d: %s\n", __LINE__, #cond);             \
        }                                                                    \
    } while (0)

/* Encodings from the RP2040 datasheet, section 3.4, as the driver runs them
 * before it starts a state machine. */
#define OP_IN_PINS_2    0x4002U
#define OP_MOV_OSR_ISR  0xA0E6U
#define OP_SET_X_0      0xE020U
#define OP_SET_Y_0      0xE040U
#define STEP_LIMIT      64U

/* One state machine: IN shifts left, OUT shifts right, no autopush. */
typedef struct
{
    uint32_t x;
    uint32_t y;
    uint32_t isr;
    uint32_t osr;
    uint32_t pins; /* bit 0 is A, bit 1 is B */
    uint8_t  pc;
    bool     b_pushed;
    uint32_t pushed;
} sm_t;

static uint32_t mask_of (uint32_t bits)
{
    return (bits >= 32U) ? 0xFFFFFFFFU : ((1U << bits) - 1U);
}

static uint32_t source (const sm_t * p_sm, uint32_t code)
{
    uint32_t value = 0U;

    switch (code)
    {
        case 0U: value = p_sm->pins; break;
        case 1U: value = p_sm->x; break;
        case 2U: value = p_sm->y; break;
        case 6U: value = p_sm->isr; break;
        case 7U: value = p_sm->osr; break;
        default: value = 0U; break;
    }
    return value;
}

/* Run one instruction. Returns false for an encoding the model lacks. */
static bool exec (sm_t * p_sm, uint16_t op)
{
    uint32_t const kind  = (uint32_t)op >> 13U;
    uint32_t const arg   = ((uint32_t)op >> 5U) & 7U;
    uint32_t const low   = (uint32_t)op & 0x1FU;
    uint32_t const bits  = (0U == low) ? 32U : low;
    uint8_t const  here  = p_sm->pc;
    bool           b_jmp = false;
    bool           b_ok  = true;

    switch (kind)
    {
        case 0U: /* JMP */
            if (0U == arg)
            {
                b_jmp = true;
            }
            else if (2U == arg)
            {
                b_jmp = (0U != p_sm->x);
                p_sm->x--;
            }
            else if (4U == arg)
            {
                b_jmp = (0U != p_sm->y);
                p_sm->y--;
            }
            else
            {
                b_ok = false;
            }
            if (b_jmp)
            {
                p_sm->pc = (uint8_t)low;
            }
            break;
        case 2U: /* IN */
            p_sm->isr = ((bits >= 32U) ? 0U : (p_sm->isr << bits)) |
                        (source(p_sm, arg) & mask_of(bits));
            break;
        case 3U: /* OUT isr */
            b_ok      = (6U == arg);
            p_sm->isr = p_sm->osr & mask_of(bits);
            p_sm->osr = (bits >= 32U) ? 0U : (p_sm->osr >> bits);
            break;
        case 4U: /* PUSH noblock */
            b_ok           = (0x8000U == op);
            p_sm->pushed   = p_sm->isr;
            p_sm->b_pushed = true;
            p_sm->isr      = 0U;
            break;
        case 5U: /* MOV */
        {
            uint32_t const op_bits = (low >> 3U) & 3U;
            uint32_t       value   = source(p_sm, low & 7U);

            value = (1U == op_bits) ? ~value : value;
            switch (arg)
            {
                case 2U: p_sm->y = value; break;
                case 5U:
                    p_sm->pc = (uint8_t)(value & 0x1FU);
                    b_jmp    = true;
                    break;
                case 7U: p_sm->osr = value; break;
                default: b_ok = false; break;
            }
            break;
        }
        case 7U: /* SET */
            if (1U == arg)
            {
                p_sm->x = low;
            }
            else
            {
                p_sm->y = low;
            }
            break;
        default:
            b_ok = false;
            break;
    }
    if (!b_jmp)
    {
        p_sm->pc = (here == quadrature_wrap) ? (uint8_t)quadrature_wrap_target
                                             : (uint8_t)(here + 1U);
    }
    return b_ok;
}

/* Run the program until it pushes; return the word. */
static uint32_t run_to_push (sm_t * p_sm)
{
    unsigned steps = 0U;

    p_sm->b_pushed = false;
    while (!p_sm->b_pushed && (steps < STEP_LIMIT))
    {
        CHECK(exec(p_sm, quadrature_program_instructions[p_sm->pc]));
        steps++;
    }
    CHECK(p_sm->b_pushed);
    return p_sm->pushed;
}

/* Start as the driver does: last state from the pins, X = Y = 0. */
static uint32_t start (sm_t * p_sm, uint32_t pins)
{
    p_sm->pins = pins;
    p_sm->isr  = 0U;
    p_sm->osr  = 0U;
    CHECK(exec(p_sm, (uint16_t)OP_IN_PINS_2));
    CHECK(exec(p_sm, (uint16_t)OP_MOV_OSR_ISR));
    CHECK(exec(p_sm, (uint16_t)OP_SET_X_0));
    CHECK(exec(p_sm, (uint16_t)OP_SET_Y_0));
    p_sm->pc = (uint8_t)quadrature_wrap_target;
    return run_to_push(p_sm);
}

/* Pin states (bit 0 A, bit 1 B) for the positive direction: the order
 * A,B = 00, 01, 11, 10, as the old edge capture counted it. */
static const uint32_t g_forward[4] = { 0U, 2U, 3U, 1U };

static void feed (sm_t * p_sm, quadrature_t * p_quad, uint32_t pins)
{
    p_sm->pins = pins;
    CHECK(quadrature_apply(p_quad, ENCODER_RIGHT, run_to_push(p_sm)));
}

static void test_program_counts_steps (void)
{
    sm_t           sm    = { 0 };
    quadrature_t   quad;
    uint32_t       words[ENCODER_WHEELS] = { 0U, 0U };
    unsigned       step  = 0U;

    words[ENCODER_RIGHT] = start(&sm, g_forward[0]);
    CHECK(0U == words[ENCODER_RIGHT]);
    quadrature_init(&quad, words);

    /* Eight forward steps, a held state, then three back. */
    for (step = 1U; step <= 8U; step++)
    {
        feed(&sm, &quad, g_forward[step % 4U]);
    }
    feed(&sm, &quad, g_forward[0]);
    CHECK(8U == quad.wheel[ENCODER_RIGHT].positive);
    CHECK(0U == quad.wheel[ENCODER_RIGHT].negative);
    for (step = 3U; step >= 1U; step--)
    {
        feed(&sm, &quad, g_forward[step]);
    }
    CHECK(8U == quad.wheel[ENCODER_RIGHT].positive);
    CHECK(3U == quad.wheel[ENCODER_RIGHT].negative);
    CHECK(0U == quad.wheel[ENCODER_RIGHT].invalid);

    /* A skipped state (both pins change) counts as invalid, no distance. */
    feed(&sm, &quad, g_forward[3]); /* back at 1 -> 3 is a skip */
    CHECK(1U == quad.wheel[ENCODER_RIGHT].invalid);
    CHECK(8U == quad.wheel[ENCODER_RIGHT].positive);
    CHECK(3U == quad.wheel[ENCODER_RIGHT].negative);
    /* Counting goes on from the new state. */
    feed(&sm, &quad, g_forward[0]);
    CHECK(9U == quad.wheel[ENCODER_RIGHT].positive);
    /* The other wheel is untouched. */
    CHECK(0U == quad.wheel[ENCODER_LEFT].positive);
}

/* Every one of the 16 transitions, from every start state. */
static void test_program_table (void)
{
    uint32_t from = 0U;
    uint32_t to   = 0U;

    for (from = 0U; from < 4U; from++)
    {
        for (to = 0U; to < 4U; to++)
        {
            sm_t         sm = { 0 };
            quadrature_t quad;
            uint32_t     words[ENCODER_WHEELS] = { 0U, 0U };
            uint32_t     pos_from = 0U;
            uint32_t     pos_to   = 0U;
            uint32_t     ahead    = 0U;

            words[ENCODER_RIGHT] = start(&sm, from);
            quadrature_init(&quad, words);
            feed(&sm, &quad, to);
            while (g_forward[pos_from] != from)
            {
                pos_from++;
            }
            while (g_forward[pos_to] != to)
            {
                pos_to++;
            }
            ahead = (pos_to + 4U - pos_from) % 4U;
            CHECK(quad.wheel[ENCODER_RIGHT].positive == ((1U == ahead) ? 1U : 0U));
            CHECK(quad.wheel[ENCODER_RIGHT].negative == ((3U == ahead) ? 1U : 0U));
            CHECK(quad.wheel[ENCODER_RIGHT].invalid == ((2U == ahead) ? 1U : 0U));
        }
    }
}

/* The decoder extends the 24-bit net count and the 8-bit skip count. */
static void test_decoder_wraps (void)
{
    quadrature_t   quad;
    uint32_t const words[ENCODER_WHEELS] = { 0x00FFFFF0U, 0x01000005U };

    quadrature_init(&quad, words);
    CHECK(0U == quad.wheel[ENCODER_LEFT].positive);
    CHECK(0U == quad.wheel[ENCODER_RIGHT].invalid);

    /* Net +32 across the 24-bit wrap. */
    CHECK(quadrature_apply(&quad, ENCODER_LEFT, 0x00000010U));
    CHECK(32U == quad.wheel[ENCODER_LEFT].positive);
    /* Net -20 across the wrap back. */
    CHECK(quadrature_apply(&quad, ENCODER_LEFT, 0x00FFFFFCU));
    CHECK(20U == quad.wheel[ENCODER_LEFT].negative);

    /* X counts down: from 1 to 0xFE is three skips, across the wrap. */
    CHECK(quadrature_apply(&quad, ENCODER_RIGHT, 0xFE000005U));
    CHECK(3U == quad.wheel[ENCODER_RIGHT].invalid);
    CHECK(0U == quad.wheel[ENCODER_RIGHT].positive);
    CHECK(0U == quad.wheel[ENCODER_RIGHT].negative);

    CHECK(!quadrature_apply(&quad, ENCODER_WHEELS, 0U));
    CHECK(!quadrature_apply(NULL, ENCODER_LEFT, 0U));
    quadrature_init(NULL, words); /* must not crash */
}

int main (void)
{
    test_program_counts_steps();
    test_program_table();
    test_decoder_wraps();

    if (g_fail != 0U)
    {
        (void)printf("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf("test_quadrature: all checks passed\n");
    return 0;
}
