/** @file encoder.c
 * @brief The PIO quadrature counters and a protected snapshot.
 */
#include <tk/tkernel.h>
#include <tk/smp.h>

#include "drv_encoder_pads.h"
#include "motion_pins.h"
#include "encoder.h"
#include "drv_encoder.h"
#include "platform/drv_timer.h"

static const unsigned int g_pin_a[ENCODER_WHEELS] = {
    MOTOR_ENCODER_LEFT_A, MOTOR_ENCODER_RIGHT_A,
};

/* Serialises PIO FIFO access and state updates without masking IRQs during
 * FIFO waits. */
static ID           g_pio_mtx = 0;
static T_SPLOCK     g_capture_lock;
static quadrature_t g_captured;
/* Lifecycle belongs to the initialisation task, before any readers start. */
static bool g_initialised = false;

ER encoder_init (void)
{
    T_CMTX   cmtx                  = {0};
    UINT     saved                 = 0U;
    ER       result                = E_OK;
    ID       mtx_id                = 0;
    uint32_t words[ENCODER_WHEELS] = {0U};

    if (g_initialised)
    {
        return E_OBJ;
    }

    drv_encoder_pads_init(g_pin_a);

    /* TA_INHERIT alone: TA_INHERIT | TA_TPRI has the value of TA_CEILING. */
    cmtx.mtxatr = TA_INHERIT;
    mtx_id      = tk_cre_mtx(&cmtx);
    if (mtx_id <= 0)
    {
        return (mtx_id < 0) ? (ER)mtx_id : E_SYS;
    }

    if (!encoder_pio_start(g_pin_a))
    {
        (void)tk_del_mtx(mtx_id);
        return E_LIMIT;
    }

    (void)InitSpinLock(&g_capture_lock);
    result = tk_loc_mtx(mtx_id, TMO_FEVR);
    if (E_OK != result)
    {
        encoder_pio_stop();
        (void)tk_del_mtx(mtx_id);
        return result;
    }
    if (!encoder_pio_read(words))
    {
        encoder_pio_stop();
        (void)tk_unl_mtx(mtx_id);
        (void)tk_del_mtx(mtx_id);
        return E_IO;
    }

    (void)ISpinLock(&g_capture_lock, &saved);
    quadrature_init(&g_captured, words);
    g_pio_mtx     = mtx_id;
    g_initialised = true;
    (void)ISpinUnlock(&g_capture_lock, saved);
    (void)tk_unl_mtx(mtx_id);
    return E_OK;
}

ER encoder_read (encoder_sample_t * p_sample)
{
    UINT     saved                 = 0U;
    ER       result                = E_OK;
    uint32_t words[ENCODER_WHEELS] = {0U};

    if (p_sample == NULL)
    {
        return E_PAR;
    }
    /* Initialisation must finish before consumers are started. */
    if (!g_initialised)
    {
        return E_OBJ;
    }

    result = tk_loc_mtx(g_pio_mtx, TMO_FEVR);
    if (E_OK != result)
    {
        return result;
    }
    if (!encoder_pio_read(words))
    {
        (void)tk_unl_mtx(g_pio_mtx);
        return E_IO;
    }

    (void)ISpinLock(&g_capture_lock, &saved);
    for (unsigned int wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        (void)quadrature_apply(&g_captured, wheel, words[wheel]);
        p_sample->wheel[wheel] = g_captured.wheel[wheel];
    }
    p_sample->timestamp_us = now_us();
    (void)ISpinUnlock(&g_capture_lock, saved);
    (void)tk_unl_mtx(g_pio_mtx);
    return E_OK;
}

ER encoder_reset (void)
{
    UINT     saved                 = 0U;
    ER       result                = E_OK;
    uint32_t words[ENCODER_WHEELS] = {0U};

    if (!g_initialised)
    {
        return E_OBJ;
    }

    result = tk_loc_mtx(g_pio_mtx, TMO_FEVR);
    if (E_OK != result)
    {
        return result;
    }
    if (!encoder_pio_read(words))
    {
        (void)tk_unl_mtx(g_pio_mtx);
        return E_IO;
    }

    (void)ISpinLock(&g_capture_lock, &saved);
    quadrature_init(&g_captured, words);
    (void)ISpinUnlock(&g_capture_lock, saved);
    (void)tk_unl_mtx(g_pio_mtx);
    return E_OK;
}

/*** end of file ***/
