/* Inject initialization failures at the kernel and GPIO boundaries. */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "platform/isr.h"
#include "platform/log.h"
#include "track/track.h"

static bool g_semaphores[3];
static unsigned g_created;
static bool g_b_fail_second_sem;
static bool g_b_fail_gpio;
static bool g_b_fail_barcode_handler;
static bool g_b_barcode_handler;
static uint32_t g_timer_enable;
static unsigned g_failures;

FP g_shim_inthdr;
int g_shim_int_enabled;
INT g_shim_prc = TP_PRC1;

#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            (void)fprintf (stderr, "FAIL line %d: %s\n", __LINE__, #cond); \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

ID tk_cre_sem (const T_CSEM * p_def)
{
    (void)p_def;
    g_created++;
    if (g_b_fail_second_sem && (2U == g_created))
    {
        return E_NOMEM;
    }
    if (g_created >= (sizeof (g_semaphores) / sizeof (g_semaphores[0])))
    {
        return E_LIMIT;
    }
    g_semaphores[g_created] = true;
    return (ID)g_created;
}

ER tk_del_sem (ID sem)
{
    if ((sem <= 0) || (sem > 2) || !g_semaphores[sem])
    {
        return E_ID;
    }
    g_semaphores[sem] = false;
    return E_OK;
}

ER tk_sig_sem (ID sem, INT count)
{
    (void)sem;
    (void)count;
    return E_OK;
}

ER tk_wai_sem (ID sem, INT count, TMO timeout)
{
    (void)sem;
    (void)count;
    (void)timeout;
    return E_TMOUT;
}

ER gpio_set_pin (UINT pin, UINT mode)
{
    (void)pin;
    (void)mode;
    return g_b_fail_gpio ? E_PAR : E_OK;
}

ER ir_capture_test_def_int (UINT vector, const T_DINT * p_definition)
{
    (void)vector;
    g_shim_inthdr = (NULL != p_definition) ? p_definition->inthdr : NULL;
    return E_OK;
}

uint32_t shim_reg_read (uint32_t addr)
{
    return TIMER_INTE == addr ? g_timer_enable : 0U;
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    if (TIMER_INTE == addr)
    {
        g_timer_enable = value;
    }
}

ER gpio_irq_add_handler (UW proc, uint32_t mask, gpio_irq_handler_t handler)
{
    (void)proc;
    (void)mask;
    (void)handler;
    g_b_barcode_handler = !g_b_fail_barcode_handler;
    return g_b_barcode_handler ? E_OK : E_LIMIT;
}

void log_write (log_sev_t severity, log_task_t task, log_mod_t module, const char * p_format, ...)
{
    (void)severity;
    (void)task;
    (void)module;
    (void)p_format;
}

int main (int argc, char ** argv)
{
    ER anticipated = E_OK;

    if (2 != argc)
    {
        (void)fprintf (stderr, "Usage: %s second-semaphore|pin-setup|barcode-handler\n", argv[0]);
        return 2;
    }
    if (0 == strcmp (argv[1], "second-semaphore"))
    {
        g_b_fail_second_sem = true;
        anticipated = E_NOMEM;
    }
    else if (0 == strcmp (argv[1], "pin-setup"))
    {
        g_b_fail_gpio = true;
        anticipated = E_PAR;
    }
    else if (0 == strcmp (argv[1], "barcode-handler"))
    {
        g_b_fail_barcode_handler = true;
        anticipated = E_LIMIT;
    }
    else
    {
        return 2;
    }

    CHECK (anticipated == ir_capture_init ());
    CHECK (!g_semaphores[1]);
    CHECK (!g_semaphores[2]);
    CHECK (!g_b_barcode_handler);
    CHECK (NULL == g_shim_inthdr);
    CHECK (0 == g_shim_int_enabled);
    CHECK (0U == g_timer_enable);
    return (0U == g_failures) ? 0 : 1;
}
