#include "obstacle/extras/drv_sonar.h"
#include "platform/faults.h"

INT g_shim_prc = 1;
static UINT g_pattern;

ID tk_cre_flg (const T_CFLG * p_config)
{
    g_pattern = p_config->iflgptn;
    return 1;
}

ER tk_set_flg (ID id, UINT pattern)
{
    (void)id;
    g_pattern |= pattern;
    return E_OK;
}

ER tk_wai_flg (ID id, UINT pattern, UINT mode, UINT * p_result, TMO timeout)
{
    (void)id;
    (void)timeout;
    if (0U == (g_pattern & pattern))
    {
        return E_TMOUT;
    }
    *p_result = g_pattern;
    if (0U != (mode & TWF_BITCLR))
    {
        g_pattern &= ~pattern;
    }
    return E_OK;
}

ER tk_snd_mbf (ID id, const void * p_message, INT size, TMO timeout)
{
    (void)id;
    (void)p_message;
    (void)size;
    (void)timeout;
    return E_OK;
}

ER extras_sonar_init (ID h_echo_flag)
{
    (void)h_echo_flag;
    return E_OK;
}

void system_fault_assign (fault_owner_t owner, uint32_t mask, uint32_t active)
{
    (void)owner;
    (void)mask;
    (void)active;
}
