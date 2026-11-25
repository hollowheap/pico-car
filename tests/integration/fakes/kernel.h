#ifndef INTEGRATION_KERNEL_H
#define INTEGRATION_KERNEL_H

/* Force-included into every firmware file of the integration binary. It adds
 * the kernel calls the shared platform/fakes headers leave out, and defines
 * the supervisor configuration the way the supervisor harness does. */

#include <stddef.h>
#include <stdint.h>

/* The IRQ shim uses a no-argument handler. Firmware task APIs also pass
 * their entry points as the kernel's generic function pointer. */
#define FP integration_irq_handler_t
#include <tk/flg_fake.h>
#undef FP
typedef void (*FP)();

#define E_SYS     (-5)
#define E_IO      (-57)
#define TSK_SELF  (0)
#define TA_STA    (0x02U)
#define TA_RNG3   (0x300U)
#define TA_ASSPRC (0x00010000U)
#define CNF_MAX_TSKID (32U)
#define CNF_MAX_CYCID (8U)

typedef uint32_t RELTIM;
typedef struct { int32_t hi; uint32_t lo; } SYSTIM;

typedef struct
{
    void * exinf;
    ATR    tskatr;
    FP     task;
    PRI    itskpri;
    SZ     stksz;
    void * bufptr;
    UINT   assprc;
} T_CTSK;

typedef struct
{
    void * exinf;
    ATR    cycatr;
    FP     cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
    UINT   assprc;
} T_CCYC;

ER  tk_get_otm (SYSTIM * p_time);
ID  tk_get_tid (void);
ER  tk_slp_tsk (TMO timeout);
ER  tk_wup_tsk (ID task);
INT tk_can_wup (ID task);
ID  tk_cre_tsk (const T_CTSK * p_task);
ER  tk_sta_tsk (ID task, INT code);
ER  tk_exd_tsk (void);
ID  tk_cre_cyc (const T_CCYC * p_config);
ER  tk_del_cyc (ID cyclic);
ER  tk_clr_flg (ID flag, UINT pattern);

#pragma pack(push, 4)
#include "supervisor/supervisor.h"
#pragma pack(pop)

#endif
