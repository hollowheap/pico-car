/** @file tkernel.h
 * @brief Host shim for the uT-Kernel 3.0 API used by src/platform/isr.c.
 *
 * Only the names isr.c and its header need are provided. Register access goes
 * through two hooks that the test defines, so a test can model the hardware.
 */
#ifndef SHIM_TK_TKERNEL_H
#define SHIM_TK_TKERNEL_H

#include <stdint.h>

#define LOCAL  static
#define EXPORT

typedef int          INT;
typedef unsigned int UINT;
typedef unsigned int UW;
typedef int          ER;
typedef void (*FP)(void);

#define E_OK    (0)
#define E_OBJ   (-41)
#define E_LIMIT (-34)

#define TA_HLNG 0x00000001UL
#define TP_PRC1 0x00000001UL
#define TP_PRC2 0x00000002UL

#define TK_SUPPORT_SMP 1

#define IO_BANK0_BASE 0x40014000U
#define TIMER_BASE    0x40054000U

typedef struct
{
    void * exinf;
    UINT   intatr;
    FP     inthdr;
} T_DINT;

/** Register hooks, defined by the test. */
extern uint32_t shim_reg_read (uint32_t addr);
extern void     shim_reg_write (uint32_t addr, uint32_t value);

/** Handler captured by tk_def_int, defined by the test. */
extern FP g_shim_inthdr;

static inline UW in_w (UW addr)
{
    return shim_reg_read(addr);
}

static inline void out_w (UW addr, UW value)
{
    shim_reg_write(addr, value);
}

static inline void set_w (UW addr, UW bits)
{
    shim_reg_write(addr, shim_reg_read(addr) | bits);
}

static inline ER tk_def_int (UINT intno, const T_DINT * pk_dint)
{
    (void)intno;
    g_shim_inthdr = pk_dint->inthdr;
    return E_OK;
}

static inline void EnableInt (UINT intno, INT intpri)
{
    (void)intno;
    (void)intpri;
}

static inline void DisableInt (UINT intno)
{
    (void)intno;
}

static inline void ClearInt (UINT intno)
{
    (void)intno;
}

#endif /* SHIM_TK_TKERNEL_H */
