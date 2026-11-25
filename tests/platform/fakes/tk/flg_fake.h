/** @file flg_fake.h
 * @brief Host shim for the uT-Kernel event flag API and the interrupt-saving
 *        spinlock, on top of tkernel.h and smp.h.
 *
 * Force-included (-include) into src/navigation/obstacle/comms.c and
 * obstacle.c, which include only <tk/tkernel.h> and <tk/smp.h>. Values match
 * lib/mtk3smp-rp2040/include/tk. The test defines the flag functions.
 */
#ifndef SHIM_TK_FLG_FAKE_H
#define SHIM_TK_FLG_FAKE_H

/* The real tkernel.h brings NULL in; the shared shim does not. */
#include <stddef.h>

#include <tk/tkernel.h>
#include <tk/smp.h>

#define E_BUSY (-65)

#define TA_TFIFO   0x00000000UL
#define TA_WMUL    0x00000008UL
#define TWF_ORW    0x00000001U
#define TWF_BITCLR 0x00000020U

typedef struct
{
    void * exinf;
    ATR    flgatr;
    UINT   iflgptn;
} T_CFLG;

extern ID tk_cre_flg (const T_CFLG * pk_cflg);
extern ER tk_set_flg (ID flgid, UINT setptn);
extern ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout);

static inline ER ISpinLock (T_SPLOCK * lock, UINT * intsts)
{
    *intsts = 0U;
    *lock   = 1U;
    return E_OK;
}

static inline ER ISpinUnlock (T_SPLOCK * lock, UINT intsts)
{
    (void)intsts;
    *lock = 0U;
    return E_OK;
}

#endif /* SHIM_TK_FLG_FAKE_H */
