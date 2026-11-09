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

/* Message buffer API used by src/telemetry/queue.c and net_queue.c. Values
   match lib/mtk3smp-rp2040/include/tk. The functions are defined by the test,
   normally through tests/shim/tk/mbf_fake.h. */
typedef int          ID;
typedef int32_t      TMO;
typedef int32_t      SZ;
typedef unsigned int ATR;

#define E_PAR   (-17)
#define E_CTX   (-25)
#define E_ILUSE (-28)
#define E_ID    (-18)
#define E_NOMEM (-33)
#define E_NOEXS (-42)
#define E_TMOUT (-50)

#define TA_TPRI    0x00000001UL
#define TA_INHERIT 0x00000002UL
#define TA_USERBUF 0x00000020UL

#define TMO_POL  (0)
#define TMO_FEVR (-1)

typedef struct
{
    void * exinf;
    ATR    mbfatr;
    SZ     bufsz;
    INT    maxmsz;
    void * bufptr;
} T_CMBF;

/* Message buffer state, as in lib/mtk3smp-rp2040/include/tk/syscall.h. */
typedef struct
{
    void * exinf;
    ID     wtsk;
    ID     stsk;
    INT    msgsz;   /* size of the next message to receive, 0 when empty */
    SZ     frbufsz;
    INT    maxmsz;
} T_RMBF;

/* Mutex API used by src/telemetry/queue.c, as in
   lib/mtk3smp-rp2040/include/tk/syscall.h; defined by
   tests/shim/tk/mbf_fake.h. */
typedef int PRI;

typedef struct
{
    void * exinf;
    ATR    mtxatr;
    PRI    ceilpri;
} T_CMTX;

extern ID tk_cre_mtx (const T_CMTX * pk_cmtx);
extern ER tk_del_mtx (ID mtxid);
extern ER tk_loc_mtx (ID mtxid, TMO tmout);
extern ER tk_unl_mtx (ID mtxid);

extern ID  tk_cre_mbf (const T_CMBF * pk_cmbf);
extern ER  tk_snd_mbf (ID mbfid, const void * msg, INT msgsz, TMO tmout);
extern INT tk_rcv_mbf (ID mbfid, void * msg, TMO tmout);
extern ER  tk_ref_mbf (ID mbfid, T_RMBF * pk_rmbf);

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
