/** @file mbf_fake.h
 * @brief Host fake of the uT-Kernel 3.0 message buffer (tk_cre_mbf, tk_snd_mbf,
 *        tk_rcv_mbf, tk_ref_mbf) and of one mutex (tk_cre_mtx, tk_del_mtx,
 *        tk_loc_mtx, tk_unl_mtx), for tests of src/telemetry/queue.c.
 *
 * Include this header in exactly one translation unit per test; it defines the
 * functions that tests/shim/tk/tkernel.h declares.
 *
 * The storage model is copied from lib/mtk3smp-rp2040/kernel/tkernel/
 * messagebuf.c and messagebuf.h, so capacity and wrap-around match the target:
 *   - A message costs a 4-byte header plus its size rounded up to 4 bytes.
 *   - A send fits when `4 + msgsz <= free bytes` (the kernel tests the
 *     unrounded size, which is equivalent because free space is a multiple of
 *     4).
 *   - Messages sit in a ring over the user buffer; header and body wrap as in
 *     knl_msg_to_mbf and knl_mbf_to_msg.
 *   - tk_snd_mbf rejects msgsz <= 0 and msgsz > maxmsz with E_PAR.
 *   - tk_rcv_mbf returns the stored message size, not the rounded size.
 *   - tk_ref_mbf reports `msgsz` as the stored size of the oldest message
 *     (the 4-byte header at the head, as in the kernel), or 0 when the buffer
 *     is empty and no sender waits (the fake has no wait queues, so `wtsk`
 *     and `stsk` are 0). It also reports `exinf`, `frbufsz` and `maxmsz`.
 *     Errors: E_ID for an out-of-range id, E_NOEXS for an unused slot.
 *
 * Differences from the kernel:
 *   - `recv_hook`, when set, runs once at the start of the next tk_rcv_mbf
 *     (it is cleared first, so the hook may call tk_rcv_mbf itself). A test
 *     uses it to model another core that pops between a peek (tk_ref_mbf)
 *     and the receive.
 *   - Nothing can block. A send to a full buffer, or a receive from an empty
 *     one, returns E_TMOUT whatever the timeout. When the timeout is not
 *     TMO_POL the call also increments `would_block`, because the kernel would
 *     have put the caller to sleep.
 *   - A user buffer pointer that is NULL, or a size that is not a multiple of
 *     4, is rejected with E_PAR (the kernel rejects only the size).
 *   - There is no wait queue, so there is no direct hand-off to a waiting task.
 */
#ifndef SHIM_TK_MBF_FAKE_H
#define SHIM_TK_MBF_FAKE_H

#include <stdint.h>
#include <string.h>

#include <tk/tkernel.h>

#define MBF_FAKE_SLOTS  2
#define MBF_FAKE_HEADER 4U

/** Bytes a message of `size` bytes occupies in a message buffer. */
#define MBF_FAKE_COST(size) \
    (MBF_FAKE_HEADER + (((uint32_t)(size) + 3U) & ~(uint32_t)3U))

typedef struct
{
    int      used;
    T_CMBF   cfg;
    uint8_t *buf;
    int32_t  bufsz;
    int32_t  frbufsz;
    int32_t  head;
    int32_t  tail;
} mbf_fake_slot_t;

typedef struct
{
    mbf_fake_slot_t slot[MBF_FAKE_SLOTS];

    /** One-shot error injection: a nonzero value is returned by the next call
     *  of that function and then cleared. */
    ER create_error;
    ER send_error;
    ER recv_error;
    ER ref_error;

    /** One-shot callback at the start of the next tk_rcv_mbf. */
    void (*recv_hook)(void);

    unsigned create_calls;
    unsigned send_calls;
    unsigned recv_calls;
    unsigned ref_calls;
    unsigned would_block; /* calls that the kernel would have blocked */

    /* Mutex used by queue.c to serialise producers (see the mutex fake at the
       end of this file). */
    int      mtx_used;
    int      mtx_locked;
    ATR      mtx_atr;
    ER       mtx_create_error;
    ER       mtx_lock_error;
    unsigned mtx_create_calls;
    unsigned mtx_del_calls;
    unsigned mtx_lock_calls;
    unsigned mtx_unlock_calls;
    unsigned mtx_wait_forever; /* tk_loc_mtx calls with TMO_FEVR */
    unsigned snd_ref_unlocked; /* tk_snd_mbf/tk_ref_mbf outside the mutex */
    unsigned rcv_locked;       /* tk_rcv_mbf calls made while it was held */

    T_CMBF last_cmbf;     /* copy of the last tk_cre_mbf argument */
    TMO    last_snd_tmout;
    TMO    last_rcv_tmout;
} mbf_fake_t;

static mbf_fake_t g_mbf_fake;

/** Records a send or peek made while the producer mutex is not held. The
 *  queue must call tk_snd_mbf and tk_ref_mbf only inside it. */
static inline void mbf_fake_note_unlocked (void)
{
    if (!g_mbf_fake.mtx_locked)
    {
        g_mbf_fake.snd_ref_unlocked++;
    }
}

static inline void mbf_fake_reset (void)
{
    memset(&g_mbf_fake, 0, sizeof g_mbf_fake);
}

/** Free bytes in buffer `id`, or -1 for an unknown id. */
static inline int32_t mbf_fake_free (ID id)
{
    if ((id <= 0) || (id > MBF_FAKE_SLOTS) || !g_mbf_fake.slot[id - 1].used)
    {
        return -1;
    }
    return g_mbf_fake.slot[id - 1].frbufsz;
}

static inline mbf_fake_slot_t * mbf_fake_slot (ID id)
{
    if ((id <= 0) || (id > MBF_FAKE_SLOTS))
    {
        return 0;
    }
    return &g_mbf_fake.slot[id - 1];
}

ID tk_cre_mbf (const T_CMBF * pk_cmbf)
{
    int i;

    g_mbf_fake.create_calls++;
    g_mbf_fake.last_cmbf = *pk_cmbf;
    if (g_mbf_fake.create_error != 0)
    {
        const ER injected = g_mbf_fake.create_error;

        g_mbf_fake.create_error = 0;
        return injected;
    }
    if ((pk_cmbf->bufsz < 0) || (pk_cmbf->maxmsz <= 0))
    {
        return E_PAR;
    }
    if ((pk_cmbf->mbfatr & TA_USERBUF) == 0U)
    {
        return E_PAR; /* the firmware has no kernel heap for buffers */
    }
    if (((uint32_t)pk_cmbf->bufsz % MBF_FAKE_HEADER) != 0U)
    {
        return E_PAR;
    }
    if ((pk_cmbf->bufsz > 0) && (pk_cmbf->bufptr == 0))
    {
        return E_PAR;
    }
    for (i = 0; i < MBF_FAKE_SLOTS; i++)
    {
        mbf_fake_slot_t * s = &g_mbf_fake.slot[i];

        if (!s->used)
        {
            s->used    = 1;
            s->cfg     = *pk_cmbf;
            s->buf     = (uint8_t *)pk_cmbf->bufptr;
            s->bufsz   = (int32_t)pk_cmbf->bufsz;
            s->frbufsz = s->bufsz;
            s->head    = 0;
            s->tail    = 0;
            return i + 1;
        }
    }
    return E_LIMIT;
}

static inline void mbf_fake_store (mbf_fake_slot_t * s, const uint8_t * msg,
                                   INT msgsz)
{
    int32_t tail = s->tail;
    int32_t remsz;

    s->frbufsz -= (int32_t)MBF_FAKE_COST(msgsz);
    memcpy(&s->buf[tail], &msgsz, sizeof msgsz);
    tail += (int32_t)MBF_FAKE_HEADER;
    if (tail >= s->bufsz)
    {
        tail = 0;
    }
    remsz = s->bufsz - tail;
    if (remsz < (int32_t)msgsz)
    {
        memcpy(&s->buf[tail], msg, (size_t)remsz);
        msg += remsz;
        msgsz -= (INT)remsz;
        tail = 0;
    }
    memcpy(&s->buf[tail], msg, (size_t)msgsz);
    tail += (int32_t)(MBF_FAKE_COST(msgsz) - MBF_FAKE_HEADER);
    if (tail >= s->bufsz)
    {
        tail = 0;
    }
    s->tail = tail;
}

ER tk_snd_mbf (ID mbfid, const void * msg, INT msgsz, TMO tmout)
{
    mbf_fake_slot_t * s = mbf_fake_slot(mbfid);

    g_mbf_fake.send_calls++;
    mbf_fake_note_unlocked();
    g_mbf_fake.last_snd_tmout = tmout;
    if (g_mbf_fake.send_error != 0)
    {
        const ER injected = g_mbf_fake.send_error;

        g_mbf_fake.send_error = 0;
        return injected;
    }
    if (s == 0)
    {
        return E_ID;
    }
    if (msgsz <= 0)
    {
        return E_PAR;
    }
    if (!s->used)
    {
        return E_NOEXS;
    }
    if (msgsz > s->cfg.maxmsz)
    {
        return E_PAR;
    }
    if ((MBF_FAKE_HEADER + (uint32_t)msgsz) > (uint32_t)s->frbufsz)
    {
        if (tmout != TMO_POL)
        {
            g_mbf_fake.would_block++;
        }
        return E_TMOUT;
    }
    mbf_fake_store(s, (const uint8_t *)msg, msgsz);
    return E_OK;
}

INT tk_rcv_mbf (ID mbfid, void * msg, TMO tmout)
{
    mbf_fake_slot_t * s = mbf_fake_slot(mbfid);
    uint8_t *         out = (uint8_t *)msg;
    int32_t           head;
    int32_t           remsz;
    INT               msgsz;
    INT               actsz;

    if (g_mbf_fake.recv_hook != 0)
    {
        void (*hook)(void) = g_mbf_fake.recv_hook;

        g_mbf_fake.recv_hook = 0;
        hook();
    }
    g_mbf_fake.recv_calls++;
    if (g_mbf_fake.mtx_locked)
    {
        g_mbf_fake.rcv_locked++;
    }
    g_mbf_fake.last_rcv_tmout = tmout;
    if (g_mbf_fake.recv_error != 0)
    {
        const ER injected = g_mbf_fake.recv_error;

        g_mbf_fake.recv_error = 0;
        return injected;
    }
    if (s == 0)
    {
        return E_ID;
    }
    if (!s->used)
    {
        return E_NOEXS;
    }
    if (s->frbufsz == s->bufsz)
    {
        if (tmout != TMO_POL)
        {
            g_mbf_fake.would_block++;
        }
        return E_TMOUT;
    }

    head = s->head;
    memcpy(&msgsz, &s->buf[head], sizeof msgsz);
    actsz = msgsz;
    s->frbufsz += (int32_t)MBF_FAKE_COST(msgsz);
    head += (int32_t)MBF_FAKE_HEADER;
    if (head >= s->bufsz)
    {
        head = 0;
    }
    remsz = s->bufsz - head;
    if (remsz < (int32_t)msgsz)
    {
        memcpy(out, &s->buf[head], (size_t)remsz);
        out += remsz;
        msgsz -= (INT)remsz;
        head = 0;
    }
    memcpy(out, &s->buf[head], (size_t)msgsz);
    head += (int32_t)(MBF_FAKE_COST(msgsz) - MBF_FAKE_HEADER);
    if (head >= s->bufsz)
    {
        head = 0;
    }
    s->head = head;
    return actsz;
}

ER tk_ref_mbf (ID mbfid, T_RMBF * pk_rmbf)
{
    mbf_fake_slot_t * s = mbf_fake_slot(mbfid);

    g_mbf_fake.ref_calls++;
    mbf_fake_note_unlocked();
    if (g_mbf_fake.ref_error != 0)
    {
        const ER injected = g_mbf_fake.ref_error;

        g_mbf_fake.ref_error = 0;
        return injected;
    }
    if (s == 0)
    {
        return E_ID;
    }
    if (!s->used)
    {
        return E_NOEXS;
    }
    pk_rmbf->exinf = s->cfg.exinf;
    pk_rmbf->wtsk  = 0;
    pk_rmbf->stsk  = 0;
    if (s->frbufsz != s->bufsz)
    {
        INT msgsz;

        memcpy(&msgsz, &s->buf[s->head], sizeof msgsz);
        pk_rmbf->msgsz = msgsz;
    }
    else
    {
        pk_rmbf->msgsz = 0;
    }
    pk_rmbf->frbufsz = s->frbufsz;
    pk_rmbf->maxmsz  = s->cfg.maxmsz;
    return E_OK;
}

/* ---- mutex fake ----------------------------------------------------------
 * One mutex, enough for queue.c. The host has one thread, so a lock never
 * waits: tk_loc_mtx on a held mutex returns E_ILUSE (the kernel's code for a
 * task locking a mutex it already holds), which a test sees as a failure.
 * tk_loc_mtx returns E_CTX when `mtx_lock_error` is set to it, as the kernel
 * does under disabled dispatch. */

ID tk_cre_mtx (const T_CMTX * pk_cmtx)
{
    g_mbf_fake.mtx_create_calls++;
    if (g_mbf_fake.mtx_create_error != 0)
    {
        const ER injected = g_mbf_fake.mtx_create_error;

        g_mbf_fake.mtx_create_error = 0;
        return injected;
    }
    if (g_mbf_fake.mtx_used)
    {
        return E_LIMIT;
    }
    g_mbf_fake.mtx_used = 1;
    g_mbf_fake.mtx_locked = 0;
    g_mbf_fake.mtx_atr = pk_cmtx->mtxatr;
    return 1;
}

ER tk_del_mtx (ID mtxid)
{
    g_mbf_fake.mtx_del_calls++;
    if ((mtxid != 1) || !g_mbf_fake.mtx_used)
    {
        return E_NOEXS;
    }
    g_mbf_fake.mtx_used = 0;
    g_mbf_fake.mtx_locked = 0;
    return E_OK;
}

ER tk_loc_mtx (ID mtxid, TMO tmout)
{
    g_mbf_fake.mtx_lock_calls++;
    if (tmout == TMO_FEVR)
    {
        g_mbf_fake.mtx_wait_forever++;
    }
    if (g_mbf_fake.mtx_lock_error != 0)
    {
        const ER injected = g_mbf_fake.mtx_lock_error;

        g_mbf_fake.mtx_lock_error = 0;
        return injected;
    }
    if ((mtxid != 1) || !g_mbf_fake.mtx_used)
    {
        return E_NOEXS;
    }
    if (g_mbf_fake.mtx_locked)
    {
        return E_ILUSE;
    }
    g_mbf_fake.mtx_locked = 1;
    return E_OK;
}

ER tk_unl_mtx (ID mtxid)
{
    g_mbf_fake.mtx_unlock_calls++;
    if ((mtxid != 1) || !g_mbf_fake.mtx_used)
    {
        return E_NOEXS;
    }
    if (!g_mbf_fake.mtx_locked)
    {
        return E_ILUSE;
    }
    g_mbf_fake.mtx_locked = 0;
    return E_OK;
}

#endif /* SHIM_TK_MBF_FAKE_H */
