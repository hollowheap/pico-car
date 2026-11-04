/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	tests/shim/tk/smp.h
 *	Host-only stand-in for the kernel's tk/smp.h.  Declares only the atomic
 *	bit operations with the same signatures as lib/mtk3smp-rp2040/include/tk/smp.h.
 */

#ifndef TEST_SHIM_TK_SMP_H
#define TEST_SHIM_TK_SMP_H

#include <tk/tkernel.h>

extern UW atomic_bitset(UW *addr, UW setptn);
extern UW atomic_bitclr(UW *addr, UW clrptn);

#endif /* TEST_SHIM_TK_SMP_H */
