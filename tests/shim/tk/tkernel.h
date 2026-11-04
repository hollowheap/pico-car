/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	tests/shim/tk/tkernel.h
 *	Host-only stand-in for the kernel's tk/tkernel.h.  Defines the UW type
 *	used by the SMP atomic prototypes.  Not used by firmware builds.
 */

#ifndef TEST_SHIM_TK_TKERNEL_H
#define TEST_SHIM_TK_TKERNEL_H

#include <stdint.h>

typedef uint32_t UW;
typedef int      ER;

#endif /* TEST_SHIM_TK_TKERNEL_H */
