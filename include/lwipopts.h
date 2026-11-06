#ifndef PROJECT_LWIPOPTS_H
#define PROJECT_LWIPOPTS_H

/* Use the kernel's lwIP port configuration, then enable the application TCP
 * path without pulling in the kernel's fixed TCP test harness. */
#include_next <lwipopts.h>

#undef LWIP_TCP
#define LWIP_TCP 1

#endif /* PROJECT_LWIPOPTS_H */
