#ifndef SHIM_TCP_LWIPOPTS_H
#define SHIM_TCP_LWIPOPTS_H

#define NO_SYS                  1
#define SYS_LIGHTWEIGHT_PROT    0
#define LWIP_TCP                1
#define LWIP_DNS                1
#define LWIP_IPV4               1
#define LWIP_IPV6               0
#define LWIP_SOCKET             0
#define LWIP_NETCONN            0
#define MEM_ALIGNMENT           4
/* The firmware value; the radio timer needs one slot beyond the stack's own. */
#define MEMP_NUM_SYS_TIMEOUT    8

#endif /* SHIM_TCP_LWIPOPTS_H */
