#ifndef SHIM_TCP_TCP_STUBS_H
#define SHIM_TCP_TCP_STUBS_H

#include <stdint.h>

#include "lwip/err.h"
#include "lwip/tcp.h"

typedef struct
{
    struct tcp_pcb pcb;
    tcp_connected_fn on_connected;
    tcp_recv_fn on_recv;
    tcp_err_fn on_error;
    err_t connect_result;
    err_t write_result;
    err_t close_result;
    unsigned aborts;
    unsigned closes;
    unsigned writes;
    unsigned written_bytes;
    uint32_t now_ms;
    unsigned sys_now_calls;
    unsigned log_calls;
    void (*abort_hook)(void);
    void (*sys_now_hook)(void);
    void (*log_hook)(void);
    uint32_t link_up;
    uint32_t dhcp_complete;
    int32_t dhcp_result;
} tcp_stubs_t;

extern tcp_stubs_t g_stubs;

void tcp_stubs_reset (void);

#endif /* SHIM_TCP_TCP_STUBS_H */
