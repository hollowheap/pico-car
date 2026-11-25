#ifndef TELEMETRY_CONNECTION_H
#define TELEMETRY_CONNECTION_H

#include <tk/tkernel.h>

#include "platform/static_assert.h"
#include "telemetry/cmd.h"

typedef enum
{
    CONNECTION_DISCONNECTED = 0,
    CONNECTION_JOINING,
    CONNECTION_WAIT_DHCP,
    CONNECTION_READY,
    CONNECTION_FAULT
} connection_state_t;

STATIC_ASSERT (sizeof(connection_state_t) <= 4U, "connection_state_t size");

/* Injected by the composition root so telemetry never calls mission or
 * motion directly. */
typedef void (*connection_cmd_dispatch_t)(const telemetry_cmd_t * cmd,
                                          void * context);

ER connection_init (connection_cmd_dispatch_t dispatch, void * context);
void connection_run (void);
connection_state_t connection_state (void);

/* A copy of the command state for the diag frame: the last accepted cmd_id
   and the refusals per cause. Any task may call it. */
void connection_cmd_state (telemetry_cmd_state_t * p_out);

#endif /* TELEMETRY_CONNECTION_H */
