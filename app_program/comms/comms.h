#ifndef PICO_CAR_COMMS_H
#define PICO_CAR_COMMS_H

#include "telemetry/telemetry.h"
#include "telemetry/wire_schema.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/hug_word.h"
#include "telemetry/queue.h"

/* Runtime override for a decoded letter A=0 through Z=25. */
bool telemetry_barcode_binding (uint8_t letter, uint8_t * p_opcode);

#endif /* PICO_CAR_COMMS_H */
