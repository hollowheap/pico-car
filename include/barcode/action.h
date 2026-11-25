/** @file action.h
 * @brief What a decoded barcode asks the car to do (D130). Pure C.
 *
 * A barcode is one letter. S, L, R and U mean stop, turn left, turn right and
 * U-turn. Any other letter is rejected. Two helpers belong with it: the 100 mm
 * re-arm gate, which stops one marker from acting twice, and the 250 ms
 * window after a barcode edge in which the line follower ignores the stripes.
 */

#ifndef BARCODE_ACTION_H
#define BARCODE_ACTION_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/* Action codes. They match the barcode_action enum of the wire schema. */
#define BARCODE_ACTION_NONE     (0U)
#define BARCODE_ACTION_STOP     (1U)
#define BARCODE_ACTION_LEFT     (2U)
#define BARCODE_ACTION_RIGHT    (3U)
#define BARCODE_ACTION_UTURN    (4U)
#define BARCODE_ACTION_REJECTED (5U)
#define BARCODE_ACTION_STRAIGHT (6U)

/* Extra reasons for a rejected barcode, beside BARCODE_REJECT_* of the parser. */
#define BARCODE_REJECT_SPEED     (4U) /* the speed was stale, zero or negative */
#define BARCODE_REJECT_DUPLICATE (5U) /* the same marker, inside the re-arm gate */

/* The car must travel this far after a barcode before the next one counts. */
#define BARCODE_REARM_MM (100U)

/* The line follower ignores the line sensors this long after a barcode edge. */
#define BARCODE_SUPPRESS_US (250000U)

typedef struct
{
    uint8_t action; /* BARCODE_ACTION_* */
    uint8_t opcode; /* VEHICLE_CMD_*, valid when action is not NONE or REJECTED */
} barcode_action_t;

STATIC_ASSERT (sizeof(barcode_action_t) == 2U, "barcode_action_t size");

/* The action of a decoded character (0..25 for 'A'..'Z'). */
barcode_action_t barcode_action_of (uint8_t code);

typedef struct
{
    uint32_t last_mm;
    bool     b_armed_once;
} barcode_gate_t;

STATIC_ASSERT (sizeof(barcode_gate_t) == 8U, "barcode_gate_t size");

void barcode_gate_init (barcode_gate_t * p_gate);

/* Returns true when the barcode may act, and records its place. Returns false
 * when the car has not travelled BARCODE_REARM_MM since the last one that
 * acted. */
bool barcode_gate_pass (barcode_gate_t * p_gate, uint32_t odometer_mm);

/* True while the line follower must ignore the line sensors: an edge was seen
 * and less than BARCODE_SUPPRESS_US has passed since its sample timestamp. */
bool barcode_suppress_active (bool b_seen, uint32_t now_us, uint32_t edge_us);

#endif /* BARCODE_ACTION_H */

/*** end of file ***/
