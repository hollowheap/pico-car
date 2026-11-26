/** @file action.c
 * @brief Barcode action mapping, re-arm gate and suppression window.
 */

#include "ir/barcode/action.h"
#include "ir/barcode/action_config.h"
#include "supervisor/command.h"

#define BARCODE_CHAR_IS_LETTER(letter) (('A' <= (letter)) && ((letter) <= 'Z'))

STATIC_ASSERT (BARCODE_CHAR_IS_LETTER(BARCODE_CHAR_STOP) &&
               BARCODE_CHAR_IS_LETTER(BARCODE_CHAR_ALT_STOP) &&
               BARCODE_CHAR_IS_LETTER(BARCODE_CHAR_LEFT) &&
               BARCODE_CHAR_IS_LETTER(BARCODE_CHAR_RIGHT) &&
               BARCODE_CHAR_IS_LETTER(BARCODE_CHAR_STRAIGHT) &&
               BARCODE_CHAR_IS_LETTER(BARCODE_CHAR_UTURN),
               "barcode bindings must use uppercase A-Z");

barcode_action_t barcode_action_for_command (uint8_t opcode)
{
    switch (opcode)
    {
        case VEHICLE_CMD_STOP:
            return (barcode_action_t){ BARCODE_ACTION_STOP, opcode };
        case VEHICLE_CMD_LEFT:
            return (barcode_action_t){ BARCODE_ACTION_LEFT, opcode };
        case VEHICLE_CMD_RIGHT:
            return (barcode_action_t){ BARCODE_ACTION_RIGHT, opcode };
        case VEHICLE_CMD_UTURN:
            return (barcode_action_t){ BARCODE_ACTION_UTURN, opcode };
        case VEHICLE_CMD_STRAIGHT:
            return (barcode_action_t){ BARCODE_ACTION_STRAIGHT, opcode };
        default:
            return (barcode_action_t){ BARCODE_ACTION_REJECTED, 0U };
    }
}

barcode_action_t barcode_action_of (uint8_t code)
{
    /* The parser returns 0..25. Duplicate bindings fail as duplicate cases. */
    switch (code)
    {
        case (BARCODE_CHAR_STRAIGHT - 'A'):
            return barcode_action_for_command(VEHICLE_CMD_STRAIGHT);

        case (BARCODE_CHAR_STOP - 'A'):
        case (BARCODE_CHAR_ALT_STOP - 'A'):
            return barcode_action_for_command(VEHICLE_CMD_STOP);

        case (BARCODE_CHAR_LEFT - 'A'):
            return barcode_action_for_command(VEHICLE_CMD_LEFT);

        case (BARCODE_CHAR_RIGHT - 'A'):
            return barcode_action_for_command(VEHICLE_CMD_RIGHT);

        case (BARCODE_CHAR_UTURN - 'A'):
            return barcode_action_for_command(VEHICLE_CMD_UTURN);

        default:
            return (barcode_action_t){ BARCODE_ACTION_REJECTED, 0U };
    }

}

void barcode_gate_init (barcode_gate_t * p_gate)
{
    p_gate->last_mm      = 0U;
    p_gate->b_armed_once = false;
}

bool barcode_gate_pass (barcode_gate_t * p_gate, uint32_t odometer_mm)
{
    /* Signed, so a car that went back (a pivot) must still travel forward. */
    bool const b_pass = !p_gate->b_armed_once ||
                        ((int32_t) (odometer_mm - p_gate->last_mm) >= (int32_t) BARCODE_REARM_MM);

    if (b_pass)
    {
        p_gate->last_mm      = odometer_mm;
        p_gate->b_armed_once = true;
    }

    return b_pass;
}

bool barcode_suppress_active (bool b_seen, uint32_t now_us, uint32_t edge_us)
{
    return b_seen && ((uint32_t) (now_us - edge_us) < BARCODE_SUPPRESS_US);
}
