/** @file action.c
 * @brief Barcode action mapping, re-arm gate and suppression window.
 */

#include "barcode/action.h"
#include "supervisor/command.h"

/* Letters, as codes 0..25 for 'A'..'Z'. */
#define CODE_H (7U)
#define CODE_L (11U)
#define CODE_R (17U)
#define CODE_S (18U)
#define CODE_U (20U)

barcode_action_t barcode_action_of (uint8_t code)
{
    barcode_action_t result = { BARCODE_ACTION_REJECTED, 0U };

    switch (code)
    {
        /* The letters of the four junction commands (D164). H is a fifth,
         * a halt; delete its row to have only four. */
        case CODE_S:
            result.action = BARCODE_ACTION_STRAIGHT;
            result.opcode = VEHICLE_CMD_STRAIGHT;
        break;

        case CODE_H:
            result.action = BARCODE_ACTION_STOP;
            result.opcode = VEHICLE_CMD_STOP;
        break;

        case CODE_L:
            result.action = BARCODE_ACTION_LEFT;
            result.opcode = VEHICLE_CMD_LEFT;
        break;

        case CODE_R:
            result.action = BARCODE_ACTION_RIGHT;
            result.opcode = VEHICLE_CMD_RIGHT;
        break;

        case CODE_U:
            result.action = BARCODE_ACTION_UTURN;
            result.opcode = VEHICLE_CMD_UTURN;
        break;

        default:
            /* Every other letter has no action. */
        break;
    }

    return result;
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
