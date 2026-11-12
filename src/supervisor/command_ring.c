/*
 *	command_ring.c
 *	Inbox for vehicle commands. See command_ring.h.
 */

#include "command_ring.h"

static bool is_stop (uint8_t opcode)
{
    return ((VEHICLE_CMD_STOP == opcode) || (VEHICLE_CMD_ESTOP == opcode));
}

static uint8_t slot_of (const command_ring_t * p_ring, uint8_t offset)
{
    return (uint8_t) ((p_ring->head + offset) % COMMAND_RING_LEN);
}

void command_ring_init (command_ring_t * p_ring)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < COMMAND_RING_LEN; idx++)
    {
        p_ring->item[idx].opcode = 0U;
        p_ring->item[idx].source = 0U;
    }
    p_ring->head    = 0U;
    p_ring->count   = 0U;
    p_ring->dropped = 0U;
}

bool command_ring_push (command_ring_t * p_ring, uint8_t opcode, uint8_t source)
{
    bool    b_stored = true;
    uint8_t offset   = 0U;

    if (COMMAND_RING_LEN == p_ring->count)
    {
        /* Find the oldest command that is not a stop and remove it. */
        bool b_removed = false;

        for (offset = 0U; (offset < p_ring->count) && !b_removed; offset++)
        {
            if (!is_stop (p_ring->item[slot_of (p_ring, offset)].opcode))
            {
                uint8_t from = offset;

                while ((uint8_t) (from + 1U) < p_ring->count)
                {
                    p_ring->item[slot_of (p_ring, from)] =
                        p_ring->item[slot_of (p_ring, (uint8_t) (from + 1U))];
                    from++;
                }
                p_ring->count--;
                p_ring->dropped++;
                b_removed = true;
            }
        }
        if (!b_removed)
        {
            p_ring->dropped++;
            b_stored = false;
        }
    }

    if (b_stored)
    {
        uint8_t const slot = slot_of (p_ring, p_ring->count);

        p_ring->item[slot].opcode = opcode;
        p_ring->item[slot].source = source;
        p_ring->count++;
    }

    return b_stored;
}

bool command_ring_pop (command_ring_t * p_ring, vehicle_cmd_t * p_cmd)
{
    bool b_taken = (p_ring->count > 0U);

    if (b_taken)
    {
        *p_cmd       = p_ring->item[p_ring->head];
        p_ring->head = (uint8_t) ((p_ring->head + 1U) % COMMAND_RING_LEN);
        p_ring->count--;
    }

    return b_taken;
}
