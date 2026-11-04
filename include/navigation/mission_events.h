#ifndef NAVIGATION_MISSION_EVENTS_H
#define NAVIGATION_MISSION_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

#include "navigation/navigation.h"

/** Apply one event batch; STOP/FAULT dominate RESET/START. */
mission_state_t mission_apply_events (mission_state_t state, uint32_t events,
                                      bool * p_stop_requested);

#endif /* NAVIGATION_MISSION_EVENTS_H */
