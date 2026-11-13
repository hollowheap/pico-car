#ifndef QR_OBSTACLE_H
#define QR_OBSTACLE_H

#include "navigation/obstacle.h"

#define B5_FLG_ECHO      (0x01U)
#define B5_FLG_CMD       (0x02U)
#define B5_FLG_STOP      (0x04U)
#define B5_FLG_NAV_EVT   (0x08U)
#define B5_FLG_NAV_STEER (0x10U)

uint32_t obstacle_get_steer (void);
void     obstacle_set_look (int16_t deci);
uint32_t obstacle_get_status (void);
uint8_t  obstacle_dropped_events (void);

#endif
