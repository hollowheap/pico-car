#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include <stdint.h>
#include "controller.h"

typedef struct
{
    controller_config_t control;
    uint32_t            poll_delay_ms;
} motion_config_t;

void motion_config_default (motion_config_t * p_config);

#endif /* MOTION_CONFIG_H */
