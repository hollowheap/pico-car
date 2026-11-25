/** @file drv_lwip.h
 * @brief Pico SDK hooks used by the lwIP port.
 */
#ifndef TELEMETRY_DRV_LWIP_H
#define TELEMETRY_DRV_LWIP_H

#ifndef LWIP_PLATFORM_ASSERT
#include "pico.h"
#define LWIP_PLATFORM_ASSERT(x) panic(x)
#endif

#ifndef LWIP_RAND
#include "pico/rand.h"
/* Use the pico_rand library which goes to reasonable lengths to try to
 * provide good entropy */
#define LWIP_RAND() get_rand_32()
#endif

#endif /* TELEMETRY_DRV_LWIP_H */

/*** end of file ***/
