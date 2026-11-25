#ifndef PROJECT_LWIPOPTS_H
#define PROJECT_LWIPOPTS_H

/* Use the kernel's lwIP port configuration, then enable the application TCP
 * path without pulling in the kernel's fixed TCP test harness. */
#include_next <lwipopts.h>

#undef LWIP_TCP
#define LWIP_TCP 1

/* Sized to the car's traffic: one MQTT connection out, small frames, a few
 * bytes in. */
/* Wi-Fi receive copies each frame into a pool buffer, and the CYW43 task
 * hands it to lwIP at once. Four cover a burst; a dropped broadcast costs
 * nothing, and TCP resends a dropped segment. */
#undef PBUF_POOL_SIZE
#define PBUF_POOL_SIZE 4
/* The heap holds copied TCP segments (TCP_SND_BUF, 1072 B, plus headers),
 * DHCP and DNS packets, and one SDK MQTT client. */
#undef MEM_SIZE
#define MEM_SIZE (5 * 1024)
/* The SDK protocol and request timers run on the radio owner. Leave slots
 * for MQTT and the application's recurring radio timer. */
#undef MEMP_NUM_SYS_TIMEOUT
#define MEMP_NUM_SYS_TIMEOUT (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 2)
#define MQTT_CYCLIC_TIMER_INTERVAL 1
#define MQTT_REQ_TIMEOUT 10
#define MQTT_CONNECT_TIMOUT 10
/* Keep room for command PUBACKs when pending telemetry cannot reach TCP. */
#define MQTT_REQ_MAX_IN_FLIGHT 8
#define MQTT_OUTPUT_RINGBUF_SIZE 2048

/* The live connection and the one in TIME_WAIT after a reconnect. */
#define MEMP_NUM_TCP_PCB 2
/* One name to resolve: the broker. */
#define DNS_TABLE_SIZE 1

#endif /* PROJECT_LWIPOPTS_H */
