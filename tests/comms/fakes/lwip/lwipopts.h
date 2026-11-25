#ifndef SHIM_TCP_LWIPOPTS_H
#define SHIM_TCP_LWIPOPTS_H

#define NO_SYS                  1
#define SYS_LIGHTWEIGHT_PROT    0
#define LWIP_TCP                1
#define LWIP_DNS                1
#define LWIP_IPV4               1
#define LWIP_IPV6               0
#define LWIP_SOCKET             0
#define LWIP_NETCONN            0
#define MEM_ALIGNMENT           4
/* The host builds with the same MQTT timer settings and timeout budget. */
#define MEMP_NUM_SYS_TIMEOUT    (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 2)
#define MQTT_CYCLIC_TIMER_INTERVAL 1
#define MQTT_REQ_TIMEOUT 10
#define MQTT_CONNECT_TIMOUT 10
/* Keep room for command PUBACKs when pending telemetry cannot reach TCP. */
#define MQTT_REQ_MAX_IN_FLIGHT 8
#define MQTT_OUTPUT_RINGBUF_SIZE 2048

#endif /* SHIM_TCP_LWIPOPTS_H */
