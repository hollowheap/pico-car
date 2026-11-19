#ifndef TELEMETRY_MQTT_H
#define TELEMETRY_MQTT_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

#define MQTT_TX_BUFFER_SIZE 256u
#define MQTT_RX_BUFFER_SIZE 128u
#define MQTT_DEFAULT_KEEPALIVE_S 30u

/* Reply timeout and reconnect backoff. */
#define MQTT_REPLY_TIMEOUT_MS 10000u
#define MQTT_RECONNECT_MIN_MS 1000u
#define MQTT_RECONNECT_MAX_MS 30000u

typedef enum
{
    MQTT_OK = 0,
    MQTT_ERR_ARG,
    MQTT_ERR_STATE,
    MQTT_ERR_TRANSPORT,
    MQTT_ERR_PROTOCOL,
    /* The transport had no room (send returned 0): the packet was dropped and
       the session stays up. A PUBACK that finds no room is not dropped: the
       command has already run, and the next poll retries the PUBACK before
       it parses any further packet, so a QoS 0 PUBLISH behind it is delayed
       until the PUBACK fits. */
    MQTT_ERR_BUSY
} mqtt_result_t;

STATIC_ASSERT (sizeof(mqtt_result_t) <= 4U, "mqtt_result_t size");

typedef enum
{
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_WAIT_CONNACK,
    MQTT_STATE_WAIT_SUBACK,
    MQTT_STATE_CONNECTED
} mqtt_state_t;

STATIC_ASSERT (sizeof(mqtt_state_t) <= 4U, "mqtt_state_t size");

typedef struct
{
    void * context;
    int (*send)(void * context, const uint8_t * data, uint16_t len);
    int (*recv)(void * context, uint8_t * data, uint16_t len);
} mqtt_transport_t;

STATIC_ASSERT (sizeof(mqtt_transport_t) == (3U * sizeof(void *)),
               "mqtt_transport_t size");

typedef struct
{
    const char * client_id;
    const char * username;
    const char * password;
    const char * command_topic;
    uint16_t     keepalive_s;
} mqtt_config_t;

STATIC_ASSERT (sizeof(mqtt_config_t) == (5U * sizeof(void *)),
               "mqtt_config_t size");

typedef void (*mqtt_command_fn_t)(const uint8_t * payload, uint16_t len,
                                  void * context);

typedef struct
{
    mqtt_transport_t  transport;
    mqtt_config_t     config;
    mqtt_command_fn_t command_handler;
    void *            command_context;
    mqtt_state_t      state;
    uint32_t          last_tx_ms;
    uint32_t          next_attempt_ms;
    uint32_t          reply_deadline_ms;
    uint32_t          reconnect_delay_ms;
    bool              attempt_pending;
    bool              reply_pending;
    bool              ping_owed;
    bool              puback_due;
    uint16_t          puback_id;
    uint8_t           rx_buffer[MQTT_RX_BUFFER_SIZE];
    uint16_t          rx_len;
    bool              valid;
    uint8_t           tx_buffer[MQTT_TX_BUFFER_SIZE];
} mqtt_client_t;

/* 416 bytes beside nine pointers and the padded keepalive word, rounded up
   to the pointer alignment. */
STATIC_ASSERT (sizeof(mqtt_client_t) ==
                   (((416U + (10U * sizeof(void *))) + (sizeof(void *) - 1U)) &
                    ~(sizeof(void *) - 1U)),
               "mqtt_client_t size");

void mqtt_client_init (mqtt_client_t * client, const mqtt_config_t * config,
                       const mqtt_transport_t * transport,
                       mqtt_command_fn_t handler, void * context);

mqtt_result_t mqtt_client_poll (mqtt_client_t * client, uint32_t now_ms);

mqtt_result_t mqtt_client_publish (mqtt_client_t * client, const char * topic,
                                   const uint8_t * payload, uint16_t len,
                                   uint32_t now_ms);

mqtt_state_t mqtt_client_state (const mqtt_client_t * client);

#endif /* TELEMETRY_MQTT_H */
