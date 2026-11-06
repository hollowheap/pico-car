#include <string.h>

#include "telemetry/mqtt.h"

#define MQTT_CONNECT      0x10u
#define MQTT_CONNACK      0x20u
#define MQTT_PUBLISH      0x30u
#define MQTT_PUBACK       0x40u
#define MQTT_SUBSCRIBE    0x82u
#define MQTT_SUBACK       0x90u
#define MQTT_PINGREQ      0xC0u
#define MQTT_PINGRESP     0xD0u
#define MQTT_RECONNECT_MS 1000u

static uint16_t
read_u16 (const uint8_t * data)
{
    return (uint16_t) (((uint16_t) data[0] << 8U) | (uint16_t) data[1]);
}

static uint16_t
put_u16 (uint8_t * data, uint16_t value)
{
    data[0] = (uint8_t) (value >> 8U);
    data[1] = (uint8_t) value;
    return 2u;
}

static bool
put_string (uint8_t * data, uint16_t capacity, uint16_t * index,
            const char * text)
{
    uint16_t len = (uint16_t) strlen(text);

    if ((uint32_t) *index + 2u + len > capacity)
    {
        return false;
    }
    (void) put_u16(&data[*index], len);
    (void) memcpy(&data[*index + 2u], text, len);
    *index = (uint16_t) (*index + len + 2u);
    return true;
}

static uint16_t
remaining_length (uint16_t len, uint8_t out[2])
{
    uint16_t count = 0u;

    do
    {
        uint8_t byte = (uint8_t) (len % 128u);
        len = (uint16_t) (len / 128u);
        if (len > 0u)
        {
            byte = (uint8_t) (byte | 0x80u);
        }
        out[count] = byte;
        count++;
    } while (len > 0u);

    return count;
}

static mqtt_result_t
send_packet (mqtt_client_t * client, uint8_t header, const uint8_t * body,
             uint16_t body_len, uint32_t now_ms)
{
    uint8_t  rem[2];
    uint16_t rem_len = remaining_length(body_len, rem);
    uint16_t index = 0u;
    int      sent;

    if ((uint16_t) (1u + rem_len + body_len) > MQTT_TX_BUFFER_SIZE)
    {
        return MQTT_ERR_PROTOCOL;
    }

    client->tx_buffer[index] = header;
    index++;
    (void) memcpy(&client->tx_buffer[index], rem, rem_len);
    index = (uint16_t) (index + rem_len);
    if (body_len > 0u)
    {
        (void) memcpy(&client->tx_buffer[index], body, body_len);
        index = (uint16_t) (index + body_len);
    }

    sent = client->transport.send(client->transport.context, client->tx_buffer,
                                  index);
    if (sent != (int) index)
    {
        return MQTT_ERR_TRANSPORT;
    }
    client->last_tx_ms = now_ms;
    return MQTT_OK;
}

static mqtt_result_t
send_connect (mqtt_client_t * client, uint32_t now_ms)
{
    uint8_t  body[MQTT_TX_BUFFER_SIZE];
    uint16_t index = 0u;
    uint8_t  flags = 0x02u;

    body[index++] = 0x00u;
    body[index++] = 0x04u;
    body[index++] = (uint8_t) 'M';
    body[index++] = (uint8_t) 'Q';
    body[index++] = (uint8_t) 'T';
    body[index++] = (uint8_t) 'T';
    body[index++] = 0x04u;
    if ((client->config.username != 0) && (client->config.password != 0))
    {
        flags = (uint8_t) (flags | 0xC0u);
    }
    body[index++] = flags;
    index = (uint16_t) (index + put_u16(&body[index],
                                        client->config.keepalive_s));
    if (!put_string(body, sizeof(body), &index, client->config.client_id))
    {
        return MQTT_ERR_ARG;
    }
    if (flags == 0xC2u)
    {
        if (!put_string(body, sizeof(body), &index, client->config.username) ||
            !put_string(body, sizeof(body), &index, client->config.password))
        {
            return MQTT_ERR_ARG;
        }
    }

    return send_packet(client, MQTT_CONNECT, body, index, now_ms);
}

static mqtt_result_t
send_subscribe (mqtt_client_t * client, uint32_t now_ms)
{
    uint8_t  body[64];
    uint16_t index = 0u;

    index = (uint16_t) (index + put_u16(&body[index], 1u));
    if (!put_string(body, sizeof(body), &index, client->config.command_topic))
    {
        return MQTT_ERR_ARG;
    }
    if (index >= sizeof(body))
    {
        return MQTT_ERR_ARG;
    }
    body[index++] = 1u;

    return send_packet(client, MQTT_SUBSCRIBE, body, index, now_ms);
}

static void
disconnect (mqtt_client_t * client, uint32_t now_ms)
{
    client->state = MQTT_STATE_DISCONNECTED;
    client->next_attempt_ms = now_ms + MQTT_RECONNECT_MS;
    client->rx_len = 0u;
}

static mqtt_result_t
send_puback (mqtt_client_t * client, uint16_t packet_id, uint32_t now_ms)
{
    uint8_t body[2];

    (void) put_u16(body, packet_id);
    return send_packet(client, MQTT_PUBACK, body, 2u, now_ms);
}

static mqtt_result_t
handle_publish (mqtt_client_t * client, const uint8_t * packet,
                uint16_t packet_len, uint32_t now_ms)
{
    uint8_t  flags = (uint8_t) (packet[0] & 0x0Fu);
    uint8_t  qos = (uint8_t) ((flags >> 1U) & 0x03u);
    uint32_t index;
    uint16_t topic_len;
    uint16_t packet_id = 0u;
    const uint8_t * topic;
    const uint8_t * payload;
    uint16_t payload_len;

    if ((qos == 3u) || (packet_len < 4u))
    {
        return MQTT_ERR_PROTOCOL;
    }

    topic_len = read_u16(&packet[2]);
    if (topic_len > (uint16_t) (packet_len - 4u))
    {
        return MQTT_ERR_PROTOCOL;
    }
    topic = &packet[4];
    index = 4u + topic_len;
    if (qos > 0u)
    {
        if ((index + 2u) > packet_len)
        {
            return MQTT_ERR_PROTOCOL;
        }
        packet_id = read_u16(&packet[index]);
        index += 2u;
    }
    payload = &packet[index];
    payload_len = (uint16_t) (packet_len - index);

    if (qos == 1u)
    {
        mqtt_result_t result = send_puback(client, packet_id, now_ms);
        if (result != MQTT_OK)
        {
            return result;
        }
    }

    if ((topic_len == strlen(client->config.command_topic)) &&
        (memcmp(topic, client->config.command_topic, topic_len) == 0) &&
        (payload_len >= 2u))
    {
        uint16_t cmd_id = read_u16(payload);
        int16_t  delta;

        if (client->has_last_cmd)
        {
            delta = (int16_t) (cmd_id - client->last_cmd_id);
            if (delta <= 0)
            {
                return MQTT_OK;
            }
        }
        client->last_cmd_id = cmd_id;
        client->has_last_cmd = true;
        if (client->command_handler != 0)
        {
            client->command_handler(payload, payload_len,
                                    client->command_context);
        }
    }

    return MQTT_OK;
}

static mqtt_result_t
handle_packet (mqtt_client_t * client, const uint8_t * packet,
               uint16_t packet_len, uint32_t now_ms)
{
    uint8_t  type = (uint8_t) (packet[0] >> 4U);
    uint16_t remaining;
    uint16_t index = 1u;

    if (packet_len < 2u)
    {
        return MQTT_ERR_PROTOCOL;
    }
    remaining = packet[1];
    if ((remaining & 0x80u) != 0u)
    {
        if (packet_len < 3u)
        {
            return MQTT_ERR_PROTOCOL;
        }
        remaining = (uint16_t) ((remaining & 0x7Fu) |
                                ((uint16_t) packet[2] << 7U));
        index = 2u;
    }
    if ((uint16_t) (index + 1u + remaining) != packet_len)
    {
        return MQTT_ERR_PROTOCOL;
    }

    if ((type == (MQTT_CONNACK >> 4U)) && (client->state == MQTT_STATE_WAIT_CONNACK))
    {
        if ((remaining != 2u) || (packet[index + 1u] != 0u))
        {
            disconnect(client, now_ms);
            return MQTT_ERR_PROTOCOL;
        }
        client->state = MQTT_STATE_WAIT_SUBACK;
        return send_subscribe(client, now_ms);
    }
    if ((type == (MQTT_SUBACK >> 4U)) && (client->state == MQTT_STATE_WAIT_SUBACK))
    {
        client->state = MQTT_STATE_CONNECTED;
        client->last_tx_ms = now_ms;
        return MQTT_OK;
    }
    if ((type == (MQTT_PUBLISH >> 4U)) && (client->state == MQTT_STATE_CONNECTED))
    {
        return handle_publish(client, packet, packet_len, now_ms);
    }
    if (type == (MQTT_PINGRESP >> 4U))
    {
        return MQTT_OK;
    }

    return MQTT_ERR_PROTOCOL;
}

static mqtt_result_t
receive_packets (mqtt_client_t * client, uint32_t now_ms)
{
    int got;

    if (client->rx_len < MQTT_RX_BUFFER_SIZE)
    {
        got = client->transport.recv(client->transport.context,
                                     &client->rx_buffer[client->rx_len],
                                     (uint16_t) (MQTT_RX_BUFFER_SIZE -
                                                 client->rx_len));
        if (got < 0)
        {
            disconnect(client, now_ms);
            return MQTT_ERR_TRANSPORT;
        }
        if (got > 0)
        {
            client->rx_len = (uint16_t) (client->rx_len + got);
        }
    }

    while (client->rx_len >= 2u)
    {
        uint16_t rem = client->rx_buffer[1];
        uint16_t header_len = 2u;
        uint16_t total_len;

        if ((rem & 0x80u) != 0u)
        {
            if (client->rx_len < 3u)
            {
                break;
            }
            rem = (uint16_t) ((rem & 0x7Fu) |
                              ((uint16_t) client->rx_buffer[2] << 7U));
            header_len = 3u;
        }
        total_len = (uint16_t) (header_len + rem);
        if (client->rx_len < total_len)
        {
            break;
        }
        if (handle_packet(client, client->rx_buffer, total_len, now_ms) !=
            MQTT_OK)
        {
            return MQTT_ERR_PROTOCOL;
        }
        (void) memmove(client->rx_buffer, &client->rx_buffer[total_len],
                       (size_t) (client->rx_len - total_len));
        client->rx_len = (uint16_t) (client->rx_len - total_len);
    }

    if (client->rx_len == MQTT_RX_BUFFER_SIZE)
    {
        disconnect(client, now_ms);
        return MQTT_ERR_PROTOCOL;
    }

    return MQTT_OK;
}

void
mqtt_client_init (mqtt_client_t * client, const mqtt_config_t * config,
                  const mqtt_transport_t * transport, mqtt_command_fn handler,
                  void * context)
{
    if (client == 0)
    {
        return;
    }
    (void) memset(client, 0, sizeof(*client));
    if ((config == 0) || (transport == 0) || (transport->send == 0) ||
        (transport->recv == 0) || (config->client_id == 0) ||
        (config->command_topic == 0))
    {
        return;
    }
    client->valid = true;
    client->config = *config;
    client->transport = *transport;
    client->command_handler = handler;
    client->command_context = context;
    client->state = MQTT_STATE_DISCONNECTED;
    client->next_attempt_ms = 0u;
    if (client->config.keepalive_s == 0u)
    {
        client->config.keepalive_s = MQTT_DEFAULT_KEEPALIVE_S;
    }
}

mqtt_result_t
mqtt_client_poll (mqtt_client_t * client, uint32_t now_ms)
{
    mqtt_result_t result;

    if ((client == 0) || !client->valid)
    {
        return MQTT_ERR_ARG;
    }

    if (client->state == MQTT_STATE_DISCONNECTED)
    {
        if ((int32_t) (now_ms - client->next_attempt_ms) < 0)
        {
            return MQTT_OK;
        }
        result = send_connect(client, now_ms);
        if (result != MQTT_OK)
        {
            disconnect(client, now_ms);
            return result;
        }
        client->state = MQTT_STATE_WAIT_CONNACK;
        return MQTT_OK;
    }

    result = receive_packets(client, now_ms);
    if (result != MQTT_OK)
    {
        if (client->state != MQTT_STATE_DISCONNECTED)
        {
            disconnect(client, now_ms);
        }
        return result;
    }

    if ((client->state == MQTT_STATE_CONNECTED) &&
        (client->config.keepalive_s > 0u) &&
        ((now_ms - client->last_tx_ms) >=
         ((uint32_t) client->config.keepalive_s * 1000u)))
    {
        result = send_packet(client, MQTT_PINGREQ, 0, 0u, now_ms);
        if (result != MQTT_OK)
        {
            disconnect(client, now_ms);
        }
        return result;
    }

    return MQTT_OK;
}

mqtt_result_t
mqtt_client_publish (mqtt_client_t * client, const char * topic,
                     const uint8_t * payload, uint16_t len, uint32_t now_ms)
{
    uint8_t  body[MQTT_TX_BUFFER_SIZE];
    uint16_t index = 0u;
    uint16_t topic_len;
    mqtt_result_t result;

    if ((client == 0) || !client->valid || (topic == 0) || (payload == 0))
    {
        return MQTT_ERR_ARG;
    }
    if (client->state != MQTT_STATE_CONNECTED)
    {
        return MQTT_ERR_STATE;
    }

    topic_len = (uint16_t) strlen(topic);
    if ((uint32_t) 2u + topic_len + len > MQTT_TX_BUFFER_SIZE)
    {
        return MQTT_ERR_PROTOCOL;
    }
    if (!put_string(body, sizeof(body), &index, topic))
    {
        return MQTT_ERR_ARG;
    }
    (void) memcpy(&body[index], payload, len);
    index = (uint16_t) (index + len);

    result = send_packet(client, MQTT_PUBLISH, body, index, now_ms);
    if (result != MQTT_OK)
    {
        disconnect(client, now_ms);
    }
    return result;
}

mqtt_state_t
mqtt_client_state (const mqtt_client_t * client)
{
    return (client == 0) ? MQTT_STATE_DISCONNECTED : client->state;
}
