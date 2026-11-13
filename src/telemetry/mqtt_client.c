#include <string.h>

#include "telemetry/cmd.h"
#include "telemetry/mqtt.h"

#define MQTT_CONNECT      0x10u
#define MQTT_CONNACK      0x20u
#define MQTT_PUBLISH      0x30u
#define MQTT_PUBACK       0x40u
#define MQTT_SUBSCRIBE    0x82u
#define MQTT_SUBACK       0x90u
#define MQTT_PINGREQ      0xC0u
#define MQTT_PINGRESP     0xD0u

/* Byte and field geometry. */
#define MQTT_BYTE_BITS            8U
#define MQTT_U16_SIZE             2U
#define MQTT_MS_PER_S             1000U
#define MQTT_TYPE_SHIFT           4U

/* Remaining-length encoding (at most two bytes are supported). */
#define MQTT_RL_MAX_BYTES         2U
#define MQTT_RL_RADIX             128U
#define MQTT_RL_CONT_FLAG         0x80U
#define MQTT_RL_VALUE_MASK        0x7FU
#define MQTT_RL_SHIFT             7U
#define MQTT_RL_BYTE2_OFFSET      2U
#define MQTT_MIN_PACKET_1B_RL     2U
#define MQTT_MIN_PACKET_2B_RL     3U
#define MQTT_RX_MIN_HEADER        2U
#define MQTT_HEADER_LEN_1B_RL     2U
#define MQTT_HEADER_LEN_2B_RL     3U
#define MQTT_RL_INDEX_2B          2U

/* CONNECT variable header. */
#define MQTT_PROTO_NAME_LEN       0x04U
#define MQTT_PROTO_LEVEL          0x04U
#define MQTT_CONN_FLAG_CLEAN      0x02U
#define MQTT_CONN_FLAG_USER       0x80U
#define MQTT_CONN_FLAG_PASS       0x40U

/* SUBSCRIBE body. */
#define MQTT_SUBSCRIBE_BODY_MAX   64U

/* PUBLISH fixed-header flags and offsets. */
#define MQTT_PUBLISH_FLAGS_MASK   0x0FU
#define MQTT_PUBLISH_QOS_SHIFT    1U
#define MQTT_PUBLISH_QOS_MASK     0x03U
#define MQTT_PUBLISH_QOS_UNSUPPORTED 2U
#define MQTT_PUBLISH_MIN_LEN      4U
#define MQTT_PUBLISH_TOPIC_LEN_AT 2U
#define MQTT_PUBLISH_TOPIC_AT     4U

/* CONNACK. */
#define MQTT_CONNACK_REMAINING    2U
#define MQTT_CONNACK_CODE_AT      2U
#define MQTT_CONNACK_ACCEPTED     0U

/* SUBACK. */
#define MQTT_SUBACK_REMAINING     3U
#define MQTT_SUBACK_CODE_AT       3U
#define MQTT_SUBACK_FAILURE       0x80U

/* Reconnect delay growth per failed attempt. */
#define MQTT_RECONNECT_FACTOR    2U

/* Largest body that still encodes a one-byte remaining length. */
#define MQTT_RL_1B_MAX           127U

static uint16_t
read_u16 (const uint8_t * data)
{
    return (uint16_t) (((uint32_t) data[0] << MQTT_BYTE_BITS) |
                       (uint32_t) data[1]);
}

static uint16_t
put_u16 (uint8_t * data, uint16_t value)
{
    data[0] = (uint8_t) (value >> MQTT_BYTE_BITS);
    data[1] = (uint8_t) value;
    return MQTT_U16_SIZE;
}

static bool
put_string (uint8_t * data, uint16_t capacity, uint16_t * index,
            const char * text)
{
    uint16_t len = (uint16_t) strlen(text);

    if ((uint32_t) *index + MQTT_U16_SIZE + len > capacity)
    {
        return false;
    }
    (void) put_u16(&data[*index], len);
    (void) memcpy(&data[*index + MQTT_U16_SIZE], text, len);
    *index = (uint16_t) (*index + len + MQTT_U16_SIZE);
    return true;
}

static uint16_t
remaining_length (uint16_t len, uint8_t out[MQTT_RL_MAX_BYTES])
{
    uint16_t count = 0u;

    do
    {
        uint8_t byte = (uint8_t) (len % MQTT_RL_RADIX);
        len = (uint16_t) (len / MQTT_RL_RADIX);
        if (len > 0u)
        {
            byte = (uint8_t) (byte | MQTT_RL_CONT_FLAG);
        }
        out[count] = byte;
        count++;
    } while (len > 0u);

    return count;
}

/* True when a packet with this body length, fixed header included, fits the
   TX buffer. */
static bool
packet_fits (uint32_t body_len)
{
    uint32_t rem_len = (body_len > MQTT_RL_1B_MAX) ? MQTT_RL_MAX_BYTES : 1u;

    return (1u + rem_len + body_len) <= MQTT_TX_BUFFER_SIZE;
}

static mqtt_result_t
send_packet (mqtt_client_t * client, uint8_t header, const uint8_t * body,
             uint16_t body_len, uint32_t now_ms)
{
    uint8_t  rem[MQTT_RL_MAX_BYTES];
    uint16_t rem_len = remaining_length(body_len, rem);
    uint16_t index = 0u;
    int      sent = 0;

    if (!packet_fits(body_len))
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
    if (sent == 0)
    {
        return MQTT_ERR_BUSY;
    }
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
    uint8_t  flags = MQTT_CONN_FLAG_CLEAN;

    body[index++] = 0x00u;
    body[index++] = MQTT_PROTO_NAME_LEN;
    body[index++] = (uint8_t) 'M';
    body[index++] = (uint8_t) 'Q';
    body[index++] = (uint8_t) 'T';
    body[index++] = (uint8_t) 'T';
    body[index++] = MQTT_PROTO_LEVEL;
    /* 3.1.2.9: the password flag requires the username flag. */
    if (client->config.username != 0)
    {
        flags = (uint8_t) (flags | MQTT_CONN_FLAG_USER);
        if (client->config.password != 0)
        {
            flags = (uint8_t) (flags | MQTT_CONN_FLAG_PASS);
        }
    }
    body[index++] = flags;
    index = (uint16_t) (index + put_u16(&body[index],
                                        client->config.keepalive_s));
    if (!put_string(body, sizeof(body), &index, client->config.client_id))
    {
        return MQTT_ERR_ARG;
    }
    if (((flags & MQTT_CONN_FLAG_USER) != 0u) &&
        !put_string(body, sizeof(body), &index, client->config.username))
    {
        return MQTT_ERR_ARG;
    }
    if (((flags & MQTT_CONN_FLAG_PASS) != 0u) &&
        !put_string(body, sizeof(body), &index, client->config.password))
    {
        return MQTT_ERR_ARG;
    }

    return send_packet(client, MQTT_CONNECT, body, index, now_ms);
}

static mqtt_result_t
send_subscribe (mqtt_client_t * client, uint32_t now_ms)
{
    uint8_t  body[MQTT_SUBSCRIBE_BODY_MAX];
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

/* Close the session and schedule the next attempt; the delay doubles up to
   MQTT_RECONNECT_MAX_MS until a SUBACK resets it. */
static void
disconnect (mqtt_client_t * client, uint32_t now_ms)
{
    uint32_t next_delay = client->reconnect_delay_ms * MQTT_RECONNECT_FACTOR;

    if (client->transport.restart != 0)
    {
        client->transport.restart(client->transport.context);
    }
    client->state = MQTT_STATE_DISCONNECTED;
    client->next_attempt_ms = now_ms + client->reconnect_delay_ms;
    client->reconnect_delay_ms = (next_delay > MQTT_RECONNECT_MAX_MS) ?
                                     MQTT_RECONNECT_MAX_MS : next_delay;
    client->attempt_pending = false;
    client->reply_pending = false;
    client->rx_len = 0u;
}

static void
start_reply_timer (mqtt_client_t * client, uint32_t now_ms)
{
    client->reply_deadline_ms = now_ms + MQTT_REPLY_TIMEOUT_MS;
    client->reply_pending = true;
}

static mqtt_result_t
send_puback (mqtt_client_t * client, uint16_t packet_id, uint32_t now_ms)
{
    uint8_t body[MQTT_U16_SIZE];

    (void) put_u16(body, packet_id);
    return send_packet(client, MQTT_PUBACK, body, MQTT_U16_SIZE, now_ms);
}

static mqtt_result_t
handle_publish (mqtt_client_t * client, const uint8_t * packet,
                uint16_t packet_len, uint32_t now_ms)
{
    uint8_t  flags = (uint8_t) (packet[0] & MQTT_PUBLISH_FLAGS_MASK);
    uint8_t  qos = (uint8_t) (((uint32_t) flags >> MQTT_PUBLISH_QOS_SHIFT) &
                              MQTT_PUBLISH_QOS_MASK);
    uint32_t index = 0u;
    uint16_t topic_len = 0u;
    uint16_t packet_id = 0u;
    const uint8_t * topic = 0;
    const uint8_t * payload = 0;
    uint16_t payload_len = 0u;

    /* The client subscribes at QoS 1, so QoS 2 (and the invalid QoS 3) is a
       broker protocol violation. */
    if ((qos >= MQTT_PUBLISH_QOS_UNSUPPORTED) ||
        (packet_len < MQTT_PUBLISH_MIN_LEN))
    {
        return MQTT_ERR_PROTOCOL;
    }

    topic_len = read_u16(&packet[MQTT_PUBLISH_TOPIC_LEN_AT]);
    if (topic_len > (uint16_t) (packet_len - MQTT_PUBLISH_MIN_LEN))
    {
        return MQTT_ERR_PROTOCOL;
    }
    topic = &packet[MQTT_PUBLISH_TOPIC_AT];
    index = MQTT_PUBLISH_TOPIC_AT + topic_len;
    if (qos > 0u)
    {
        if ((index + MQTT_U16_SIZE) > packet_len)
        {
            return MQTT_ERR_PROTOCOL;
        }
        packet_id = read_u16(&packet[index]);
        index += MQTT_U16_SIZE;
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
        (payload_len >= MQTT_U16_SIZE))
    {
        uint16_t cmd_id = read_u16(payload);
        int16_t  delta = 0;

        if (client->has_last_cmd &&
            ((uint32_t) (now_ms - client->last_cmd_ms) <= TELEMETRY_CMD_MAX_AGE_MS))
        {
            delta = (int16_t) (cmd_id - client->last_cmd_id);
            if (delta <= 0)
            {
                return MQTT_OK;
            }
        }
        client->last_cmd_ms = now_ms;
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
    uint8_t  type = (uint8_t) (packet[0] >> MQTT_TYPE_SHIFT);
    uint16_t remaining = 0u;
    uint16_t index = 1u;
    mqtt_result_t result = MQTT_OK;

    if (packet_len < MQTT_MIN_PACKET_1B_RL)
    {
        return MQTT_ERR_PROTOCOL;
    }
    remaining = packet[1];
    if ((remaining & MQTT_RL_CONT_FLAG) != 0u)
    {
        if (packet_len < MQTT_MIN_PACKET_2B_RL)
        {
            return MQTT_ERR_PROTOCOL;
        }
        remaining = (uint16_t) ((remaining & MQTT_RL_VALUE_MASK) |
                                ((uint32_t) packet[MQTT_RL_BYTE2_OFFSET]
                                 << MQTT_RL_SHIFT));
        index = MQTT_RL_INDEX_2B;
    }
    if ((uint16_t) (index + 1u + remaining) != packet_len)
    {
        return MQTT_ERR_PROTOCOL;
    }

    if ((type == (MQTT_CONNACK >> MQTT_TYPE_SHIFT)) &&
        (client->state == MQTT_STATE_WAIT_CONNACK))
    {
        /* Byte 3 of the packet is the return code; byte 2 is the
           session-present flag, which never causes a reject. */
        if ((remaining != MQTT_CONNACK_REMAINING) ||
            (packet[index + MQTT_CONNACK_CODE_AT] != MQTT_CONNACK_ACCEPTED))
        {
            disconnect(client, now_ms);
            return MQTT_ERR_PROTOCOL;
        }
        result = send_subscribe(client, now_ms);
        if (result == MQTT_OK)
        {
            client->state = MQTT_STATE_WAIT_SUBACK;
            start_reply_timer(client, now_ms);
        }
        return (result == MQTT_ERR_ARG) ? MQTT_ERR_PROTOCOL : result;
    }
    if ((type == (MQTT_SUBACK >> MQTT_TYPE_SHIFT)) &&
        (client->state == MQTT_STATE_WAIT_SUBACK))
    {
        if ((remaining != MQTT_SUBACK_REMAINING) ||
            (packet[index + MQTT_SUBACK_CODE_AT] == MQTT_SUBACK_FAILURE))
        {
            disconnect(client, now_ms);
            return MQTT_ERR_PROTOCOL;
        }
        client->state = MQTT_STATE_CONNECTED;
        client->last_tx_ms = now_ms;
        client->reply_pending = false;
        client->reconnect_delay_ms = MQTT_RECONNECT_MIN_MS;
        return MQTT_OK;
    }
    if ((type == (MQTT_PUBLISH >> MQTT_TYPE_SHIFT)) &&
        (client->state == MQTT_STATE_CONNECTED))
    {
        return handle_publish(client, packet, packet_len, now_ms);
    }
    if (type == (MQTT_PINGRESP >> MQTT_TYPE_SHIFT))
    {
        if (client->state == MQTT_STATE_CONNECTED)
        {
            client->reply_pending = false;
        }
        return MQTT_OK;
    }

    return MQTT_ERR_PROTOCOL;
}

static mqtt_result_t
receive_packets (mqtt_client_t * client, uint32_t now_ms)
{
    int got = 0;

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

    while (client->rx_len >= MQTT_RX_MIN_HEADER)
    {
        uint16_t rem = client->rx_buffer[1];
        uint16_t header_len = MQTT_HEADER_LEN_1B_RL;
        uint16_t total_len = 0u;

        if ((rem & MQTT_RL_CONT_FLAG) != 0u)
        {
            if (client->rx_len < MQTT_MIN_PACKET_2B_RL)
            {
                break;
            }
            rem = (uint16_t) (
                (rem & MQTT_RL_VALUE_MASK) |
                ((uint32_t) client->rx_buffer[MQTT_RL_BYTE2_OFFSET]
                 << MQTT_RL_SHIFT));
            header_len = MQTT_HEADER_LEN_2B_RL;
        }
        total_len = (uint16_t) (header_len + rem);
        if (client->rx_len < total_len)
        {
            break;
        }
        const mqtt_result_t result =
            handle_packet(client, client->rx_buffer, total_len, now_ms);
        if (result != MQTT_OK)
        {
            return result;
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
                  const mqtt_transport_t * transport, mqtt_command_fn_t handler,
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
    client->attempt_pending = true;
    client->reconnect_delay_ms = MQTT_RECONNECT_MIN_MS;
    if (client->config.keepalive_s == 0u)
    {
        client->config.keepalive_s = MQTT_DEFAULT_KEEPALIVE_S;
    }
}

mqtt_result_t
mqtt_client_poll (mqtt_client_t * client, uint32_t now_ms)
{
    mqtt_result_t result = MQTT_OK;

    if ((client == 0) || !client->valid)
    {
        return MQTT_ERR_ARG;
    }

    if (client->state == MQTT_STATE_DISCONNECTED)
    {
        /* The first attempt is due at once, whatever now_ms is. */
        if (!client->attempt_pending &&
            ((int32_t) (now_ms - client->next_attempt_ms) < 0))
        {
            return MQTT_OK;
        }
        client->attempt_pending = false;
        result = send_connect(client, now_ms);
        if (result == MQTT_ERR_BUSY)
        {
            return result;
        }
        if (result != MQTT_OK)
        {
            disconnect(client, now_ms);
            return result;
        }
        client->state = MQTT_STATE_WAIT_CONNACK;
        start_reply_timer(client, now_ms);
        return MQTT_OK;
    }

    result = receive_packets(client, now_ms);
    if ((result != MQTT_OK) && (result != MQTT_ERR_BUSY))
    {
        if (client->state != MQTT_STATE_DISCONNECTED)
        {
            disconnect(client, now_ms);
        }
        return result;
    }

    /* Replies are processed first, so one arriving at the deadline counts. */
    if (client->reply_pending &&
        ((int32_t) (now_ms - client->reply_deadline_ms) >= 0))
    {
        disconnect(client, now_ms);
        return MQTT_ERR_TRANSPORT;
    }

    if (result == MQTT_ERR_BUSY)
    {
        return result;
    }

    if ((client->state == MQTT_STATE_CONNECTED) &&
        (client->config.keepalive_s > 0u) &&
        ((now_ms - client->last_tx_ms) >=
         ((uint32_t) client->config.keepalive_s * MQTT_MS_PER_S)))
    {
        result = send_packet(client, MQTT_PINGREQ, 0, 0u, now_ms);
        if ((result != MQTT_OK) && (result != MQTT_ERR_BUSY))
        {
            disconnect(client, now_ms);
        }
        else if ((result == MQTT_OK) && !client->reply_pending)
        {
            start_reply_timer(client, now_ms);
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
    uint16_t topic_len = 0u;
    mqtt_result_t result = MQTT_OK;

    if ((client == 0) || !client->valid || (topic == 0) || (payload == 0))
    {
        return MQTT_ERR_ARG;
    }
    if (client->state != MQTT_STATE_CONNECTED)
    {
        return MQTT_ERR_STATE;
    }

    topic_len = (uint16_t) strlen(topic);
    if (!packet_fits((uint32_t) MQTT_U16_SIZE + topic_len + len))
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
    if ((result != MQTT_OK) && (result != MQTT_ERR_BUSY))
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
