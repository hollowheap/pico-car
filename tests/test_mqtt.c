/*
 * Host unit test for src/telemetry/mqtt_client.c (spec S19).
 *
 * The client only talks to an injected transport port.  This test supplies a
 * fake transport that records every byte sent and replays scripted received
 * bytes (optionally in small chunks), then checks the exact MQTT 3.1.1 bytes
 * of CONNECT, SUBSCRIBE, PUBLISH, PUBACK and PINGREQ, the handshake state
 * machine, QoS 1 command delivery with duplicate suppression, keepalive
 * timing and reconnect behaviour.
 *
 * The assertions pin MQTT 3.1.1 behaviour plus decision D91 (10 s reply
 * timeouts, 1 s to 30 s reconnect backoff).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry/mqtt.h"

static unsigned g_failures = 0u;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            (void) printf ("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define FAKE_SENT_MAX 2048u
#define FAKE_RX_MAX   1024u
#define RECONNECT_MS  MQTT_RECONNECT_MIN_MS
#define CMD_TOPIC     "c"

/* ---- fake transport ---------------------------------------------------- */

typedef struct
{
    uint8_t  sent[FAKE_SENT_MAX];
    uint32_t sent_len;
    unsigned send_calls;
    uint8_t  rx[FAKE_RX_MAX];
    uint32_t rx_len;
    uint32_t rx_pos;
    uint16_t rx_chunk; /* 0 = deliver everything available */
    unsigned recv_calls;
    bool     fail_send;
    bool     short_send;
    bool     busy_send;
    unsigned restarts;
    bool     fail_recv;
} fake_t;

static int
fake_send (void * context, const uint8_t * data, uint16_t len)
{
    fake_t * f = (fake_t *) context;

    f->send_calls++;
    if (f->fail_send)
    {
        return -1;
    }
    if (f->busy_send)
    {
        return 0;
    }
    if (f->short_send)
    {
        return (int) len - 1;
    }
    CHECK ((f->sent_len + len) <= FAKE_SENT_MAX); /* fixture capacity */
    if ((f->sent_len + len) <= FAKE_SENT_MAX)
    {
        (void) memcpy (&f->sent[f->sent_len], data, len);
        f->sent_len += len;
    }
    return (int) len;
}

static int
fake_recv (void * context, uint8_t * data, uint16_t len)
{
    fake_t * f = (fake_t *) context;
    uint32_t n = f->rx_len - f->rx_pos;

    f->recv_calls++;
    if (f->fail_recv)
    {
        return -1;
    }
    if (n > len)
    {
        n = len;
    }
    if ((f->rx_chunk != 0u) && (n > f->rx_chunk))
    {
        n = f->rx_chunk;
    }
    (void) memcpy (data, &f->rx[f->rx_pos], n);
    f->rx_pos += n;
    return (int) n;
}

static void
fake_restart (void * context)
{
    fake_t * f = (fake_t *) context;

    f->restarts++;
}

static void
fake_feed (fake_t * f, const uint8_t * data, uint16_t len)
{
    CHECK ((f->rx_len + len) <= FAKE_RX_MAX); /* fixture capacity */
    if ((f->rx_len + len) <= FAKE_RX_MAX)
    {
        (void) memcpy (&f->rx[f->rx_len], data, len);
        f->rx_len += len;
    }
}

static uint32_t
fake_mark (const fake_t * f)
{
    return f->sent_len;
}

/* True when exactly want[0..n) was sent since mark. */
static bool
sent_since_is (const fake_t * f, uint32_t mark, const uint8_t * want,
               uint16_t n)
{
    return ((f->sent_len - mark) == n) &&
           (memcmp (&f->sent[mark], want, n) == 0);
}

static bool
nothing_sent_since (const fake_t * f, uint32_t mark)
{
    return f->sent_len == mark;
}

/* ---- command handler probe -------------------------------------------- */

typedef struct
{
    unsigned calls;
    uint8_t  payload[32];
    uint16_t len;
    void *   ctx;
} handler_probe_t;

static void
probe_handler (const uint8_t * payload, uint16_t len, void * context)
{
    handler_probe_t * p = (handler_probe_t *) context;

    p->calls++;
    p->len = len;
    p->ctx = context;
    if (len <= sizeof (p->payload))
    {
        (void) memcpy (p->payload, payload, len);
    }
}

/* ---- fixture ----------------------------------------------------------- */

typedef struct
{
    fake_t          fake;
    handler_probe_t probe;
    mqtt_client_t   client;
    mqtt_config_t   config;
    mqtt_transport_t transport;
} rig_t;

static void
rig_init_cfg (rig_t * r, const char * id, const char * user,
              const char * pass, const char * topic, uint16_t keepalive,
              bool with_handler)
{
    (void) memset (r, 0, sizeof (*r));
    r->config.client_id = id;
    r->config.username = user;
    r->config.password = pass;
    r->config.command_topic = topic;
    r->config.keepalive_s = keepalive;
    r->transport.context = &r->fake;
    r->transport.send = fake_send;
    r->transport.recv = fake_recv;
    r->transport.restart = fake_restart;
    mqtt_client_init (&r->client, &r->config, &r->transport,
                      with_handler ? probe_handler : NULL, &r->probe);
}

static void
rig_init (rig_t * r)
{
    rig_init_cfg (r, "car1", NULL, NULL, CMD_TOPIC, 0u, true);
}

static const uint8_t CONNACK_OK[4] = { 0x20, 0x02, 0x00, 0x00 };
static const uint8_t SUBACK_OK[5] = { 0x90, 0x03, 0x00, 0x01, 0x01 };

/* Drive CONNECT -> CONNACK -> SUBSCRIBE -> SUBACK.  The CONNECT goes out at
   t_connect; the broker replies are processed at t_ack. */
static void
rig_handshake_at (rig_t * r, uint32_t t_connect, uint32_t t_ack)
{
    CHECK (mqtt_client_poll (&r->client, t_connect) == MQTT_OK);
    CHECK (mqtt_client_state (&r->client) == MQTT_STATE_WAIT_CONNACK);
    fake_feed (&r->fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r->client, t_ack) == MQTT_OK);
    CHECK (mqtt_client_state (&r->client) == MQTT_STATE_WAIT_SUBACK);
    fake_feed (&r->fake, SUBACK_OK, sizeof (SUBACK_OK));
    CHECK (mqtt_client_poll (&r->client, t_ack) == MQTT_OK);
    CHECK (mqtt_client_state (&r->client) == MQTT_STATE_CONNECTED);
}

static void
rig_handshake (rig_t * r, uint32_t t)
{
    rig_handshake_at (r, t, t);
}

/* Build a PUBLISH with a one-byte remaining length.  flags is the low
   nibble of the first byte (bit 3 DUP, bits 2..1 QoS, bit 0 RETAIN). */
static uint16_t
build_publish (uint8_t * out, uint8_t flags, const char * topic,
               uint16_t packet_id, const uint8_t * payload, uint16_t plen)
{
    uint16_t tl = (uint16_t) strlen (topic);
    uint16_t qos = (uint16_t) ((flags >> 1) & 0x03u);
    uint16_t rem = (uint16_t) (2u + tl + ((qos != 0u) ? 2u : 0u) + plen);
    uint16_t i = 0u;

    out[i++] = (uint8_t) (0x30u | flags);
    out[i++] = (uint8_t) rem;
    out[i++] = (uint8_t) (tl >> 8);
    out[i++] = (uint8_t) tl;
    (void) memcpy (&out[i], topic, tl);
    i = (uint16_t) (i + tl);
    if (qos != 0u)
    {
        out[i++] = (uint8_t) (packet_id >> 8);
        out[i++] = (uint8_t) packet_id;
    }
    (void) memcpy (&out[i], payload, plen);
    return (uint16_t) (i + plen);
}

/* 8-byte command payload; only cmd_id matters to the MQTT layer. */
static void
cmd_payload (uint8_t * out, uint16_t cmd_id)
{
    out[0] = (uint8_t) (cmd_id >> 8);
    out[1] = (uint8_t) cmd_id;
    out[2] = 0x12u;
    out[3] = 0x34u;
    out[4] = 0x00u;
    out[5] = 0x64u;
    out[6] = 0x01u;
    out[7] = 0x00u;
}

/* Deliver a QoS 1 command publish and poll once. */
static mqtt_result_t
deliver_cmd (rig_t * r, uint8_t flags, uint16_t packet_id, uint16_t cmd_id,
             uint32_t now)
{
    uint8_t payload[8];
    uint8_t pkt[64];
    uint16_t n;

    cmd_payload (payload, cmd_id);
    n = build_publish (pkt, flags, CMD_TOPIC, packet_id, payload, 8u);
    fake_feed (&r->fake, pkt, n);
    return mqtt_client_poll (&r->client, now);
}

/* ---- init / argument handling ----------------------------------------- */

static void
test_init_validation (void)
{
    rig_t r;
    mqtt_client_t c;
    mqtt_config_t good;
    mqtt_transport_t t;
    fake_t f;

    mqtt_client_init (NULL, NULL, NULL, NULL, NULL); /* must not crash */
    CHECK (mqtt_client_state (NULL) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (NULL, 0u) == MQTT_ERR_ARG);
    CHECK (mqtt_client_publish (NULL, "s", (const uint8_t *) "x", 1u, 0u)
           == MQTT_ERR_ARG);

    (void) memset (&f, 0, sizeof (f));
    good.client_id = "car1";
    good.username = NULL;
    good.password = NULL;
    good.command_topic = CMD_TOPIC;
    good.keepalive_s = 0u;
    t.context = &f;
    t.send = fake_send;
    t.recv = fake_recv;

    /* Each invalid input leaves an invalid client: poll says ERR_ARG and
       nothing is sent. */
    mqtt_client_init (&c, NULL, &t, NULL, NULL);
    CHECK (mqtt_client_poll (&c, 0u) == MQTT_ERR_ARG);
    mqtt_client_init (&c, &good, NULL, NULL, NULL);
    CHECK (mqtt_client_poll (&c, 0u) == MQTT_ERR_ARG);

    t.send = NULL;
    mqtt_client_init (&c, &good, &t, NULL, NULL);
    CHECK (mqtt_client_poll (&c, 0u) == MQTT_ERR_ARG);
    t.send = fake_send;
    t.recv = NULL;
    mqtt_client_init (&c, &good, &t, NULL, NULL);
    CHECK (mqtt_client_poll (&c, 0u) == MQTT_ERR_ARG);
    t.recv = fake_recv;

    good.client_id = NULL;
    mqtt_client_init (&c, &good, &t, NULL, NULL);
    CHECK (mqtt_client_poll (&c, 0u) == MQTT_ERR_ARG);
    good.client_id = "car1";
    good.command_topic = NULL;
    mqtt_client_init (&c, &good, &t, NULL, NULL);
    CHECK (mqtt_client_poll (&c, 0u) == MQTT_ERR_ARG);
    CHECK (mqtt_client_publish (&c, "s", (const uint8_t *) "x", 1u, 0u)
           == MQTT_ERR_ARG);
    CHECK (f.send_calls == 0u);
    CHECK (mqtt_client_state (&c) == MQTT_STATE_DISCONNECTED);

    rig_init (&r);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.client.config.keepalive_s == MQTT_DEFAULT_KEEPALIVE_S);
    CHECK (MQTT_DEFAULT_KEEPALIVE_S == 30u);
}

/* ---- CONNECT ------------------------------------------------------------ */

static void
test_connect_bytes_default (void)
{
    rig_t r;
    /* 0x10, RL 16; "MQTT", level 4, clean session, keepalive 30, "car1". */
    static const uint8_t want[] = { 0x10, 0x10, 0x00, 0x04, 'M', 'Q', 'T',
                                    'T',  0x04, 0x02, 0x00, 0x1E, 0x00, 0x04,
                                    'c',  'a',  'r',  '1' };

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK (r.fake.send_calls == 1u);
    CHECK (sent_since_is (&r.fake, 0u, want, sizeof (want)));
    /* CONNECT does not read from the socket. */
    CHECK (r.fake.recv_calls == 0u);
}

static void
test_connect_keepalive_config (void)
{
    rig_t r;
    static const uint8_t want[] = { 0x10, 0x10, 0x00, 0x04, 'M', 'Q', 'T',
                                    'T',  0x04, 0x02, 0x00, 0x3C, 0x00, 0x04,
                                    'c',  'a',  'r',  '1' };

    rig_init_cfg (&r, "car1", NULL, NULL, CMD_TOPIC, 60u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, 0u, want, sizeof (want)));
}

static void
test_connect_credentials (void)
{
    rig_t r;
    /* flags 0xC2 = user + pass + clean; RL = 10 + 6 + 3 + 4 = 23. */
    static const uint8_t want[] = { 0x10, 0x17, 0x00, 0x04, 'M',  'Q',  'T',
                                    'T',  0x04, 0xC2, 0x00, 0x1E, 0x00, 0x04,
                                    'c',  'a',  'r',  '1',  0x00, 0x01, 'u',
                                    0x00, 0x02, 'p',  'w' };
    static const uint8_t want_user_only[] = {
        0x10, 0x13, 0x00, 0x04, 'M',  'Q',  'T',  'T',  0x04, 0x82,
        0x00, 0x1E, 0x00, 0x04, 'c',  'a',  'r',  '1',  0x00, 0x01, 'u' };
    static const uint8_t want_plain[] = { 0x10, 0x10, 0x00, 0x04, 'M', 'Q',
                                          'T',  'T',  0x04, 0x02, 0x00, 0x1E,
                                          0x00, 0x04, 'c',  'a',  'r',  '1' };

    rig_init_cfg (&r, "car1", "u", "pw", CMD_TOPIC, 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, 0u, want, sizeof (want)));

    /* MQTT 3.1.1 allows a username without a password: only the username
       flag (0x80 | clean = 0x82) is set and only the username is sent.
       RL = 10 + 6 + 3 = 19. */
    rig_init_cfg (&r, "car1", "u", NULL, CMD_TOPIC, 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, 0u, want_user_only,
                          sizeof (want_user_only)));

    /* 3.1.2.9: a password without a username is not sent. */
    rig_init_cfg (&r, "car1", NULL, "pw", CMD_TOPIC, 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, 0u, want_plain, sizeof (want_plain)));
}

static void
test_connect_client_id_too_long (void)
{
    rig_t r;
    char long_id[300];

    (void) memset (long_id, 'x', sizeof (long_id) - 1u);
    long_id[sizeof (long_id) - 1u] = '\0';

    rig_init_cfg (&r, long_id, NULL, NULL, CMD_TOPIC, 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_ERR_ARG);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.fake.send_calls == 0u);
}

/* ---- handshake ---------------------------------------------------------- */

static void
test_handshake_bytes (void)
{
    rig_t r;
    uint32_t mark;
    /* SUBSCRIBE: 0x82, RL 6, packet id 1, topic "c", requested QoS 1. */
    static const uint8_t want_sub[] = { 0x82, 0x06, 0x00, 0x01,
                                        0x00, 0x01, 'c',  0x01 };

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 5u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
    CHECK (sent_since_is (&r.fake, mark, want_sub, sizeof (want_sub)));

    fake_feed (&r.fake, SUBACK_OK, sizeof (SUBACK_OK));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
    CHECK (nothing_sent_since (&r.fake, mark));
}

static void
test_handshake_longer_topic (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t want_sub[] = { 0x82, 0x0B, 0x00, 0x01, 0x00, 0x06,
                                        'c',  'a',  'r',  '/',  'c',  'm',
                                        0x01 };

    /* topic "car/cm" is 6 bytes: RL = 2 + 2 + 6 + 1 = 11 */
    rig_init_cfg (&r, "car1", NULL, NULL, "car/cm", 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 5u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, mark, want_sub, sizeof (want_sub)));
}

static void
test_handshake_topic_limit (void)
{
    rig_t r;
    char topic[80];
    uint32_t mark;

    /* The SUBSCRIBE body buffer holds a topic of up to 59 bytes. */
    (void) memset (topic, 't', sizeof (topic));
    topic[59] = '\0';
    rig_init_cfg (&r, "car1", NULL, NULL, topic, 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 5u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
    /* 0x82, RL 64 (2 + 2 + 59 + 1), packet id 1, topic length 59. */
    CHECK ((r.fake.sent_len - mark) == 66u);
    CHECK (r.fake.sent[mark] == 0x82u);
    CHECK (r.fake.sent[mark + 1u] == 0x40u);
    CHECK (r.fake.sent[mark + 2u] == 0x00u);
    CHECK (r.fake.sent[mark + 3u] == 0x01u);
    CHECK (r.fake.sent[mark + 4u] == 0x00u);
    CHECK (r.fake.sent[mark + 5u] == 59u);
    CHECK (memcmp (&r.fake.sent[mark + 6u], topic, 59u) == 0);
    CHECK (r.fake.sent[mark + 65u] == 0x01u);

    topic[59] = 't';
    topic[60] = '\0';
    rig_init_cfg (&r, "car1", NULL, NULL, topic, 0u, true);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    /* The SUBSCRIBE failure is reported as a protocol error by the receive
       path, and the connection is dropped. */
    CHECK (mqtt_client_poll (&r.client, 5u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

static void
test_handshake_single_burst (void)
{
    rig_t r;
    uint8_t both[9];

    /* CONNACK and SUBACK arriving in the same read are both processed. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    (void) memcpy (both, CONNACK_OK, 4u);
    (void) memcpy (&both[4], SUBACK_OK, 5u);
    fake_feed (&r.fake, both, sizeof (both));
    CHECK (mqtt_client_poll (&r.client, 1u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
test_handshake_chunked (void)
{
    rig_t r;
    uint32_t mark;
    unsigned i;

    rig_init (&r);
    r.fake.rx_chunk = 1u;
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    mark = fake_mark (&r.fake);

    /* One byte per poll: no reaction until the fourth byte arrives. */
    for (i = 0u; i < 3u; i++)
    {
        CHECK (mqtt_client_poll (&r.client, 1u + i) == MQTT_OK);
        CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
        CHECK (nothing_sent_since (&r.fake, mark));
    }
    CHECK (mqtt_client_poll (&r.client, 4u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
    CHECK (!nothing_sent_since (&r.fake, mark));

    fake_feed (&r.fake, SUBACK_OK, sizeof (SUBACK_OK));
    for (i = 0u; i < 4u; i++)
    {
        CHECK (mqtt_client_poll (&r.client, 5u + i) == MQTT_OK);
        CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
    }
    CHECK (mqtt_client_poll (&r.client, 9u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
test_handshake_bad_connack (void)
{
    rig_t r;
    static const uint8_t bad_len[5] = { 0x20, 0x03, 0x00, 0x00, 0x00 };
    static const uint8_t session_present[4] = { 0x20, 0x02, 0x01, 0x00 };
    static const uint8_t refused[4] = { 0x20, 0x02, 0x00, 0x05 };
    static const uint8_t refused_sp[4] = { 0x20, 0x02, 0x01, 0x05 };
    uint32_t mark;

    /* Wrong remaining length. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, bad_len, sizeof (bad_len));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (nothing_sent_since (&r.fake, mark));

    /* Session-present set with return code 0 is accepted (3.2.2.2). */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, session_present, sizeof (session_present));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);

    /* The return code is byte 3: 0x05 "not authorized" closes the session. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, refused, sizeof (refused));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (nothing_sent_since (&r.fake, mark));

    /* A refusal that also sets session-present is still a refusal. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, refused_sp, sizeof (refused_sp));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

static void
test_handshake_suback_failure_code (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t suback_fail[5] = { 0x90, 0x03, 0x00, 0x01, 0x80 };

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, 1u) == MQTT_OK);
    fake_feed (&r.fake, suback_fail, sizeof (suback_fail));
    /* MQTT 3.1.1 3.9.3: 0x80 is "subscription failure".  The session is
       closed and a reconnect is scheduled one backoff step later. */
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 2u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_poll (&r.client, 2u + RECONNECT_MS - 1u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, 2u + RECONNECT_MS) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
}

static void
test_handshake_connack_timeout (void)
{
    rig_t r;
    uint32_t mark;
    const uint32_t t0 = 500u;

    /* A silent broker: the session is closed 10 s after the CONNECT. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, t0) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t0 + 1u) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t0 + MQTT_REPLY_TIMEOUT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK (r.fake.send_calls == 1u);
    CHECK (mqtt_client_poll (&r.client, t0 + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);

    /* Reconnect one backoff step later (1 s). */
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client,
                             t0 + MQTT_REPLY_TIMEOUT_MS + RECONNECT_MS - 1u)
           == MQTT_OK);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_poll (&r.client,
                             t0 + MQTT_REPLY_TIMEOUT_MS + RECONNECT_MS)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK (r.fake.sent[mark] == 0x10u);

    /* A CONNACK just before the deadline keeps the session. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, t0) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, t0 + MQTT_REPLY_TIMEOUT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);

    /* A CONNACK that arrives on the deadline poll is still processed. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, t0) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, t0 + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
}

static void
test_handshake_suback_timeout (void)
{
    rig_t r;
    const uint32_t t_connack = 2000u;

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, t_connack) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);

    /* The SUBACK timer starts when the SUBSCRIBE is sent, not at CONNECT. */
    CHECK (mqtt_client_poll (&r.client, t_connack + MQTT_REPLY_TIMEOUT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
    CHECK (mqtt_client_poll (&r.client, t_connack + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client,
                             t_connack + MQTT_REPLY_TIMEOUT_MS +
                                 RECONNECT_MS)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);

    /* A SUBACK before the deadline stops the timer for good. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (mqtt_client_poll (&r.client, MQTT_REPLY_TIMEOUT_MS * 2u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
test_handshake_timeout_tick_wrap (void)
{
    rig_t r;
    const uint32_t t0 = UINT32_MAX - 4000u; /* deadline wraps to 5999 */

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, t0) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t0 + MQTT_REPLY_TIMEOUT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK (mqtt_client_poll (&r.client, t0 + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

static void
test_publish_before_suback_is_protocol_error (void)
{
    rig_t r;
    uint8_t pkt[64];
    uint8_t payload[8];
    uint16_t n;

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, 1u) == MQTT_OK);
    cmd_payload (payload, 1u);
    n = build_publish (pkt, 0x00u, CMD_TOPIC, 0u, payload, 8u);
    fake_feed (&r.fake, pkt, n);
    CHECK (mqtt_client_poll (&r.client, 2u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.probe.calls == 0u);
}

/* ---- outbound PUBLISH --------------------------------------------------- */

static void
test_publish_qos0_bytes (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t body[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    /* 0x30 = PUBLISH, QoS 0, no DUP, no RETAIN; RL = 2 + 1 + 8 = 11. */
    static const uint8_t want[] = { 0x30, 0x0B, 0x00, 0x01, 's', 1, 2, 3,
                                    4,    5,    6,    7,    8 };
    static const uint8_t want_empty[] = { 0x30, 0x03, 0x00, 0x01, 'h' };

    rig_init (&r);
    rig_handshake (&r, 0u);

    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_publish (&r.client, "s", body, 8u, 100u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, mark, want, sizeof (want)));

    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_publish (&r.client, "h", body, 0u, 100u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, mark, want_empty, sizeof (want_empty)));
}

static void
test_publish_rejects (void)
{
    rig_t r;
    static const uint8_t body[2] = { 1, 2 };
    uint32_t mark;

    rig_init (&r);

    /* Not connected yet. */
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_publish (&r.client, "s", body, 2u, 0u)
           == MQTT_ERR_STATE);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (r.fake.send_calls == 0u);

    rig_handshake (&r, 0u);
    CHECK (mqtt_client_publish (&r.client, NULL, body, 2u, 1u)
           == MQTT_ERR_ARG);
    CHECK (mqtt_client_publish (&r.client, "s", NULL, 2u, 1u) == MQTT_ERR_ARG);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
publish_n (rig_t * r, uint16_t n, mqtt_result_t * result)
{
    uint8_t body[300];

    (void) memset (body, 0x5A, sizeof (body));
    *result = mqtt_client_publish (&r->client, "s", body, n, 100u);
}

static void
test_publish_remaining_length_encoding (void)
{
    rig_t r;
    mqtt_result_t res;
    uint32_t mark;

    /* Body = 2 + 1 + n.  n = 124 -> 127 (one-byte RL 0x7F). */
    rig_init (&r);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    publish_n (&r, 124u, &res);
    CHECK (res == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == (1u + 1u + 127u));
    CHECK (r.fake.sent[mark] == 0x30u);
    CHECK (r.fake.sent[mark + 1u] == 0x7Fu);

    /* n = 125 -> 128 (two-byte RL 0x80 0x01). */
    mark = fake_mark (&r.fake);
    publish_n (&r, 125u, &res);
    CHECK (res == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == (1u + 2u + 128u));
    CHECK (r.fake.sent[mark + 1u] == 0x80u);
    CHECK (r.fake.sent[mark + 2u] == 0x01u);
    CHECK (r.fake.sent[mark + 3u] == 0x00u);
    CHECK (r.fake.sent[mark + 4u] == 0x01u);
    CHECK (r.fake.sent[mark + 5u] == (uint8_t) 's');

    /* n = 250 -> 253 (0xFD 0x01), the largest packet that fits the 256-byte
       TX buffer (1 + 2 + 253). */
    mark = fake_mark (&r.fake);
    publish_n (&r, 250u, &res);
    CHECK (res == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == 256u);
    CHECK (r.fake.sent[mark + 1u] == 0xFDu);
    CHECK (r.fake.sent[mark + 2u] == 0x01u);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
test_publish_oversize (void)
{
    rig_t r;
    mqtt_result_t res;
    uint32_t mark;
    uint16_t n;

    /* 251..253 bytes fit the body but not the fixed header: the same
       up-front rejection, nothing sent, session kept. */
    for (n = 251u; n <= 254u; n++)
    {
        rig_init (&r);
        rig_handshake (&r, 0u);
        mark = fake_mark (&r.fake);
        publish_n (&r, n, &res);
        CHECK (res == MQTT_ERR_PROTOCOL);
        CHECK (nothing_sent_since (&r.fake, mark));
        CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
    }

    /* 250 bytes is the largest that fits, and it still goes out. */
    mark = fake_mark (&r.fake);
    publish_n (&r, 250u, &res);
    CHECK (res == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == MQTT_TX_BUFFER_SIZE);
}

static void
test_publish_transport_failure (void)
{
    rig_t r;
    static const uint8_t body[4] = { 1, 2, 3, 4 };

    rig_init (&r);
    rig_handshake (&r, 0u);
    r.fake.fail_send = true;
    CHECK (mqtt_client_publish (&r.client, "s", body, 4u, 5000u)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    r.fake.fail_send = false;

    /* Retry only after the reconnect interval. */
    CHECK (mqtt_client_poll (&r.client, 5000u + RECONNECT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, 5000u + RECONNECT_MS) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);

    /* A short write counts as failure too. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    r.fake.short_send = true;
    CHECK (mqtt_client_publish (&r.client, "s", body, 4u, 10u)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

/* A full transport buffer must not schedule a second CONNECT. */
static void
test_publish_retries_without_restarting (void)
{
    rig_t r;
    const uint8_t payload[1] = { 0x42u };
    uint32_t mark;

    rig_init (&r);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    r.fake.busy_send = true;
    CHECK (mqtt_client_publish (&r.client, "s", payload, sizeof (payload), 10u)
           == MQTT_ERR_BUSY);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
    CHECK (r.fake.restarts == 0u);
    CHECK (mqtt_client_poll (&r.client, 10u + RECONNECT_MS) == MQTT_OK);
    CHECK (nothing_sent_since (&r.fake, mark));

    r.fake.busy_send = false;
    CHECK (mqtt_client_publish (&r.client, "s", payload, sizeof (payload),
                                20u + RECONNECT_MS) == MQTT_OK);
    CHECK (r.fake.sent[mark] == 0x30u);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);

    r.fake.fail_send = true;
    CHECK (mqtt_client_publish (&r.client, "s", payload, sizeof (payload),
                                30u + RECONNECT_MS) == MQTT_ERR_TRANSPORT);
    CHECK (r.fake.restarts == 1u);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

static void
test_handshake_waits_for_writable_transport (void)
{
    rig_t r;
    uint32_t mark;

    rig_init (&r);
    r.fake.busy_send = true;
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_ERR_BUSY);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.fake.restarts == 0u);

    r.fake.busy_send = false;
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);

    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    mark = fake_mark (&r.fake);
    r.fake.busy_send = true;
    CHECK (mqtt_client_poll (&r.client, 20u) == MQTT_ERR_BUSY);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK (r.fake.restarts == 0u);
    CHECK (nothing_sent_since (&r.fake, mark));

    r.fake.busy_send = false;
    CHECK (mqtt_client_poll (&r.client, 30u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
    CHECK (r.fake.sent[mark] == 0x82u);
    fake_feed (&r.fake, SUBACK_OK, sizeof (SUBACK_OK));
    CHECK (mqtt_client_poll (&r.client, 40u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
test_command_retries_puback_after_backpressure (void)
{
    rig_t r;
    const uint8_t ack[] = { 0x40u, 0x02u, 0x00u, 0x2Au };
    uint32_t mark;

    rig_init (&r);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    r.fake.busy_send = true;
    CHECK (deliver_cmd (&r, 0x02u, 0x002Au, 1u, 10u) == MQTT_ERR_BUSY);
    CHECK (r.probe.calls == 0u);
    CHECK (r.fake.restarts == 0u);
    CHECK (nothing_sent_since (&r.fake, mark));

    r.fake.busy_send = false;
    CHECK (mqtt_client_poll (&r.client, 20u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (sent_since_is (&r.fake, mark, ack, sizeof (ack)));
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

/* ---- inbound PUBLISH / PUBACK ------------------------------------------ */

static void
test_cmd_qos1_puback_and_dispatch (void)
{
    rig_t r;
    uint32_t mark;
    uint8_t want_payload[8];
    static const uint8_t puback[4] = { 0x40, 0x02, 0x00, 0x2A };

    rig_init (&r);
    rig_handshake (&r, 0u);

    mark = fake_mark (&r.fake);
    CHECK (deliver_cmd (&r, 0x02u, 0x002Au, 1u, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (r.probe.len == 8u);
    cmd_payload (want_payload, 1u);
    CHECK (memcmp (r.probe.payload, want_payload, 8u) == 0);
    CHECK (r.probe.ctx == (void *) &r.probe);
    /* PUBACK: 0x40, RL 2, the packet ID of the PUBLISH. */
    CHECK (sent_since_is (&r.fake, mark, puback, sizeof (puback)));
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
}

static void
test_cmd_duplicate_delivery (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t puback_2b[4] = { 0x40, 0x02, 0x00, 0x2B };
    static const uint8_t puback_2c[4] = { 0x40, 0x02, 0x00, 0x2C };

    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (deliver_cmd (&r, 0x02u, 0x002Au, 1u, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);

    /* Broker retransmission: DUP set, same packet ID.  Acked again, not
       dispatched again. */
    mark = fake_mark (&r.fake);
    CHECK (deliver_cmd (&r, 0x0Au, 0x002Au, 1u, 11u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (!nothing_sent_since (&r.fake, mark));
    CHECK (r.fake.sent[mark] == 0x40u);

    /* Same cmd_id under a fresh packet ID: acked, still not dispatched. */
    mark = fake_mark (&r.fake);
    CHECK (deliver_cmd (&r, 0x02u, 0x002Bu, 1u, 12u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (sent_since_is (&r.fake, mark, puback_2b, sizeof (puback_2b)));

    /* A newer cmd_id is dispatched. */
    mark = fake_mark (&r.fake);
    CHECK (deliver_cmd (&r, 0x02u, 0x002Cu, 2u, 13u) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
    CHECK (sent_since_is (&r.fake, mark, puback_2c, sizeof (puback_2c)));

    /* An older cmd_id is acked but dropped. */
    CHECK (deliver_cmd (&r, 0x02u, 0x002Du, 1u, 14u) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
}

static void
test_cmd_id_wrap (void)
{
    rig_t r;

    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (deliver_cmd (&r, 0x02u, 1u, 0xFFFEu, 10u) == MQTT_OK);
    CHECK (deliver_cmd (&r, 0x02u, 2u, 0xFFFFu, 11u) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
    CHECK (deliver_cmd (&r, 0x02u, 3u, 0x0000u, 12u) == MQTT_OK);
    CHECK (r.probe.calls == 3u);
    CHECK (deliver_cmd (&r, 0x02u, 4u, 0xFFFFu, 13u) == MQTT_OK);
    CHECK (r.probe.calls == 3u);
    /* +0x8000 from the last ID (0x0000) counts as older. */
    CHECK (deliver_cmd (&r, 0x02u, 5u, 0x8000u, 14u) == MQTT_OK);
    CHECK (r.probe.calls == 3u);
    CHECK (deliver_cmd (&r, 0x02u, 6u, 0x7FFFu, 15u) == MQTT_OK);
    CHECK (r.probe.calls == 4u);
}

static void
test_cmd_first_id_zero (void)
{
    rig_t r;

    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (deliver_cmd (&r, 0x02u, 1u, 0u, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (deliver_cmd (&r, 0x02u, 2u, 0u, 11u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
}

static void
test_cmd_qos0 (void)
{
    rig_t r;
    uint32_t mark;

    rig_init (&r);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    CHECK (deliver_cmd (&r, 0x00u, 0u, 1u, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (nothing_sent_since (&r.fake, mark)); /* no PUBACK for QoS 0 */
    /* The ID filter applies to QoS 0 as well. */
    CHECK (deliver_cmd (&r, 0x00u, 0u, 1u, 11u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
}

static void
test_cmd_other_topic_and_short_payload (void)
{
    rig_t r;
    uint8_t pkt[64];
    uint8_t payload[8];
    uint16_t n;
    uint32_t mark;
    static const uint8_t puback[4] = { 0x40, 0x02, 0x00, 0x07 };
    static const uint8_t one[1] = { 0x01 };

    rig_init (&r);
    rig_handshake (&r, 0u);
    cmd_payload (payload, 1u);

    /* QoS 1 on a different topic: acked, not delivered to the handler. */
    n = build_publish (pkt, 0x02u, "x", 7u, payload, 8u);
    fake_feed (&r.fake, pkt, n);
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 0u);
    CHECK (sent_since_is (&r.fake, mark, puback, sizeof (puback)));

    /* Topic that only starts with the command topic. */
    n = build_publish (pkt, 0x02u, "cc", 7u, payload, 8u);
    fake_feed (&r.fake, pkt, n);
    CHECK (mqtt_client_poll (&r.client, 11u) == MQTT_OK);
    CHECK (r.probe.calls == 0u);

    /* Command topic but fewer than 2 payload bytes: acked, ignored. */
    n = build_publish (pkt, 0x02u, CMD_TOPIC, 7u, one, 1u);
    fake_feed (&r.fake, pkt, n);
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 12u) == MQTT_OK);
    CHECK (r.probe.calls == 0u);
    CHECK (sent_since_is (&r.fake, mark, puback, sizeof (puback)));
}

static void
test_cmd_qos2_is_protocol_error (void)
{
    rig_t r;
    uint32_t mark;

    rig_init (&r);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    /* The client subscribes at QoS 1, so a QoS 2 PUBLISH is a broker
       protocol violation: no dispatch, no PUBREC, session closed. */
    CHECK (deliver_cmd (&r, 0x04u, 0x0010u, 1u, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (r.probe.calls == 0u);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, 10u + RECONNECT_MS) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
}

static void
test_cmd_chunked_delivery (void)
{
    rig_t r;
    uint8_t payload[8];
    uint8_t pkt[64];
    uint16_t n;
    unsigned polls;
    static const uint8_t puback[4] = { 0x40, 0x02, 0x12, 0x34 };

    rig_init (&r);
    rig_handshake (&r, 0u);
    r.fake.rx_chunk = 1u;
    cmd_payload (payload, 9u);
    n = build_publish (pkt, 0x02u, CMD_TOPIC, 0x1234u, payload, 8u);
    fake_feed (&r.fake, pkt, n);

    for (polls = 1u; polls < n; polls++)
    {
        CHECK (mqtt_client_poll (&r.client, 10u + polls) == MQTT_OK);
        CHECK (r.probe.calls == 0u);
    }
    {
        uint32_t mark = fake_mark (&r.fake);
        CHECK (mqtt_client_poll (&r.client, 100u) == MQTT_OK);
        CHECK (r.probe.calls == 1u);
        CHECK (sent_since_is (&r.fake, mark, puback, sizeof (puback)));
    }
    CHECK (r.client.rx_len == 0u);

    /* Three bytes at a time. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    r.fake.rx_chunk = 3u;
    cmd_payload (payload, 10u);
    n = build_publish (pkt, 0x02u, CMD_TOPIC, 5u, payload, 8u);
    fake_feed (&r.fake, pkt, n);
    for (polls = 0u; polls < 10u; polls++)
    {
        CHECK (mqtt_client_poll (&r.client, 10u + polls) == MQTT_OK);
    }
    CHECK (r.probe.calls == 1u);
}

static void
test_cmd_back_to_back (void)
{
    rig_t r;
    uint8_t payload[8];
    uint8_t pkts[128];
    uint16_t n1;
    uint16_t n2;
    uint32_t mark;

    rig_init (&r);
    rig_handshake (&r, 0u);
    cmd_payload (payload, 1u);
    n1 = build_publish (pkts, 0x02u, CMD_TOPIC, 1u, payload, 8u);
    cmd_payload (payload, 2u);
    n2 = build_publish (&pkts[n1], 0x02u, CMD_TOPIC, 2u, payload, 8u);
    fake_feed (&r.fake, pkts, (uint16_t) (n1 + n2));

    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
    CHECK ((r.fake.sent_len - mark) == 8u); /* two 4-byte PUBACKs */
}

static void
test_cmd_null_handler (void)
{
    rig_t r;
    uint32_t mark;

    rig_init_cfg (&r, "car1", NULL, NULL, CMD_TOPIC, 0u, false);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    CHECK (deliver_cmd (&r, 0x02u, 3u, 1u, 10u) == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == 4u); /* still acked */
    CHECK (r.probe.calls == 0u);
}

static void
test_cmd_dedup_survives_reconnect (void)
{
    rig_t r;

    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (deliver_cmd (&r, 0x02u, 1u, 5u, 10u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);

    r.fake.fail_recv = true;
    CHECK (mqtt_client_poll (&r.client, 20u) == MQTT_ERR_TRANSPORT);
    r.fake.fail_recv = false;
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);

    r.fake.rx_pos = r.fake.rx_len;
    rig_handshake_at (&r, 20u + RECONNECT_MS, 20u + RECONNECT_MS);
    /* The broker redelivers the unacknowledged command after reconnect. */
    CHECK (deliver_cmd (&r, 0x0Au, 1u, 5u, 2000u) == MQTT_OK);
    CHECK (r.probe.calls == 1u);
    CHECK (deliver_cmd (&r, 0x02u, 2u, 6u, 2001u) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
}

static void
test_cmd_id_after_idle (void)
{
    rig_t r;
    const uint32_t later = 40010u;

    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (deliver_cmd (&r, 0x02u, 1u, 10u, 10u) == MQTT_OK);
    CHECK (deliver_cmd (&r, 0x02u, 2u, (uint16_t) later, later) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
    CHECK (deliver_cmd (&r, 0x0Au, 2u, (uint16_t) later, later + 1u) == MQTT_OK);
    CHECK (r.probe.calls == 2u);
}

static void
test_inbound_protocol_errors (void)
{
    rig_t r;
    static const uint8_t qos3[] = { 0x36, 0x05, 0x00, 0x01, 'c', 0x00, 0x01 };
    static const uint8_t topic_overrun[] = { 0x32, 0x03, 0x00, 0x05, 'c' };
    static const uint8_t no_packet_id[] = { 0x32, 0x04, 0x00, 0x01,
                                            'c',  'x' };
    static const uint8_t too_short[] = { 0x30, 0x02, 0x00, 0x01 };
    static const uint8_t pubrec[] = { 0x50, 0x02, 0x00, 0x01 };

    rig_init (&r);
    rig_handshake (&r, 0u);
    fake_feed (&r.fake, qos3, sizeof (qos3));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);

    rig_init (&r);
    rig_handshake (&r, 0u);
    fake_feed (&r.fake, topic_overrun, sizeof (topic_overrun));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.probe.calls == 0u);

    rig_init (&r);
    rig_handshake (&r, 0u);
    fake_feed (&r.fake, no_packet_id, sizeof (no_packet_id));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);

    rig_init (&r);
    rig_handshake (&r, 0u);
    fake_feed (&r.fake, too_short, sizeof (too_short));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);

    /* A packet type the client never expects (PUBREC). */
    rig_init (&r);
    rig_handshake (&r, 0u);
    fake_feed (&r.fake, pubrec, sizeof (pubrec));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

static void
test_rx_buffer_overflow (void)
{
    rig_t r;
    uint8_t junk[MQTT_RX_BUFFER_SIZE];

    /* A packet header announcing 127 bytes (129 total) can never fit the
       128-byte RX buffer.  The client must drop the connection, not wedge. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    (void) memset (junk, 0, sizeof (junk));
    junk[0] = 0x30u;
    junk[1] = 0x7Fu;
    fake_feed (&r.fake, junk, sizeof (junk));
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.client.rx_len == 0u);
}

/* ---- PINGREQ / keepalive ------------------------------------------------ */

static void
test_keepalive_ping (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t pingreq[2] = { 0xC0, 0x00 };
    static const uint8_t pingresp[2] = { 0xD0, 0x00 };
    const uint32_t t0 = 1000u;

    rig_init (&r);
    rig_handshake (&r, t0);

    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, t0 + 29999u) == MQTT_OK);
    CHECK (nothing_sent_since (&r.fake, mark));

    CHECK (mqtt_client_poll (&r.client, t0 + 30000u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, mark, pingreq, sizeof (pingreq)));

    /* The broker answers, so the PINGRESP timer is cleared. */
    fake_feed (&r.fake, pingresp, sizeof (pingresp));
    CHECK (mqtt_client_poll (&r.client, t0 + 30001u) == MQTT_OK);

    /* The next one is due 30 s after the previous PINGREQ. */
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, t0 + 30001u) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t0 + 59999u) == MQTT_OK);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_poll (&r.client, t0 + 60000u) == MQTT_OK);
    CHECK (sent_since_is (&r.fake, mark, pingreq, sizeof (pingreq)));
}

static void
test_keepalive_custom_interval (void)
{
    rig_t r;
    uint32_t mark;

    rig_init_cfg (&r, "car1", NULL, NULL, CMD_TOPIC, 5u, true);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 4999u) == MQTT_OK);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_poll (&r.client, 5000u) == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == 2u);
    CHECK (r.fake.sent[mark] == 0xC0u);
}

static void
test_keepalive_publish_resets_timer (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t body[1] = { 0 };
    const uint32_t t0 = 1000u;

    rig_init (&r);
    rig_handshake (&r, t0);
    CHECK (mqtt_client_publish (&r.client, "s", body, 1u, t0 + 20000u)
           == MQTT_OK);

    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, t0 + 30000u) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t0 + 49999u) == MQTT_OK);
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_poll (&r.client, t0 + 50000u) == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == 2u);
    CHECK (r.fake.sent[mark] == 0xC0u);
}

static void
test_keepalive_tick_wrap (void)
{
    rig_t r;
    uint32_t mark;
    const uint32_t t0 = UINT32_MAX - 5000u; /* 30 s later wraps the tick */

    rig_init (&r);
    rig_handshake_at (&r, 100u, t0);

    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, t0 + 29999u) == MQTT_OK); /* wrapped */
    CHECK (nothing_sent_since (&r.fake, mark));
    CHECK (mqtt_client_poll (&r.client, t0 + 30000u) == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == 2u);
    CHECK (r.fake.sent[mark] == 0xC0u);
}

static void
test_pingresp_accepted (void)
{
    rig_t r;
    static const uint8_t pingresp[2] = { 0xD0, 0x00 };
    uint32_t mark;

    rig_init (&r);
    rig_handshake (&r, 0u);
    fake_feed (&r.fake, pingresp, sizeof (pingresp));
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
    CHECK (nothing_sent_since (&r.fake, mark));
}

static void
test_pingresp_timeout (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t pingresp[2] = { 0xD0, 0x00 };
    const uint32_t t_ping = 30000u;

    /* No PINGRESP within 10 s of the PINGREQ closes the session. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, t_ping) == MQTT_OK);
    CHECK ((r.fake.sent_len - mark) == 2u);
    CHECK (mqtt_client_poll (&r.client, t_ping + MQTT_REPLY_TIMEOUT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
    CHECK (mqtt_client_poll (&r.client, t_ping + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client,
                             t_ping + MQTT_REPLY_TIMEOUT_MS + RECONNECT_MS)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);

    /* A PINGRESP clears the timer: the session survives past the deadline. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (mqtt_client_poll (&r.client, t_ping) == MQTT_OK);
    fake_feed (&r.fake, pingresp, sizeof (pingresp));
    CHECK (mqtt_client_poll (&r.client, t_ping + 5000u) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t_ping + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);

    /* No ping outstanding: a long silence alone never times out. */
    rig_init (&r);
    rig_handshake (&r, 0u);
    CHECK (mqtt_client_poll (&r.client, 29999u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);

    /* An unsolicited PINGRESP during the handshake does not clear the
       CONNACK timer. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, pingresp, sizeof (pingresp));
    CHECK (mqtt_client_poll (&r.client, 1u) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
}

static void
test_pingresp_timeout_tick_wrap (void)
{
    rig_t r;
    const uint32_t t_ping = UINT32_MAX - 4000u; /* deadline wraps */

    rig_init (&r);
    rig_handshake_at (&r, 100u, t_ping - 30000u);
    CHECK (mqtt_client_poll (&r.client, t_ping) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t_ping + MQTT_REPLY_TIMEOUT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);
    CHECK (mqtt_client_poll (&r.client, t_ping + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

static void
test_ping_send_failure (void)
{
    rig_t r;

    rig_init (&r);
    rig_handshake (&r, 0u);
    r.fake.fail_send = true;
    CHECK (mqtt_client_poll (&r.client, 30000u) == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
}

/* ---- reconnect / backoff ----------------------------------------------- */

static void
test_reconnect_after_recv_failure (void)
{
    rig_t r;
    uint32_t mark;
    static const uint8_t partial[3] = { 0x30, 0x0B, 0x00 };

    rig_init (&r);
    rig_handshake (&r, 0u);

    /* A partial packet is in the RX buffer when the socket dies. */
    fake_feed (&r.fake, partial, sizeof (partial));
    CHECK (mqtt_client_poll (&r.client, 50u) == MQTT_OK);
    CHECK (r.client.rx_len == 3u);
    r.fake.fail_recv = true;
    CHECK (mqtt_client_poll (&r.client, 10000u) == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.client.rx_len == 0u);
    r.fake.fail_recv = false;
    r.fake.rx_pos = r.fake.rx_len;

    /* Within the interval: poll returns at once, touching no transport call. */
    {
        unsigned sends = r.fake.send_calls;
        unsigned recvs = r.fake.recv_calls;

        CHECK (mqtt_client_poll (&r.client, 10000u) == MQTT_OK);
        CHECK (mqtt_client_poll (&r.client, 10000u + RECONNECT_MS - 1u)
               == MQTT_OK);
        CHECK (r.fake.send_calls == sends);
        CHECK (r.fake.recv_calls == recvs);
    }

    /* After the interval a fresh CONNECT is sent. */
    mark = fake_mark (&r.fake);
    CHECK (mqtt_client_poll (&r.client, 10000u + RECONNECT_MS) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK ((r.fake.sent_len - mark) == 18u);
    CHECK (r.fake.sent[mark] == 0x10u);

    /* The stale partial packet is gone: the handshake completes normally. */
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, 12000u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_SUBACK);
}

/* Poll until the next CONNECT attempt (which fails) and return its time.
   The attempt must not happen one tick early. */
static uint32_t
expect_failed_attempt_at (rig_t * r, uint32_t earliest, uint32_t delay)
{
    unsigned calls = r->fake.send_calls;

    CHECK (mqtt_client_poll (&r->client, earliest + delay - 1u) == MQTT_OK);
    CHECK (r->fake.send_calls == calls);
    CHECK (mqtt_client_poll (&r->client, earliest + delay)
           == MQTT_ERR_TRANSPORT);
    CHECK (r->fake.send_calls == (calls + 1u));
    return earliest + delay;
}

static void
test_reconnect_connect_send_failure (void)
{
    rig_t r;
    uint32_t t;
    uint32_t delay;

    rig_init (&r);
    r.fake.fail_send = true;

    /* The first attempt is immediate; each failure doubles the delay. */
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (r.fake.send_calls == 1u);

    t = expect_failed_attempt_at (&r, 0u, 1000u);
    t = expect_failed_attempt_at (&r, t, 2000u);
    t = expect_failed_attempt_at (&r, t, 4000u);
    t = expect_failed_attempt_at (&r, t, 8000u);
    t = expect_failed_attempt_at (&r, t, 16000u);

    /* The delay is capped at 30 s and stays there. */
    delay = MQTT_RECONNECT_MAX_MS;
    t = expect_failed_attempt_at (&r, t, delay);
    t = expect_failed_attempt_at (&r, t, delay);
    t = expect_failed_attempt_at (&r, t, delay);

    /* Recovery. */
    r.fake.fail_send = false;
    CHECK (mqtt_client_poll (&r.client, t + delay) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
}

static void
test_reconnect_backoff_resets_after_suback (void)
{
    rig_t r;
    uint32_t t;

    /* Two failed handshakes push the delay to 4 s. */
    rig_init (&r);
    r.fake.fail_send = true;
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_ERR_TRANSPORT);
    t = expect_failed_attempt_at (&r, 0u, 1000u);
    r.fake.fail_send = false;
    CHECK (mqtt_client_poll (&r.client, t + 1999u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, t + 2000u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    t += 2000u;

    /* A CONNACK alone does not reset the backoff: a SUBACK timeout still
       schedules the next step (4 s). */
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, t) == MQTT_OK);
    CHECK (mqtt_client_poll (&r.client, t + MQTT_REPLY_TIMEOUT_MS)
           == MQTT_ERR_TRANSPORT);
    t += MQTT_REPLY_TIMEOUT_MS;
    CHECK (mqtt_client_poll (&r.client, t + 3999u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, t + 4000u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    t += 4000u;

    /* A full handshake resets the delay to 1 s. */
    fake_feed (&r.fake, CONNACK_OK, sizeof (CONNACK_OK));
    CHECK (mqtt_client_poll (&r.client, t) == MQTT_OK);
    fake_feed (&r.fake, SUBACK_OK, sizeof (SUBACK_OK));
    CHECK (mqtt_client_poll (&r.client, t) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_CONNECTED);

    r.fake.fail_recv = true;
    CHECK (mqtt_client_poll (&r.client, t + 100u) == MQTT_ERR_TRANSPORT);
    r.fake.fail_recv = false;
    CHECK (mqtt_client_poll (&r.client, t + 100u + RECONNECT_MS - 1u)
           == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, t + 100u + RECONNECT_MS) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
}

static void
test_reconnect_backoff_wrap (void)
{
    rig_t r;
    const uint32_t t0 = UINT32_MAX - 1500u; /* 1 s + 2 s steps wrap */

    rig_init (&r);
    r.fake.fail_send = true;
    CHECK (mqtt_client_poll (&r.client, t0) == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_poll (&r.client, t0 + 999u) == MQTT_OK);
    CHECK (r.fake.send_calls == 1u);
    CHECK (mqtt_client_poll (&r.client, t0 + 1000u) == MQTT_ERR_TRANSPORT);
    CHECK (r.fake.send_calls == 2u);
    CHECK (mqtt_client_poll (&r.client, t0 + 2999u) == MQTT_OK); /* wrapped */
    CHECK (r.fake.send_calls == 2u);
    CHECK (mqtt_client_poll (&r.client, t0 + 3000u) == MQTT_ERR_TRANSPORT);
    CHECK (r.fake.send_calls == 3u);
}

static void
test_reconnect_after_bad_connack (void)
{
    rig_t r;
    static const uint8_t bad[5] = { 0x20, 0x03, 0x00, 0x00, 0x00 };

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0u) == MQTT_OK);
    fake_feed (&r.fake, bad, sizeof (bad));
    CHECK (mqtt_client_poll (&r.client, 100u) == MQTT_ERR_PROTOCOL);
    CHECK (mqtt_client_poll (&r.client, 100u + RECONNECT_MS - 1u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, 100u + RECONNECT_MS) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
}

static void
test_reconnect_tick_wrap (void)
{
    rig_t r;
    const uint32_t t_fail = UINT32_MAX - 500u; /* retry time wraps to 499 */

    rig_init (&r);
    rig_handshake_at (&r, 100u, t_fail);
    r.fake.fail_recv = true;
    CHECK (mqtt_client_poll (&r.client, t_fail) == MQTT_ERR_TRANSPORT);
    r.fake.fail_recv = false;

    CHECK (mqtt_client_poll (&r.client, UINT32_MAX) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, 498u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_DISCONNECTED);
    CHECK (mqtt_client_poll (&r.client, 499u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
}

static void
test_first_connect_late_tick (void)
{
    rig_t r;

    /* The first attempt is due at once, whatever the tick is, including a
       first poll at 2^31 ms or later. */
    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, 0x80000000u) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);
    CHECK (r.fake.send_calls == 1u);

    rig_init (&r);
    CHECK (mqtt_client_poll (&r.client, UINT32_MAX) == MQTT_OK);
    CHECK (mqtt_client_state (&r.client) == MQTT_STATE_WAIT_CONNACK);

    /* A failed first attempt at a late tick still backs off 1 s. */
    rig_init (&r);
    r.fake.fail_send = true;
    CHECK (mqtt_client_poll (&r.client, 0xC0000000u) == MQTT_ERR_TRANSPORT);
    CHECK (mqtt_client_poll (&r.client, 0xC0000000u + RECONNECT_MS - 1u)
           == MQTT_OK);
    CHECK (r.fake.send_calls == 1u);
    CHECK (mqtt_client_poll (&r.client, 0xC0000000u + RECONNECT_MS)
           == MQTT_ERR_TRANSPORT);
    CHECK (r.fake.send_calls == 2u);
}

int
main (void)
{
    test_init_validation ();
    test_connect_bytes_default ();
    test_connect_keepalive_config ();
    test_connect_credentials ();
    test_connect_client_id_too_long ();
    test_handshake_bytes ();
    test_handshake_longer_topic ();
    test_handshake_topic_limit ();
    test_handshake_single_burst ();
    test_handshake_chunked ();
    test_handshake_bad_connack ();
    test_handshake_suback_failure_code ();
    test_handshake_connack_timeout ();
    test_handshake_suback_timeout ();
    test_handshake_timeout_tick_wrap ();
    test_publish_before_suback_is_protocol_error ();
    test_publish_qos0_bytes ();
    test_publish_rejects ();
    test_publish_remaining_length_encoding ();
    test_publish_oversize ();
    test_publish_transport_failure ();
    test_publish_retries_without_restarting ();
    test_handshake_waits_for_writable_transport ();
    test_command_retries_puback_after_backpressure ();
    test_cmd_qos1_puback_and_dispatch ();
    test_cmd_duplicate_delivery ();
    test_cmd_id_wrap ();
    test_cmd_first_id_zero ();
    test_cmd_qos0 ();
    test_cmd_other_topic_and_short_payload ();
    test_cmd_qos2_is_protocol_error ();
    test_cmd_chunked_delivery ();
    test_cmd_back_to_back ();
    test_cmd_null_handler ();
    test_cmd_dedup_survives_reconnect ();
    test_cmd_id_after_idle ();
    test_inbound_protocol_errors ();
    test_rx_buffer_overflow ();
    test_keepalive_ping ();
    test_keepalive_custom_interval ();
    test_keepalive_publish_resets_timer ();
    test_keepalive_tick_wrap ();
    test_pingresp_accepted ();
    test_pingresp_timeout ();
    test_pingresp_timeout_tick_wrap ();
    test_ping_send_failure ();
    test_reconnect_after_recv_failure ();
    test_reconnect_connect_send_failure ();
    test_reconnect_backoff_resets_after_suback ();
    test_reconnect_backoff_wrap ();
    test_reconnect_after_bad_connack ();
    test_reconnect_tick_wrap ();
    test_first_connect_late_tick ();

    if (g_failures != 0u)
    {
        (void) printf ("test_mqtt: %u check(s) FAILED\n", g_failures);
        return 1;
    }
    (void) printf ("test_mqtt: all tests passed\n");
    return 0;
}
