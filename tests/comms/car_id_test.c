/* Host test for the car fingerprint and the MQTT client ID text. */
#include <stdio.h>
#include <string.h>

#include "comms/telemetry/car_id.h"

static unsigned g_failures;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static void test_from_mac (void)
{
    const uint8_t mac[CAR_ID_MAC_BYTES] = {0x28U, 0xCDU, 0xC1U, 0x0AU, 0xBCU, 0xF0U};
    const uint8_t none[CAR_ID_MAC_BYTES] = {0U};

    CHECK(car_id_from_mac(mac) == 0xC10ABCF0U);
    CHECK(car_id_from_mac(none) == 0U);
}

static void test_client_id (void)
{
    char text[32];

    CHECK(car_id_client_id(text, sizeof(text), "pico-car", 0xC10ABCF0U) == 17U);
    CHECK(strcmp(text, "pico-car-c10abcf0") == 0);
    CHECK(car_id_client_id(text, sizeof(text), "x", 0x00000001U) == 10U);
    CHECK(strcmp(text, "x-00000001") == 0);
    CHECK(car_id_client_id(text, sizeof(text), "", 0xFFFFFFFFU) == 9U);
    CHECK(strcmp(text, "-ffffffff") == 0);
}

static void test_client_id_bounds (void)
{
    char text[18];

    (void)memset(text, 'z', sizeof(text));
    CHECK(car_id_client_id(text, 17U, "pico-car", 1U) == 0U);
    CHECK(text[17] == 'z');
    CHECK(car_id_client_id(text, 18U, "pico-car", 1U) == 17U);
    CHECK(strcmp(text, "pico-car-00000001") == 0);
}

int main (void)
{
    test_from_mac();
    test_client_id();
    test_client_id_bounds();
    if (g_failures != 0U)
    {
        (void)printf("test_car_id: %u failure(s)\n", g_failures);
        return 1;
    }
    (void)printf("test_car_id: all checks passed\n");
    return 0;
}
