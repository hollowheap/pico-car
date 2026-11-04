#include <stdio.h>

#include "track/barcode_filter.h"

static unsigned int failures;

static void check (int condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

int main (void)
{
    barcode_filter_t filter;

    check(barcode_filter_init(&filter, BARCODE_REARM_UM), "init");
    check(barcode_filter_is_armed(&filter), "initially armed");

    barcode_filter_note_marker(&filter, 1000U, 0);
    check(!barcode_filter_is_armed(&filter), "disarmed after marker");
    check(barcode_filter_suppresses_line_lost(&filter, 1000U), "suppressed now");
    check(barcode_filter_suppresses_line_lost(&filter, 250999U), "suppressed later");
    check(!barcode_filter_suppresses_line_lost(&filter, 251000U), "suppression expired");

    barcode_filter_update(&filter, 99999);
    check(!barcode_filter_is_armed(&filter), "not rearmed before 100 mm");
    barcode_filter_update(&filter, 100000);
    check(barcode_filter_is_armed(&filter), "rearmed at 100 mm");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("barcode filter OK\n");
    return 0;
}
