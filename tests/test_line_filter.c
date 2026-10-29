#include <assert.h>
#include "line_filter.h"

int main (void)
{
    line_filter_t filter;

    line_filter_init(&filter, false, false);
    assert(!filter.left && !filter.right);

    line_filter_update(&filter, true, false);
    assert(!filter.left);
    line_filter_update(&filter, true, false);
    assert(!filter.left);
    line_filter_update(&filter, true, false);
    assert(filter.left);

    line_filter_update(&filter, false, true);
    assert(filter.left);
    assert(!filter.right);
    line_filter_update(&filter, false, true);
    line_filter_update(&filter, false, true);
    assert(!filter.left);
    assert(filter.right);

    return 0;
}
