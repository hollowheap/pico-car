/** @file obstacle_extras_bench.h
 * @brief Select and start the obstacle scanner bench.
 */
#ifndef OBSTACLE_EXTRAS_BENCH_H
#define OBSTACLE_EXTRAS_BENCH_H

#include <stdbool.h>
#include <tk/tkernel.h>

bool extras_bench_requested (void);
void extras_bench_start (void);
void extras_bench_run (INT stacd, void * exinf);

#endif /* OBSTACLE_EXTRAS_BENCH_H */
