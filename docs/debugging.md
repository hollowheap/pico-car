# Debugging and log output

Every task writes its console output the same way. This page explains the format and how to add a log line.

## The format

```
[<SEVERITY>](<task>) message
[<SEVERITY>](<task>:<module>) message
```

Examples:

```
[INFO](barcode) read L
[ERROR](boot) startup failed, er=-33
[ERROR](motion) fault=NO_EDGES, stopped
[WARN](connection:tcp) attempt failed, state=3
[INFO](line:nav) plan: obstacle 205 mm, start 60 deg, side offset 190 mm, box (out and back)
[DATA](physics:imu) B4,123456,12,-8,1002,...
```

## Severity

| Severity | Use it when |
|---|---|
| `ERROR` | A feature failed or the car stopped. |
| `WARN` | The car runs with reduced function, or a bad input was rejected. |
| `INFO` | A state change or a finished action. Once per event. |
| `DEBUG` | Per-pass detail for a bring-up session. |
| `DATA` | A data row for a tool, for example the physics CSV. Only printed with `LOG_NOW`. |

## Task and module

The task is the kernel task that runs the code: `boot`, `motion`, `supervisor`, `line`, `obstacle`, `physics`, `barcode`, `telemetry`, `connection`, `log`.

The module is optional. It names the code owner when that is not the whole task. Obstacle navigation runs inside the line task, so its lines show `(line:nav)`. Modules today: `nav`, `scan`, `tcp`, `mqtt`, `imu`, `hump`, `pid`.

## How to log from code

Include `platform/log.h`.

| Macro | When |
|---|---|
| `LOG_ERROR`, `LOG_WARN`, `LOG_INFO`, `LOG_DEBUG` | Always, in real-time tasks and for every error. The call stores a record and returns. |
| `LOG_NOW` | Only where the caller paces its own output (the flight log dump, the bench builds) or prints many values. It prints at once. |
| `LOG_CONT` | Continues a `LOG_NOW` line, for example a table row. It adds no prefix. |

```c
LOG_INFO (LOG_TASK_LINE, LOG_MOD_NONE, "turn done in %d ms", (int)elapsed_ms);
LOG_ERROR (LOG_TASK_BOOT, LOG_MOD_NONE, "startup failed, er=%d", (int)result);
LOG_NOW ("INFO", "line:nav", "state %d cause %d\n", (int)state, (int)cause);
```

Rules:

1. Do not call `tm_printf`, `tm_putstring` or `tm_sprintf`. `python3 artifacts/scripts/check_firmware_policy.py . logging` fails if you do. The bench builds (`bench_main.c`, `physics_check.c`, `demo_tasks.c`) are the exception.
2. Do not log from an interrupt handler. Count in the handler. Log the count from a task.
3. The format is a literal. The ring keeps its address.
4. Give at most 8 arguments (`LOG_MAX_ARGS`). Cast each to `(int)`. Use only `%d %i %u %x %X %c %s`. No floats.
5. A `%s` argument is a string literal, or text that lives for ever.
6. Keep a message under 100 characters after formatting. The line buffer holds 160 characters and the formatter does not check its bounds.
7. Name every number: `turn left done in 910 ms`, not `910`.

## How the output reaches you

1. `LOG_*` copies one record into a ring of 64 records. This takes a few microseconds and never waits for the console.
2. The telemetry task prints up to 4 records every 50 ms while a serial monitor is open and its buffer has room (`log_drain`).
3. With no monitor open, the records wait in the ring. Open the monitor and they print, oldest first.
4. When the ring is full, an `ERROR` or `WARN` replaces the oldest record. Other records are dropped and counted (`log_dropped()`).
5. Identical records in a row print once. Then the log prints `[INFO](log) previous message repeated N times`.
6. At boot and on a startup failure the code calls `log_flush()`, because the telemetry task may never run.

If the monitor opens late, only the last 64 records are there. Read the obstacle flight log for a longer history of a bypass: keep the car on, plug in USB and open the monitor. It prints as `[DATA](line:nav)` rows.

## Reading a physics capture

`tools/physics_log_summary.py LOGFILE` reads the capture of `physics_check` or the main firmware. It drops the `[DATA](physics:imu) ` prefix and uses the `B4,` rows.

## Adding a new task or module name

1. Add the value to `log_task_t` or `log_mod_t` in `include/platform/log.h`.
2. Add the name to the matching table in `src/platform/log_core.c`. A compile-time check fails if the counts differ.
3. Add the name to this page.
