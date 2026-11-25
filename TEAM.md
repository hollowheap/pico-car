# Team responsibilities

`app_program/app_main.c` assembles the firmware through `app_program/app.h`.
Each subsystem owns its lifecycle and exposes the public header in the table.

| Subsystem | Owns | Public header |
| --- | --- | --- |
| Comms | Wi-Fi, MQTT session, telemetry and command ingress | [comms.h](app_program/comms/comms.h) |
| Motion | Motors, encoders, wheel control and pilot | [motion.h](app_program/motion/motion.h) |
| IR | Line sensors, following and barcode actions | [ir.h](app_program/ir/ir.h) |
| IMU | Attitude, lift and hump detection | [imu.h](app_program/imu/imu.h) |
| Obstacle | Sonar, servo scan and clearance evidence | [obstacle.h](app_program/obstacle/obstacle.h) |
| Supervisor | Run state, motion permits and startup admission | [supervisor.h](app_program/supervisor/supervisor.h) |
| Platform | Startup barrier, GPIO interrupts, watchdog and system diagnostics | [platform.h](app_program/platform/platform.h) |

## GPIO assignments

| GPIO | Use after application setup | Owner |
| --- | --- | --- |
| GP0, GP1 | Sonar trigger and echo | Obstacle |
| GP2, GP3 | IMU I²C1 SDA and SCL | IMU |
| GP4, GP5 | Left encoder A and B | Motion |
| GP8, GP9 | Left motor inputs | Motion |
| GP10, GP11 | Right motor inputs | Motion |
| GP12 | Sonar servo PWM | Obstacle |
| GP16, GP17 | Right encoder A and B | Motion |
| GP20 | Active-low START input | Platform and supervisor |
| GP21 | Active-low emergency stop | Motion |
| GP23–25, GP29 | On-board CYW43 radio signals | Pico W and kernel |
| GP26 | Barcode input | IR |
| GP27, GP28 | Left and right line inputs | IR |

The upstream kernel boot table assigns UART0 to GP0/GP1 and drives GP16 as
its external LED before the application configures those pins. Check the
kernel boot table and the Pico W radio assignments when proposing pin changes.

## Firmware tasks

| Task | Priority | Affinity with SMP | Stack bytes |
| --- | ---: | --- | ---: |
| Startup | 1 | Core 0 | 2048 |
| Motion | 1 | Core 0 | 2560 |
| Supervisor | 2 | Core 0 | 2560 |
| Line | 3 | Core 0 | 2560 |
| Obstacle | 4 | Core 0 | 2048 |
| CYW43 and lwIP | 4 | Core 0 | Kernel configuration |
| IMU | 5 | Core 0 | 2816 |
| Connection | 6 | Core 1 | 2048 |
| Telemetry | 7 | Unpinned | 2816 |
| Barcode | 7 | Unpinned | 2304 |

Host tests are arranged by subsystem. The shared bench sources in
`app_program/bench/` build the existing `car_bench`, `wdt_check` and
`physics_check` images. See [testing](testing/README.md) for use.
