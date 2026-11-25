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

Beak Wei Xiang contributed the IMU work, TemariVirus the barcode and line
work, and unknowntitan12 the motion work.

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

## Car dimensions

[chassis.h](app_program/platform/chassis.h) defines the measured dimensions
used by the firmware. Distances along the car are from the drive axle.

| Dimension | Value | Macro |
|---|---|---|
| Wheel track, centre to centre | 107 mm | `CHASSIS_WHEEL_TRACK_MM` |
| Outer width, tyre face to tyre face | 126 mm | `CHASSIS_OUTER_WIDTH_MM` |
| Wheel diameter | 65 mm | `CHASSIS_WHEEL_TRAVEL_UM` |
| Encoder edges per wheel turn | 2530 | `CHASSIS_EDGES_PER_REV` |
| Axle to caster contact | 120 mm | `CHASSIS_AXLE_TO_CASTER_MM` |
| Axle to line sensors | 130 mm | `CHASSIS_LINE_SENSOR_LEAD_MM` |
| Line sensor spacing | 25 mm | `CHASSIS_LINE_SENSOR_SPACING_MM` |
| Axle to barcode reader corner | 45 mm | `CHASSIS_BARCODE_FORWARD_MM` |
| Barcode corner beyond left tyre | 44 mm | `CHASSIS_LEFT_REACH_MM` |
| Axle to sonar | 70 mm | `CHASSIS_SONAR_FORWARD_MM` |
| Axle to nose / IR tip | 110 / 130 mm | `CHASSIS_NOSE_FORWARD_MM`<br>`CHASSIS_FRONT_TIP_MM` |
| Nose width / IR width | 90 / 45 mm | `CHASSIS_NOSE_WIDTH_MM`<br>`CHASSIS_IR_WIDTH_MM` |
| Body envelope radius | 119 mm | `CHASSIS_BODY_RADIUS_MM` |
| IMU above / behind axle | 80 / 15 mm | `CHASSIS_IMU_ABOVE_AXLE_MM`<br>`CHASSIS_IMU_BEHIND_AXLE_MM` |

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
