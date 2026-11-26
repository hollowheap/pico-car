# Team responsibilities

`app_program/app_main.c` assembles the obstacle-extras firmware. Each
subsystem owns its lifecycle.

| Subsystem | Owner | Owns | Public header |
| --- | --- | --- | --- |
| Comms | Buddy 1, Yucky Hito | Wi-Fi, MQTT session, telemetry and command ingress | [comms.h](app_program/comms/comms.h) |
| Motion | Buddy 2, Neo Tock Peng | Motors, encoders, wheel control and pilot | [motion.h](app_program/motion/motion.h) |
| IR | Buddy 3, Kiyonn Chan | Line sensors, following and barcode actions | [ir.h](app_program/ir/ir.h) |
| IMU | Buddy 4, Beak Wei Xiang | Attitude, lift and hump detection | [imu.h](app_program/imu/imu.h) |
| Obstacle | Buddy 5, Chin Qing Rong | Sonar, servo scan and clearance evidence | [obstacle.h](app_program/obstacle/obstacle.h) |

Shared code belongs to the whole team.

| Shared code | Owns | Public header |
| --- | --- | --- |
| Supervisor | Run state, motion permits and startup admission | [supervisor.h](app_program/supervisor/supervisor.h) |
| Platform | Chassis dimensions, startup barrier,<br>GPIO interrupts, watchdog and diagnostics | [platform.h](app_program/platform/platform.h) |

## Interfaces

The calls each subsystem makes into another one:

| From | To | Interface |
| --- | --- | --- |
| IR | Motion | Reads wheel feedback (`motion_feedback_snapshot`).<br>Sets wheel speeds (`motion_set_speed`, `motion_set_obstacle_speed`). |
| IR | Supervisor | Takes queued turns (`supervisor_poll_turn`) and the armed start.<br>Posts barcode commands (`vehicle_command_post`). |
| IR | Obstacle | Reads the latest forward ping (`obstacle_pack_ping_*`). |
| IR | IMU | Reads attitude and hump state (`physics_snapshot`). |
| IR | Comms | Emits barcode samples and events (`telemetry_try_emit_*`).<br>Reads the dashboard barcode map (`telemetry_barcode_binding`). |
| IMU | Motion | Reads wheel speed and travel (`motion_feedback_snapshot`). |
| Obstacle | Motion | Reads wheel travel for the scan and bypass (`motion_feedback_snapshot`). |
| Motion | Obstacle | The extras navigator reads the scan profile, events and faults (`obstacle_extras_*`). |
| Motion | IR | The pilot follows the line through the line-control API (`line_follow_*`). |
| Motion | Comms | Emits motion events (`telemetry_try_emit_event`). |
| Supervisor | Motion | Renews the motion permit (`motion_renew_permit`) and reads motion state. |
| Supervisor | IMU | Reads attitude, starts a new run and reports free wheels (`physics_*`). |
| Supervisor | Obstacle | Checks sonar health at start (`obstacle_echo_counts`, `obstacle_pack_ping_*`). |
| Supervisor | Comms | Acknowledges commands (`telemetry_post_cmd_ack`) and emits events. |
| Comms | All | Assembles the 5 Hz status packet from every subsystem's snapshot. |
| Comms | Supervisor | `app_main.c` passes each dashboard command to `vehicle_command_post_ids`.<br>E-stop also calls `motion_request_stop`. |

## GPIO assignments

| GPIO | Use after application setup | Owner |
| --- | --- | --- |
| GP0, GP1 | Sonar trigger and echo | Obstacle |
| GP2, GP3 | IMU I²C1 SDA and SCL | IMU |
| GP4, GP5 | Left encoder A and B | Motion |
| GP6, GP7 | Right encoder A and B | Motion |
| GP8, GP9 | Left motor inputs | Motion |
| GP10, GP11 | Right motor inputs | Motion |
| GP12 | Sonar servo PWM | Obstacle |
| GP16 | External LED output at reset | Kernel |
| GP20 | Active-low START, scanner bench menu | Supervisor or bench |
| GP21 | Active-low emergency stop, scanner bench send | Motion or bench |
| GP23–25, GP29 | On-board CYW43 radio signals | Pico W and kernel |
| GP26 | Barcode input | IR |
| GP27, GP28 | Left and right line inputs | IR |

The sonar claims GP0 and GP1, so the template's `CRASH_REPORT=1` option
cannot be used. That option reserves both pins for its UART.

Move right encoder A from GP16 to GP6 and B from GP17 to GP7 on the Robo Pico
breakout header. Keep its existing power and ground connections. Both images
use this wiring. The upstream kernel drives GP16 as its external LED at reset.
Verify the encoder sign by turning the raised wheel forward before a drive test.

## Car dimensions

[chassis.h](app_program/platform/chassis.h) defines the measured dimensions
used by the firmware. Distances along the car are from the drive axle.
The code rounds half-millimetre measurements outward.

| Dimension | Value | Macro |
|---|---|---|
| Wheel track, centre to centre | 107 mm | `CHASSIS_WHEEL_TRACK_MM` |
| Outer width, tyre face to tyre face | 126 mm | `CHASSIS_OUTER_WIDTH_MM` |
| Wheel diameter | 65 mm | `CHASSIS_WHEEL_TRAVEL_UM` |
| Encoder edges per wheel turn | 2530 | `CHASSIS_EDGES_PER_REV` |
| Axle to caster contact | 120 mm | `CHASSIS_AXLE_TO_CASTER_MM` |
| Axle to line sensors | 123.5 mm (124 in code) | `CHASSIS_LINE_SENSOR_LEAD_MM` |
| Line sensor spacing | 25 mm | `CHASSIS_LINE_SENSOR_SPACING_MM` |
| Axle to barcode reader corner | 45 mm | `CHASSIS_BARCODE_FORWARD_MM` |
| Barcode corner beyond left tyre | 44 mm | `CHASSIS_LEFT_REACH_MM` |
| Axle to sonar shaft | 68.5 mm (69 in code) | `CHASSIS_SONAR_FORWARD_MM` |
| Axle to nose / IR tip | 110 / 133 mm | `CHASSIS_NOSE_FORWARD_MM`<br>`CHASSIS_FRONT_TIP_MM` |
| Nose width / IR width | 90 / 45 mm | `CHASSIS_NOSE_WIDTH_MM`<br>`CHASSIS_IR_WIDTH_MM` |
| Body envelope radius | 119 mm | `CHASSIS_BODY_RADIUS_MM` |
| Axle to tail (nominal) | 100 mm | `CHASSIS_TAIL_MM` |
| Effective track for spins on the spot (not measured) | 80.25 mm | `CHASSIS_PIVOT_TRACK_UM` |
| IMU above / behind axle | 80 / 15 mm | `CHASSIS_IMU_ABOVE_AXLE_MM`<br>`CHASSIS_IMU_BEHIND_AXLE_MM` |

## Firmware tasks

| Task in `main.uf2` | Priority | Affinity with SMP | Stack bytes |
| --- | ---: | --- | ---: |
| Startup | 1 | Core 0 | 2048 |
| Motion | 1 | Core 0 | 2560 |
| Supervisor | 2 | Core 0 | 2560 |
| Line and extras navigation | 3 | Core 0 | 2560 |
| Obstacle | 4 | Core 0 | 4096 |
| CYW43 and lwIP | 4 | Core 0 | Kernel configuration |
| Physics | 5 | Core 0 | 2816 |
| MQTT connection | 6 | Core 1 | 2048 |
| Barcode | 7 | Unpinned | 2304 |
| Telemetry | 7 | Unpinned | 2816 |

Host tests are arranged by subsystem. The shared bench sources in
`bench/` build the existing `car_bench`, `wdt_check` and
`physics_check` images. See [testing](testing/README.md) for use.
