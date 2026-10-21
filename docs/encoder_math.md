# Buddy 2 encoder calculations

This module converts encoder counts into distance and interval speed. It is
compiled with the firmware and tested on the host. The current demo does not
call it, and this change does not configure encoder pins, start a control task
or drive the motors.

## Inputs and units

Measure wheel travel per full wheel revolution in micrometres, and count the
edges captured during that same revolution. Use the same rising/falling or
quadrature counting mode during calibration and operation. Left and right
wheels can use different calibrations. No kit-specific count or wheel size is
assumed.

| Value | Unit and meaning |
| --- | --- |
| `count` | Nonnegative edge count or interval count difference |
| `wheel_travel_um` | Measured travel of the wheel per full revolution, in micrometres |
| `edges_per_revolution` | Captured edges per wheel revolution for the selected capture mode |
| `elapsed_us` | Interval duration in microseconds |
| Distance result | Whole micrometres, rounded down once |
| Speed result | Whole micrometres per second, rounded down once |

1000 micrometres equal 1 millimetre. The speed function retains fractional
distance until its final division; it does not calculate speed from an already
rounded distance. Test numbers are synthetic fixtures, not measured car limits.

## Validity and numerical limits

The calculation API accepts full-range unsigned 32-bit inputs. Calibration and
elapsed time must be nonzero. Outputs range from 0 to `UINT32_MAX`; a larger
result is rejected, and the caller's output remains unchanged. Products of two
32-bit inputs need 64-bit intermediates. The additional speed scale is applied
without forming an overflowing product. No floating point or 128-bit integer
support is required.

These arithmetic limits do not set the car's operating speed, encoder sampling
window, or final shared-message widths. The minimum-bit application schema still
needs agreed physical ranges, resolution, validity and overflow rules before
motion snapshots or telemetry are added. This module provides no packed shared
storage and makes no claim of RAM savings.

`encoder_math_count_delta()` handles modulo-2^32 rollover of an increasing edge
counter. Do not pass a signed quadrature position that can decrease. The actual
increment between samples must be less than one full counter cycle. The same restriction
applies if unsigned subtraction is used for a wrapping microsecond timestamp.
Sampling deadlines and maximum edge rates must be checked during integration.

For odometry, convert the total count or preserve a fractional remainder. Do not
sum independently rounded interval distances and assume there is no drift.

## Capture and integration still required

- Confirm the actual encoder output and wiring before assigning GPIO pins.
- Measure each wheel's calibration with the intended edge-counting mode.
- Capture count and time samples coherently using the SMP kernel's access rules.
  The pure calculation functions do not make the sampling code synchronized.
- Establish measured direction or explicitly mark commanded-direction inference
  when using a single-channel encoder. Current results are magnitudes only.
- Define stale-data and sensor-fault detection. A valid zero sampled speed is
  not proof that the car has physically stopped or that the encoder is healthy.
- Measure task timing on the board before using the conversions in the motion
  loop. Do not run these conversions in the edge ISR.

## Host verification

From the repository root in a Linux/WSL terminal:

```sh
cmake -S tests -B ~/.codex/tmp/pico-car-host-tests -G Ninja
cmake --build ~/.codex/tmp/pico-car-host-tests
ctest --test-dir ~/.codex/tmp/pico-car-host-tests --output-on-failure
```

The C executable checks 23 known-answer and failure-contract cases. The Python
reference test checks distance, speed and rollover against arbitrary-precision
arithmetic for 5000 deterministic cases, including invalid inputs and boundary
values. These are host calculation tests, not GPIO, timing or physical accuracy
measurements.
