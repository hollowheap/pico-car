# imu test records

### 2026-11-26 — imu — Lift, set-down and overturn events

- Build: 3c299b5+dirty
- Kind: host test
- Setup: physics task with scripted accelerometer and wheel readings
- Steps: `ctest --test-dir build-tests -L '^imu$' --output-on-failure`
- Expected: a 36.9-degree carry at 1.2 g raises LIFT_START, then LIFT_END, then OVERTURN
- Observed: an uncommitted change counted tilt only within 0.12 g of rest, so the carry raised no lift. The committed code counted tilt only while the car was supported. Tilt now counts in either case. All 19 imu cases pass.
- Result: pass
- Evidence: CTest output
- Follow-up: pick up and tilt the car on the bench with `physics_check.uf2`
