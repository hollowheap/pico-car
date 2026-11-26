# motion test records

### 2026-11-26 — motion — Guided bypass tape limit

- Build: 3c299b5+dirty
- Kind: host test
- Setup: hug bypass legs with scripted sonar and encoders
- Steps: `ctest --test-dir build-tests -L '^motion$' --output-on-failure`
- Expected: a sideways leg that reaches the tape limit without seeing the post stops with no exit
- Observed: the test drove the limit for the old 130 mm sensor lead. It now drives the firmware limit for the 124 mm lead (147 mm). All 79 motion cases pass.
- Result: pass
- Evidence: CTest output
- Follow-up: n/a
