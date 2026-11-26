# obstacle test records

### 2026-11-26 — obstacle — Faster bypass spins

- Build: 3c299b5+dirty
- Kind: host test
- Setup: extras scan and navigation replays
- Steps: `ctest --test-dir build-tests --no-tests=error -L '^obstacle$' --output-on-failure`
- Expected: replays keep their outcomes with spins at 140 mm/s (slowing to 46.7 mm/s for the last 20 degrees) and the chicken-head scan at 70 mm/s
- Observed: all 28 obstacle cases pass. In the recorded runs, the wheel travel of a commanded 90-degree bypass spin gives about 83 degrees with the 80.25 mm pivot track and about 62 degrees with the 107 mm wheel track. The true turn was not measured.
- Result: partial
- Evidence: CTest output, local bridge recording `testing/logs/bridge/history.jsonl` (not committed)
- Follow-up: measure the spin overshoot and the pivot track on the car
