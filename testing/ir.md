# ir test records

### 2026-11-26 — ir — Square-left exit with a stripe-like graze

- Build: 3c299b5+dirty
- Kind: host test
- Setup: line simulator, square left corner, 124 mm sensor lead, wheel profiles `matched`, `nov16` and `nov20`
- Steps: `ctest --test-dir build-tests -R 'square_left' --output-on-failure`
- Expected: the car leaves the corner without stopping a wheel in the last 250 mm of the exit
- Observed: before the fix, six cases failed. A 10 mm graze on the exit straight counted as a side stripe. Its search started after 131 mm, 31 mm into the shallow probe, and the probe needed about 33 mm. The probe now also starts two sensor spacings before a stripe's search point. All square-left cases pass.
- Result: pass
- Evidence: CTest output
- Follow-up: n/a

### 2026-11-26 — ir — Course stretch from the recorded follow runs

- Build: n/a (analysis of an earlier run)
- Kind: behaviour run
- Setup: course runs recorded by the bridge on 26 Nov in `testing/logs/bridge/history.jsonl`; all bends are 90 degrees
- Steps: rebuilt the axle path from wheel travel with the 107 mm track; fitted the straights around each bend
- Expected: two 90-degree bends about 0.5 m apart; a bend radius close to the simulator's 150 mm
- Observed: straights 470–540 mm; one 1800 mm drive ran 539 mm, bend, 465 mm, bend, 236 mm. Net yaw over whole drives was 1.003 of the true 90 and 180 degrees. Tape bend radius 160, 165, 173 and 206 mm (median about 169 mm). The left wheel realised about 1.07 times the right, the same as the `nov20` profile. Wheels did not move below a 20 mm/s target.
- Result: pass
- Evidence: local bridge recording `testing/logs/bridge/history.jsonl` (not committed)
- Follow-up: consider a 170 mm round bend in the simulator

### 2026-11-26 — ir — Recorded reverse Z barcode

- Build: 3c299b5+dirty
- Kind: host test
- Setup: element widths captured on the car while reading `*Z*` backwards
- Steps: `ctest --test-dir build-tests -R '^ir[.]barcode' --output-on-failure`
- Expected: the recorded widths decode to Z
- Observed: the reader shrinks bars and widens spaces by about 2 mm. Ranking bars and spaces apart within each character decodes Z. The old shared narrow width rejected it. A gap between characters must stay under twice the narrow space; the recorded gaps read about 1.6 times.
- Result: pass
- Evidence: CTest output, `ir.barcode_parser.default`
- Follow-up: re-read the Z marker on the car
