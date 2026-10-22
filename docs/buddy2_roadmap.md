# Buddy 2 implementation roadmap

Status after the synthetic guidance bench run,22October2026. These steps separate
working hardware evidence from code that has only been built/tested in software.

| Step | Planned work | Current position |
| --- | --- | --- |
| 1 | Personal branch and reproducible build | Done; completed steps pushed to feature/buddy2-encoder. |
| 2 | Verify encoders, pin mapping, counts and direction | Done on assembled car. |
| 3 | Single-wheel PWM and basic stop | Done; physical direction/stop observed, exact stop latency unmeasured. |
| 4 | Signed speed and displacement sampling | Implemented; forward/reverse observations recorded. |
| 5 | Wheel calibration | Provisional67mm/edge scales and two500mm rolling observations; final accuracy not accepted. |
| 6 | Paired motor output | Six-step wheels-up batch passed. |
| 7 | PID and motor-owner service | First wheels-up PI run completed cleanly; encoder-speed means near50mm/s target. Loaded/full tuning remains. |
| 8 | Distance commands and automatic completion | Optional50cm/right3wheel-revs/10cm demo passed a recorded wheels-up run with an explicit braked baseline. Floor/powerbank behaviour remains unresolved; general navigation API and final accuracy remain deferred. |
| 9 | Angle-based turns | Deferred until needed; measured wheel spacing required. |
| 10 | Line/barcode and obstacle integration | Straight/steer/hold API implemented; synthetic seven-stage wheels-up test passed. Real Buddy3/Buddy5 drivers/freshness and shared IRQ integration pending. |
| 11 | Combined timing, faults and loaded accuracy | All13 host groups pass; physical PI and synthetic guidance checks recorded. Loaded real-sensor tests and timing guarantees remain pending. |
| 12 | Handoff, pushes and lead-reviewed PR | Notes/pushes kept current; PR/review/merge later. |

**Current position:** first step7 trial completed. In the last second, observed
encoder-speed means were50.443mm/s left and49.655mm/s right for50mm/s request;
no reported software faults or invalid deltas, and owner confirmed stopping.
The subsequent step10 synthetic guidance run also passed: all seven
straight/steering/overlapping-hold/resume stages matched, no reported software
faults, and owner confirmed final stop. See `buddy2_guidance_check.md`.
**Priority update:** user confirmed normal movement is sensor-led. Continue
with step10's decoded straight/steering decisions and independent obstacle/
line-lost holds; see `buddy2_sensor_motion.md`. Keep initial gains and rough
wheel scales unchanged. Distance-quality history was invalid before the run;
fix it when implementing step8, without claiming automatic distance ready.
Do not repeat manual wheel-turn/500mm trials for ordinary line following.
Loaded validation, final calibration and physical integration remain open.

Follow the physically tested pin map in `buddy2_pins.h`; encoder left/right
labels differ from the original Week6 table. Keep scale verification false
until accepted calibration evidence exists. No need to repeat all manual
encoder trials before continuing software work.

Default team main is unchanged. Completed, checked commits are pushed to the
personal branch. Leads review/merge through a PR when the contribution is ready.
