# Buddy 2 implementation roadmap

Status after the first PI bench run,22October2026. These steps separate
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
| 8 | Distance commands and automatic completion | Next software stage; first fix explicit segment baseline/idle-quality diagnostics. |
| 9 | Angle-based turns | Pending; requires measured wheel spacing. |
| 10 | Line/barcode and obstacle integration | Proposed speed/stop/status API exists; teammate interfaces/arbitration and shared IRQ integration pending. |
| 11 | Combined timing, faults and loaded accuracy | Host policy checks exist; integrated physical tests pending. |
| 12 | Handoff, pushes and lead-reviewed PR | Notes/pushes kept current; PR/review/merge later. |

**Current position:** first step7 trial completed. In the last second, observed
encoder-speed means were50.443mm/s left and49.655mm/s right for50mm/s request;
no reported software faults or invalid deltas, and owner confirmed stopping.
Distance-quality history was already invalid before the active run and must
be handled before step8 automatic travel. Keep initial gains unchanged for
now. See `buddy2_speed_check.md` for evidence and limits; loaded validation
and final calibration remain open.

Follow the physically tested pin map in `buddy2_pins.h`; encoder left/right
labels differ from the original Week6 table. Keep scale verification false
until accepted calibration evidence exists. No need to repeat all manual
encoder trials before continuing software work.

Default team main is unchanged. Completed, checked commits are pushed to the
personal branch. Leads review/merge through a PR when the contribution is ready.
