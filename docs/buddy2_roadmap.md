# Buddy 2 implementation roadmap

Status before the first PI bench run,22October2026. These steps separate
working hardware evidence from code that has only been built/tested in software.

| Step | Planned work | Current position |
| --- | --- | --- |
| 1 | Personal branch and reproducible build | Done; completed steps pushed to feature/buddy2-encoder. |
| 2 | Verify encoders, pin mapping, counts and direction | Done on assembled car. |
| 3 | Single-wheel PWM and basic stop | Done; physical direction/stop observed, exact stop latency unmeasured. |
| 4 | Signed speed and displacement sampling | Implemented; forward/reverse observations recorded. |
| 5 | Wheel calibration | Provisional67mm/edge scales and two500mm rolling observations; final accuracy not accepted. |
| 6 | Paired motor output | Six-step wheels-up batch passed. |
| 7 | PID and motor-owner service | Software connected/tested; bounded PI tuning target built, physical trial next. |
| 8 | Distance commands and automatic completion | Pending. |
| 9 | Angle-based turns | Pending; requires measured wheel spacing. |
| 10 | Line/barcode and obstacle integration | Proposed speed/stop/status API exists; teammate interfaces/arbitration and shared IRQ integration pending. |
| 11 | Combined timing, faults and loaded accuracy | Host policy checks exist; integrated physical tests pending. |
| 12 | Handoff, pushes and lead-reviewed PR | Notes/pushes kept current; PR/review/merge later. |

**Current action:** step7, `buddy2_speed_check`. One GP20 start requests50mm/s
on lifted wheels, for nominally3seconds with20% duty cap and experimental PI
gains. See `buddy2_speed_check.md` for exact settings and limits. This is not
completed physical tuning or a final speed choice.

Follow the physically tested pin map in `buddy2_pins.h`; encoder left/right
labels differ from the original Week6 table. Keep scale verification false
until accepted calibration evidence exists. No need to repeat all manual
encoder trials before continuing software work.

Default team main is unchanged. Completed, checked commits are pushed to the
personal branch. Leads review/merge through a PR when the contribution is ready.
