# Buddy 2 first closed-loop PI bench experiment

`buddy2_speed_check` is an opt-in, single-start wheels-up experiment. It links
the real motor-owner service into a standalone firmware target; team `main`
is unchanged. It exercises a producer on core1 requesting speeds from the
core0 owner. First wheels-up observations are recorded below; it is not a
tuned/validated loaded-driving program.

```sh
cmake --build <configured-build-directory> --target buddy2_speed_check
```

## What this run does

Support both wheels completely clear of the surface. USB is the current
bench supply. Keep fingers clear and do not use carrier M1/M2 motor buttons.
After `READY speed trial service=1`, release GP20 then press it once. Both
wheels request forward speed for nominally3seconds; afterward the program
requests a terminal stop and will not run again until reboot. GP21 can stop
the run early. Holding GP20 at boot does not start it.

| Experimental setting | Value |
| --- | --- |
| Requested speed, both wheels | 50000um/s = 50mm/s |
| Maximum duty | 200permille = 20% |
| P / I / D, Q16 integers | 64 / 256 / 0 |
| Planned run duration | 3000000us |
| Command TTL | 200000us, refreshed by the producer |
| Active owner/producer gap limit | 100000us |
| No-edge timeout while commanded PWM is nonzero | 2000000us |
| Maximum sample age for PID | 60000us |
| Task delay requests | 10ms; actual periods must be observed |

These are explicit starting experiment settings, not measured motor gains,
maximum operating speeds or project stop requirements. P initially requests
about49permille with zero measured speed. I is allowed to raise duty toward
the20% cap while error persists; the cap and gains never automatically rise.
D is disabled. Slow startup or oscillation is possible; the purpose is to
measure that response before choosing the next adjustment.

Both scales remain provisional (67mm nominal tyres, left2541/right2530 x4
edges/revolution, opposite verified signs). The diagnostic explicitly allows
unverified scales; it continues to report them unverified. A reported wheel
speed is not independently measured ground speed, slip or loaded accuracy.

The start acknowledgment is printed before the first command. No serial
printing happens during commanded motion. Up to192 rows are retained in RAM,
at least20ms apart. After terminal output is acknowledged, the frozen trace
is printed with pacing so the host can drain the USB ring, then repeated.
The host should be recording before GP20 and must recover a complete trace.

## Reading the trace

Each DATA row gives:

`sample_us dt_us left_um_s right_um_s left_permille right_permille flags bad_left bad_right`

`dt_us` is the sampler's actual interval, not the producer's requested10ms
delay. Duty is the owner's latest requested output, not measured voltage.
Flags bits0/1 are L/R speed-valid,2/3 odometry-valid,4/5 scale-verified. Normal
provisional rows can therefore have flags15: both quality flags true, both
scale flags false. Bad counts are invalid transition deltas for that sample.
Rows are observer snapshots, not every owner iteration. Peaks or gaps between
observations may be absent; this is not a worst-case scheduling measurement.

The final state normally becomes STOPPED (enum3), fault0, with interrupted0
and gapfault0. A fault or GP21 interruption can finish earlier. A successful
software stop is not, by itself, successful speed tuning. Inspect whether
both wheels started, time to settle, late-run mean/error/ripple, duty
saturation, invalids, faults and missing trace rows. Do not claim stability or
accuracy from the target number or one short trial. A low-duty failure to
start is not enough to diagnose bad wiring or to justify automatically
increasing the duty cap.

`stop_ack_wait_us` is producer-request-to-observed-terminal-status time. It
includes polling/scheduling and is not GP21-edge-to-stop latency or time to
physical rest. Owner/producer task/CPU failure is outside the software time
bounds; there is no new hardware watchdog. The trace waits for owner stop
acknowledgment before printing. Existing driver/carrier-button limitations
still apply.

## Verification status

Ten host test groups pass. New policy tests cover intentional start, cap and
configuration validity,3second duration, clock rollover, stop priority,
producer-gap failure and no restart. The target links against the pinned
kernel/service with application warnings treated as errors. This verifies
software build/policy, not physical PI tracking or cross-core runtime.

Before loading, obtain a fresh wheels-lifted/BOOTSEL confirmation. After the
first trace, preserve its settings and raw evidence, then decide the next
change from the measured response. Do not repeatedly flash arbitrary gains.

## First physical PI trial,22October2026

Source107f5e1, USB supply, both wheels lifted. Owner pressed GP20 once and
confirmed both wheels moved and stopped. The continuous reader captured the
start acknowledgment and one complete96-row trace, with0 buffer drops or
incomplete reports; it then closed normally. The run reached STOPPED with
fault/init/API/gap/brake-error fields0 and no interruption.

Last-second statistics below are means/ranges of33 observed interval-speed
rows per wheel, not independently measured physical speed or full-path travel:

| Measurement | Left | Right |
| --- | ---: | ---: |
| Requested speed | 50mm/s | 50mm/s |
| Observed last-second mean | 50.443mm/s | 49.655mm/s |
| Observed last-second range | 45.539..53.846mm/s | 45.739..54.072mm/s |
| Mean requested duty in that window | 9.161% | 7.685% |
| Samples at20% cap in that window | 0 | 0 |
| First observed nonzero speed after command start | 189837us | 29838us |

Every observed speed-valid flag was true and every recorded invalid delta
was0. The gains remain unchanged: this first response supports continuing
with the initial PI values for the next bench stage, not claiming completed
tuning, ground-speed accuracy or loaded straightness. Sampling/trace
decimation can omit intermediate extrema. Largest observed owner interval
was20021us, producer gap20015us and sample age9991us. Stop request to observed
terminal status was19970us; that is not physical-stop or GP21 latency.

**Distance-quality issue remains open:** all observed rows had both odometry
valid flags false, including the first row, while speed remained valid. The
sampler latches distance quality bad after an invalid capture interval or an
excessive interval and restores it only with an explicit new baseline. Since
active feedback failure would have faulted this controller, the retained bad
history predates the active speed trial. The exact idle/startup event was not
logged, so its cause is not established. This is separate from provisional
scale flags, which are also false. Do not use this trace to approve automatic
distance travel. The next distance stage needs an explicit braked segment
baseline and retained diagnostics for any earlier idle loss; it must never
clear an active-motion fault or silently invent missing travel.

Evidence in the owner's Codex artifact folder: `speed-check/analysis.json`,
`trace.csv`, `latest-complete-report.txt` and
`physical-speed-20261021T182009Z.log`. The first recorded sample was at44549686us
after command start44539853us; final sample47529706us.96 retained rows are
intentional observer decimation, not a record of every control iteration.
