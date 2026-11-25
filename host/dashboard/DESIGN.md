---
name: SE01 Mission Control
description: Dark-first control-room dashboard for one autonomous line-following car. Signal Blue is the only action accent; the light theme is derived from the dark surface stack.

colors:
  signal-blue: "#0063e1"
  on-accent: "#ffffff"

  dark-canvas: "#050505"
  dark-card: "#0d0d0d"
  dark-elevated: "#1f1f1f"
  dark-text: "#f7f7f5"
  dark-muted: "#8a8a8a"
  dark-border: "rgba(255, 255, 255, 0.12)"
  dark-danger: "#ffb4ab"
  dark-danger-surface: "#2b2c2e"

  light-canvas: "#f7f7f5"
  light-card: "#ffffff"
  light-elevated: "#eeeeee"
  light-text: "#0d0d0d"
  light-muted: "#52525b"
  light-border: "rgba(0, 0, 0, 0.12)"
  light-danger: "#b91c1c"
  light-danger-surface: "#fef2f2"

typography:
  scale:
    "12": "0.75rem"
    "16": "1rem"
    "18": "1.125rem"
    "20": "1.25rem"
    "24": "1.5rem"
    "56": "3.5rem"
    "86": "5.375rem"
  display:
    fontFamily: "system-ui, sans-serif"
    fontSize: "3.5rem"
    fontWeight: 400
    letterSpacing: "-0.04em"
    lineHeight: 1.3
  heading:
    fontFamily: "system-ui, sans-serif"
    fontSize: "1.5rem"
    fontWeight: 400
    letterSpacing: "-0.027em"
    lineHeight: 1.33
  body:
    fontFamily: "system-ui, sans-serif"
    fontSize: "1rem"
    fontWeight: 400
    letterSpacing: "-0.007em"
    lineHeight: 1.5
  label:
    fontFamily: "system-ui, sans-serif"
    fontSize: "0.75rem"
    fontWeight: 600
    letterSpacing: "0.033em"
    lineHeight: 1.2

rounded:
  nav: "2px"
  control: "8px"
  card: "16px"
  image: "24px"

spacing:
  xs: "4px"
  sm: "8px"
  md: "12px"
  lg: "16px"
  xl: "20px"
  xxl: "24px"
  section: "80px"

components:
  button-primary:
    backgroundColor: "{colors.signal-blue}"
    textColor: "{colors.on-accent}"
    typography: "{typography.body}"
    rounded: "{rounded.control}"
    padding: "0 20px"
  button-secondary:
    backgroundColor: "transparent"
    textColor: "{colors.dark-text}"
    borderColor: "{colors.dark-border}"
    typography: "{typography.body}"
    rounded: "{rounded.control}"
    padding: "0 20px"
  card:
    backgroundColor: "{colors.dark-card}"
    textColor: "{colors.dark-text}"
    borderColor: "{colors.dark-border}"
    rounded: "{rounded.card}"
    padding: "16px"
  status-pill:
    backgroundColor: "{colors.dark-elevated}"
    textColor: "{colors.dark-text}"
    typography: "{typography.label}"
    rounded: "{rounded.control}"
    padding: "4px 12px"
---

# Design System: SE01 Mission Control

## 1. Overview

A working control room for one autonomous car. The dashboard answers three
questions in order: is the car safe, what is it doing, and what failed. It is
not a marketing site.

**Key characteristics**

- Dark-first. The light theme is a derived surface stack, not a second brand.
- Signal Blue is the only action accent. It marks the one primary action in a
  viewport, active state, or brand punctuation.
- Surfaces are flat. A 1px inset border defines card edges. No drop shadows.
- Data is literal. Labels, units, and states are always visible. No decorative
  proof numbers.
- Screen changes use a brief fade. The system reduced-motion preference
  disables transitions. Telemetry updates and safety controls appear immediately.

## 2. Themes

The dashboard follows the OS appearance. `app.json` sets
`userInterfaceStyle: "automatic"`. `lib/theme-provider.tsx` reads
`useColorScheme` and exposes one `Theme` object.

| Role | Dark | Light |
|---|---|---|
| Canvas | `#050505` | `#f7f7f5` |
| Card | `#0d0d0d` | `#ffffff` |
| Elevated | `#1f1f1f` | `#eeeeee` |
| Text | `#f7f7f5` | `#0d0d0d` |
| Muted | `#8a8a8a` | `#52525b` |
| Border | `rgba(255,255,255,0.12)` | `rgba(0,0,0,0.12)` |
| Accent | `#0063e1` | `#0063e1` |
| On accent | `#ffffff` | `#ffffff` |
| Danger | `#ffb4ab` | `#b91c1c` |
| Danger surface | `#2b2c2e` | `#fef2f2` |

Rules:

- Do not hardcode a color outside `lib/theme.ts`.
- Do not use a different accent in light mode.
- Do not put a light card on a dark canvas or a dark card on a light canvas.
- The danger hue is semantic only. It is not a second brand accent.

## 3. Typography

**Display and body:** system UI face. If a licensed face is added later, pair
one display face with the body face. Do not introduce Inter, Roboto, Fraunces,
Geist, Plus Jakarta Sans, or Space Grotesk.

### Hierarchy

| Role | Size | Weight | Tracking | Line height |
|---|---|---|---|---|
| Caption | 12px | 400 | -0.005em | 1.2 |
| Body | 16px | 400 | -0.007em | 1.5 |
| Subheading | 18px | 400 | -0.015em | 1.4 |
| Heading small | 20px | 400 | -0.027em | 1.33 |
| Heading | 24px | 400 | -0.027em | 1.33 |
| Heading large | 56px | 400 | -0.04em | 1.25 |
| Display | 86px | 400 | -0.064em | 1.0 |
| Label | 12px | 600 | +0.033em | 1.2 |

Rules:

- Display and heading weights are 400. Use 600 only for short labels and
  emphasis.
- Do not use body text below 16px. Functional text stays at or above 12px.
- Keep body line length at or below 80 characters.
- Do not use all-caps body passages.
- Do not use wide tracking on body text.

## 4. Spacing and layout

Base unit: 4px.

| Step | Value |
|---|---|
| xs | 4px |
| sm | 8px |
| md | 12px |
| lg | 16px |
| xl | 20px |
| xxl | 24px |
| section | 80px |

- On desktop and phone, the navbar displays Signal in dBm, Car uptime and
  Missed ticks as plain text. Fresh zero counters share a single 0. A missed
  tick identifies its task, and unavailable readings show a dash. The status
  rail shows the vehicle mode, a distinct run state and active holds. A
  waiting, stale or offline connection is visible when it needs attention.
- Start, Left, Straight, Right, U-turn, Reset, Stop and E-stop are directly accessible
  in the navbar. The last-reported safety condition and recovery action align
  to the right of the command row on a laptop. Start acknowledges recoverable
  faults and begins IMU calibration. E-stop sends immediately when enabled.
  Stop cancels movement and keeps the dashboard history. Reset opens a confirmation
  popup. An accepted onboard Reset clears route state and starts a new dashboard
  session, including the trace, frames, events and wheel plots. Vehicle uptime and
  bridge recordings survive. A rejected or unconfirmed request preserves the session.
  Normal Start uses continuous floor. The **Stop at obstacles** switch selects a stationary survey for that run.
  Either the floor label or the switch changes the selection. After a stationary
  obstacle profile, the rail explains how to select a continuous-floor detour.
  The calibration hold tells the operator to keep the car level and still.
  A recorded START refusal for unavailable attitude appears with that hold.
  Physics reports unknown attitude until the car has a calibrated reference.
- The controls require a connected command service and fresh vehicle status.
  Stale or offline telemetry disables every command and dismisses pending
  confirmations. The send handler checks status age again when pressed.
  The navbar distinguishes an initial wait from expired readings and clears
  the current mode when stale. An emergency latch requires a vehicle reboot.
- At 1280 × 800, Course trace and Activity share the first column, Line and
  Obstacle the middle column, and Motion, Physics and Faults the right column.
  Orientation controls sit at the end of the course header. The Course card
  is 372 px tall with a 216 px plot throughout the three-column layout.
  The map fits the recorded route within that plot. Activity is 280 px tall
  on three-column screens. Events and Frames scroll inside the card.
  Below 1120 px the instruments share one column. Below 760 px the cards
  form one column.
- Course shows measured speed in mm/s alongside distance, elapsed sample
  time and average speed. A measured stop reads “Speed 0 mm/s”, an expired
  sample reads “Last speed” and an unavailable speed reads “Speed --”.
  Travel totals cover “This run” after a recorded START, or “Recorded” when
  the retained history starts mid-run.
- Activity holds Events and Frames. Event rows show the title, timestamp
  and distance. A pickup and its applied motion hold can share a row. Selecting
  it opens the individual records. Separate commands and concurrent holds keep
  their own rows. Select a row for the event source and full detail. A course
  marker selects the same event. Nonzero event counts appear in their
  instrument cards. Motion shows turns and stops, Line shows barcode and
  search activity, Obstacle shows encounters and bypasses, Physics shows
  humps and lifts, and Faults shows recorded fault events.
- The Line card owns live IR, decoder state and firmware counters. The
  Junction cell leads with the direction or maneuver. Activity owns the
  timestamped barcode event history. Obstacle places the sonar fan beside
  its legend, current measurements and recorded bearings. Below 560 px the
  readings stack under the fan. The scan distinguishes a live ping from an
  earlier measured sweep.
- Activity tab Frames shows the newest 5,000 retained bridge receipts across
  firmware runs. Pages follow the live stream until an older page or a frame is
  selected. Live receipts use the full width of Activity. Selecting a physics
  receipt opens its timestamped fields in the Physics card. Return to live
  restores the current gauges and keyboard focus to the selected receipt.
  Other packet types open their fields alongside the receipt list. The list
  and fields scroll within Activity while the pager stays visible.
- Each web tab list supports arrows, Home and End and has one tab in the keyboard
  order. Native tabs announce the selected state. Existing Course links open
  the trace. The legacy Systems URL opens the console with its inline readings,
  and specific drive or fault links open the corresponding card. Map and event
  inspection share the selection.
- One trace follows the car from the first observed pose through faults,
  stops and restarts of motion. A gap holds the last plotted location. The
  current-position arrow returns when measurements support another step.
- Line and barcode shows the latest decoded letter, “None” for an explicit
  empty firmware value, or “Unknown” before any decoder evidence. Read issues
  and dropped edges are separate counters.
- Edges places a dot at the car's estimated axle. Its short heading line turns
  with the car. A located survey or hug stays fixed in course coordinates as the
  car passes it. The dashed connector measures from the axle to the nearest
  supported edge. With both hug dimensions the view draws an estimated rectangle
  centered on the approach heading. With width alone it shows the near face.
  Ticks on a surveyed contour mark observed bounds. A survey without a known
  course pose shows the car at its scan-time origin, labelled “At scan.”
- Sonar plots the sweep on a polar grid with a 600 mm minimum range and six
  distance rings. Short strokes mark valid ranges. Rim marks retain each
  open or unusable bearing. The recorded sweep keeps its starting heading.
  A separate live servo beam shows the current aim while the reading is fresh.
- The moving bypass reports its stationary front fan. Edges joins contiguous
  measured returns and leaves unknown bearings open. When a later scan has no
  supported face, the card dates the last mapped one. Wall-follow mode appears
  in the hug status while active.
- A scan with a supported pose contributes its observed faces to the course
  trace. They persist across later sweeps, holds, faults and same-boot tab
  reloads. The latest located hug estimate also survives an idle reload after
  the heartbeat confirms the same boot. A confirmed reboot clears both with
  the trace. The last near return appears as a distance.
- The navbar reports stale telemetry once. Cards retain their last readings.
- The compact shell and main scroll view use 12px horizontal padding.
- Group related items at 8px to 12px. Separate sections at 32px or 80px.
- Do not use the same gap everywhere.
- Do not let body text touch the viewport edge.

## 5. Components

### Button

- Primary buttons use Signal Blue fill, white text, 8px corners and a 44px minimum
  height, 20px horizontal padding.
- Secondary: transparent fill, 1px border, text color, same radius and target.
- Danger is a semantic state, not a separate brand color. Use the danger token
  for destructive copy or a danger surface, not for a third button accent.
- One primary action per viewport. Navigation and secondary actions use the
  secondary variant.
- Visible focus ring. No suppressed outline.

### Card

- 16px radius, 1px inset border, 16px padding, no shadow.
- Do not nest cards. Use spacing, headings, and dividers inside a card.
- Do not repeat the same card grid across the page.

### Status pill

- Short text label, 12px 600. The state word carries the meaning.
- Pair status colors with meaningful text such as `STALE`, `OFFLINE`,
  `READY`, `RUNNING` or `STOPPED`. Fresh routine operation needs only the
  vehicle's reported state.
- Do not use color alone. Do not add a decorative pulsing dot.

### Metric row

- Label, value, unit. Right-align values in a table or use a two-column row.
- Use tabular figures if the face supports them.
- Do not use the hero-metric template: huge number, tiny label, supporting
  stats, gradient accent.

### Wheel history

Motion gives each wheel its own signed speed graph. Both use the same speed scale,
zero baseline and ten-second time window. Sequence gaps, invalid samples and
reboots break each wheel's trace.
Each wheel also shows the latest measured speed, target, and applied duty.
An idle controller with a zero target and nonzero wheel speed reads “Zero target”.

### Fault list

- The header shows the active count, past count, clear state or stop cause.
  Keep the marker beside the Matrix and Health tabs. The detail slot
  stays in place and shows the hovered, focused or selected fault, then the
  sonar flags or a default message.
- Activity’s Frames view preserves the original bit masks.

### Panel states

- Every panel has four states besides its data: loading (waiting for the first
  packet), empty (no data and the link is down), stale (the packet is older than
  its limit and its readings remain visible) and error. Never show zeros for a packet
  that has not arrived.
- Sensor rates use the difference between two cumulative diagnostic samples
  divided by elapsed device time. The first sample after a boot establishes the
  baseline. A delayed sample within the rate window leaves the latest sample
  and baseline in place. A receipt gap long enough for the device timer to
  cycle fully starts a new baseline without deriving a rate for the gap. The
  longest recorded line recovery comes from retained line-found events.
  Recordings made with the legacy diagnostic layout still show their reported
  sensor rates.

### Event row

- Time, odometer, the name from the schema, and p1 and p2 decoded with the
  layouts in `protocol/telemetry.yaml`.
- For a type the schema lacks: `Event 12`. No invented names.

### Instruments

- Compass: a heading-up card that turns under a fixed car. The north mark is the
  only accent in it. The arc beside the car shows the turn rate.
- Level gauges: a side view for pitch and a rear view for roll, with ticks at
  the lift envelope.
- The edge drawing uses the same odometry and contact pose as Course. Open,
  missing and unusable samples leave a gap in a surveyed contour.
- Course trace: dead-reckoned from the odometer and the heading. Course has
  North up and Car up views. Car up rotates the trace, events and obstacle
  together in the dashboard and shows north relative to the car's heading.
  A square metric grid shares the course origin and rotates with those features.
  Each square spans the distance on the scale bar as the map fits the run.
  Travel segments are colored by recorded speed. Dashed segments mark unknown
  speed where the vehicle position is known.
  Without a current heading the view uses North up. A reverse draws backward along
  the heading. The odometer falls in a reverse, so it can name two places.
  A registered scan keeps its sampled trail point even when status frames
  from one bridge burst share a timestamp. A mark is drawn at the trail
  segment nearest its packet time in trail order that spans its odometer. An event older than
  the trail lands on the first segment that spans its odometer, the first pass;
  a mark whose odometer no segment spans is not drawn. The
  obstacle a hug found is a box ahead of the car's pose where the hug first
  read the face, on the heading of the contact; a side with no measured size is
  dashed. The contact and first face each use the newest available status point
  at or before their obstacle frame (up to 200 ms old). Without either pose the box
  stays off the map for that hug. An anchored box stays at its recorded pose.
  The trace starts over on a firmware boot event or heartbeat evidence of
  reboot (a changed boot ID or an uptime rollback inconsistent with timer
  wraparound). Queued boot evidence for the same startup counts once. Pre-boot events stay off the new map.
  Faults, STOP/START, missing samples and odometer discontinuities keep the
  original coordinate frame. Unsupported travel contributes no displacement.
  Older samples are simplified once the trail reaches 6,000 points. The start,
  current pose and receipt-linked obstacle and event anchors stay in that frame.
  A web tab saves its observed trail on page exit. On reload, a fresh
  heartbeat confirming the same car boot allows the tab to merge that trail
  with bridge history. Current instruments and commands use the new connection.
  A new tab can reconstruct the travel covered by retained bridge telemetry.
- SVG text uses the page font and stays at or above 12 px after scaling.

### Banner

- Persistent for offline, emergency latch, and command failure. If telemetry
  goes stale, a reported safety condition keeps its notice marked last reported.
- Text label and a clear recovery action.
- Do not use a toast for a safety condition.

### Confirmation dialog

- Center the confirmation over the instruments and dim that area. Keep the
  command toolbar available above it. Name the command and explain the
  consequence of Start. Give Cancel initial focus and return focus to the
  trigger on dismissal. Escape, native Back and a tap on the backdrop dismiss
  the popup.
- The confirmation contains Cancel and Confirm. E-stop has one persistent
  control in the navbar. A fresh E-stop sends immediately and dismisses the
  pending command.

## 6. Motion

- Use exponential ease-out only.
- Animate transform and opacity only.
- Ease the course viewport and current pose between live updates. Snapshot
  safety states, lost pose, and reduced-motion mode update immediately.
- No bounce, elastic, marquee, blinking cursor, or width/height animation.
- Pulse only when it represents genuinely changing live data.
- No image hover transforms.

## 7. Accessibility

- WCAG AA: 4.5:1 for body text, 3:1 for large text.
- 44x44 minimum touch target.
- Visible focus ring on every interactive control.
- Accessible name on every button and icon-only control.
- Do not use color alone for status.
- Do not use content that is invisible at rest.

## 8. Do and Do not

### Do

- Do use Signal Blue for the one primary action.
- Do keep surfaces dark, flat, and mineral.
- Do use a 1px inset border for card definition.
- Do show heartbeat age, packet age, and fault state at all times.
- Do use real values and units.
- Do use the same single page on desktop and mobile.

### Do not

- Do not use a camera, video, GPS, satellite or street map, or IR imagery. The
  course trace is allowed: it is drawn from the car's own
  odometer and heading.
- Do not show per-buddy health or a fleet selector.
- Do not invent run cards or run comparison. The data has no run ID. One
  summary of the latest run is allowed.
- Do not invent event names. They come from `events` in
  `protocol/telemetry.yaml`.
- Do not connect to Mosquitto directly or expose broker credentials.
- Do not add login, accounts, roles, notifications, export, or settings.
- Do not use gradients, glass, blur, glow, or decorative grid backgrounds.
- Do not use side-stripe borders, nested cards, or identical card grids.
- Do not use the hero-metric template.
- Do not use a modal as the first answer for every interaction.
- Do not use marketing copy, fake testimonials, fake logos, or fake stats.
- Do not use an off-token color, font, radius, or font size.

## 9. Impeccable anti-patterns

The negative prompt covers all 62 Impeccable detector rules: 31 slop and 31
quality. This file defines the positive system. The negative prompt defines
what not to do. Both apply.

## 10. Android preview

The `preview` profile builds a standalone, internally distributed APK for
`com.picocar.dashboard`. Link the dashboard to its Expo project and configure
Android signing credentials before building. In that project's EAS `preview`
environment, set `EXPO_PUBLIC_SERVICE_URL` to the bridge's HTTPS base URL and
`EXPO_PUBLIC_LINK_URL` to its WSS endpoint. Both URLs are bundled as public
configuration. Use a LAN hostname the test phone can resolve and a certificate
it trusts. The preview profile keeps the insecure LAN opt-in disabled.
[Expo APK profiles](https://docs.expo.dev/build-reference/apk/) · [Build environment variables](https://docs.expo.dev/eas/environment-variables/usage/)

Run `eas build --platform android --profile preview` from `host/dashboard` to
produce the signed APK. Verify the local Android bundle export before building.
The physical smoke test uses the signed APK, ADB and a synthetic TLS bridge
with Metro stopped.
