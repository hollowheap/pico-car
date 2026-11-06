---
name: SE01 Mission Control
description: Dark-first control-room dashboard for one autonomous line-following car. Signal Blue is the only action accent; the light theme is derived from the dark surface stack.

colors:
  signal-blue: "#0063e1"
  on-accent: "#ffffff"

  dark-canvas: "#000000"
  dark-card: "#0d0d0d"
  dark-elevated: "#1f1f1f"
  dark-text: "#ffffff"
  dark-muted: "#858585"
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
    lineHeight: 1.25
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
  input: "8px"
  card: "16px"
  image: "24px"
  button: "59px"

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
    rounded: "{rounded.button}"
    padding: "0 20px"
  button-secondary:
    backgroundColor: "transparent"
    textColor: "{colors.dark-text}"
    borderColor: "{colors.dark-border}"
    typography: "{typography.body}"
    rounded: "{rounded.button}"
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
    rounded: "{rounded.button}"
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
- Motion is reserved for changing data. No decorative pulse, marquee, bounce,
  or gradient.

## 2. Themes

The dashboard follows the OS appearance. `app.json` sets
`userInterfaceStyle: "automatic"`. `lib/theme-provider.tsx` reads
`useColorScheme` and exposes one `Theme` object.

| Role | Dark | Light |
|---|---|---|
| Canvas | `#000000` | `#f7f7f5` |
| Card | `#0d0d0d` | `#ffffff` |
| Elevated | `#1f1f1f` | `#eeeeee` |
| Text | `#ffffff` | `#0d0d0d` |
| Muted | `#858585` | `#52525b` |
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

- Desktop: 1440x900 reference. Status rail, main content, secondary
  fault/event column. Page max width 1200px.
- Mobile: 375x812 reference. Single column, sticky status header, bottom
  navigation.
- Page padding: 24px on mobile, 32px to 40px on desktop.
- Group related items at 8px to 12px. Separate sections at 32px or 80px.
- Do not use the same gap everywhere.
- Do not let body text touch the viewport edge.

## 5. Components

### Button

- Primary: Signal Blue fill, white text, 59px pill radius, 44px minimum
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

- Short label, 12px 600, 59px pill radius, 4px by 12px padding.
- Always pair the color with text: `HEALTHY`, `FAULT`, `STALE`, `OFFLINE`,
  `READY`, `RUNNING`, `STOPPED`.
- Do not use color alone. Do not add a decorative pulsing dot.

### Metric row

- Label, value, unit. Right-align values in a table or use a two-column row.
- Use tabular figures if the face supports them.
- Do not use the hero-metric template: huge number, tiny label, supporting
  stats, gradient accent.

### Fault row

- Fault name, bit, latch state. One row per active fault.
- Empty state: `No active faults`.
- Do not show a fault bit as an unlabeled number.

### Event row

- Timestamp, event ID, type, p1, p2.
- If names are unavailable: `Event 12 / type 3`.
- No invented names.

### Banner

- Persistent for offline, stale, emergency latch, and command failure.
- Text label and a clear recovery action.
- Do not use a toast for a safety condition.

### Confirmation dialog

- Action, target, consequence, Cancel, Confirm.
- Confirm is the single primary button.
- Cancel is secondary.
- Use only for destructive commands. Do not use a modal for navigation or
  information.

## 6. Motion

- Use exponential ease-out only.
- Animate transform and opacity only.
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
- Do use the same three-screen model on desktop and mobile.

### Do not

- Do not use a camera, video, map, GPS, or IR imagery.
- Do not show per-buddy health or a fleet selector.
- Do not invent run cards or run comparison. The data has no run ID.
- Do not invent event names. The schema has no event catalog.
- Do not connect to Mosquitto directly or expose broker credentials.
- Do not add login, accounts, roles, notifications, export, or settings.
- Do not use gradients, glass, blur, glow, or decorative grid backgrounds.
- Do not use side-stripe borders, nested cards, or identical card grids.
- Do not use the hero-metric template.
- Do not use a modal as the first answer for every interaction.
- Do not use marketing copy, fake testimonials, fake logos, or fake stats.
- Do not use an off-token color, font, radius, or font size.

## 9. Impeccable anti-patterns

The full negative prompt is in
`artifacts/design/stitch/stitch-prompt.md`. It covers all 62 Impeccable
detector rules: 31 slop and 31 quality. This file defines the positive system.
The negative prompt defines what not to do. Both apply.
