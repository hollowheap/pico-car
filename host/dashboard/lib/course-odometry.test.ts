import { describe, expect, it } from 'vitest';

import { buildCourse, courseStrokes, courseTapeRuns, courseTraceSegments, latestCourseSpeed, stabilizeStraightPoints } from './course';
import { describeEvent } from './events';
import type { LinkPacket } from './link';
import { ENUMS, FAULT_BITS } from './schema.generated';
import { emptyStore, reduceStore } from './telemetry-store';

const frame = (packet: string, timestamp: number, values: LinkPacket['values']): LinkPacket => ({ packet, timestamp, values });
const status = (seq: number, timestamp: number, distance: number, heading = 0xffff) =>
  frame('status', timestamp, { seq: seq & 0xffff, odometer_mm: distance, heading_ddeg: heading, speed_mms: 36 });
const wheels = (seq: number, timestamp: number, left = 72, right = 0) =>
  frame('motion', timestamp, { seq: seq & 0xff, left_mms: left, right_mms: right, motion_fault: 0 });
const fold = (frames: LinkPacket[]) => frames.reduce(reduceStore, emptyStore());

describe('wheel-based course reconstruction', () => {
  it.each([-1, 1])('corrects measured pivot slip before plotting subsequent travel (%i)', (direction) => {
    const pivot = (seq: number, ms: number, scale: number) =>
      frame('status', ms, { seq, odometer_mm: 100, speed_mms: 0,
        heading_ddeg: seq === 1 ? 0 : 0xffff, pivot_scale_pct: scale });
    const frames = [
      pivot(1, 1000, 60), wheels(1, 1000, direction * 100, -direction * 100),
      wheels(2, 1200, direction * 100, -direction * 100), pivot(2, 1200, 60),
      status(3, 1400, 110), wheels(3, 1400, 100, 100),
    ];
    const course = buildCourse(fold(frames).trail, []);
    const turn = 200 * 0.2 * 0.60 / 107;
    expect(course.points[1]).toEqual({ x: 0, y: 0 });
    expect(course.samples[1]?.headingDeg).toBeCloseTo((360 + direction * turn * 180 / Math.PI) % 360);
    // Half of the next interval is still the measured pivot. Its arc keeps
    // the recorded distance while rotating by the corrected amount.
    const halfArc = turn / 4;
    const chord = 10 * Math.sin(halfArc) / halfArc;
    expect(course.points[2]?.x).toBeCloseTo(direction * chord * Math.sin(turn + halfArc));
    expect(course.points[2]?.y).toBeCloseTo(chord * Math.cos(turn + halfArc));
  });

  it('uses wheel yaw when the pivot estimate is missing or unverified for this interval', () => {
    const pair = (seq: number, timestamp: number, scale: number, left = 100, right = -100) => [
      frame('status', timestamp, { seq, odometer_mm: 0, heading_ddeg: seq === 1 ? 0 : 0xffff, pivot_scale_pct: scale }),
      wheels(seq, timestamp, left, right),
    ];
    const heading = (frames: LinkPacket[]) => buildCourse(fold(frames).trail, []).samples.at(-1)?.headingDeg;
    const encoder = 200 * 0.2 / 107 * 180 / Math.PI;
    expect(heading([...pair(1, 1000, 0), ...pair(2, 1200, 20)])).toBeCloseTo(encoder);
    expect(heading([...pair(1, 1000, 60), ...pair(2, 1200, 0)])).toBeCloseTo(encoder * 0.8);
    expect(heading([...pair(1, 1000, 60), ...pair(3, 1400, 60)])).toBeCloseTo(encoder * 2);
    expect(heading([...pair(1, 1000, 60, 150, 50), ...pair(2, 1200, 60, 150, 50)]))
      .toBeCloseTo(encoder / 2);
  });

  it('draws more distance for faster travel over an equal sample interval', () => {
    const frames = [
      frame('status', 1000, { seq: 1, odometer_mm: 100, speed_mms: 50, heading_ddeg: 0 }),
      frame('status', 1200, { seq: 2, odometer_mm: 110, speed_mms: 50, heading_ddeg: 0 }),
      frame('status', 1400, { seq: 3, odometer_mm: 140, speed_mms: 150, heading_ddeg: 0 }),
    ];
    const segments = courseTraceSegments(buildCourse(fold(frames).trail, []));
    expect(segments.map(({ speedMms, points }) => [speedMms, points.at(-1)!.y - points[0]!.y]))
      .toEqual([[50, 10], [150, 30]]);
  });

  it('shows signed speed including zero, and colors each connected segment using its measured speed', () => {
    const packets = [
      frame('status', 1000, { seq: 1, odometer_mm: 100, heading_ddeg: 0, speed_mms: 0 }),
      frame('status', 1200, { seq: 2, odometer_mm: 77, heading_ddeg: 0 }),
      wheels(2, 1200, -110, -120),
      frame('status', 1400, { seq: 3, odometer_mm: 110, heading_ddeg: 0, speed_mms: 165 }),
      frame('status', 1600, { seq: 4, odometer_mm: 113, heading_ddeg: 0, speed_mms: 0, faults: 1 << FAULT_BITS.staleSpeed }),
      frame('status', 1800, { seq: 5, odometer_mm: 113, heading_ddeg: 0 }),
    ];
    const readings = packets.map((_, i) => {
      const store = fold(packets.slice(0, i + 1));
      return latestCourseSpeed(store.latest.status, store.trail.at(-1));
    });
    expect(readings).toEqual([0, null, -115, 165, null, null]);
    const strokes = courseStrokes(buildCourse(fold(packets).trail, []));
    expect(strokes.map((stroke) => stroke.band)).toEqual(['medium', 'fast', 'unknown']);
    expect(strokes[0]?.points.at(-1)).toEqual(strokes[1]?.points[0]);
    expect(strokes[1]?.points.at(-1)).toEqual(strokes[2]?.points[0]);
    expect(latestCourseSpeed(frame('status', 1900, { speed_mms: 22 }), undefined)).toBe(22);
  });

  it('closes a clockwise circle through compass dropout, low-byte wrap and reordered bridge batches', () => {
    const frames: LinkPacket[] = [];
    for (let i = 0; i < 48; i += 2) {
      const seq = 0xffef + i;
      const t = 1000 + Math.floor(i / 2) * 400;
      const first = status(seq, t, Math.round(i * 7.2), i === 0 ? 0 : 0xffff);
      const second = status(seq + 1, t, Math.round((i + 1) * 7.2), i === 24 ? 1330 : 0xffff);
      frames.push(wheels(seq, t), second, first, wheels(seq + 1, t), wheels(seq + 1, t));
    }
    const store = fold(frames);
    const course = buildCourse(store.trail, []);
    expect(course.points).toHaveLength(48);
    expect(course.points[11]?.x).toBeGreaterThan(40);
    expect(course.bounds.maxX - course.bounds.minX).toBeCloseTo(107, 0);
    expect(course.bounds.maxY - course.bounds.minY).toBeCloseTo(107, 0);
    expect(Math.hypot(course.points.at(-1)?.x ?? Infinity, course.points.at(-1)?.y ?? Infinity)).toBeLessThan(3);
  });

  it('does not draw unsupported travel or place a car at an old pose after genuine frame loss', () => {
    const store = fold([status(0, 1000, 0, 0), wheels(0, 1000), status(1, 1200, 7), wheels(1, 1200),
      status(17, 4400, 150), status(18, 4600, 160)]);
    const course = buildCourse(store.trail, []);
    expect(course.points.slice(2)).toEqual([course.points[1], course.points[1]]);
    expect(course).toMatchObject({ hasGap: true, positionKnown: false });
    expect(courseStrokes(course).flatMap((stroke) => stroke.points).every((p) => Math.hypot(p.x, p.y) < 8)).toBe(true);
  });

  it('resumes at the last mapped point after a gap without reusing old motion or rotating on a delayed compass', () => {
    const store = fold([status(0, 1000, 0, 0), wheels(0, 1000), status(1, 1200, 7), wheels(1, 1200),
      status(257, 52_400, 160), wheels(257, 1200), status(258, 52_600, 174), wheels(258, 52_600),
      status(259, 52_800, 181), wheels(259, 52_800), status(260, 53_000, 188, 2000), wheels(260, 53_000)]);
    const course = buildCourse(store.trail, []);
    expect(course).toMatchObject({ hasGap: true, positionKnown: true, headingReference: 'compass' });
    expect(course.points).toHaveLength(6);
    expect(course.points[2]).toEqual(course.points[1]);
    expect(course.points[3]).toEqual(course.points[1]);
    expect(course.points[0]).toEqual({ x: 0, y: 0 });
    expect(course.points.at(-1)?.y).toBeGreaterThan(13);
  });

  it('does not reuse pending pre-reboot motion at the same low status sequence', () => {
    const store = fold([frame('heartbeat', 1000, { run_id: 7, uptime_s: 10 }), wheels(0, 1200),
      frame('heartbeat', 1300, { run_id: 8, uptime_s: 0 }), status(0, 1300, 0),
      status(1, 1500, 7), wheels(1, 1500)]);
    expect(buildCourse(store.trail, [])).toMatchObject({ hasGap: true, positionKnown: false });
  });

  it('bridges one omitted pass when both wheel pairs and signed distance agree', () => {
    const a = status(1, 1000, 100, 0);
    const b = status(3, 1400, 114);
    const pairedA = wheels(1, 1000);
    const compatible = wheels(3, 1400);
    const optimistic = buildCourse(fold([a, b, pairedA, compatible]).trail, []);
    expect(optimistic).toMatchObject({ hasGap: false, positionKnown: true });
    expect(optimistic.points[1]?.x).toBeGreaterThan(1.8);

    const incompatible = wheels(3, 1400, 0, 72);
    expect(buildCourse(fold([a, b, pairedA, incompatible]).trail, [])).toMatchObject({ hasGap: true, positionKnown: false });
    expect(buildCourse(fold([a, status(3, 1400, 200), pairedA, compatible]).trail, []))
      .toMatchObject({ hasGap: true, positionKnown: false });
  });

  it('uses the short north-crossing arc for a pair of compass-only samples', () => {
    const course = buildCourse(fold([status(1, 1000, 100, 3590), status(2, 1200, 200, 10)]).trail, []);
    expect(course.points[1]?.x).toBeCloseTo(0);
    expect(course.points[1]?.y).toBeGreaterThan(99);
    expect(course.samples[1]?.headingDeg).toBeCloseTo(1);
  });

  it('keeps consistent wheel travel across a delayed receipt without bridging mismatched data', () => {
    const a = status(1, 1000, 100, 0);
    const pairedA = wheels(1, 1000, 65, 0);
    const pairedB = wheels(2, 2361);
    const late = buildCourse(fold([a, pairedA, status(2, 2361, 107), pairedB]).trail, []);
    expect(late).toMatchObject({ hasGap: false, positionKnown: true });
    expect(late.points).toHaveLength(2);
    expect(late.points[1]?.x).toBeGreaterThan(0.4);
    expect(buildCourse(fold([a, pairedA, status(2, 2361, 140, 900), pairedB]).trail, []))
      .toMatchObject({ hasGap: true, positionKnown: false });
  });

  it('does not use a pair of zero wheel readings to turn through a reported moving sample', () => {
    const store = fold([status(1, 1000, 100, 0), wheels(1, 1000, 0, 0),
      status(2, 1200, 107), wheels(2, 1200)]);
    expect(buildCourse(store.trail, [])).toMatchObject({ hasGap: true, positionKnown: false });
  });

  it('keeps old travel and marks on the same map through a fault and resumes from the last plotted point', () => {
    const barcode = (id: number, timestamp: number, odometer: number) => frame('event', timestamp,
      { event_id: id, event_type: 16, p1: 76, p2: 2, odometer_mm: odometer });
    const faulted = status(6, 2000, 136, 900);
    faulted.values.faults = 1 << 11;
    const store = fold([status(1, 1000, 100, 0), status(2, 1200, 120, 0), barcode(1, 1190, 119),
      barcode(2, 1700, 127), faulted, status(7, 2200, 156, 900), barcode(3, 2180, 155)]);
    const course = buildCourse(store.trail, store.events);
    expect(course).toMatchObject({ hasGap: true, positionKnown: true, headingReference: 'compass' });
    expect(course.samples.map((sample) => sample.odometerMm)).toEqual([100, 120, 136, 156]);
    expect(course.points[0]).toEqual({ x: 0, y: 0 });
    expect(course.points[2]).toEqual(course.points[1]);
    expect(course.points.at(-1)?.x).toBeCloseTo(20);
    expect(course.points.at(-1)?.y).toBeCloseTo(20);
    expect(course.markers.map(({ eventId }) => eventId)).toEqual([1, 3]);
    expect(course.markers[0]?.y).toBeCloseTo(19);
    expect(course.markers[1]?.x).toBeCloseTo(19);
  });

  it('restores device order for status receipts two milliseconds apart', () => {
    const store = fold([status(1, 1000, 100, 0), wheels(1, 1000), status(3, 1400, 114), wheels(3, 1400),
      wheels(2, 1401), status(2, 1402, 107)]);
    const course = buildCourse(store.trail, []);
    expect(course.samples.map((sample) => sample.seq)).toEqual([1, 2, 3]);
    expect(course.samples.every((sample) => sample.wheels !== undefined)).toBe(true);
    expect(store.latest.status?.values.seq).toBe(3);
    expect(store.statusLost).toBe(0);
  });
});

it('describes an external obstacle-state stop independently of sonar detection', () => {
  expect(describeEvent(ENUMS.eventType.obstacleStop, ENUMS.obstacleCause.external, 0))
    .toMatchObject({ title: 'External stop', kind: 'command' });
  expect(describeEvent(ENUMS.eventType.obstacleSeen, 200, 0))
    .toMatchObject({ title: 'Obstacle seen', kind: 'obstacle', detail: '200 mm ahead' });
});

it('damps straightaway heading jitter and attenuates overlapped historical passes', () => {
  // Oscillating ±3° steps along north (+y) should straighten closer to x=0.
  const headings = [0, 30, 0, -30, 0, 30, 0, -30, 0];
  const straightStore = fold(headings.map((h, i) => status(i + 1, 1000 + i * 200, i * 40, h)));
  const straightCourse = buildCourse(straightStore.trail, []);
  const stabilized = stabilizeStraightPoints(straightCourse.points, straightCourse.samples);
  const rawMaxX = Math.max(...straightCourse.points.map((p) => Math.abs(p.x)));
  const stabMaxX = Math.max(...stabilized.map((p) => Math.abs(p.x)));
  expect(stabMaxX).toBeLessThan(rawMaxX);

  // Back-and-forth pass across the same corridor marks the earlier pass as overlapped with lower opacity.
  const overlapFrames = [
    status(1, 0, 0, 0),
    status(2, 200, 40, 0),
    status(3, 400, 80, 0),
    status(4, 10_000, 120, 1800),
    status(5, 10_200, 160, 1800),
  ];
  const segments = courseTraceSegments(buildCourse(fold(overlapFrames).trail, []));
  expect(segments.some((s) => s.overlapped && s.opacity < 0.2)).toBe(true);
  expect(segments.some((s) => s.recent && s.opacity > 0.5 && s.opacity < 0.8)).toBe(true);

  // Tape runs stay continuous across brief off-tape straddles and break only on large gaps/jumps.
  const tapeFrames = [
    frame('status', 0, { seq: 1, odometer_mm: 0, heading_ddeg: 0, speed_mms: 100, line_bits: 1 }),
    frame('status', 200, { seq: 2, odometer_mm: 40, heading_ddeg: 0, speed_mms: 100, line_bits: 1 }),
    frame('status', 400, { seq: 3, odometer_mm: 80, heading_ddeg: 0, speed_mms: 100, line_bits: 0 }),
    frame('status', 600, { seq: 4, odometer_mm: 120, heading_ddeg: 0, speed_mms: 100, line_bits: 0 }),
    frame('status', 800, { seq: 5, odometer_mm: 160, heading_ddeg: 0, speed_mms: 100, line_bits: 2 }),
  ];
  expect(courseTapeRuns(buildCourse(fold(tapeFrames).trail, []))).toHaveLength(1);

  // Short packet gap with continuous odometer delta produces a straight estimated bridge.
  const gapFrames = [
    status(1, 0, 0, 0),
    status(2, 200, 40, 0),
    status(6, 1000, 200, 0),
    status(7, 1200, 240, 0),
  ];
  const gapCourse = buildCourse(fold(gapFrames).trail, []);
  const gapSegments = courseTraceSegments(gapCourse);
  expect(gapSegments.some((s) => s.estimated === true)).toBe(true);
});

it('projects 2D tape to front IR sensors, detects slip, and places 2D hump footprints', () => {
  // Heading north (0 deg): left IR hit (bit 0) projects +130 mm north and -22.5 mm west of axle.
  const tapeStore = fold([
    frame('status', 0, { seq: 1, odometer_mm: 0, heading_ddeg: 0, speed_mms: 100, line_bits: 1 }),
    frame('status', 200, { seq: 2, odometer_mm: 40, heading_ddeg: 0, speed_mms: 100, line_bits: 1 }),
  ]);
  const tapeRuns = courseTapeRuns(buildCourse(tapeStore.trail, []));
  expect(tapeRuns).toHaveLength(1);
  expect(tapeRuns[0]![0]!.x).toBeCloseTo(-22.5, 1);
  expect(tapeRuns[0]![0]!.y).toBeCloseTo(130, 1);

  // One-wheel slip (left spinning 120 mm/s, right stalled 0 mm/s, heading unchanged) flags slip.
  const slipStore = fold([
    status(1, 0, 0, 0),
    frame('motion', 200, { seq: 2, left_mms: 120, right_mms: 0 }),
    status(2, 200, 40, 0),
  ]);
  const slipSegs = courseTraceSegments(buildCourse(slipStore.trail, []));
  expect(slipSegs.some((s) => s.slip)).toBe(true);

  // Hump state samples + humpEnd event project an oriented 2D ramp footprint,
  // shifted laterally toward the raised wheel from rollDeg, while unflagged pitch noise is ignored.
  const humpStore = fold([
    frame('physics', 0, { flags: 8, pitch_cdeg: 1200, roll_cdeg: 1200, hump_state: 1 }),
    status(1, 0, 0, 0),
    frame('physics', 200, { flags: 8, pitch_cdeg: 1400, roll_cdeg: 1200, hump_state: 2 }),
    status(2, 200, 100, 0),
    frame('physics', 400, { flags: 8, pitch_cdeg: -1100, roll_cdeg: 1200, hump_state: 3 }),
    status(3, 400, 200, 0),
    frame('event', 420, { event_id: 1, event_type: ENUMS.eventType.humpEnd, p1: 24, p2: 1, odometer_mm: 200 }),
    frame('physics', 600, { flags: 8, pitch_cdeg: 800, roll_cdeg: 0, hump_state: 0 }),
    status(4, 600, 240, 0),
  ]);
  const humpCourse = buildCourse(humpStore.trail, humpStore.events);
  expect(humpCourse.humps).toHaveLength(1);
  expect(humpCourse.humps[0]).toMatchObject({ peakMm: 24, headingDeg: 0, widthMm: 160 });
  expect(humpCourse.humps[0]!.x).toBeCloseTo(53.5, 1);
  expect(humpCourse.humps[0]!.y).toBeCloseTo(100, 0);
  expect(humpCourse.bounds.maxX).toBeGreaterThanOrEqual(53);

  // Calibrated floor pivot (pivot_scale_pct: 50) does not falsely trigger slip.
  const pivotStore = fold([
    status(1, 0, 0, 0),
    frame('motion', 200, { seq: 2, left_mms: 150, right_mms: -150 }),
    frame('status', 200, { seq: 2, odometer_mm: 0, heading_ddeg: 160, speed_mms: 0, pivot_scale_pct: 50 }),
  ]);
  expect(courseTraceSegments(buildCourse(pivotStore.trail, [])).some((s) => s.slip)).toBe(false);

  // Passing a post-reset trailStart excludes samples and humps from before the reset.
  const postReset = buildCourse(humpStore.trail, humpStore.events, null, 500);
  expect(postReset.samples).toHaveLength(1);
  expect(postReset.humps).toHaveLength(0);
});
