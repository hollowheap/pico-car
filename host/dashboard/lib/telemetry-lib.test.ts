import { describe, expect, it } from 'vitest';

import { buildCourse, orientCourse, placeContact } from './course';
import { describeEvent, eventName, isCurrentStartAttitudeRefusal } from './events';
import { FAULT_META, faultTiles, summarizeFaults } from './faults';
import type { LinkPacket } from './link';
import { longestLineRecovery, summarizeRun } from './run-summary';
import { currentScanReturn, polarToXY, rangeRings, scanPoints } from './scan';
import { ENUMS, FAULT_BITS } from './schema.generated';
import { storeFromHistory } from './history';
import { panelStatus } from './panel-state';
import { odometerDiscontinuous } from './odometer';
import { emptyStore, reduceStore, statusLossPercent, TRAIL_MAX } from './telemetry-store';

const packet = (name: string, values: Record<string, number>, timestamp = 1000): LinkPacket => ({
  timestamp,
  packet: name,
  values,
});
const status = (seq: number, odo: number, headingDeg: number, timestamp: number) =>
  packet('status', { seq, odometer_mm: odo, heading_ddeg: headingDeg * 10, speed_mms: 160 }, timestamp);
const event = (id: number, type: number, p1 = 0, p2 = 0, odo = 0, timestamp = 1000) =>
  packet('event', { event_id: id, event_type: type, source: 2, p1, p2, odometer_mm: odo }, timestamp);

describe('obstacle ping evidence', () => {
  const sample = (timestamp: number, seq: number | undefined, range = 145, echo: number = ENUMS.echoClass.valid) =>
    packet('obstacle', {
      ...(seq === undefined ? {} : { seq }), dist_mm: range, echo_class: echo, servo_deci: 0,
    }, timestamp);

  it('ages the ping independently of repeated snapshots and retains its original course pose', () => {
    const acquired = sample(1000, 41);
    const updatedState = { ...sample(3000, 41), values: { ...acquired.values, cause: 2 } };
    const first = [status(1, 0, 0, 1000), acquired].reduce(reduceStore, emptyStore());
    expect(currentScanReturn(first.obstaclePing, 1500, true)?.distMm).toBe(145);
    const continued = [status(2, 100, 0, 2900), updatedState].reduce(reduceStore, first);

    expect(continued.latest.obstacle).toBe(updatedState);
    expect(continued.obstaclePing).toBe(acquired);
    expect(continued.forwardReturn).toBe(first.forwardReturn);
    expect(currentScanReturn(continued.obstaclePing, 3025, true)).toBeNull();
  });

  it('handles ping sequence rollover, startup placeholders and older replays', () => {
    const blank = sample(1000, 0, 0);
    const first = reduceStore(emptyStore(), blank);
    expect(first.obstaclePing).toBeUndefined();
    expect(reduceStore(first, sample(1200, 0, 0, ENUMS.echoClass.noEcho)).obstaclePing).toBeUndefined();
    const wrap = sample(2000, 0);
    const store = [sample(1500, 63), wrap, sample(1800, 62, 175)].reduce(reduceStore, first);
    expect(store.latest.obstacle).toBe(wrap);
    expect(store.obstaclePing).toBe(wrap);
    expect(currentScanReturn(store.obstaclePing, 2100, true)?.distMm).toBe(145);
  });

  it('uses changed legacy evidence and resets the baseline with the run', () => {
    const history = [sample(3000, undefined), sample(1000, undefined)].map((record) => ({ ...record, topic: 'car/o' }));
    const replayed = storeFromHistory(history);
    expect(replayed.latest.obstacle?.timestamp).toBe(3000);
    expect(replayed.obstaclePing?.timestamp).toBe(1000);
    const next = sample(3100, undefined, 150);
    const updated = reduceStore(replayed, next);
    expect(updated.obstaclePing).toBe(next);

    const restarted = reduceStore(updated, event(1, 0x01, 0, 0, 0, 3200));
    expect(restarted.obstaclePing).toBeUndefined();
    expect(reduceStore(restarted, next).latest.obstacle).toBeUndefined();
    expect(reduceStore(restarted, sample(3300, 0, 150)).obstaclePing?.timestamp).toBe(3300);
  });
});

describe('sensor diagnostics', () => {
  const sample = (clock: number, edges: number, valid: number, timestamp = 1000) =>
    packet('diag', { sensor_sample_us: clock, line_edges_total: edges, echo_total_0: valid }, timestamp);

  it('uses device time and raw counters across a missed frame, including a counter wrap', () => {
    const first = reduceStore(emptyStore(), sample(0xfff0bdc0, 40, 0xfffffffe));
    expect(first.diagRates.echo_valid).toBeUndefined();
    const next = reduceStore(first, sample(1_000_000, 46, 4));
    expect(next.diagRates.line_edges_per_s).toBe(3);
    expect(next.diagRates.echo_valid).toBe(3);
    expect(next.latest.diag?.values).toEqual({ sensor_sample_us: 1_000_000, line_edges_total: 46, echo_total_0: 4 });
    expect(reduceStore(next, sample(1_000_000, 46, 4)).diagRates.echo_valid).toBe(3);
  });

  it('starts a fresh baseline after boot and reads legacy rates', () => {
    const store = [sample(1_000_000, 10, 20), sample(2_000_000, 16, 25), event(10, 0x01, 0, 0, 0, 1200),
      sample(3_000_000, 3, 5, 1300)].reduce(reduceStore, emptyStore());
    expect(store.diagRates.echo_valid).toBeUndefined();
    expect(reduceStore(store, packet('diag', { echo_valid: 9, line_edges_per_s: 5 }, 1400)).diagRates)
      .toMatchObject({ echo_valid: 9, line_edges_per_s: 5 });
  });

  it('keeps the latest diagnostic baseline when a recent older sample arrives after it', () => {
    const latest = sample(3_000_000, 30, 30, 3000);
    const store = [sample(1_000_000, 10, 10, 1000), latest].reduce(reduceStore, emptyStore());
    const delayed = reduceStore(store, sample(2_000_000, 10, 10, 3100));

    expect(delayed.latest.diag).toBe(latest);
    expect(delayed.diagRates.echo_valid).toBe(10);
    expect(reduceStore(delayed, sample(4_000_000, 40, 40, 4000)).diagRates.echo_valid).toBe(10);
  });

  it('accepts an interval too long to compute a rate as a fresh baseline', () => {
    const store = [sample(1_000_000, 10, 10, 1000), sample(50_000_000, 40, 40, 50_000)]
      .reduce(reduceStore, emptyStore());

    expect(store.latest.diag?.values.sensor_sample_us).toBe(50_000_000);
    expect(store.diagRates.echo_valid).toBeUndefined();
    expect(reduceStore(store, sample(51_000_000, 50, 50, 51_000)).diagRates.echo_valid).toBe(10);
  });

  it('accepts a smaller timer reading after enough time for a wrap', () => {
    const clock = 1_000_000;
    const longGapMs = Math.ceil((2 ** 32 - clock) / 1000);
    const store = [sample(2 * clock, 10, 10, 1000), sample(clock, 20, 20, 1000 + longGapMs)]
      .reduce(reduceStore, emptyStore());

    expect(store.latest.diag?.values.sensor_sample_us).toBe(clock);
    expect(store.diagRates.echo_valid).toBeUndefined();
    expect(reduceStore(store, sample(2 * clock, 30, 30, 2000 + longGapMs)).diagRates.echo_valid).toBe(10);
  });

  it.each([0, 2_000_000])('clears rates when the receipt gap permits a full wrap and the timer advances %s μs', (advance) => {
    const resumedAt = 2000 + Math.ceil((2 ** 32 + advance) / 1000);
    const resumed = sample(2_000_000 + advance, 40, 40, resumedAt);
    const store = [sample(1_000_000, 10, 10, 1000), sample(2_000_000, 20, 20, 2000), resumed]
      .reduce(reduceStore, emptyStore());

    expect(store.latest.diag).toBe(resumed);
    expect(store.diagRates.echo_valid).toBeUndefined();
    const next = reduceStore(store, sample(3_000_000 + advance, 50, 50, resumedAt + 1000));
    expect(next.diagRates.echo_valid).toBe(10);
  });

  it('keeps diagnostics across an odometer discontinuity and starts a new baseline on a reboot heartbeat', () => {
    const store = [packet('heartbeat', { uptime_s: 50, run_id: 7 }, 48_900), sample(49_000_000, 30, 30, 49_000),
      status(1, 2000, 0, 49_100), sample(50_000_000, 40, 40, 50_000)]
      .reduce(reduceStore, emptyStore());
    expect(store.diagRates.echo_valid).toBe(10);

    const continued = reduceStore(store, status(2, 0, 0, 50_100));
    expect(continued.latest.diag).toBe(store.latest.diag);
    expect(continued.diagRates.echo_valid).toBe(10);
    const confirmed = reduceStore(continued, packet('heartbeat', { uptime_s: 1, run_id: 8 }, 50_200));
    expect(confirmed.trail).toEqual([]);
    expect(confirmed.latest.diag).toBeUndefined();
    const baseline = reduceStore(confirmed, sample(1_000_000, 2, 2, 50_250));
    expect(baseline.diagRates.echo_valid).toBeUndefined();
    expect(reduceStore(baseline, sample(2_000_000, 4, 4, 51_250)).diagRates.echo_valid).toBe(2);
  });
});

it('replays history and buffered live frames once, including distinct frames in the same millisecond', () => {
  const before = status(45, 1234, 0, 1000);
  const boot = event(1, 0x01, 0, 0, 0, 1100);
  const first = status(1, 0, 90, 1100);
  const second = status(2, 10, 90, 1110);
  const third = status(3, 20, 90, 1110);
  const records = [second, first, boot, before].map((row) => ({ ...row, topic: 'car/test' }));
  const store = storeFromHistory(records, [before, boot, first, second, third]);

  expect(store.trail.map((row) => row.odometerMm)).toEqual([0, 10, 20]);
  expect(store.statusReceived).toBe(4);
  expect(store.events.map((row) => row.name)).toEqual(['boot']);
  expect(store.latest.status?.values.seq).toBe(3);
});

describe('near sonar return', () => {
  const ping = (dist: number, time: number, overrides = {}) =>
    packet('obstacle', { dist_mm: dist, echo_class: 0, servo_deci: 0, hug_state: 0, ...overrides }, time);
  const courseFor = (store: ReturnType<typeof emptyStore>) =>
    buildCourse(store.trail, store.events, store.obstacle, store.trailStart, store.forwardReturn);

  it('places an idle forward return from the sonar origin at the recorded pose', () => {
    const store = [
      status(1, 0, 90, 1000),
      status(2, 100, 90, 1200),
      event(1, 0x1a, 0, 0x010b, 100, 1250),
      ping(234, 1300),
      status(3, 60, 0, 1400),
      ping(234, 1500, { echo_class: 1 }),
    ].reduce(reduceStore, emptyStore());
    const course = courseFor(store);
    expect(course.obstacle).toBeNull();
    expect(course.forwardReturn?.rangeMm).toBe(234);
    expect(course.forwardReturn?.beam?.from.x).toBeCloseTo(170);
    expect(course.forwardReturn?.beam?.to.x).toBeCloseTo(404);
    expect(course.forwardReturn?.beam?.to.y).toBeCloseTo(0);
    expect(course.bounds.maxX).toBeCloseTo(100); // An isolated range is shown as text outside the map.
  });

  it('keeps the forward return at its pose across later status frames sharing its time and odometer', () => {
    const store = [status(1, 0, 0, 900), status(2, 100, 0, 1000), ping(120, 1000),
      status(3, 250, 90, 1000), status(4, 100, 0, 1000)].reduce(reduceStore, emptyStore());

    const beam = courseFor(store).forwardReturn?.beam;
    expect(beam?.from.x).toBeCloseTo(0);
    expect(beam?.from.y).toBeCloseTo(170);
    expect(beam?.to.y).toBeCloseTo(290);
  });

  it('keeps the range when the heading or a recent position is missing', () => {
    const unknown = reduceStore(emptyStore(), status(1, 0, 6553.5, 1000));
    const old = reduceStore(emptyStore(), status(1, 0, 90, 1000));
    expect(courseFor(reduceStore(unknown, ping(234, 1100))).forwardReturn).toEqual({ rangeMm: 234, beam: null });
    expect(courseFor(reduceStore(old, ping(234, 3000))).forwardReturn).toEqual({ rangeMm: 234, beam: null });
  });

  it('ignores invalid returns, keeps the anchored return across an odometer jump and clears it on boot', () => {
    const ready = reduceStore(emptyStore(), status(1, 0, 0, 1000));
    const ignored = [ping(0, 1100), ping(251, 1100), ping(234, 1100, { echo_class: 5 }),
      ping(234, 1100, { servo_deci: 20 }), ping(234, 1100, { hug_state: 4 })];
    expect(courseFor(ignored.reduce(reduceStore, ready)).forwardReturn).toBeNull();
    const seen = reduceStore(ready, ping(250, 1100));
    expect(courseFor(seen).forwardReturn?.rangeMm).toBe(250);
    expect(courseFor(reduceStore(seen, event(2, 0x01, 0, 0, 0, 1200))).forwardReturn).toBeNull();
    const continued = reduceStore(seen, status(2, 9000, 90, 1200));
    expect(continued.forwardReturn).toBe(seen.forwardReturn);
    expect(courseFor(continued).forwardReturn).toEqual(courseFor(seen).forwardReturn);
  });
});

describe('course orientation', () => {
  it('turns the entire scene around the car while preserving its world positions', () => {
    const store = [status(1, 0, 0, 1000), obstacle(1, 250, 100, 0, 1000),
      event(1, 0x20, 250, 0, 0, 1000), status(2, 100, 90, 1200)]
      .reduce(reduceStore, emptyStore());
    const world = buildCourse(store.trail, store.events, store.obstacle);
    const view = orientCourse(world, 'car-up');

    expect(view.headingDeg).toBe(0);
    expect(view.northDeg).toBe(-90);
    expect(view.course.points.at(-1)).toEqual({ x: 0, y: 0 });
    expect(view.course.points[0]?.y).toBeCloseTo(-100);
    expect(view.course.markers[0]?.y).toBeCloseTo(-100);
    expect(view.course.markers[0]?.event).toBe(world.markers[0]?.event);
    expect(view.course.obstacle?.x).toBeCloseTo(-250);
    expect(view.course.obstacle?.y).toBeCloseTo(-100);
    expect(view.course.bounds.minX).toBeCloseTo(-250);
    expect(world.points.at(-1)?.x).toBeCloseTo(100);
    expect(orientCourse(world, 'north-up').course).toBe(world);
  });

  it('projects the last sonar return and falls back to north when the heading is missing', () => {
    const store = [status(1, 0, 90, 1000), packet('obstacle', { dist_mm: 250, echo_class: 0, servo_deci: 0 }, 1100)]
      .reduce(reduceStore, emptyStore());
    const world = buildCourse(store.trail, [], null, null, store.forwardReturn);
    const beam = orientCourse(world, 'car-up').course.forwardReturn?.beam;

    expect(beam?.from.x).toBeCloseTo(0);
    expect(beam?.from.y).toBeCloseTo(70);
    expect(beam?.to.y).toBeCloseTo(320);
    const noHeading = buildCourse([{ timestamp: 1000, odometerMm: 0, headingDeg: null, speedMms: 0 }], []);
    expect(orientCourse(noHeading, 'car-up')).toEqual({
      course: noHeading, headingDeg: null, northDeg: 0, origin: { x: 0, y: 0 },
    });
  });
});

describe('faults', () => {
  it('has a label and a severity for every bit of the schema', () => {
    for (const bit of Object.values(FAULT_BITS)) {
      expect(FAULT_META[bit], `bit ${bit}`).toBeDefined();
    }
    expect(Object.keys(FAULT_META)).toHaveLength(Object.keys(FAULT_BITS).length);
  });

  it('separates set now, set before and ok', () => {
    const tiles = faultTiles(1 << 11, (1 << 11) | (1 << 4));
    expect(tiles).toHaveLength(16);
    expect(tiles[11]?.state).toBe('set');
    expect(tiles[4]?.state).toBe('was-set');
    expect(tiles[0]?.state).toBe('ok');
    expect(summarizeFaults(tiles)).toEqual({ set: 1, critical: 1, wasSet: 1 });
  });

  it('reports no data, never ok, without a status word', () => {
    expect(faultTiles(undefined, undefined).every((tile) => tile.state === 'unknown')).toBe(true);
  });

  it('treats the line-lost warning as non-critical', () => {
    const tiles = faultTiles(1 << 4, 1 << 4);
    expect(summarizeFaults(tiles)).toEqual({ set: 1, critical: 0, wasSet: 0 });
  });
});

describe('events', () => {
  it('names a barcode with its action, direction and bar width', () => {
    const text = describeEvent(0x10, 'L'.charCodeAt(0), (150 << 4) | (1 << 3) | 2);
    expect(text.title).toBe('Barcode L');
    expect(text.detail).toContain('Left');
    expect(text.detail).toContain('reverse');
    expect(text.detail).toContain('600 µm');
  });

  it('distinguishes a failed stripe read from a decoded letter without a driving action', () => {
    expect(describeEvent(0x11, 1, 0)).toEqual({
      kind: 'barcode', title: 'Barcode read failed',
      detail: 'Unexpected stripe width or bar/space order. The failed stripe is unavailable.',
    });
    const decoded = describeEvent(0x10, 'A'.charCodeAt(0), (150 << 4) | 5);
    expect(decoded.title).toBe('Barcode A');
    expect(decoded.detail).toContain('Decoded, no driving action');
    expect(decoded.detail).toContain('600 µm');
  });

  it('displays a rejected barcode element width when the firmware recorded one', () => {
    expect(describeEvent(0x11, 1, 1750).detail)
      .toBe('Unexpected stripe width or bar/space order. Estimated last element width 1750 µm.');
    expect(describeEvent(0x11, 2, 2400).detail)
      .toBe('Parity check failed. Estimated last element width 2400 µm.');
    expect(describeEvent(0x11, 2, 0).detail).toBe('Parity check failed.');
    expect(describeEvent(0x11, 6, 0).detail).toBe('Symbol outside the A–Z route alphabet.');
    expect(describeEvent(0x11, 7, 0).detail).toBe('Barcode sensor transitions were lost during the read.');
  });

  it('decodes a refused command with its source, reason and id', () => {
    const text = describeEvent(0x05, (2 << 12) | (1 << 8) | 3, 77);
    expect(text.title).toBe('Command refused');
    expect(text.detail).toBe('left from telemetry: busy (command 77)');
    expect(describeEvent(0x05, (8 << 12) | (1 << 8) | 3, 77).detail).toContain(': stalled');
    expect(describeEvent(0x05, (9 << 12) | (1 << 8) | 3, 77).detail).toContain(': IMU attitude unavailable');
  });

  it('shows why a startup motion check stopped the car', () => {
    expect(describeEvent(0x03, 2, 1).title).toBe('Startup motion unverified');
    expect(describeEvent(0x03, 2, 0).title).toBe('Run stopped');
  });

  it('decodes a pilot decision with its behaviour, reason and speed', () => {
    const stop = describeEvent(0x1a, 0, (1 << 8) | 6);
    expect(stop.title).toBe('Pilot decision');
    expect(stop.detail).toBe('safety; reason sonar; 0 mm/s');
    expect(describeEvent(0x1a, 0xffc4, (3 << 8) | 0).detail).toBe('follow line; reason none; -60 mm/s');
    expect(describeEvent(0x1a, 0, (1 << 8) | 1).detail).toContain('stale wheel feedback');
    expect(describeEvent(0x1a, 0, (1 << 8) | 9).detail).toBe('safety; reason sonar (latched); 0 mm/s');
    expect(describeEvent(0x1a, 0, (1 << 8) | 10).detail).toContain('stale wheel feedback (latched)');
    expect(describeEvent(0x1a, 0, (1 << 8) | 99).detail).toContain('reason 99');
  });

  it('labels the course-clearance stop as an obstacle alert', () => {
    expect(describeEvent(0x1a, 0, 0x010b)).toMatchObject({ kind: 'obstacle', title: 'Course clearance hold' });
    expect(describeEvent(0x1a, 0, 0x010c)).toEqual({
      kind: 'obstacle',
      title: 'Checking course clearance',
      detail: 'The pilot is holding for fresh forward clearance.',
    });
    expect(describeEvent(0x1a, 0, 0x040d)).toMatchObject({
      kind: 'obstacle', title: 'Searching for tape behind obstacle',
    });
    expect(describeEvent(0x1a, 0, 0x040e)).toMatchObject({
      kind: 'obstacle', title: 'Rejoin search stopped',
    });
  });

  it('labels signed sonar commands without implying shaft feedback', () => {
    expect(eventName(0x25)).toBe('sonarAim');
    expect(describeEvent(0x25, -65, 0)).toEqual({
      kind: 'obstacle', title: 'Sonar aim', detail: 'Commanded 6.5° right',
    });
    expect(describeEvent(0x25, 200, 0).detail).toBe('Commanded 20° left');
    expect(describeEvent(0x25, 0, 0).detail).toBe('Commanded 0° (center)');
  });

  it('decodes a motion transition with the applied hold mask and fault', () => {
    expect(eventName(0x06)).toBe('motionState');
    expect(describeEvent(0x06, (1 << 8) | 4, (4 << 8) | 4)).toEqual({
      kind: 'system', title: 'Motion state changed', detail: 'running → fault, fault gap, hold operator',
    });
  });

  it('keeps old attitude events readable and shows the new transition evidence', () => {
    expect(describeEvent(0x38, 0, 0).detail).toBe('pickup detected');
    expect(describeEvent(0x38, 640, 720).detail).toBe('pickup detected, tilt 6.4°, acceleration 720 mg');
    expect(describeEvent(0x39, 0, 1010).detail).toBe('START releases the hold, tilt 0°, acceleration 1010 mg');
    expect(describeEvent(0x3a, 17_430, 1000).detail).toBe('Set down upright and still before START, tilt 174.3°, acceleration 1000 mg');
    expect(describeEvent(0x38, 0x7fff, 0).detail).toBe('pickup detected, tilt unavailable, acceleration 0 mg');
  });

  it('decodes an accepted command; a barcode command has no id', () => {
    expect(describeEvent(0x04, (1 << 8) | 1, 5).detail).toBe('start from telemetry (command 5)');
    expect(describeEvent(0x04, (2 << 8) | 3, 0).detail).toBe('left from barcode');
  });

  it('preserves historical event decoding for reserved wire opcode 10', () => {
    expect(describeEvent(0x04, (1 << 8) | 10, 12)).toEqual({
      kind: 'command',
      title: 'Command accepted',
      detail: 'start stationary from telemetry (command 12)',
    });
    expect(describeEvent(0x05, (6 << 12) | (1 << 8) | 10, 13)).toEqual({
      kind: 'command',
      title: 'Command refused',
      detail: 'start stationary from telemetry: unknown command (command 13)',
    });
    expect(isCurrentStartAttitudeRefusal(0x05, (9 << 12) | (1 << 8) | 10, 14, 14)).toBe(true);
    expect(isCurrentStartAttitudeRefusal(0x05, (9 << 12) | (1 << 8) | 10, 14, 15)).toBe(false);
  });

  it('tells a line search start, a retry and a stop apart', () => {
    const start = describeEvent(0x18, 1, 0);
    expect(start.title).toBe('Line lost');
    const retry = describeEvent(0x18, 1, 0x8000 | 2);
    expect(retry.title).toBe('Line search retry 2');
    expect(retry.detail).toContain('backed up and searching again');
    const stop = describeEvent(0x18, 1, 135);
    expect(stop.title).toBe('Line search stopped');
    expect(stop.detail).toContain('135 degree');
  });

  it('reads a line_lost p2 of exactly 0x8000 as a retry, not as a stop', () => {
    const text = describeEvent(0x18, 1, 0x8000);
    expect(text.title).toBe('Line search retry 0');
    expect(text.detail).toContain('backed up and searching again');
  });

  it('decodes the stall codes of the supervisor', () => {
    expect(describeEvent(0x48, 0x82, 0).detail).toContain('crosswise');
    expect(describeEvent(0x48, 1, 5).detail).toContain('line task');
  });

  it('reads a fault event by name', () => {
    expect(describeEvent(0x40, 11, 0).detail).toBe('Car lifted');
  });

  it('names an obstacle stop: the cause and the echo', () => {
    expect(describeEvent(0x24, 7, 2).detail).toBe('cause collision; echo too close');
    expect(describeEvent(0x24, 15, 2).detail).toBe('cause bypass gave up; echo too close');
  });

  it('names the long dark reading from barcode IR with its length', () => {
    const text = describeEvent(0x15, 120, 0);
    expect(text.title).toBe('Long dark barcode reading');
    expect(text.detail).toBe('Barcode IR saw a dark surface for 120 mm.');
    expect(text.kind).toBe('barcode');
  });

  it('tells the start of a line search from the stop at its end', () => {
    expect(describeEvent(0x18, 1, 0).detail).toBe('last seen on the left; the car turns on the spot to find it');
    const stop = describeEvent(0x18, 2, 135);
    expect(stop.title).toBe('Line search stopped');
    expect(stop.detail).toBe('no line in a 135 degree turn each way; last seen on the right');
  });

  it('falls back for a type the schema lacks, without inventing a name', () => {
    expect(eventName(0x77)).toBe('unknown');
    expect(describeEvent(0x77, 1, 2).title).toBe('Event 119');
  });
});

describe('scan', () => {
  it('unpacks the slot words', () => {
    const values: Record<string, number> = {};
    values.slots_18 = (1 << 15) | (0 << 12) | 245; // measured, valid, 245 mm
    values.slots_19 = (1 << 15) | (1 << 12) | 0; // measured, open
    values.slots_20 = (1 << 15) | (3 << 12); // measured, no echo
    const points = scanPoints(values);
    expect(points.find((p) => p.slot === 18)).toMatchObject({ angleDeg: 0, distMm: 245, mark: 'echo' });
    expect(points.find((p) => p.slot === 19)).toMatchObject({ angleDeg: 5, mark: 'open' });
    expect(points.find((p) => p.slot === 20)).toMatchObject({ mark: 'bad' });
  });

  it('puts a left angle to the left of the car', () => {
    const left = polarToXY(30, 100, 1, 0, 0);
    expect(left.x).toBeLessThan(0);
    expect(left.y).toBeLessThan(0);
    const ahead = polarToXY(0, 100, 1, 0, 0);
    expect(ahead.x).toBeCloseTo(0, 9);
    expect(ahead.y).toBeCloseTo(-100, 9);
  });

  it('lists range rings', () => {
    expect(rangeRings(400, 100)).toEqual([100, 200, 300, 400]);
  });
});

describe('telemetry store', () => {
  const previousBoot = () => [
    packet('heartbeat', { run_id: 7, uptime_s: 50 }, 1000),
    status(40, 2100, 0, 1100),
    packet('status', { seq: 41, odometer_mm: 2100, speed_mms: 0, faults: 1 << FAULT_BITS.estopLatched }, 1200),
    packet('motion', { left_mms: 0 }, 1201),
    packet('physics', { pitch_cdeg: 125 }, 1202),
    packet('obstacle', { dist_mm: 200 }, 1203),
    packet('profile', { closest_mm: 200 }, 1204),
    packet('diag', { sticky_faults: 1 << FAULT_BITS.estopLatched }, 1205),
  ].reduce(reduceStore, emptyStore());
  const nextHeartbeat = packet('heartbeat', { run_id: 8, uptime_s: 1 }, 1300);
  const bootEvent = event(1, 0x01, 0, 0, 0, 1300);

  it.each([nextHeartbeat, bootEvent])('clears previous-boot instrument readings on $packet', (restart) => {
    const before = previousBoot();
    expect(before.latest.status?.values.faults).toBe(1 << FAULT_BITS.estopLatched);
    expect(panelStatus('open', before.latest.status, restart.timestamp, 'status').kind).toBe('ready');
    expect(before.trail.map((sample) => sample.speedMms)).toEqual([160, 0]);

    const after = reduceStore(before, restart);
    expect(after.latest).toEqual({ [restart.packet]: restart });
    expect(panelStatus('open', after.latest.status, restart.timestamp, 'status').kind).toBe('loading');
    expect(after.trail.map((sample) => sample.speedMms)).toEqual([]);
    expect(after.trail).toEqual([]);
  });

  it('keeps latched status through backwards or duplicate sequences and waits for post-reboot status', () => {
    const currentStatus = packet('status', { seq: 1, odometer_mm: 0, speed_mms: 0, faults: 0 }, 1250);
    const initial = previousBoot();
    const delayed = reduceStore(initial, currentStatus);
    expect(delayed).toBe(initial);
    expect(reduceStore(delayed, status(41, 9000, 0, 1251))).toBe(initial);
    const rebooted = reduceStore(delayed, nextHeartbeat);
    expect(rebooted.latest).toEqual({ heartbeat: nextHeartbeat });
    const fresh = { ...currentStatus, timestamp: nextHeartbeat.timestamp + 1 };
    const after = reduceStore(rebooted, fresh);
    expect(after.latest).toEqual({ status: fresh, heartbeat: nextHeartbeat });
    expect(after.trail.map((sample) => sample.speedMms)).toEqual([0]);
    expect(after.trail.map((sample) => sample.odometerMm)).toEqual([0]);
  });

  it('keeps readings after a heartbeat and boot event received in the same millisecond', () => {
    const current = status(1, 5, 0, nextHeartbeat.timestamp);
    const after = [nextHeartbeat, current, bootEvent].reduce(reduceStore, previousBoot());

    expect(after.latest).toEqual({ heartbeat: nextHeartbeat, status: current, event: bootEvent });
    expect(after.trail.map((sample) => sample.speedMms)).toEqual([160]);
    expect(after.events.map((row) => row.name)).toEqual(['boot']);
  });

  it('coalesces a queued boot event with its reboot heartbeat and keeps intervening readings and events', () => {
    const statusAfterReset = status(1, 5, 0, 1320);
    const barcode = event(2, 0x10, 76, 1, 5, 1340);
    const delayedBoot = { ...bootEvent, timestamp: 1400 };
    const old = reduceStore(previousBoot(), event(1, 0x10, 82, 1, 30, 1250));
    const after = [nextHeartbeat, statusAfterReset, barcode, delayedBoot].reduce(reduceStore, old);
    expect(after.trailStart).toBe(nextHeartbeat.timestamp);
    expect(after.latest.status).toBe(statusAfterReset);
    expect(after.trail.map((sample) => sample.odometerMm)).toEqual([5]);
    expect(after.events.map(({ name }) => name)).toEqual(['barcode', 'boot']);
    const repeated = Array.from({ length: 525 }, (_, i) => event(i + 3, 0x12, 0, 0, 5, 1410 + i))
      .reduce(reduceStore, after);
    expect(repeated.events.some(({ name }) => name === 'boot')).toBe(false);
    expect(reduceStore(repeated, delayedBoot)).toBe(repeated);
    const nextBoot = { ...delayedBoot, timestamp: 4000 };
    expect(reduceStore(repeated, nextBoot).trail).toEqual([]);
  });

  it('accepts the next boot event after the first boot lost its startup event', () => {
    const rebooted = reduceStore(previousBoot(), nextHeartbeat);
    const running = reduceStore(rebooted, status(1400, 2480, 0, 281_300));
    const laterBoot = { ...bootEvent, timestamp: 282_000 };
    const restarted = reduceStore(running, laterBoot);
    expect(restarted.trailStart).toBe(laterBoot.timestamp);
    expect(restarted.trail).toEqual([]);
    expect(restarted.latest.status).toBeUndefined();
  });

  it('compares boot reset reasons when events arrive within the startup window', () => {
    const initial = [packet('heartbeat', { run_id: 8, uptime_s: 0, reset_reason: 1 }, 1300),
      status(1, 15, 0, 2600)].reduce(reduceStore, previousBoot());
    const selfTest = event(1, 0x01, 1, 0, 0, 3300);
    expect(reduceStore(initial, selfTest).trail).toBe(initial.trail);
    const differentReset = event(1, 0x01, 2, 0, 0, 3300);
    expect(reduceStore(initial, differentReset).trail).toEqual([]);
  });

  it('ignores earlier boot evidence replayed after current-boot readings', () => {
    const rebooted = reduceStore(previousBoot(), nextHeartbeat);
    const fresh = reduceStore(rebooted, status(1, 5, 0, 1320));
    const current = reduceStore(fresh, packet('heartbeat', { run_id: 8, uptime_s: 2 }, 1350));
    expect(reduceStore(current, packet('heartbeat', { run_id: 7, uptime_s: 50 }, 1299))).toBe(current);
    expect(reduceStore(current, packet('heartbeat', { run_id: 8, uptime_s: 1 }, 1340))).toBe(current);
    expect(reduceStore(current, event(1, 0x01, 0, 0, 0, 1299))).toBe(current);
    expect(current.trail.map((sample) => sample.odometerMm)).toEqual([5]);
  });

  it('counts status frames lost from gaps in seq', () => {
    let store = emptyStore();
    for (const seq of [1, 2, 3, 6, 7]) store = reduceStore(store, status(seq, seq * 30, 0, seq * 200));
    expect(store.statusLost).toBe(2);
    expect(statusLossPercent(store)).toBeCloseTo((2 / 7) * 100);
  });

  it('wraps seq and ignores a reboot-sized jump', () => {
    let store = emptyStore();
    for (const seq of [65534, 65535, 0, 1]) store = reduceStore(store, status(seq, 100, 0, seq));
    expect(store.statusLost).toBe(0);
    store = reduceStore(store, status(30000, 130, 0, 9));
    expect(store.statusLost).toBe(0);
  });

  it('keeps the full trail across a discontinuous odometer drop', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 500, 0, 1));
    store = reduceStore(store, status(2, 530, 0, 2));
    store = reduceStore(store, status(3, 10, 0, 3));
    expect(store.trail.map((sample) => sample.odometerMm)).toEqual([500, 530, 10]);
    expect(store.trailStart).toBeNull();
  });

  it('keeps the trail and the box through a 150 mm line-search reverse', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 300, 0, 2));
    store = reduceStore(store, obstacle(1, 100, 50, 50, 2));
    store = reduceStore(store, status(3, 150, 0, 3));
    expect(store.trail.map((point) => point.odometerMm)).toEqual([0, 300, 150]);
    expect(store.obstacle).not.toBeNull();
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.points[1]?.y).toBeCloseTo(300, 5);
    expect(course.points[2]?.y).toBeCloseTo(150, 5);
    expect(course.points[2]?.x).toBeCloseTo(0, 5);
    expect(course.obstacle?.y).toBeCloseTo(400, 5);
  });

  it('classifies abnormal odometer changes without erasing the trail', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 1000, 0, 1));
    expect(reduceStore(store, status(2, 501, 0, 2)).trail).toHaveLength(2);
    expect(reduceStore(store, status(2, 499, 0, 2)).trail).toHaveLength(2);
    expect(odometerDiscontinuous(1000, 500)).toBe(false);
    expect(odometerDiscontinuous(1000, 499)).toBe(true);
    expect(odometerDiscontinuous(1000, 6000)).toBe(false);
    expect(odometerDiscontinuous(1000, 6001)).toBe(true);
  });

  it('restarts the trail and clears the box when the heartbeat run id changes', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 100, 0, 2));
    store = reduceStore(store, status(2, 150, 0, 3));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 3));
    expect(store.obstacle).not.toBeNull();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 51, run_id: 7 }, 4));
    expect(store.trail).toHaveLength(2);
    expect(store.obstacle).not.toBeNull();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 52, run_id: 9 }, 5));
    expect(store.trail).toHaveLength(0);
    expect(store.obstacle).toBeNull();
  });

  it('restarts the trail when the heartbeat uptime falls', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 100, 0, 2));
    store = reduceStore(store, status(2, 150, 0, 3));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 3));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 3, run_id: 7 }, 4));
    expect(store.trail).toHaveLength(0);
    expect(store.obstacle).toBeNull();
  });

  it('keeps the route when the firmware millisecond clock wraps with the same boot ID', () => {
    const before = [packet('heartbeat', { uptime_s: 4_294_966, run_id: 9 }, 1000),
      status(100, 10, 0, 1200)].reduce(reduceStore, emptyStore());
    const wrapped = reduceStore(before, packet('heartbeat', { uptime_s: 0, run_id: 9 }, 2400));
    expect(wrapped.trail).toBe(before.trail);
    expect(wrapped.trailStart).toBeNull();
    const rebooted = reduceStore(before, packet('heartbeat', { uptime_s: 0, run_id: 10 }, 2400));
    expect(rebooted.trail).toEqual([]);
  });

  it('keeps the trail on the first heartbeat and on one with the same uptime', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 100, 0, 1));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 2));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 3));
    expect(store.trail).toHaveLength(1);
  });

  it('keeps the trail through the small odometer drop of a pivot', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 100, 0, 1));
    store = reduceStore(store, status(2, 96, 40, 2));
    expect(store.trail).toHaveLength(2);
  });

  it('starts a new trail on a boot event', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 300, 0, 2));
    store = reduceStore(store, event(1, 0x01, 0, 0, 0, 3));
    expect(store.trail).toHaveLength(0);
    store = reduceStore(store, status(3, 5, 0, 4));
    expect(store.trail).toHaveLength(1);
  });

  it('keeps events once and in order, and a boot starts over', () => {
    let store = emptyStore();
    store = reduceStore(store, event(1, 0x12, 1, 0, 100, 10));
    store = reduceStore(store, event(1, 0x12, 1, 0, 100, 10));
    store = reduceStore(store, event(2, 0x13, 1, 900, 220, 20));
    expect(store.events.map((e) => e.name)).toEqual(['turnStart', 'turnDone']);
    store = reduceStore(store, event(1, 0x01, 0, 0, 0, 30));
    expect(store.events.map((e) => e.name)).toEqual(['boot']);
  });

  it('keeps the newest packet of each kind', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 5 }));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 6 }));
    expect(store.latest.heartbeat?.values.uptime_s).toBe(6);
  });
});

describe('course', () => {
  it('dead-reckons a square and puts an event at its odometer', () => {
    let store = emptyStore();
    let odo = 0;
    let seq = 0;
    store = reduceStore(store, status(seq, odo, 0, 0));
    for (const heading of [0, 90, 180, 270]) {
      for (let i = 0; i < 10; i += 1) {
        odo += 100;
        seq += 1;
        store = reduceStore(store, status(seq, odo, heading, seq * 200));
      }
    }
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 500, 5000));
    const course = buildCourse(store.trail, store.events);
    expect(course.points).toHaveLength(41);
    const end = course.points[course.points.length - 1];
    expect(end?.x).toBeCloseTo(0, 5);
    expect(end?.y).toBeCloseTo(0, 5);
    expect(course.bounds.maxY).toBeCloseTo(1000, 5);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.kind).toBe('barcode');
  });

  it('leaves travel unlocated when both heading and wheel turn are missing', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 90, 1));
    store = reduceStore(store, packet('status', { seq: 2, odometer_mm: 100, heading_ddeg: 0xffff, speed_mms: 0 }, 2));
    const course = buildCourse(store.trail, store.events);
    expect(course.points).toEqual([{ x: 0, y: 0 }, { x: 0, y: 0 }]);
    expect(course).toMatchObject({ hasGap: true, positionKnown: false });
  });
});

describe('course while the odometer has fallen', () => {
  it('draws the mark and the box at 300 mm after a 150 mm reverse', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 300, 0, 2));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 300, 2));
    store = reduceStore(store, obstacle(1, 100, 50, 50, 2));
    store = reduceStore(store, status(3, 150, 0, 3));
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.y).toBeCloseTo(300, 5);
    expect(course.obstacle?.y).toBeCloseTo(400, 5);
  });

  it('puts a mark on the pass it came in, not on the first pass through the same odometer', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 1000, 0, 2));
    store = reduceStore(store, status(3, 750, 0, 3));
    store = reduceStore(store, status(4, 750, 90, 4));
    store = reduceStore(store, status(5, 1000, 90, 5));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 900, 4.5));
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.x).toBeCloseTo(150, 5);
    expect(course.markers[0]?.y).toBeCloseTo(750, 5);
  });

  it('places the box on the pass of its stamp', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 1000, 0, 2));
    store = reduceStore(store, status(3, 750, 0, 3));
    store = reduceStore(store, status(4, 750, 90, 4));
    store = reduceStore(store, status(5, 900, 90, 5));
    store = reduceStore(store, obstacle(1, 100, 50, 50, 5));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(store.obstacle?.sample.timestamp).toBe(5);
    expect(box?.x).toBeCloseTo(150 + 100, 5);
    expect(box?.y).toBeCloseTo(750, 5);
  });

  it('places an event older than the trail on the first segment', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 100));
    store = reduceStore(store, status(2, 400, 0, 200));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 100, 10));
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.markers[0]?.y).toBeCloseTo(100, 5);
  });

  it('places an event older than the trail on the first pass through its odometer', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 100));
    store = reduceStore(store, status(2, 1000, 0, 200));
    store = reduceStore(store, status(3, 750, 0, 300));
    store = reduceStore(store, status(4, 1000, 90, 400));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 900, 10));
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.x).toBeCloseTo(0, 5);
    expect(course.markers[0]?.y).toBeCloseTo(900, 5);
  });

  it('places an older event outside the first-to-last odometer on the segment that spans it', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 500, 0, 100));
    store = reduceStore(store, status(2, 100, 0, 200));
    store = reduceStore(store, status(3, 300, 0, 300));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 200, 10));
    store = reduceStore(store, event(2, 0x10, 'S'.charCodeAt(0), 1, 600, 10));
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.y).toBeCloseTo(-300, 5);
  });

  it('places the box on the newest of status points with the same timestamp', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 5));
    store = reduceStore(store, status(3, 200, 0, 5));
    store = reduceStore(store, obstacle(1, 100, 50, 50, 5));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(box?.y).toBeCloseTo(300, 5);
  });

  it('keeps the box at its recorded pose when another status with the same timestamp arrives', () => {
    const store = [status(1, 0, 0, 900), status(2, 100, 0, 1000), obstacle(1, 100, 50, 50, 1000),
      status(3, 250, 0, 1000)].reduce(reduceStore, emptyStore());

    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle?.y).toBeCloseTo(200);
  });

  it('retains the exact obstacle position when old trail readings are compacted', () => {
    let store = emptyStore();
    store = reduceStore(store, status(0, 10, 0, 1));
    store = reduceStore(store, obstacle(1, 100, 50, 50, 1));
    const original = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(original).not.toBeNull();
    for (let i = 1; i <= TRAIL_MAX; i += 1) store = reduceStore(store, status(i, 10, 0, i));
    expect(store.trail[0]?.timestamp).toBe(1);
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toEqual(original);
  });

  it('draws no mark on the new trail for an event of the run before a heartbeat restart', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 100, 0, 2));
    store = reduceStore(store, status(2, 200, 0, 3));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 150, 3));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 2, run_id: 9 }, 5));
    store = reduceStore(store, status(3, 100, 0, 6));
    store = reduceStore(store, status(4, 200, 0, 7));
    store = reduceStore(store, event(2, 0x10, 'S'.charCodeAt(0), 1, 150, 7));
    expect(store.events).toHaveLength(2);
    const course = buildCourse(store.trail, store.events, store.obstacle, store.trailStart);
    expect(course.markers.map((marker) => marker.eventId)).toEqual([2]);
  });

  it('clears the old trail and starts locating events again when a heartbeat confirms a reboot', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 2000, 0, 2));
    store = reduceStore(store, status(2, 2100, 0, 3));
    store = reduceStore(store, status(3, 0, 0, 6));
    store = reduceStore(store, status(4, 100, 0, 7));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 50, 7));
    expect(store.trail.map((point) => point.odometerMm)).toEqual([2000, 2100, 0, 100]);
    store = reduceStore(store, packet('heartbeat', { uptime_s: 2, run_id: 9 }, 8));
    expect(store.trail).toHaveLength(0);
    store = reduceStore(store, status(0, 0, 0, 9));
    store = reduceStore(store, status(1, 100, 0, 10));
    store = reduceStore(store, event(2, 0x10, 'S'.charCodeAt(0), 1, 50, 10));
    expect(store.trail).toHaveLength(2);
    const course = buildCourse(store.trail, store.events, store.obstacle, store.trailStart);
    expect(course.markers.map((marker) => marker.eventId)).toEqual([2]);
  });

  it('keeps the trail across a same-boot heartbeat confirming an odometer discontinuity', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 2000, 0, 2));
    store = reduceStore(store, status(2, 0, 0, 3));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 51, run_id: 7 }, 4));
    expect(store.trail.map((point) => point.odometerMm)).toEqual([2000, 0]);
    store = reduceStore(store, packet('heartbeat', { uptime_s: 2, run_id: 9 }, 5));
    expect(store.trail).toHaveLength(0);
  });

  it('keeps an earlier marker at its original position across an odometer discontinuity', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 1000, 0, 2));
    store = reduceStore(store, status(2, 1100, 0, 3));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 1050, 3));
    store = reduceStore(store, status(3, 0, 0, 6));
    store = reduceStore(store, status(4, 100, 0, 7));
    expect(store.events).toHaveLength(1);
    const course = buildCourse(store.trail, store.events, store.obstacle, store.trailStart);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.y).toBeCloseTo(50);
  });
});

const obstacle = (hugState: number, faceMm: number, widthMm: number, breadthMm: number, timestamp: number) =>
  packet('obstacle', { hug_state: hugState, hug_side: 0, face_mm: faceMm, width_mm: widthMm, breadth_mm: breadthMm }, timestamp);

describe('obstacle box', () => {
  it.each([999, 1201, 5000])('does not use a future or stale contact pose at %s ms', (timestamp) => {
    const store = [status(1, 0, 0, 1000), obstacle(1, 120, 50, 50, timestamp)].reduce(reduceStore, emptyStore());

    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
    expect(store.latest.obstacle?.values.face_mm).toBe(120);
  });

  it('uses the fresh preceding pose even if a later pose arrived first', () => {
    const store = [status(1, 0, 0, 1000), status(2, 100, 0, 1250), obstacle(1, 120, 50, 50, 1200)]
      .reduce(reduceStore, emptyStore());

    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle?.y).toBeCloseTo(120);
  });

  it('does not restamp an unlocated first face at a later pose', () => {
    const initial = [status(1, 0, 0, 1000), obstacle(1, 0, 0, 0, 1000), obstacle(3, 120, 50, 50, 5000)]
      .reduce(reduceStore, emptyStore());
    const later = [status(2, 150, 90, 5100), obstacle(4, 120, 50, 50, 5200)].reduce(reduceStore, initial);

    expect(buildCourse(initial.trail, initial.events, initial.obstacle).obstacle).toBeNull();
    expect(buildCourse(later.trail, later.events, later.obstacle).obstacle).toBeNull();
  });

  it('places a single front contact along the reported heading', () => {
    const contact = placeContact({ x: 10, y: 20, headingDeg: 90 }, 200);
    expect(contact.x).toBeCloseTo(210);
    expect(contact.y).toBeCloseTo(20);
  });

  it('fixes the pose when the hug starts and keeps the box after it ends', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(0, 0, 0, 0, 2));
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
    store = reduceStore(store, obstacle(1, 120, 0, 0, 2));
    store = reduceStore(store, status(3, 200, 90, 3));
    store = reduceStore(store, obstacle(4, 80, 90, 0, 3));
    store = reduceStore(store, obstacle(0, 0, 0, 0, 3));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // Pose is (0, 100) heading north when the hug left idle; the later frames do not move it.
    expect(box?.y).toBeCloseTo(220, 5);
    expect(box?.x).toBeCloseTo(0, 5);
  });

  it('keeps the box on an equal or discontinuous odometer and clears it on a boot event', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 1));
    expect(store.obstacle).not.toBeNull();
    const sameOdometer = reduceStore(store, status(2, 0, 0, 2));
    expect(sameOdometer.obstacle).not.toBeNull();
    expect(reduceStore(store, status(2, 9000, 0, 2)).obstacle).toBe(store.obstacle);
    expect(reduceStore(store, event(1, 0x01)).obstacle).toBeNull();
  });

  it('places the box from the first face reading when the hug backs off first and the odometer falls', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 200, 0, 2));
    store = reduceStore(store, obstacle(0, 0, 0, 0, 2));
    store = reduceStore(store, obstacle(1, 0, 0, 0, 2));
    store = reduceStore(store, status(3, 160, 0, 3));
    store = reduceStore(store, obstacle(2, 0, 0, 0, 3));
    store = reduceStore(store, status(4, 120, 0, 4));
    store = reduceStore(store, obstacle(1, 0, 0, 0, 4));
    store = reduceStore(store, status(5, 120, 0, 5));
    store = reduceStore(store, obstacle(3, 80, 0, 0, 5));
    expect(store.trail).toHaveLength(5);
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // Pose (0, 120) heading north when the face first showed, not (0, 200) at the first contact.
    expect(box?.y).toBeCloseTo(200, 5);
    expect(box?.x).toBeCloseTo(0, 5);
  });

  it('keeps the heading of the contact when the face first shows after the car has turned', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(0, 0, 0, 0, 2));
    store = reduceStore(store, obstacle(1, 0, 0, 0, 2));
    store = reduceStore(store, status(3, 104, 50, 3));
    store = reduceStore(store, obstacle(3, 90, 0, 0, 3));
    expect(store.obstacle).toMatchObject({ sample: { odometerMm: 104 }, headingDeg: 0, faceMm: 90 });
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // The last 4 mm follows the short arc while the box retains the north-facing contact heading.
    const halfTurn = (25 * Math.PI) / 180;
    const chord = 4 * Math.sin(halfTurn) / halfTurn;
    const pose = { x: chord * Math.sin(halfTurn), y: 100 + chord * Math.cos(halfTurn) };
    expect(box?.y).toBeCloseTo(pose.y + 90, 5);
    expect(box?.x).toBeCloseTo(pose.x, 5);
  });

  it('gives no box for a first obstacle frame in a leg after the trail has points', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(4, 120, 50, 50, 2));
    expect(store.obstacle).toBeNull();
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
  });

  it('freezes the face at its first reading while the hug runs', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(0, 0, 0, 0, 1));
    store = reduceStore(store, obstacle(1, 0, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(3, 80, 0, 0, 2));
    store = reduceStore(store, status(3, 300, 0, 3));
    store = reduceStore(store, obstacle(5, 200, 60, 0, 3));
    expect(store.obstacle).toMatchObject({ sample: { odometerMm: 100 }, faceMm: 80, widthMm: 60 });
  });

  it('draws no box while the face is 0', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 0, 40, 0, 2));
    const contact = store.obstacle;
    expect(contact?.faceMm).toBe(0);
    expect(buildCourse(store.trail, store.events, contact).obstacle).toBeNull();
    store = reduceStore(store, status(3, 200, 0, 3));
    store = reduceStore(store, obstacle(5, 90, 40, 0, 3));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(box?.x).toBeCloseTo(0, 5);
    expect(box?.y).toBeCloseTo(290, 5);
    expect(buildCourse(store.trail, [], contact).obstacle).toBeNull();
  });

  it('starts a new box when a hug returns to contact and the idle frame between was lost', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 2));
    store = reduceStore(store, status(3, 300, 0, 3));
    store = reduceStore(store, obstacle(8, 120, 50, 50, 3));
    const first = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(first?.x).toBeCloseTo(0, 5);
    expect(first?.y).toBeCloseTo(220, 5);
    store = reduceStore(store, status(4, 500, 90, 4));
    store = reduceStore(store, obstacle(1, 200, 60, 70, 4));
    const second = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // Pose (200, 300) heading east: the second hug has its own pose, face and sizes.
    expect(second?.x).toBeCloseTo(400, 5);
    expect(second?.y).toBeCloseTo(300, 5);
  });

  it('keeps one pose across repeated contact frames of the same hug', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 0, 0, 2));
    store = reduceStore(store, status(3, 200, 90, 3));
    store = reduceStore(store, obstacle(1, 120, 50, 0, 3));
    expect(store.obstacle).toMatchObject({ sample: { odometerMm: 100 }, headingDeg: 0, faceMm: 120, widthMm: 50 });
  });

  it('gives no box for a hug whose frames come before the first trail point', () => {
    let store = emptyStore();
    store = reduceStore(store, obstacle(1, 120, 50, 50, 0));
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(4, 120, 50, 50, 1));
    expect(store.obstacle).toBeNull();
    store = reduceStore(store, obstacle(0, 0, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 2));
    expect(store.obstacle).toMatchObject({ sample: { odometerMm: 100 }, faceMm: 120 });
  });

  it('gives no box for the rest of a hug interrupted by a reboot', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 500, 0, 1));
    store = reduceStore(store, status(2, 600, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 2));
    expect(store.obstacle).not.toBeNull();
    store = reduceStore(store, event(1, 0x01, 0, 0, 0, 3));
    store = reduceStore(store, status(0, 10, 0, 3));
    expect(store.obstacle).toBeNull();
    store = reduceStore(store, obstacle(4, 120, 50, 50, 3));
    expect(store.obstacle).toBeNull();
  });

  it('uses the last valid heading when the heading is invalid at the stamp', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 90, 1));
    store = reduceStore(store, packet('status', { seq: 2, odometer_mm: 100, heading_ddeg: 0xffff, speed_mms: 0 }, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50, 2));
    expect(store.obstacle?.headingDeg).toBe(90);
  });

  it('keeps the same stamp when a hug frame repeats the face and the sizes', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(0, 0, 0, 0, 1));
    store = reduceStore(store, obstacle(4, 120, 50, 50, 1));
    const stamp = store.obstacle;
    expect(stamp).not.toBeNull();
    expect(reduceStore(store, obstacle(4, 120, 50, 50, 1)).obstacle).toBe(stamp);
    expect(reduceStore(store, obstacle(5, 120, 0, 0, 1)).obstacle).toBe(stamp);
    expect(reduceStore(store, obstacle(5, 120, 70, 50, 1)).obstacle).not.toBe(stamp);
  });

  it('wraps the contact bearing and saturates the reported range', () => {
    const pose = { x: 5, y: 7, headingDeg: 350 };
    const west = placeContact(pose, 200);
    const wrapped = placeContact({ ...pose, headingDeg: -10 }, 200);
    expect(west.x).toBeCloseTo(wrapped.x);
    expect(west.y).toBeCloseTo(wrapped.y);
    expect(placeContact({ x: 0, y: 0, headingDeg: 0 }, 60000).y).toBe(4095);
    const sample = { timestamp: 1, odometerMm: 50, headingDeg: 0, speedMms: 0 };
    const invalidFace = { sample, headingDeg: 0, faceMm: Infinity, widthMm: 50, breadthMm: 50 };
    expect(buildCourse([sample], [], invalidFace).obstacle).toBeNull();
  });
});

describe('run summary', () => {
  it('does not include a previous turn drained in the same bridge batch as START', () => {
    const store = [event(1, 0x13, 1, 900, 0, 100), event(2, 0x02, 0, 0, 0, 100),
      event(3, 0x13, 1, 900, 0, 100)].reduce(reduceStore, emptyStore());
    expect(summarizeRun([], store.events)).toMatchObject({ turns: 1, hasTravel: false });
  });

  it('labels late joins as recorded travel, counts reverse distance and discards earlier boot events', () => {
    const store = [event(1, 0x02, 0, 0, 0, 100), event(2, 0x13, 1, 900, 0, 300),
      status(0, 100, 0, 1000), status(1, 300, 0, 2000), status(2, 200, 0, 3000)]
      .reduce(reduceStore, emptyStore());
    expect(summarizeRun(store.trail, store.events, 1000)).toMatchObject({
      scope: 'Recorded', turns: 0, distanceMm: 300, hasTravel: true,
    });
    expect(summarizeRun([store.trail[0]!], store.events, 1000).hasTravel).toBe(false);
    expect(summarizeRun(store.trail, store.events).scope).toBe('This run');
  });

  it('finds the longest recorded line recovery from events', () => {
    const store = [event(1, 0x18, 0, 0, 200, 1000), event(2, 0x19, 0, 75, 275, 1100),
      event(3, 0x18, 0, 0x8001, 800, 1200), event(4, 0x19, 0, 120, 920, 1300)]
      .reduce(reduceStore, emptyStore());
    expect(longestLineRecovery(store.events)).toBe(120);
    expect(longestLineRecovery([])).toBeNull();
  });

  it('summarises the run from the last run_start', () => {
    let store = emptyStore();
    store = reduceStore(store, event(1, 0x02, 0, 0, 0, 1000));
    for (let i = 0; i < 6; i += 1) store = reduceStore(store, status(i, i * 160, 0, 1000 + i * 1000));
    store = reduceStore(store, event(2, 0x13, 1, 900, 300, 3000));
    store = reduceStore(store, event(3, 0x10, 'L'.charCodeAt(0), 2, 400, 4000));
    store = reduceStore(store, event(4, 0x11, 5, 0, 410, 4100));
    store = reduceStore(store, event(5, 0x31, 55, 1, 700, 5000));
    store = reduceStore(store, event(6, 0x40, 4, 0, 710, 5100));
    const summary = summarizeRun(store.trail, store.events);
    expect(summary.distanceMm).toBe(800);
    expect(summary.durationS).toBe(5);
    expect(summary.averageMms).toBe(160);
    expect(summary).toMatchObject({ turns: 1, barcodesOk: 1, barcodesNoAction: 0, barcodesBad: 1, humps: 1, tallestHumpMm: 55, faults: 1, faultBits: [4] });
  });

  it('counts a decoded no-action letter separately from driving actions, read issues and dark blocks', () => {
    const store = [
      event(1, 0x10, 'L'.charCodeAt(0), 2, 200, 1000),
      event(2, 0x10, 'A'.charCodeAt(0), 5, 300, 2000),
      event(3, 0x11, 1, 0, 350, 3000),
      event(4, 0x15, 120, 0, 400, 4000),
    ].reduce(reduceStore, emptyStore());
    expect(summarizeRun(store.trail, store.events)).toMatchObject({ barcodesOk: 2, barcodesNoAction: 1, barcodesBad: 1 });
  });

  it('counts a line loss once when the search also stops at its limit', () => {
    let store = emptyStore();
    store = reduceStore(store, event(1, 0x02, 0, 0, 0, 1000));
    store = reduceStore(store, event(2, 0x18, 1, 0, 100, 2000));
    store = reduceStore(store, event(3, 0x18, 1, 1000, 1000, 9000));
    expect(summarizeRun(store.trail, store.events).lineLosses).toBe(1);
  });

  it('counts a line loss once when the search retries before it stops', () => {
    let store = emptyStore();
    store = reduceStore(store, event(1, 0x02, 0, 0, 0, 1000));
    store = reduceStore(store, event(2, 0x18, 1, 0, 100, 2000));
    store = reduceStore(store, event(3, 0x18, 1, 0x8000 | 1, 120, 4000));
    store = reduceStore(store, event(4, 0x18, 1, 0x8000 | 2, 140, 6000));
    store = reduceStore(store, event(5, 0x18, 1, 135, 160, 9000));
    expect(summarizeRun(store.trail, store.events).lineLosses).toBe(1);
  });
});

it('starts a clean dashboard session only on an onboard accepted reset, keeping the vehicle identity', () => {
  const command = (timestamp: number, type: number, opcode: number, id = 40) => ({
    packet: 'event', timestamp, values: { event_type: type, event_id: id, p1: 256 + opcode, source: 9, p2: 30 }});
  const before = [packet('heartbeat', {run_id: 42, uptime_s: 20}, 1000),
    packet('status', {seq: 2, odometer_mm: 350}, 1100), command(1150, 4, 0, 38),
    command(1160, 5, 2, 39)].reduce(reduceStore, emptyStore());
  expect(before.trail).toHaveLength(1);
  const reset = command(1200, 4, 2);
  const after = reduceStore(before, reset);
  expect(after.trail).toEqual([]);
  expect(after.events.map(e => e.eventId)).toEqual([40]);
  expect(after.latest.heartbeat?.values.run_id).toBe(42);
  expect(after.latest.status).toBeUndefined();
  const live = reduceStore(after, packet('status', {seq: 3, odometer_mm: 355}, 1250));
  expect(reduceStore(live, reset).trail).toEqual(live.trail);
  expect(reduceStore(live, packet('motion', {left_mms: 80}, 1199)).latest.motion).toBeUndefined();
});
