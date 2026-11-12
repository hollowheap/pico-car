import { describe, expect, it } from 'vitest';

import { buildCourse } from './course';
import { describeEvent, eventName } from './events';
import { FAULT_META, faultTiles, summarizeFaults } from './faults';
import type { LinkPacket } from './link';
import { summarizeRun } from './run-summary';
import { polarToXY, rangeRings, scanPoints } from './scan';
import { FAULT_BITS } from './schema.generated';
import { emptyStore, reduceStore, statusLossPercent } from './telemetry-store';

const packet = (name: string, values: Record<string, number>, timestamp = 1000): LinkPacket => ({
  timestamp,
  packet: name,
  values,
});
const status = (seq: number, odo: number, headingDeg: number, timestamp: number) =>
  packet('status', { seq, odometer_mm: odo, heading_ddeg: headingDeg * 10, speed_mms: 160 }, timestamp);
const event = (id: number, type: number, p1 = 0, p2 = 0, odo = 0, timestamp = 1000) =>
  packet('event', { event_id: id, event_type: type, source: 2, p1, p2, odometer_mm: odo }, timestamp);

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

  it('decodes a refused command with its source and reason', () => {
    const text = describeEvent(0x05, 3, (1 << 8) | 2);
    expect(text.title).toBe('Command refused');
    expect(text.detail).toBe('left from telemetry: busy');
  });

  it('decodes the stall codes of the supervisor', () => {
    expect(describeEvent(0x48, 0x82, 0).detail).toContain('crosswise');
    expect(describeEvent(0x48, 1, 5).detail).toContain('line task');
  });

  it('reads a fault event by name', () => {
    expect(describeEvent(0x40, 11, 0).detail).toBe('Car lifted');
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

  it('starts a new trail when the odometer goes backwards', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 500, 0, 1));
    store = reduceStore(store, status(2, 530, 0, 2));
    store = reduceStore(store, status(3, 10, 0, 3));
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

  it('holds the last heading while it is invalid', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 90, 1));
    store = reduceStore(store, packet('status', { seq: 2, odometer_mm: 100, heading_ddeg: 0xffff, speed_mms: 0 }, 2));
    const course = buildCourse(store.trail, store.events);
    expect(course.points[1]?.x).toBeCloseTo(100, 5);
  });
});

describe('run summary', () => {
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
    expect(summary).toMatchObject({ turns: 1, barcodesOk: 1, barcodesRefused: 1, humps: 1, tallestHumpMm: 55, faults: 1, faultBits: [4] });
  });
});
