import { describe, expect, it } from 'vitest';

import { buildCourse, obstacleLabel, OBSTACLE_PLACEHOLDER_MM, placeObstacle } from './course';
import { describeEvent, eventName } from './events';
import { FAULT_META, faultTiles, summarizeFaults } from './faults';
import type { LinkPacket } from './link';
import { summarizeRun } from './run-summary';
import { polarToXY, rangeRings, scanPoints } from './scan';
import { FAULT_BITS } from './schema.generated';
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

  it('decodes a refused command with its source, reason and id', () => {
    const text = describeEvent(0x05, (2 << 12) | (1 << 8) | 3, 77);
    expect(text.title).toBe('Command refused');
    expect(text.detail).toBe('left from telemetry: busy (command 77)');
    expect(describeEvent(0x05, (8 << 12) | (1 << 8) | 3, 77).detail).toContain(': stalled');
    expect(describeEvent(0x05, (9 << 12) | (1 << 8) | 3, 77).detail).toContain(': lean unknown');
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

  it('decodes an accepted command; a barcode command has no id', () => {
    expect(describeEvent(0x04, (1 << 8) | 1, 5).detail).toBe('start from telemetry (command 5)');
    expect(describeEvent(0x04, (2 << 8) | 3, 0).detail).toBe('left from barcode');
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

  it('names a block on the track with its length', () => {
    const text = describeEvent(0x15, 120, 0);
    expect(text.title).toBe('Block on the track');
    expect(text.detail).toBe('black for 120 mm; not a barcode');
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

  it('starts a new trail when the odometer drops by more than 500 mm', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 500, 0, 1));
    store = reduceStore(store, status(2, 530, 0, 2));
    store = reduceStore(store, status(3, 10, 0, 3));
    expect(store.trail).toHaveLength(1);
  });

  it('keeps the trail and the box through a 150 mm line-search reverse', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 300, 0, 2));
    store = reduceStore(store, obstacle(1, 100, 50, 50));
    store = reduceStore(store, status(3, 150, 0, 3));
    expect(store.trail.map((point) => point.odometerMm)).toEqual([0, 300, 150]);
    expect(store.obstacle).not.toBeNull();
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.points[1]?.y).toBeCloseTo(300, 5);
    expect(course.points[2]?.y).toBeCloseTo(150, 5);
    expect(course.points[2]?.x).toBeCloseTo(0, 5);
    expect(course.obstacle?.corners[0]?.y).toBeCloseTo(400, 5);
  });

  it('keeps a drop of 499 mm and restarts on a drop of 501 mm', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 1000, 0, 1));
    expect(reduceStore(store, status(2, 501, 0, 2)).trail).toHaveLength(2);
    expect(reduceStore(store, status(2, 499, 0, 2)).trail).toHaveLength(1);
  });

  it('restarts the trail and clears the box when the heartbeat run id changes', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 100, 0, 2));
    store = reduceStore(store, status(2, 150, 0, 3));
    store = reduceStore(store, obstacle(1, 120, 50, 50));
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
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 3, run_id: 7 }, 4));
    expect(store.trail).toHaveLength(0);
    expect(store.obstacle).toBeNull();
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

  it('holds the last heading while it is invalid', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 90, 1));
    store = reduceStore(store, packet('status', { seq: 2, odometer_mm: 100, heading_ddeg: 0xffff, speed_mms: 0 }, 2));
    const course = buildCourse(store.trail, store.events);
    expect(course.points[1]?.x).toBeCloseTo(100, 5);
  });
});

describe('course while the odometer has fallen', () => {
  it('draws the mark and the box at 300 mm after a 150 mm reverse', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 300, 0, 2));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 300, 2));
    store = reduceStore(store, obstacle(1, 100, 50, 50));
    store = reduceStore(store, status(3, 150, 0, 3));
    const course = buildCourse(store.trail, store.events, store.obstacle);
    expect(course.markers).toHaveLength(1);
    expect(course.markers[0]?.y).toBeCloseTo(300, 5);
    expect(course.obstacle?.corners[0]?.y).toBeCloseTo(400, 5);
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
    store = reduceStore(store, obstacle(1, 100, 50, 50));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(store.obstacle?.timestamp).toBe(5);
    expect(box?.corners[0]?.x).toBeCloseTo(150 + 100, 5);
    expect(box?.corners[0]?.y).toBeCloseTo(750 + 25, 5);
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
    store = reduceStore(store, obstacle(1, 100, 50, 50));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(box?.corners[0]?.y).toBeCloseTo(300, 5);
  });

  it('gives no box when the trail cap has evicted the point of its stamp', () => {
    let store = emptyStore();
    store = reduceStore(store, status(0, 10, 0, 1));
    store = reduceStore(store, obstacle(1, 100, 50, 50));
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).not.toBeNull();
    for (let i = 1; i <= TRAIL_MAX; i += 1) store = reduceStore(store, status(i, 10, 0, 1 + i));
    expect(store.trail[0]?.timestamp).toBeGreaterThan(1);
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
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

  it('keeps the new trail and its marks when the first heartbeat of the run follows an odometer restart', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 2000, 0, 2));
    store = reduceStore(store, status(2, 2100, 0, 3));
    store = reduceStore(store, status(3, 0, 0, 6));
    store = reduceStore(store, status(4, 100, 0, 7));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 50, 7));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 2, run_id: 9 }, 8));
    expect(store.trail).toHaveLength(2);
    const course = buildCourse(store.trail, store.events, store.obstacle, store.trailStart);
    expect(course.markers).toHaveLength(1);
  });

  it('still restarts on a heartbeat after an earlier odometer restart of the same run', () => {
    let store = emptyStore();
    store = reduceStore(store, packet('heartbeat', { uptime_s: 50, run_id: 7 }, 1));
    store = reduceStore(store, status(1, 2000, 0, 2));
    store = reduceStore(store, status(2, 0, 0, 3));
    store = reduceStore(store, packet('heartbeat', { uptime_s: 51, run_id: 7 }, 4));
    expect(store.trail).toHaveLength(1);
    store = reduceStore(store, packet('heartbeat', { uptime_s: 2, run_id: 9 }, 5));
    expect(store.trail).toHaveLength(0);
  });

  it('draws no mark on the new trail for an event of the run before an odometer restart', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 1000, 0, 2));
    store = reduceStore(store, status(2, 1100, 0, 3));
    store = reduceStore(store, event(1, 0x10, 'S'.charCodeAt(0), 1, 50, 3));
    store = reduceStore(store, status(3, 0, 0, 6));
    store = reduceStore(store, status(4, 100, 0, 7));
    expect(store.events).toHaveLength(1);
    const course = buildCourse(store.trail, store.events, store.obstacle, store.trailStart);
    expect(course.markers).toHaveLength(0);
  });
});

const obstacle = (hugState: number, faceMm: number, widthMm: number, breadthMm: number) =>
  packet('obstacle', { hug_state: hugState, hug_side: 0, face_mm: faceMm, width_mm: widthMm, breadth_mm: breadthMm });

describe('obstacle box', () => {
  it('puts the near face the face distance ahead of the pose, along the heading', () => {
    const box = placeObstacle({ x: 10, y: 20, headingDeg: 90 }, 200, 100, 60);
    // East: ahead is +x, the car's right is -y. Order: near left, near right, far right, far left.
    const expected = [[210, 70], [210, -30], [270, -30], [270, 70]];
    box.corners.forEach((c, i) => {
      expect(c.x).toBeCloseTo(expected[i]?.[0] ?? NaN, 5);
      expect(c.y).toBeCloseTo(expected[i]?.[1] ?? NaN, 5);
    });
    expect(box.widthDashed).toBe(false);
    expect(box.depthDashed).toBe(false);
  });

  it('uses the placeholder and dashes a side whose size is 0', () => {
    const box = placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 150, 0, 0);
    expect(box.widthDashed).toBe(true);
    expect(box.depthDashed).toBe(true);
    expect(box.corners[1]?.x).toBeCloseTo(OBSTACLE_PLACEHOLDER_MM / 2, 5);
    expect(box.corners[2]?.y).toBeCloseTo(150 + OBSTACLE_PLACEHOLDER_MM, 5);
  });

  it('fixes the pose when the hug starts and keeps the box after it ends', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
    store = reduceStore(store, obstacle(1, 120, 0, 0));
    store = reduceStore(store, status(3, 200, 90, 3));
    store = reduceStore(store, obstacle(4, 80, 90, 0));
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // Pose is (0, 100) heading north when the hug left idle; the later frames do not move it.
    expect(box?.corners[0]?.y).toBeCloseTo(220, 5);
    expect(box?.corners[0]?.x).toBeCloseTo(-45, 5);
    expect(box?.widthDashed).toBe(false);
    expect(box?.depthDashed).toBe(true);
  });

  it('keeps the box on an equal odometer and clears it on an odometer jump and on a boot event', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    expect(store.obstacle).not.toBeNull();
    const sameOdometer = reduceStore(store, status(2, 0, 0, 2));
    expect(sameOdometer.obstacle).not.toBeNull();
    expect(reduceStore(store, status(1, 9000, 0, 2)).obstacle).toBeNull();
    expect(reduceStore(store, event(1, 0x01)).obstacle).toBeNull();
  });

  it('places the box from the first face reading when the hug backs off first and the odometer falls', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 200, 0, 2));
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    store = reduceStore(store, obstacle(1, 0, 0, 0));
    store = reduceStore(store, status(3, 160, 0, 3));
    store = reduceStore(store, obstacle(2, 0, 0, 0));
    store = reduceStore(store, status(4, 120, 0, 4));
    store = reduceStore(store, obstacle(1, 0, 0, 0));
    store = reduceStore(store, status(5, 120, 0, 5));
    store = reduceStore(store, obstacle(3, 80, 0, 0));
    expect(store.trail).toHaveLength(5);
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // Pose (0, 120) heading north when the face first showed, not (0, 200) at the first contact.
    expect(box?.corners[0]?.y).toBeCloseTo(200, 5);
    expect(box?.corners[0]?.x).toBeCloseTo(-50, 5);
  });

  it('keeps the heading of the contact when the face first shows after the car has turned', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    store = reduceStore(store, obstacle(1, 0, 0, 0));
    store = reduceStore(store, status(3, 104, 50, 3));
    store = reduceStore(store, obstacle(3, 90, 0, 0));
    expect(store.obstacle).toMatchObject({ odometerMm: 104, headingDeg: 0, faceMm: 90 });
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // The last step of 4 mm ran at 50 degrees; the box still points north.
    const pose = { x: 4 * Math.sin((50 * Math.PI) / 180), y: 100 + 4 * Math.cos((50 * Math.PI) / 180) };
    expect(box?.corners[0]?.y).toBeCloseTo(pose.y + 90, 5);
    expect(box?.corners[0]?.x).toBeCloseTo(pose.x - 50, 5);
    expect(box?.corners[1]?.x).toBeCloseTo(pose.x + 50, 5);
  });

  it('gives no box for a first obstacle frame in a leg after the trail has points', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(4, 120, 50, 50));
    expect(store.obstacle).toBeNull();
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
  });

  it('freezes the face at its first reading while the hug runs', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    store = reduceStore(store, obstacle(1, 0, 0, 0));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(3, 80, 0, 0));
    store = reduceStore(store, status(3, 300, 0, 3));
    store = reduceStore(store, obstacle(5, 200, 60, 0));
    expect(store.obstacle).toMatchObject({ odometerMm: 100, faceMm: 80, widthMm: 60 });
  });

  it('draws no box while the face is 0', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 0, 40, 0));
    expect(buildCourse(store.trail, store.events, store.obstacle).obstacle).toBeNull();
    store = reduceStore(store, status(3, 200, 0, 3));
    store = reduceStore(store, obstacle(5, 90, 40, 0));
    const box = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(box?.corners[0]?.x).toBeCloseTo(-20, 5);
    expect(box?.corners[0]?.y).toBeCloseTo(290, 5);
    expect(buildCourse(store.trail, [], { timestamp: 2, odometerMm: 100, headingDeg: 0, faceMm: 0, widthMm: 40, breadthMm: 0 }).obstacle).toBeNull();
  });

  it('starts a new box when a hug returns to contact and the idle frame between was lost', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    store = reduceStore(store, status(3, 300, 0, 3));
    store = reduceStore(store, obstacle(8, 120, 50, 50));
    const first = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    expect(first?.corners[0]?.x).toBeCloseTo(-25, 5);
    expect(first?.corners[0]?.y).toBeCloseTo(220, 5);
    store = reduceStore(store, status(4, 500, 90, 4));
    store = reduceStore(store, obstacle(1, 200, 60, 70));
    const second = buildCourse(store.trail, store.events, store.obstacle).obstacle;
    // Pose (200, 300) heading east: the second hug has its own pose, face and sizes.
    expect(second?.corners[0]?.x).toBeCloseTo(400, 5);
    expect(second?.corners[0]?.y).toBeCloseTo(330, 5);
    expect(second?.corners[2]?.x).toBeCloseTo(470, 5);
    expect(second?.widthDashed).toBe(false);
    expect(second?.depthDashed).toBe(false);
  });

  it('keeps one pose across repeated contact frames of the same hug', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 0, 0));
    store = reduceStore(store, status(3, 200, 90, 3));
    store = reduceStore(store, obstacle(1, 120, 50, 0));
    expect(store.obstacle).toMatchObject({ odometerMm: 100, headingDeg: 0, faceMm: 120, widthMm: 50 });
  });

  it('gives no box for a hug whose frames come before the first trail point', () => {
    let store = emptyStore();
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(4, 120, 50, 50));
    expect(store.obstacle).toBeNull();
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    store = reduceStore(store, status(2, 100, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    expect(store.obstacle).toMatchObject({ odometerMm: 100, faceMm: 120 });
  });

  it('gives no box for the rest of a hug whose trail restarted in the middle', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 500, 0, 1));
    store = reduceStore(store, status(2, 600, 0, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    expect(store.obstacle).not.toBeNull();
    store = reduceStore(store, status(3, 10, 0, 3));
    expect(store.obstacle).toBeNull();
    store = reduceStore(store, obstacle(4, 120, 50, 50));
    expect(store.obstacle).toBeNull();
  });

  it('uses the last valid heading when the heading is invalid at the stamp', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 90, 1));
    store = reduceStore(store, packet('status', { seq: 2, odometer_mm: 100, heading_ddeg: 0xffff, speed_mms: 0 }, 2));
    store = reduceStore(store, obstacle(1, 120, 50, 50));
    expect(store.obstacle?.headingDeg).toBe(90);
  });

  it('keeps the same stamp when a hug frame repeats the face and the sizes', () => {
    let store = emptyStore();
    store = reduceStore(store, status(1, 0, 0, 1));
    store = reduceStore(store, obstacle(0, 0, 0, 0));
    store = reduceStore(store, obstacle(4, 120, 50, 50));
    const stamp = store.obstacle;
    expect(stamp).not.toBeNull();
    expect(reduceStore(store, obstacle(4, 120, 50, 50)).obstacle).toBe(stamp);
    expect(reduceStore(store, obstacle(5, 120, 0, 0)).obstacle).toBe(stamp);
    expect(reduceStore(store, obstacle(5, 120, 70, 50)).obstacle).not.toBe(stamp);
  });

  it('names the unmeasured sides in the label of the box', () => {
    expect(obstacleLabel(placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 100, 50, 50))).toBe('obstacle estimate');
    expect(obstacleLabel(placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 100, 0, 50))).toBe('obstacle estimate, width not measured');
    expect(obstacleLabel(placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 100, 50, 0))).toBe('obstacle estimate, depth not measured');
    expect(obstacleLabel(placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 100, 0, 0))).toBe('obstacle estimate, width and depth not measured');
  });

  it('places the box alike for 350 and -10 degrees', () => {
    const west = placeObstacle({ x: 5, y: 7, headingDeg: 350 }, 200, 100, 60);
    const wrapped = placeObstacle({ x: 5, y: 7, headingDeg: -10 }, 200, 100, 60);
    west.corners.forEach((c, i) => {
      expect(c.x).toBeCloseTo(wrapped.corners[i]?.x ?? NaN, 9);
      expect(c.y).toBeCloseTo(wrapped.corners[i]?.y ?? NaN, 9);
    });
    // Ten degrees west of north: ahead leans to -x.
    expect(west.corners[0]?.x).toBeLessThan(5);
    expect(west.corners[0]?.y).toBeGreaterThan(7);
  });

  it('clamps a size to the 12-bit wire range and reads a size that is not finite as unknown', () => {
    const big = placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 60000, 65535, 65535);
    expect(big.corners[0]?.y).toBeCloseTo(4095, 5);
    expect(big.corners[1]?.x).toBeCloseTo(4095 / 2, 5);
    expect(big.corners[2]?.y).toBeCloseTo(2 * 4095, 5);
    const bad = placeObstacle({ x: 0, y: 0, headingDeg: 0 }, 150, NaN, Infinity);
    expect(bad.widthDashed).toBe(true);
    expect(bad.depthDashed).toBe(true);
    expect(bad.corners[1]?.x).toBeCloseTo(OBSTACLE_PLACEHOLDER_MM / 2, 5);
    expect(bad.corners[2]?.y).toBeCloseTo(150 + OBSTACLE_PLACEHOLDER_MM, 5);
    expect(buildCourse([{ timestamp: 1, odometerMm: 0, headingDeg: 0, speedMms: 0 }, { timestamp: 2, odometerMm: 100, headingDeg: 0, speedMms: 0 }], [], { timestamp: 1, odometerMm: 50, headingDeg: 0, faceMm: Infinity, widthMm: 50, breadthMm: 50 }).obstacle).toBeNull();
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
