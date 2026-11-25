import { expect, it } from 'vitest';

import { buildCourse } from './course';
import { courseSession } from './course-session';
import { emptyStore, reduceStore } from './telemetry-store';

const storage = () => {
  const records = new Map<string, string>();
  return { getItem: (key: string) => records.get(key) ?? null,
    setItem: (key: string, value: string) => { records.set(key, value); },
    removeItem: (key: string) => { records.delete(key); } };
};
const hb = (timestamp: number, id = 7, uptime = 15) => ({
  packet: 'heartbeat', timestamp, values: { run_id: id, uptime_s: uptime },
});
const status = (timestamp: number, seq: number, odometer_mm: number, heading_ddeg = 0) => ({
  packet: 'status', timestamp, values: { seq, odometer_mm, heading_ddeg, speed_mms: 100 },
});
const old = () => [hb(1000), status(1100, 5, 0), status(1200, 6, 20), status(1200, 7, 40, 900),
  { packet: 'physics', timestamp: 1210, values: { attitude_state: 2 } }].reduce(reduceStore, emptyStore());

it('rejoins a same-tab replay on the same boot without duplicating overlapping poses or cached readings', () => {
  const memory = storage();
  courseSession('car-a', memory).save(old());
  const replay = [status(1200, 6, 20), status(1200, 7, 40, 900), status(1400, 8, 60, 900),
    { packet: 'obstacle', timestamp: 1401, values: { dist_mm: 170, echo_class: 0, servo_deci: 0 } },
    hb(1500)].reduce(reduceStore, emptyStore());
  const restored = courseSession('car-a', memory).restore(replay, 1500);
  expect(restored.latest).toBe(replay.latest);
  expect(restored.latest.physics).toBeUndefined();
  expect(restored.pendingWheels).toBe(replay.pendingWheels);
  expect(restored.trail.map((sample) => sample.seq)).toEqual([5, 6, 7, 8]);
  const course = buildCourse(restored.trail, [], null, null, restored.forwardReturn);
  expect(course.points.at(-1)?.x).toBeCloseTo(40);
  expect(course.points.at(-1)?.y).toBeCloseTo(20);
  expect(course.forwardReturn?.beam?.from.x).toBeCloseTo(110);
});

it('waits for current boot evidence and discards a snapshot from another endpoint, ID, uptime or explicit boot', () => {
  const memory = storage();
  courseSession('car-a', memory).save(old());
  const recent = [status(1800, 9, 210), hb(2000, 7, 16)].reduce(reduceStore, emptyStore());
  expect(courseSession('car-b', memory).restore(recent, 2000)).toBe(recent);
  const session = courseSession('car-a', memory);
  const missing = reduceStore(emptyStore(), status(1800, 9, 210));
  expect(session.restore(missing, 2000)).toBe(missing);
  expect(session.restore(recent, 20_000)).toBe(recent);
  expect(session.restore(recent, 2000).trail[0]?.seq).toBe(5);
  const boot = { packet: 'event', timestamp: 1700, values: { event_type: 1, event_id: 1 } };
  const cases = [[hb(2000, 8)], [hb(2000, 7, 2)], [boot, hb(2000)],
    [{ packet: 'heartbeat', timestamp: 2000, values: { uptime_s: 16 } }]];
  for (const frames of cases) {
    const store = frames.reduce(reduceStore, emptyStore());
    expect(courseSession('car-a', memory).restore(store, 2000)).toBe(store);
  }
});

it('continues across the firmware millisecond clock wrap in the same boot', () => {
  const memory = storage();
  const before = [hb(1000, 9, 4_294_966), status(1200, 100, 10)].reduce(reduceStore, emptyStore());
  courseSession('car', memory).save(before);
  const after = [hb(2400, 9, 0), status(2410, 106, 10)].reduce(reduceStore, emptyStore());
  expect(courseSession('car', memory).restore(after, 2450).trail.map((p) => p.seq)).toEqual([100, 106]);
});

it('waits for replay to catch up before appending duplicate cached status receipts', () => {
  const memory = storage();
  courseSession('car', memory).save(old());
  const session = courseSession('car', memory);
  const behind = [status(1200, 6, 20), hb(1400)].reduce(session.reduce, emptyStore());
  expect(session.restore(behind, 1400)).toBe(behind);
  const caughtUp = [status(1200, 7, 40, 900), status(1400, 8, 60, 900)].reduce(session.reduce, behind);
  expect(session.restore(caughtUp, 1400).trail.map((p) => p.seq)).toEqual([5, 6, 7, 8]);
});

it('cuts a truncated replay when the first observed heartbeat confirms a new boot', () => {
  const memory = storage();
  courseSession('car', memory).save(old());
  const session = courseSession('car', memory);
  const replayed = [status(1200, 7, 40, 900), hb(1600, 8, 0), status(1700, 0, 0)]
    .reduce(session.reduce, emptyStore());
  const fresh = session.restore(replayed, 1700);
  expect(fresh.trail.map((point) => point.seq)).toEqual([0]);
  expect(fresh.trailStart).toBe(1600);
  expect(fresh.latest.status?.timestamp).toBe(1700);
});

it('reconstructs unmatched cached poses in the earlier replay origin', () => {
  const memory = storage();
  const remembered = [hb(1000), status(1100, 5, 50), status(1200, 6, 60), status(1300, 7, 70)]
    .reduce(reduceStore, emptyStore());
  remembered.trail[1] = { ...remembered.trail[1]!, checkpoint: { x: 0, y: 10, headingDeg: 0,
    reference: 'compass', hasGap: false } };
  courseSession('car', memory).save(remembered);
  const replay = [status(900, 4, 0), status(1100, 5, 50), status(1300, 7, 70), hb(1500)]
    .reduce(reduceStore, emptyStore());
  const joined = courseSession('car', memory).restore(replay, 1500);
  expect(buildCourse(joined.trail, []).points.map((p) => p.y)).toEqual([0, 50, 60, 70]);
});

it('keeps distinct legacy poses delivered in the same millisecond on both sides of a reload', () => {
  const memory = storage();
  const legacy = (distance: number) => ({ packet: 'status', timestamp: 1200,
    values: { odometer_mm: distance, heading_ddeg: 0, speed_mms: 100 } });
  const saved = [hb(1000), legacy(0), legacy(20)].reduce(reduceStore, emptyStore());
  courseSession('car-a', memory).save(saved);
  const replay = [legacy(20), legacy(40), hb(1500)].reduce(reduceStore, emptyStore());
  const restored = courseSession('car-a', memory).restore(replay, 1500);
  expect(restored.trail.map((sample) => sample.odometerMm)).toEqual([0, 20, 40]);
  expect(buildCourse(restored.trail, []).points.at(-1)?.y).toBeCloseTo(40);
});

it('keeps the reset boundary on reload even when bridge history ends before the acceptance', () => {
  const memory = storage();
  const first = courseSession('car-a', memory);
  first.save(old());
  const ack = { packet: 'event', timestamp: 1400, values: {event_type: 4, source: 9, p1: 258, p2: 9} };
  first.reduce(old(), ack);
  const reload = courseSession('car-a', memory);
  const earlier = [hb(1000), status(1100, 5, 0)].reduce(reload.reduce, emptyStore());
  expect(reload.restore(earlier, 1400).trail).toEqual([]);
  expect(earlier.resetAt).toBe(1400);
  const after = [hb(1500), status(1600, 6, 35)].reduce(reload.reduce, earlier);
  expect(reload.restore(after, 1600).trail.map(p => p.seq)).toEqual([6]);
});

it('keeps the exact reset boundary when receipts share a timestamp', () => {
  const memory = storage();
  const before = { ...status(1400, 49, 240), receiptId: 'before', receiptOrder: 7 };
  const reset = { packet: 'event', timestamp: 1400, receiptId: 'reset', receiptOrder: 8,
    values: { event_type: 4, source: 9, p1: 258, p2: 9, event_id: 10 } };
  const after = { ...status(1400, 50, 0), receiptId: 'after', receiptOrder: 9 };
  const replay = (frames: Array<typeof before | typeof reset>) =>
    frames.reduce(courseSession('car', memory).reduce, emptyStore());

  expect(replay([before, reset, after]).trail.map(p => p.odometerMm)).toEqual([0]);
  expect(replay([before, reset, after]).trail.map(p => p.odometerMm)).toEqual([0]);
  expect(replay([before, after]).trail.map(p => p.odometerMm)).toEqual([0]);
});
