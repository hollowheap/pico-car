import { expect, it } from 'vitest';
import { buildCourse } from '../../src/lib/course';
import { compactTrail } from '../../src/lib/course-retention';
import { courseSession } from '../../src/lib/course-session';
import type { LinkPacket } from '../../src/lib/link';
import { emptyStore, reduceStore, type TelemetryStore } from '../../src/lib/telemetry-store';

const status = (t: number, seq: number, heading: number, odo = 0): LinkPacket => ({
  timestamp: t, packet: 'status', values: { seq, heading_ddeg: heading, odometer_mm: odo, speed_mms: 0 },
});
const hug = (t: number, state: number, body: number | undefined, face = 0): LinkPacket => ({
  timestamp: t, packet: 'obstacle', values: { hug_state: state, face_mm: face, width_mm: 60,
    ...(body === undefined ? {} : { body_deci: body }) },
});
const wheel = (t: number, seq: number, left: number, right: number): LinkPacket => ({
  timestamp: t, packet: 'motion', values: { seq, left_mms: left, right_mms: right },
});
const replay = (...frames: LinkPacket[]) => frames.reduce(reduceStore, emptyStore());
const course = (store: TelemetryStore) => buildCourse(store.trail, [], store.obstacle);
const bearing = (store: TelemetryStore) => {
  const box = course(store).obstacle;
  if (!box) throw new Error('Missing hug estimate');
  return box.headingDeg;
};

it('removes the signed body pivot from the mapped bearing when CONTACT first arrives mid-turn', () => {
  const store = replay(status(7600, 38, 3596), status(7800, 39, 6), status(8000, 40, 136),
    hug(8000, 1, -131), status(8200, 41, 295), hug(8200, 3, -290, 230));
  expect(bearing(store)).toBeCloseTo(0.5, 5);
});

it('pairs an off-cadence hug with both status receipts across north, including a later arrival', () => {
  const first = [status(1000, 5, 3500), hug(1100, 1, 100, 200)];
  expect(course(replay(...first)).obstacle).toBeNull();
  expect(bearing(replay(...first, status(1200, 6, 100)))).toBeCloseTo(10, 5);
  expect(course(replay(...first, status(1400, 7, 900))).obstacle).toBeNull();
  expect(bearing(replay(status(1000, 5, 3500), status(1200, 6, 100), hug(1100, 1, 100, 200)))).toBeCloseTo(10, 5);
});

it('pairs a modern CONTACT and its first status in either order within one bridge batch', () => {
  const contact = hug(1000, 1, -131, 180);
  const current = status(1003, 5, 136);
  expect(bearing(replay(contact, current))).toBeCloseTo(0.5);
  expect(bearing(replay(current, contact))).toBeCloseTo(0.5);
  expect(course(replay(contact, status(1010, 5, 136))).obstacle).toBeNull();
  expect(course(replay(current, hug(1003, 1, 32767, 180))).obstacle).toBeNull();
});

it('uses the course wheel bearing and first measured face axle through backoff and compaction', () => {
  const beforeFace = replay(status(1000, 5, 0xffff), wheel(1000, 5, 0, 0),
    status(1200, 6, 0xffff), hug(1200, 1, -100), wheel(1200, 6, 100, -100),
    status(1400, 7, 0xffff, -50), wheel(1400, 7, -250, -250));
  const expectedDegrees = (course(beforeFace).samples[1]!.headingDeg! - 10 + 360) % 360;
  const store = reduceStore(beforeFace, hug(1400, 3, -100, 200));
  expect(bearing(store)).toBeCloseTo(expectedDegrees, 5);
  const result = course(store);
  const contact = result.obstacle!;
  const axle = result.points[2]!;
  expect(contact.x).toBeCloseTo(axle.x + 200 * Math.sin(expectedDegrees * Math.PI / 180));
  expect(contact.y).toBeCloseTo(axle.y + 200 * Math.cos(expectedDegrees * Math.PI / 180));
  let extended = store;
  for (let i = 8; i <= 60; ++i) extended = reduceStore(extended, status(i * 200, i, 0, -50));
  expect(bearing(compactTrail(extended))).toBeCloseTo(expectedDegrees, 5);
});

it('retains the original contact-heading interpretation for older packets lacking body yaw', () => {
  expect(bearing(replay(status(200, 1, 900), hug(200, 1, undefined, 120)))).toBeCloseTo(90);
});

it('restores the latest located hug across an idle same-boot reload, and clears it on reboot', () => {
  let record: string | null = null;
  const memory = { getItem: () => record, setItem: (_key: string, value: string) => { record = value; },
    removeItem: () => { record = null; } };
  const hb = (timestamp: number, run_id = 7): LinkPacket => ({ packet: 'heartbeat', timestamp, values: { run_id, uptime_s: 15 } });
  const saved = replay(hb(800), status(800, 4, 3500), status(1000, 5, 3500), hug(1100, 1, 100, 200),
    status(1200, 6, 100), hug(1250, 0, 0), status(1400, 7, 100), hb(1400));
  courseSession('yaw', memory).save(saved);
  const original = record;
  const idle = replay(status(1400, 7, 100), status(1600, 8, 100), hug(1650, 0, 0), hb(1650));
  const restored = courseSession('yaw', memory).restore(idle, 1650);
  expect(course(restored).obstacle).toEqual(course(saved).obstacle);
  expect(restored.latest.obstacle).toBe(idle.latest.obstacle);

  const replacing = replay(status(1400, 7, 100), status(1600, 8, 100), hug(1600, 1, 300, 125), hb(1650));
  expect(bearing(courseSession('yaw', memory).restore(replacing, 1650))).toBeCloseTo(40);
  const partial = replay(status(1400, 7, 100), hug(1600, 4, 450, 75), hb(1650));
  expect(course(courseSession('yaw', memory).restore(partial, 1650)).obstacle).toBeNull();
  const rebooted = reduceStore(idle, hb(1700, 8));
  expect(course(courseSession('yaw', memory).restore(rebooted, 1700)).obstacle).toBeNull();
  const legacy = JSON.parse(original!);
  legacy.obstacle.sample = 99999;
  record = JSON.stringify(legacy);
  expect(course(courseSession('yaw', memory).restore(idle, 1650)).obstacle).toBeNull();
  delete legacy.obstacle;
  record = JSON.stringify(legacy);
  const old = courseSession('yaw', memory).restore(idle, 1650);
  expect(course(old).obstacle).toBeNull();
  expect(old.trail[0]?.seq).toBe(4);
});
