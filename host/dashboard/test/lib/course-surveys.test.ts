import { expect, it } from 'vitest';

import { buildCourse, orientCourse } from '../../../bridge/src/console/course.js';
import { compactTrail } from '../../../bridge/src/console/course-retention.js';
import { courseSession } from '../../../bridge/src/console/course-session.js';
import type { LinkPacket } from '../../src/lib/link';
import { obstacleView } from '../../../bridge/src/console/obstacle-view.js';
import { profileSurfaces } from '../../../bridge/src/console/scan.js';
import { emptyStore, reduceStore } from '../../../bridge/src/console/telemetry-store.js';
import { BIT_FIELDS, ENUMS, FAULT_BITS } from '../../src/lib/schema.generated';

const profile = (timestamp: number, first = 16, status = 0, req_id = 1): LinkPacket => {
  const slots = Object.fromEntries(Array.from({ length: 37 }, (_, i) => [
    `slots_${i}`, i >= first && i <= first + 3 ? 0x8000 | 240 : i === first - 1 || i === first + 4 ? 0x9000 : 0,
  ]));
  return { packet: 'profile', timestamp, values: { ...slots, req_id, status } };
};
const pose = (timestamp: number, seq: number, odo = 0, heading = 0, faults = 0): LinkPacket =>
  ({ packet: 'status', timestamp, values: { seq, odometer_mm: odo, heading_ddeg: heading * 10, speed_mms: 0, faults } });
const obstacle = (timestamp: number, state: number, body_deci = 0): LinkPacket => ({ packet: 'obstacle', timestamp, values: { state, body_deci } });
const hb = (timestamp: number, run_id = 2, uptime_s = 10): LinkPacket => ({ packet: 'heartbeat', timestamp, values: { run_id, uptime_s } });
const start = [hb(100), pose(100, 1), obstacle(110, 2), pose(300, 2, 20, 90), obstacle(310, 3)];
const complete = [...start, obstacle(450, 3, 400), pose(500, 3, 20, 50), obstacle(510, 4), profile(520, 26)];
const fold = (packets: LinkPacket[]) => packets.reduce(reduceStore, emptyStore());
const courseOf = (packets: LinkPacket[]) => buildCourse(fold(packets).trail, []);

const forwardPing = (timestamp: number, seq: number, dist_mm: number): LinkPacket => ({
  packet: 'obstacle', timestamp,
  values: { seq, dist_mm, servo_deci: 0, echo_class: ENUMS.echoClass.valid, state: ENUMS.obstacleState.stopped },
});
const attitude = (timestamp: number, attitude_state: number): LinkPacket => ({
  packet: 'physics', timestamp, values: { attitude_state, flags: 1 << BIT_FIELDS.physicsFlags.calibrated },
});

it('retains the grounded map return through a pickup and resumes with a new ping after set-down', () => {
  const previous = fold([pose(1000, 1), forwardPing(1050, 1, 200)]);
  const offFloor = forwardPing(1250, 2, 105);
  const held = pose(1200, 2, 0, 0, 1 << FAULT_BITS.lifted);
  held.values.mode = ENUMS.vehicleMode.lifted;
  const lifted = [held, attitude(1250, ENUMS.attitudeState.lifted), offFloor].reduce(reduceStore, previous);
  expect(lifted.latest.obstacle).toBe(offFloor);
  expect(lifted.obstaclePing).toBe(offFloor);
  expect(lifted.forwardReturn).toBe(previous.forwardReturn);

  const grounded = [attitude(1430, ENUMS.attitudeState.ok), { ...offFloor, timestamp: 1450 }].reduce(reduceStore, lifted);
  expect(grounded.forwardReturn).toBe(previous.forwardReturn);
  const fresh = reduceStore(grounded, forwardPing(1470, 3, 205));
  const course = buildCourse(fresh.trail, [], fresh.obstacle, fresh.trailStart, fresh.forwardReturn);
  expect(course.forwardReturn?.rangeMm).toBe(205);
});

it('uses recent lift and overturn status when there is no usable IMU attitude', () => {
  const lifted = pose(1100, 1);
  lifted.values.mode = ENUMS.vehicleMode.lifted;
  const overturned = pose(1100, 1, 0, 0, 1 << FAULT_BITS.overturned);
  const stale = attitude(600, ENUMS.attitudeState.ok);
  const uncalibrated = attitude(1150, ENUMS.attitudeState.ok);
  uncalibrated.values.flags = 0;
  expect(fold([attitude(1150, ENUMS.attitudeState.lifted), forwardPing(1200, 1, 105)]).forwardReturn).toBeNull();
  expect(fold([stale, lifted, forwardPing(1200, 1, 105)]).forwardReturn).toBeNull();
  expect(fold([overturned, uncalibrated, forwardPing(1200, 1, 105)]).forwardReturn).toBeNull();
  expect(fold([lifted, forwardPing(1900, 1, 105)]).forwardReturn).toEqual({ rangeMm: 105, pose: null });
});

it('keeps airborne contact and survey packets out of the map', () => {
  const lifted = pose(1100, 1, 0, 0, 1 << FAULT_BITS.lifted);
  const contact: LinkPacket = { packet: 'obstacle', timestamp: 1130, values: { hug_state: 1, face_mm: 105, body_deci: 0 } };
  const fan = profile(1150, 16, ENUMS.profileStatus.pilotSparse);
  fan.values.flags = 1 << BIT_FIELDS.profileFlags.stationaryContact;
  const store = fold([obstacle(1090, ENUMS.obstacleState.waitStop), lifted, contact, fan]);
  expect(store.latest.obstacle).toBe(contact);
  expect(store.latest.profile).toBe(fan);
  expect(store.obstacle).toBeNull();
  expect(buildCourse(store.trail, []).surfaces).toHaveLength(0);
});

it('places each surveyed bearing from the sonar mount at its body pivot, in the scan-start axle frame', () => {
  const [local] = profileSurfaces(profile(520, 26).values); // +40° slot is straight on a +40° body pivot.
  const rad = 40 * Math.PI / 180;
  expect(local?.points[0]?.x).toBeCloseTo(-310 * Math.sin(rad));
  expect(local?.points[0]?.y).toBeCloseTo(310 * Math.cos(rad));
  const course = courseOf(complete);
  expect(course.surfaces).toHaveLength(1);
  const [world] = course.surfaces;
  expect(world?.points[0]?.x).toBeCloseTo(20 + 310 * Math.cos(rad));
  expect(world?.points[0]?.y).toBeCloseTo(310 * Math.sin(rad));
  expect(world?.boundedStart).toBe(true);
  expect(world?.boundedEnd).toBe(true);
  expect(course.bounds.maxX).toBeGreaterThan(250);
  const rotated = orientCourse(course, 'car-up').course;
  expect(rotated.surfaces[0]?.points[0]?.x).not.toBeCloseTo(world!.points[0]!.x);
});

it('retains distinct measured runs across later sparse scans, faults and stops, and clears them on reboot', () => {
  const first = courseOf(complete).surfaces;
  const later = [...complete, obstacle(600, 0), pose(700, 4, 20, 50, 2048), profile(710, 16, 4),
    obstacle(800, 2), pose(900, 5, 40, 90), obstacle(910, 3), profile(1000, 16, 0, 2)];
  const mapped = courseOf(later).surfaces;
  expect(mapped).toHaveLength(2);
  expect(mapped[0]).toEqual(first[0]);
  expect(courseOf([...later, hb(1200, 3, 0)]).surfaces).toHaveLength(0);
  expect(courseOf([pose(100, 1), profile(110)]).surfaces).toHaveLength(0);
  expect(courseOf([...start, profile(330, 16, 4)]).surfaces).toHaveLength(0);
  expect(courseOf([...start, profile(330, 16, 3)]).surfaces).toHaveLength(0);
  expect(courseOf([...start, obstacle(350, 0), profile(351)]).surfaces).toHaveLength(0);
  expect(courseOf([...start, pose(500, 3, 45, 90), profile(530)]).surfaces).toHaveLength(0);
  expect(courseOf([...start, profile(9000)]).surfaces).toHaveLength(0);
  expect(courseOf([pose(100, 1), obstacle(110, 2), obstacle(1200, 3), profile(1300)]).surfaces).toHaveLength(0);
});

it('selects the current survey and its axle frame without borrowing geometry from an older hug', () => {
  const hug: LinkPacket = { packet: 'obstacle', timestamp: 120, values: { state: 2, hug_state: 1, face_mm: 180, width_mm: 90, breadth_mm: 75 } };
  const observed = fold([...start.slice(0, 3), hug, ...complete.slice(3)]);
  const course = buildCourse(observed.trail, [], observed.obstacle);
  expect(course.obstacle).not.toBeNull();
  const placed = obstacleView(course, profile(520, 26));
  expect(placed.contact).toBeNull();
  expect(placed.surfaces).toEqual(course.surfaces);
  expect(placed.car).toEqual(course.points.at(-1));
  expect(placed.carLabel).toBe('Car');
  const sparse = obstacleView(course, profile(600, 16, 4));
  expect(sparse.contact).toBeNull();
  expect(sparse.surfaces).toEqual(course.surfaces);
  expect(sparse.lastMappedAt).toBe(520);
  const unlocated = obstacleView(course, profile(700, 16, 0, 2));
  expect(unlocated.carLabel).toBe('At scan');
  expect(unlocated.car).toEqual({ x: 0, y: 0 });
  expect(unlocated.surfaces).toEqual(profileSurfaces(profile(700, 16, 0, 2).values));
});

it('confirms the stationary scan origin on the first status after the scan starts between 5 Hz samples', () => {
  const lastMoving = pose(2400, 12, 271);
  lastMoving.values.speed_mms = 5;
  const arrival = [obstacle(2000, 2), lastMoving, obstacle(2500, 3)];
  expect(courseOf([...arrival, pose(2600, 13, 271), profile(2700)]).surfaces).toHaveLength(1);
  expect(courseOf([...arrival, profile(2575)]).surfaces).toHaveLength(0);
  expect(courseOf([...arrival, pose(2600, 13, 280), profile(2700)]).surfaces).toHaveLength(0);
  expect(courseOf([...arrival, pose(2600, 13, 271, 45), profile(2700)]).surfaces).toHaveLength(0);
  expect(courseOf([...arrival, obstacle(2550, 3, 400), pose(2600, 13, 271), profile(2700)]).surfaces).toHaveLength(0);
});

it('retains original geometry through compaction and same-boot reload including overlapping replay', () => {
  let store = fold(complete);
  for (let i = 0; i < 30; i++) store = reduceStore(store, pose(700 + i * 200, 4 + i, 20 + 5 * i, 50));
  const original = buildCourse(store.trail, []).surfaces;
  const compacted = compactTrail(store);
  expect(buildCourse(compacted.trail, []).surfaces).toEqual(original);
  let cache: string | null = null;
  const storage = { getItem: () => cache, setItem: (_: string, data: string) => { cache = data; }, removeItem: () => { cache = null; } };
  courseSession('car', storage).save(compacted);
  for (const prefix of [[], complete]) {
    const fresh = fold([...prefix, pose(6700, 34, 180, 50), hb(7100, 2, 17)]);
    const after = courseSession('car', storage).restore(fresh, 7100);
    expect(buildCourse(after.trail, []).surfaces).toEqual(original);
  }
  const reboot = fold([pose(6700, 0), hb(7100, 3, 0)]);
  expect(buildCourse(courseSession('car', storage).restore(reboot, 7100).trail, []).surfaces).toEqual([]);
});

it('keeps the face of every survey when a dense old trail loses its original anchor samples', () => {
  const surfaces = profileSurfaces(profile(1).values);
  const store = fold(Array.from({ length: 60 }, (_, i) => pose(100 + 200 * i, i, i * 20, i < 5 ? 45 : 90)));
  store.trail = store.trail.map((p, i) => ({ ...p, surveys: [{ timestamp: p.timestamp + 1, request: i, surfaces }] }));
  const compacted = compactTrail(store);
  expect(compacted.trail.length).toBeLessThan(store.trail.length);
  const positions = (trail: typeof store.trail) => buildCourse(trail, []).surfaces.map((face) =>
    face.points.map((p) => [p.x.toFixed(6), p.y.toFixed(6)]).join('/')).sort();
  expect(positions(compacted.trail)).toEqual(positions(store.trail));
});

it('places a moving bypass front fan only at a recent stationary axle fix', () => {
  const fan = profile(310, 16, 4);
  fan.values.flags = 4;
  const fixed = pose(300, 5, 100, 90);
  const moving = { ...fixed, values: {...fixed.values, speed_mms: 100} };
  expect(courseOf([fixed, fan]).surfaces).toHaveLength(1);
  expect(courseOf([moving, fan]).surfaces).toHaveLength(0);
  expect(courseOf([pose(20, 4), fan]).surfaces).toHaveLength(0);
  expect(courseOf([pose(300, 4, 100, 90, 1 << 6), fan]).surfaces).toHaveLength(0);
});

it('keeps separate mapped edge strokes when new dimensions arrive without another measured face', () => {
  const replay = [...complete, obstacle(600, 0), pose(700, 4, 20, 50),
    obstacle(800, 2), pose(900, 5, 40, 90), obstacle(910, 3), profile(1000, 16, 0, 2)];
  const course = courseOf(replay);
  expect(course.surfaces).toHaveLength(2);
  expect(obstacleView(course, undefined).surfaces).toEqual(course.surfaces);
});
