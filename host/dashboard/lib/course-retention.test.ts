import { expect, it } from 'vitest';

import { buildCourse } from './course';
import { emptyStore, reduceStore, TRAIL_MAX } from './telemetry-store';

it('keeps the boot origin, early corner and current pose across repeated trace compaction', () => {
  const end = 2 * TRAIL_MAX;
  const corner = 27;
  let store = emptyStore();
  let largest = 0;
  for (let seq = 0; seq <= end; seq++) {
    store = reduceStore(store, {
      timestamp: seq * 200, packet: 'status',
      values: { seq, odometer_mm: seq * 10, speed_mms: 50, heading_ddeg: seq > corner ? 900 : 0 },
    });
    largest = Math.max(largest, store.trail.length);
  }
  const course = buildCourse(store.trail, []);
  expect(largest).toBeLessThanOrEqual(TRAIL_MAX);
  expect(store.trail[0]?.timestamp).toBe(0);
  expect(course.points[0]).toEqual({ x: 0, y: 0 });
  expect(course.points).toContainEqual({ x: 0, y: corner * 10 });
  expect(course.points.at(-1)?.x).toBeCloseTo((end - corner) * 10);
  expect(course.points.at(-1)?.y).toBeCloseTo(corner * 10);
  expect(course.hasGap).toBe(false);

  const paired = reduceStore(store, { timestamp: end * 200, packet: 'motion',
    values: { seq: end & 255, left_mms: 50, right_mms: 50 } });
  expect(paired.trail.at(-1)?.wheels).toEqual({ leftMms: 50, rightMms: 50 });
  const reboot = reduceStore(paired, { timestamp: end * 200 + 1, packet: 'event',
    values: { event_type: 1, event_id: 1 } });
  expect(buildCourse(reboot.trail, []).points).toEqual([]);
});

it('resumes wheel-only travel on the last bearing after compacting an unlocated interval', () => {
  let store = emptyStore();
  const resume = TRAIL_MAX / 2;
  for (let seq = 0; seq <= TRAIL_MAX; seq++) {
    const timestamp = seq * 200;
    const heading_ddeg = seq < 27 ? 0 : seq === 27 ? 900 : 0xffff;
    store = reduceStore(store, { timestamp, packet: 'status',
      values: { seq, heading_ddeg, speed_mms: seq > resume ? 50 : 0, odometer_mm: Math.max(0, seq - resume) * 10 } });
    if (seq >= resume) {
      store = reduceStore(store, { timestamp, packet: 'motion',
        values: { seq: seq & 255, left_mms: 50, right_mms: 50 } });
    }
  }
  const course = buildCourse(store.trail, []);
  expect(course.hasGap).toBe(true);
  expect(course.samples.at(-1)?.headingDeg).toBeCloseTo(90);
  expect(course.points.at(-1)?.x).toBeCloseTo((TRAIL_MAX - resume) * 10);
  expect(course.points.at(-1)?.y).toBeCloseTo(0);
});
