import { it, expect } from 'vitest';
import { emptyStore, reduceStore } from './telemetry-store';
import { steeringFraction, wheelPlotSeries, wheelSegments } from './wheel-history';

it('shows full steering during a pivot at any wheel speed and partial steering on a moving arc', () => {
  expect(steeringFraction(-40, 60)).toBe(-1);
  expect(steeringFraction(100, -100)).toBe(1);
  expect(steeringFraction(49, 73)).toBeCloseTo(-24 / 122);
  expect(steeringFraction(98, 146)).toBeCloseTo(-24 / 122);
  expect(steeringFraction(80, 80)).toBe(0);
  expect(steeringFraction(0, 0)).toBe(0);
  expect(steeringFraction(-1, 1)).toBe(0);
  expect(steeringFraction(3, -2)).toBe(0);
  expect(steeringFraction(-8, 7)).toBe(-1);
  expect(steeringFraction(undefined, 100)).toBe(0);
});

it('plots signed wheel measurements across a pivot and leaves gaps for bad or missing feedback', () => {
  const frames = [[100, 1, -40, 0], [300, 2, -50, 0], [500, 3, 0, 1], [700, 4, 60, 0], [2100, 5, 20, 0]];
  let store = emptyStore();
  for (const [timestamp, seq, left_mms, motion_fault] of frames) {
    const p = { packet: 'motion', timestamp: timestamp!, values: { seq: seq!, left_mms: left_mms!, right_mms: 70, motion_fault: motion_fault! } };
    store = reduceStore(reduceStore(store, p), p);
  }
  expect(wheelSegments(store.wheelHistory, 2100, 'left').map(run => run.map(p => p.value))).toEqual([[-40,-50],[60],[20]]);
  expect(wheelSegments(store.wheelHistory, 2100, 'right')[0]?.map(p => p.value)).toEqual([70,70]);
  expect(wheelSegments(store.wheelHistory, 13000, 'left')).toEqual([]);
  for (const id of [1,2]) store = reduceStore(store, {packet: 'heartbeat', timestamp: 2400 + id, values: {run_id: id}});
  expect(store.wheelHistory).toEqual([]);
});

it('clips left-edge wheel segments smoothly and bridges short packet losses without hiding long dropouts', () => {
  const frames = [
    { timestamp: 0, seq: 1, left_mms: 0, left_target_mms: 100 },
    { timestamp: 200, seq: 2, left_mms: 100, left_target_mms: 100 },
    // seq 3 dropped (400 ms gap -> bridgeable)
    { timestamp: 600, seq: 4, left_mms: 140, left_target_mms: 120 },
    // long 2 s dropout -> must stay open
    { timestamp: 2600, seq: 14, left_mms: 80, left_target_mms: 80 },
  ];
  const store = frames.reduce(
    (s, f) => reduceStore(s, { packet: 'motion', timestamp: f.timestamp, values: { ...f, right_mms: 0, motion_fault: 0 } }),
    emptyStore(),
  );
  // At now=10100, window is [100, 10100]: segment 0..200 clips cleanly at t=100 -> value=50.
  const clipped = wheelPlotSeries(store.wheelHistory, 10100, 'left');
  expect(clipped.measured[0]?.[0]).toEqual({ at: 100, value: 50 });
  expect(clipped.estimated).toEqual([[{ at: 200, value: 100 }, { at: 600, value: 140 }]]);
  expect(clipped.targets.flat().length).toBeGreaterThan(0);

  // At now=2800 (200 ms after last packet), right edge holds smoothly to now.
  const live = wheelPlotSeries(store.wheelHistory, 2800, 'left');
  expect(live.estimated.at(-1)).toEqual([{ at: 2600, value: 80 }, { at: 2800, value: 80 }]);
});
