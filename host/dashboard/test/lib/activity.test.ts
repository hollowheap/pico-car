import { expect, it } from 'vitest';
import { groupActivity } from '../../../bridge/src/console/activity.js';
import { emptyStore, reduceStore } from '../../../bridge/src/console/telemetry-store.js';

function records(gap = 0, boundary = false, mask = 16) {
  // The six records from the captured pickup arrived over 71 ms.
  const fields = [[0,64,6,11,2048], [15,56,4,2639,1229], [15,3,9,3,0],
    [15,26,2,0,263], [15 + gap,6,1,256,mask], [71 + gap,36,3,2,1]];
  if (boundary) fields.splice(3, 0, [15,4,6,257,7]);
  return fields.reduce((store, [ms, type, source, p1, p2], i) => reduceStore(store, {
    timestamp: 2000 + ms!, packet: 'event', values: { event_id: i + 1, event_type: type!, source: source!, p1: p1!, p2: p2! },
  }), emptyStore()).events;
}

it('summarizes a reported pickup and its applied motor hold, retaining every original record', () => {
  const events = records();
  const groups = groupActivity(events);
  expect(groups).toHaveLength(1);
  expect(groups[0]?.event).toBe(events[1]);
  expect(groups[0]?.members).toEqual(events);
});

it('keeps separate stops visible across delays, operator commands and simultaneous holds', () => {
  for (const events of [records(1500), records(0, true), records(0, false, 20)]) {
    expect(groupActivity(events).every(row => row.members.length === 1)).toBe(true);
  }
});
