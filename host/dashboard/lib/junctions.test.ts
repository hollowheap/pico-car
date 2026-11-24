import { expect, it } from 'vitest';

import { describeEvent } from './events';
import { deriveJunctionState } from './junctions';
import { ENUMS, EVENTS } from './schema.generated';
import { emptyStore, reduceStore } from './telemetry-store';

it('tracks barcode-armed junction choice, active junction turn, and completion outcome', () => {
  const packets = [
    { packet: 'event', timestamp: 100, values: { event_id: 1, event_type: EVENTS.barcode, p1: 'L'.charCodeAt(0), p2: ENUMS.barcodeAction.left } },
    { packet: 'event', timestamp: 500, values: { event_id: 2, event_type: EVENTS.turnStart, p1: 1, p2: 0 } },
    { packet: 'event', timestamp: 1120, values: { event_id: 3, event_type: EVENTS.turnDone, p1: 1, p2: 620 } },
  ];
  const armedStore = [packets[0]!].reduce(reduceStore, emptyStore());
  const armedState = deriveJunctionState(armedStore.events, null);
  expect(armedState.armed).toEqual({ direction: 'Left', source: 'barcode', letter: 'L', timestamp: 100 });
  expect(armedState.active).toBeNull();

  const activeStore = packets.slice(0, 2).reduce(reduceStore, emptyStore());
  const activeState = deriveJunctionState(activeStore.events, null);
  expect(activeState.armed).toBeNull();
  expect(activeState.active).toEqual({ direction: 'Left', timestamp: 500 });
  expect(activeState.markers).toEqual([{ timestamp: 500, direction: 'Left', outcome: 'active' }]);

  const doneStore = packets.reduce(reduceStore, emptyStore());
  const doneState = deriveJunctionState(doneStore.events, null);
  expect(doneState.active).toBeNull();
  expect(doneState.last).toEqual({ direction: 'Left', outcome: 'done', durationMs: 620, timestamp: 1120 });
  expect(doneState.markers).toEqual([{ timestamp: 500, direction: 'Left', outcome: 'done' }]);
});

it('decodes junction stripe and probe events while keeping armed instruction pending on noBranch/fallbackExit', () => {
  expect(describeEvent(EVENTS.junction, ENUMS.junctionKind.stripe, 7)).toEqual({
    kind: 'turn',
    title: 'Junction crossing',
    detail: 'Both detectors crossed transverse tape, corroborated.',
  });
  expect(describeEvent(EVENTS.junction, ENUMS.junctionKind.probe, (1 << 8) | ENUMS.junctionProbe.noBranch)).toEqual({
    kind: 'turn',
    title: 'Turn left probe: no branch',
    detail: 'Requested exit not found; instruction stays pending.',
  });
  expect(describeEvent(EVENTS.pilotDecision, 80, (3 << 8) | 15)).toMatchObject({
    kind: 'obstacle',
    title: 'Crawling without sonar',
  });

  const packets = [
    { packet: 'event', timestamp: 100, values: { event_id: 1, event_type: EVENTS.junction, p1: ENUMS.junctionKind.stripe, p2: 7 } },
    { packet: 'event', timestamp: 200, values: { event_id: 2, event_type: EVENTS.barcode, p1: 'L'.charCodeAt(0), p2: ENUMS.barcodeAction.left } },
    { packet: 'event', timestamp: 400, values: { event_id: 3, event_type: EVENTS.junction, p1: ENUMS.junctionKind.probe, p2: (1 << 8) | ENUMS.junctionProbe.fallbackExit } },
  ];
  const state = deriveJunctionState(packets.reduce(reduceStore, emptyStore()).events, null);
  expect(state.lastStripe).toEqual({ shape: 'T / cross', corroborated: true, timestamp: 100 });
  expect(state.armed).toMatchObject({ direction: 'Left', letter: 'L', probeNote: 'fallback exit' });
  expect(state.markers).toHaveLength(2);

  // Neither left nor right bit set labels as generic 'Stripe', not 'Right branch'.
  const neither = [
    { packet: 'event', timestamp: 500, values: { event_id: 4, event_type: EVENTS.junction, p1: ENUMS.junctionKind.stripe, p2: 4 } },
  ].reduce(reduceStore, emptyStore());
  expect(deriveJunctionState(neither.events, null).lastStripe).toEqual({
    shape: 'Stripe',
    corroborated: true,
    timestamp: 500,
  });
});
