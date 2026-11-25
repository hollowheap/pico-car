import { describe, expect, it } from 'vitest';

import type { LinkPacket } from '../../src/lib/link';
import { currentScanReturn, profileHits, profileSurfaces, scanPoints, scanSurfaces, sparseProfileSurfaces, SCAN_MAX_MM, SLOT_COUNT } from '../../../bridge/src/console/scan.js';
import { ENUMS } from '../../src/lib/schema.generated';

const { valid, open, noEcho } = ENUMS.echoClass;
const measured = (echo: number, range = 0) => (1 << 15) | (echo << 12) | range;
const fixture = (slots: Record<number, number>) => Object.fromEntries(
  Array.from({ length: SLOT_COUNT }, (_, slot) => [`slots_${slot}`, slots[slot] ?? 0]),
);

type PingOverride = { dist_mm?: number; servo_deci?: number; echo_class?: number };

function ping(values: PingOverride = {}, timestamp = 10_000): LinkPacket {
  return { packet: 'obstacle', timestamp, values: { dist_mm: 234, servo_deci: -65, echo_class: valid, ...values } };
}

describe('current sonar return', () => {
  it('places one recent valid echo at its reported servo bearing', () => {
    expect(currentScanReturn(ping(), 10_400, true)).toEqual({ angleDeg: -6.5, distMm: 234 });
  });

  it.each([
    { echo_class: open }, { echo_class: noEcho }, { dist_mm: 0 },
    { dist_mm: 19 }, { dist_mm: 601 }, { dist_mm: NaN },
    { servo_deci: 201 }, { servo_deci: Infinity },
  ])('omits a false return for %j', (values) => {
    expect(currentScanReturn(ping(values), 10_400, true)).toBeNull();
  });

  it('omits missing, unready, stale, and future readings', () => {
    expect(currentScanReturn(undefined, 10_400, true)).toBeNull();
    expect(currentScanReturn(ping(), 10_400, false)).toBeNull();
    expect(currentScanReturn(ping(), 11_501, true)).toBeNull();
    expect(currentScanReturn(ping(), 9_999, true)).toBeNull();
  });
});

describe('observed scan surfaces', () => {
  const hit = measured(valid, 240);
  const gap = measured(open);

  it('marks the end sample only when its immediate outside slot was measured open', () => {
    const profile = fixture({ 15: gap, 16: hit, 17: hit, 18: hit, 19: hit, 20: measured(noEcho) });
    const [surface] = scanSurfaces(scanPoints(profile));
    expect(surface?.points.map(({ slot }) => slot)).toEqual([16, 17, 18, 19]);
    expect(surface?.edges.map(({ slot }) => slot)).toEqual([16]);

    profile.slots_20 = gap;
    expect(scanSurfaces(scanPoints(profile))[0]?.edges.map(({ slot }) => slot)).toEqual([16, 19]);
  });

  it('keeps runs separate across unknown slots and a depth jump without inventing an edge', () => {
    const deeper = measured(valid, 385);
    const profile = fixture({
      13: gap, 14: hit, 15: hit, 16: hit, 18: hit, 19: hit, 20: hit, 21: deeper, 22: deeper, 23: deeper,
    });
    const surfaces = scanSurfaces(scanPoints(profile));
    expect(surfaces.map(({ points }) => points.map(({ slot }) => slot))).toEqual([[14, 15, 16], [18, 19, 20], [21, 22, 23]]);
    expect(surfaces.map(({ edges }) => edges.map(({ slot }) => slot))).toEqual([[14], [], []]);
  });

  it('omits incomplete packets and isolated or out-of-range echoes', () => {
    const incomplete = fixture({ 16: hit, 17: hit, 18: hit });
    delete incomplete.slots_0;
    expect(scanSurfaces(scanPoints(incomplete))).toEqual([]);
    expect(scanSurfaces(scanPoints(fixture({ 16: hit, 17: hit, 18: measured(valid, 0), 19: hit, 20: measured(valid, 601) })))).toEqual([]);
  });

  it('plots no geometry across sparse pilot sweeps, even with apparently adjacent hits', () => {
    const points = scanPoints(fixture({ 14: hit, 16: hit, 17: hit, 18: hit, 19: gap, 22: hit }));
    expect(scanSurfaces(points, SCAN_MAX_MM, ENUMS.profileStatus.ok)).toHaveLength(1);
    expect(scanSurfaces(points, SCAN_MAX_MM, ENUMS.profileStatus.pilotSparse)).toEqual([]);
  });

  it('leaves a grazing return unbounded even beside explicitly open slots', () => {
    const profile = fixture({ 15: gap, 16: measured(valid, 150), 17: measured(valid, 195), 18: measured(valid, 240), 19: gap });
    const [surface] = scanSurfaces(scanPoints(profile));
    expect(surface?.points).toHaveLength(3);
    expect(surface?.edges).toEqual([]);
  });
});

it('draws adjacent measured returns from a stationary pilot fan without joining an unknown bearing', () => {
  const v = {...fixture({15: measured(valid, 205), 16: measured(valid, 204), 17: measured(valid, 202),
    18: measured(noEcho), 19: measured(valid, 205), 20: measured(valid, 206), 21: measured(valid, 207)}),
    status: ENUMS.profileStatus.pilotSparse, flags: 4};
  expect(profileSurfaces(v).map(f => [f.firstSlot, f.points.length, f.boundedStart, f.boundedEnd]))
    .toEqual([[15, 3, false, false], [19, 3, false, false]]);
  expect(profileSurfaces({...v, flags: 0})).toEqual([]);
});

it('clusters sparse pilot echoes smoothly without legacy body-pivot discontinuities', () => {
  const sparse = {
    ...fixture({ 14: measured(open), 16: measured(valid, 200), 18: measured(valid, 202), 20: measured(valid, 204), 21: measured(open) }),
    status: ENUMS.profileStatus.pilotSparse,
    flags: 0,
  };
  expect(profileHits(sparse)).toHaveLength(3);
  const faces = sparseProfileSurfaces(sparse);
  expect(faces).toHaveLength(1);
  expect(faces[0]?.points).toHaveLength(3);
  expect(faces[0]?.boundedEnd).toBe(true);
});

it('retains a short oblique side and marks only the endpoint confirmed by the car', () => {
  const side = { ...fixture({17: measured(valid, 105), 18: measured(valid, 109), 19: measured(open)}),
    status: ENUMS.profileStatus.pilotSparse, flags: 4 | 2 | (5 << 4) };
  const [face] = profileSurfaces(side);
  expect(face?.points).toHaveLength(2);
  expect(face?.boundedStart).toBe(false);
  expect(face?.boundedEnd).toBe(true);
  expect(profileSurfaces({...side, slots_18: 0})).toHaveLength(0);
});
