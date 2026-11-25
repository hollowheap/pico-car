import { describe, expect, it } from 'vitest';

import { planDetour, type DetourSegment, type DetourSurvey, type Face, type SonarObservation } from './detour.js';

const AXLE_TO_SONAR_MM = 70;

function measureFace(face: Face, surface: number, tangentOffsets: readonly number[], rangeMm = 160): SonarObservation[] {
  const horizontal = face === 'front' || face === 'back';
  const facingPositive = face === 'front' || face === 'right';
  const yawDeg = horizontal ? (facingPositive ? 0 : 180) : (facingPositive ? 90 : -90);
  const normalPos = surface - (facingPositive ? 1 : -1) * (AXLE_TO_SONAR_MM + rangeMm);
  const base = 100 * (1 + ['front', 'back', 'left', 'right'].indexOf(face));
  return tangentOffsets.map((tangent, i) => {
    const capturedAtMs = base + i * 10;
    return {
      face, capturedAtMs, bearingDeg: 0, echo: 'valid', rangeMm,
      pose: { xMm: horizontal ? normalPos : tangent, yMm: horizontal ? tangent : normalPos, yawDeg, capturedAtMs },
    };
  });
}

function survey(overrides: Partial<DetourSurvey> = {}): DetourSurvey {
  return {
    model: 'single-tape-aligned-rectangle',
    verifiedArea: { minX: -250, maxX: 2200, minY: -450, maxY: 450 },
    strip: { minY: -400, maxY: 400 },
    supportingFloor: { minX: -250, maxX: 2200, minY: -400, maxY: 400 },
    footprint: { frontMm: 130, rearMm: 120, leftMm: 63, rightMm: 63 },
    trackingErrorMm: 10,
    clearanceMm: 120,
    sonar: { forwardOffsetMm: AXLE_TO_SONAR_MM, beamHalfDeg: 9.5, rangeErrorMm: 3, poseErrorMm: 2, headingErrorDeg: 1 },
    observations: [
      ...measureFace('front', 850, [-24, 0, 24]),
      ...measureFace('back', 970, [-24, 0, 24]),
      ...measureFace('left', 30, [870, 910, 950]),
      ...measureFace('right', -30, [870, 910, 950]),
    ],
    ...overrides,
  };
}

// Integrate independently of the planner. Enclose the 250 x 126 mm body in
// circles covering 25 x 21 mm tiles, then test each circle against the true
// floor and the fixture box (front 850, back 970, sides ±30 mm).
function sweepFixture(segments: readonly DetourSegment[]) {
  const tileRadius = Math.hypot(12.5, 10.5);
  let x = 0;
  let y = 0;
  let yaw = 0;
  let furthestAxleY = 0;
  let floorMargin = Infinity;
  let boxClearance = Infinity;
  for (const { lengthMm, turnDeci } of segments) {
    const steps = Math.ceil(lengthMm);
    const ds = lengthMm / steps;
    const dyaw = turnDeci * Math.PI / (1800 * steps);
    const chord = dyaw === 0 ? ds : ds * Math.sin(dyaw / 2) / (dyaw / 2);
    const stepPad = ds + Math.hypot(130, 63) * Math.abs(dyaw);
    for (let step = 0; step < steps; step++) {
      x += chord * Math.cos(yaw + dyaw / 2);
      y += chord * Math.sin(yaw + dyaw / 2);
      yaw += dyaw;
      furthestAxleY = Math.max(furthestAxleY, Math.abs(y));
      const cos = Math.cos(yaw);
      const sin = Math.sin(yaw);
      for (let u = -120 + 12.5; u < 130; u += 25) {
        for (let v = -63 + 10.5; v < 63; v += 21) {
          const cx = x + u * cos - v * sin;
          const cy = y + u * sin + v * cos;
          floorMargin = Math.min(floorMargin, Math.min(cx + 250, 2200 - cx, 400 - Math.abs(cy)) - tileRadius - stepPad);
          const dx = Math.max(850 - cx, 0, cx - 970);
          const dy = Math.max(0, Math.abs(cy) - 30);
          boxClearance = Math.min(boxClearance, Math.hypot(dx, dy) - tileRadius - stepPad);
        }
      }
    }
  }
  return { x, y, yaw, furthestAxleY, floorMargin, boxClearance };
}

describe('surveyed detour', () => {
  it('encloses the measured box and makes a rounded route with an exact mirrored return to tape', () => {
    const route = planDetour(survey());
    expect(route.kind).toBe('route');
    if (route.kind !== 'route') throw new Error(route.reason);
    expect(route.box.minX).toBeLessThan(850);
    expect(route.box.maxX).toBeGreaterThan(970);
    expect(route.box.minY).toBeLessThan(-30);
    expect(route.box.maxY).toBeGreaterThan(30);
    expect(route.segments.length).toBeLessThanOrEqual(7);
    const bends = route.segments.filter((segment) => segment.turnDeci !== 0);
    expect(bends).toHaveLength(4);
    expect(bends.map((segment) => segment.turnDeci)).toEqual([
      bends[0]?.turnDeci, -(bends[0]?.turnDeci ?? 0), -(bends[0]?.turnDeci ?? 0), bends[0]?.turnDeci,
    ]);
    expect(new Set(bends.map((segment) => segment.lengthMm)).size).toBe(1);
    expect(route.segments.every(({ lengthMm }) => lengthMm > 0 && Number.isInteger(lengthMm))).toBe(true);
    expect(route.end.yMm).toBeCloseTo(0);
    expect(route.end.xMm).toBeGreaterThan(970 + 120);
  });

  it('uses independently supported adjacent floor to rejoin past a fixed box on a 300 mm track', () => {
    const route = planDetour(survey({
      strip: { minY: -150, maxY: 150 },
      terminalTape: { sensorOffsetMm: 12.5, captureToleranceMm: 2 },
    }));
    expect(route.kind).toBe('route');
    if (route.kind !== 'route') throw new Error(route.reason);
    const swept = sweepFixture(route.segments);
    // The sweep includes chassis-point travel between samples. Add tracking error.
    expect(swept.furthestAxleY).toBeGreaterThan(150);
    expect(swept.floorMargin).toBeGreaterThan(10);
    expect(swept.boxClearance).toBeGreaterThan(120 + 10);
    expect(swept.x).toBeGreaterThan(970 + 120);
    expect(Math.abs(Math.abs(swept.y) - 12.5)).toBeLessThanOrEqual(2);
    expect(swept.yaw).toBeCloseTo(0);
  });

  it('holds with unknown supporting floor despite complete obstacle observations', () => {
    const fixture = survey();
    delete fixture.supportingFloor;
    expect(planDetour(fixture)).toEqual({ kind: 'hold', reason: 'unverified survey' });
  });

  it('holds when only the track has supporting floor beneath the swept footprint', () => {
    expect(planDetour(survey({ supportingFloor: { minX: -250, maxX: 2200, minY: -150, maxY: 150 } })))
      .toEqual({ kind: 'hold', reason: 'no safe route' });
  });

  it('holds a centered box when its surveyed obstacle-clear area is too narrow', () => {
    expect(planDetour(survey({ verifiedArea: { minX: -250, maxX: 2200, minY: -150, maxY: 150 } })))
      .toEqual({ kind: 'hold', reason: 'no safe route' });
  });

  it('holds with a complete surveyed box at the installed 100 mm contact', () => {
    const front = AXLE_TO_SONAR_MM + 100;
    const back = front + 120;
    const observations = [
      ...measureFace('front', front, [-24, 0, 24], 100),
      ...measureFace('back', back, [-24, 0, 24]),
      ...measureFace('left', 30, [front + 20, front + 60, back - 20]),
      ...measureFace('right', -30, [front + 20, front + 60, back - 20]),
    ];
    const contact = survey({ observations, clearanceMm: 50 });
    const narrowFloor = { minX: -250, maxX: 2200, minY: -150, maxY: 150 };
    expect(planDetour({ ...contact, supportingFloor: narrowFloor })).toEqual({ kind: 'hold', reason: 'no safe route' });
    expect(planDetour(contact)).toEqual({ kind: 'hold', reason: 'no safe route' });

    const earlier = {
      ...contact,
      observations: observations.map((hit) => ({ ...hit, pose: { ...hit.pose, xMm: hit.pose.xMm + 680 } })),
    };
    expect(planDetour({ ...earlier, supportingFloor: narrowFloor })).toEqual({ kind: 'hold', reason: 'no safe route' });
    expect(planDetour(earlier).kind).toBe('route');
  });

  it('accepts a surveyed 300 mm approach with a wide passing side and an earlier anchor', () => {
    const route = planDetour(survey({
      supportingFloor: { minX: -250, maxX: 2200, minY: -210, maxY: 90 },
      trackingErrorMm: 5,
      clearanceMm: 50,
      terminalTape: { sensorOffsetMm: 12.5, captureToleranceMm: 2 },
      observations: [
        ...measureFace('front', 850, [26, 50, 74]),
        ...measureFace('back', 970, [26, 50, 74]),
        ...measureFace('left', 80, [870, 910, 950]),
        ...measureFace('right', 20, [870, 910, 950]),
      ],
    }));
    expect(route.kind).toBe('route');
    if (route.kind !== 'route') throw new Error(route.reason);
    expect(route.end.xMm).toBeGreaterThan(970 + 120 + 50);
    expect(Math.abs(route.end.yMm + 12.5)).toBeLessThanOrEqual(2);
    expect(route.segments.reduce((yaw, { turnDeci }) => yaw + turnDeci, 0)).toBe(0);
  });

  it.each([1, -1])('plans a parallel endpoint over a tape sensor on side %s', (side) => {
    const route = planDetour(survey({
      supportingFloor: { minX: -250, maxX: 2200, minY: side > 0 ? -140 : -400, maxY: side > 0 ? 400 : 140 },
      terminalTape: { sensorOffsetMm: 12.5, captureToleranceMm: 2 },
    }));
    expect(route.kind).toBe('route');
    if (route.kind !== 'route') throw new Error(route.reason);
    expect(Math.abs(route.end.yMm - side * 12.5)).toBeLessThanOrEqual(2);
    expect(route.segments.reduce((yaw, { turnDeci }) => yaw + turnDeci, 0)).toBe(0);
  });

  it('holds when the outline cannot leave room for the exit', () => {
    expect(planDetour(survey({ verifiedArea: { minX: -250, maxX: 1150, minY: -450, maxY: 450 } })))
      .toEqual({ kind: 'hold', reason: 'no safe route' });
  });

  it('requires independent measured returns on all four faces', () => {
    const source = survey();
    for (const face of ['front', 'back', 'left', 'right']) {
      const observations = source.observations.filter((observation) => observation.face !== face);
      expect(planDetour({ ...source, observations })).toEqual({ kind: 'hold', reason: 'incomplete contour' });
    }
    const observation = source.observations[0]!;
    expect(planDetour({ ...source, observations: [...source.observations.filter(({ face }) => face !== 'front'), observation, observation, observation] }))
      .toEqual({ kind: 'hold', reason: 'incomplete contour' });
    expect(planDetour({ ...source, observations: [...source.observations.filter(({ face }) => face !== 'front'),
      ...measureFace('front', 850, [0, 0, 0])] })).toEqual({ kind: 'hold', reason: 'incomplete contour' });
  });

  it.each(['open', 'stale', 'grazing', 'conflicting'] as const)('refuses %s evidence for a face', (mode) => {
    const source = survey();
    const observations = source.observations.map((item): SonarObservation => {
      if (item.face !== 'back') return item;
      if (mode === 'open') return { ...item, echo: 'open', rangeMm: 0 };
      if (mode === 'stale') return { ...item, capturedAtMs: item.pose.capturedAtMs + 26 };
      if (mode === 'grazing') return { ...item, bearingDeg: 16 };
      return item.capturedAtMs === 200 ? { ...item, rangeMm: 207 } : item;
    });
    expect(planDetour({ ...source, observations })).toEqual({ kind: 'hold', reason: 'incomplete contour' });
  });

  it('refuses an unknown model and nonfinite geometry', () => {
    expect(planDetour(survey({ model: 'unverified' }))).toEqual({ kind: 'hold', reason: 'unverified survey' });
    expect(planDetour(survey({ footprint: { frontMm: NaN, rearMm: 120, leftMm: 63, rightMm: 63 } })))
      .toEqual({ kind: 'hold', reason: 'unverified survey' });
  });

  it('holds when a clear straight route exceeds the pilot step limit', () => {
    const source = survey({
      verifiedArea: { minX: -250, maxX: 2500, minY: -150, maxY: 650 },
      supportingFloor: { minX: -250, maxX: 2500, minY: -150, maxY: 650 },
      observations: [
        ...measureFace('front', 1600, [306, 330, 354]),
        ...measureFace('back', 1720, [306, 330, 354]),
        ...measureFace('left', 360, [1620, 1660, 1700]),
        ...measureFace('right', 300, [1620, 1660, 1700]),
      ],
    });
    expect(planDetour(source)).toEqual({ kind: 'hold', reason: 'no safe route' });
  });

  it('holds when individually admissible bends exceed the pilot total-distance limit', () => {
    const source = survey({
      verifiedArea: { minX: -250, maxX: 4000, minY: -1400, maxY: 1400 },
      supportingFloor: { minX: -250, maxX: 4000, minY: -1400, maxY: 1400 },
      observations: [
        ...measureFace('front', 1600, [-300, 0, 300]),
        ...measureFace('back', 2450, [-300, 0, 300]),
        ...measureFace('left', 450, [1700, 2000, 2300]),
        ...measureFace('right', -450, [1700, 2000, 2300]),
      ],
    });
    expect(planDetour(source)).toEqual({ kind: 'hold', reason: 'no safe route' });
  });
});
