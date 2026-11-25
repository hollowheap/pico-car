import { ENUMS } from './schema.generated';

/** The profile packet holds 37 slots, 5 degrees apart, slot 18 straight ahead. */
export const SLOT_COUNT = 37;
export const SLOT_STEP_DEG = 5;
export const SLOT_FIRST_DEG = -90;

const DIST_MASK = 0x0fff;
const CLASS_SHIFT = 12;
const CLASS_MASK = 0x7;
const MEASURED_BIT = 15;

export type ScanMark = 'echo' | 'open' | 'bad' | 'none';

export type ScanPoint = {
  slot: number;
  /** Positive is left of the car. */
  angleDeg: number;
  distMm: number;
  echoClass: number;
  measured: boolean;
  mark: ScanMark;
};

function markOf(measured: boolean, echoClass: number): ScanMark {
  if (!measured) return 'none';
  if (echoClass === ENUMS.echoClass.valid) return 'echo';
  if (echoClass === ENUMS.echoClass.open) return 'open';
  return 'bad';
}

/** Unpack the `slots_<n>` fields of a decoded profile packet. */
export function scanPoints(values: Record<string, number>): ScanPoint[] {
  const points: ScanPoint[] = [];
  for (let slot = 0; slot < SLOT_COUNT; slot += 1) {
    const word = values[`slots_${slot}`];
    if (word === undefined) continue;
    const measured = ((word >> MEASURED_BIT) & 1) === 1;
    const echoClass = (word >> CLASS_SHIFT) & CLASS_MASK;
    points.push({
      slot,
      angleDeg: SLOT_FIRST_DEG + slot * SLOT_STEP_DEG,
      distMm: word & DIST_MASK,
      echoClass,
      measured,
      mark: markOf(measured, echoClass),
    });
  }
  return points;
}

/**
 * Place a point on the plot. The car is at (cx, cy) facing up. A positive angle
 * is left of the car, so it moves toward smaller x.
 */
export function polarToXY(
  angleDeg: number,
  distMm: number,
  mmPerUnit: number,
  cx: number,
  cy: number,
): { x: number; y: number } {
  const rad = (angleDeg * Math.PI) / 180;
  const r = distMm / mmPerUnit;
  return { x: cx - r * Math.sin(rad), y: cy - r * Math.cos(rad) };
}

/** Range rings every `stepMm` up to and including `maxMm`. */
export function rangeRings(maxMm: number, stepMm: number): number[] {
  const rings: number[] = [];
  for (let mm = stepMm; mm <= maxMm; mm += stepMm) rings.push(mm);
  return rings;
}
