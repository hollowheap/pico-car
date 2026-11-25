import type { LinkPacket } from './link';
export type { MeasuredSurface, ScanPoint } from '../../../bridge/src/console/scan.js';

export const SCAN_MAX_MM = 600;

// Obstacle telemetry arrives at 2 Hz. A "current" mark allows three frame intervals.
const CURRENT_RETURN_MAX_AGE_MS = 1500;

// LOOK_SONAR_FWD_MM and OBSTACLE_SONAR_FWD_MM measure from the axle.
export const SONAR_FORWARD_MM = 70;

export function isRecentSonarPing(packet: LinkPacket | undefined, nowMs: number): packet is LinkPacket {
  if (packet === undefined) return false;
  const age = nowMs - packet.timestamp;
  return age >= 0 && age <= CURRENT_RETURN_MAX_AGE_MS;
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
