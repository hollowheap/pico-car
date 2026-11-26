import type { LinkPacket } from './link.js';
import { BIT_FIELDS, ENUMS } from './schema.generated.js';

/** The profile packet holds 37 slots, 5 degrees apart, slot 18 straight ahead. */
export const SLOT_COUNT = 37;
export const SLOT_STEP_DEG = 5;
export const SLOT_FIRST_DEG = -90;
export const SCAN_MAX_MM = 600;

const DIST_MASK = 0x0fff;
const CLASS_SHIFT = 12;
const CLASS_MASK = 0x7;
const MEASURED_BIT = 15;
const MIN_RANGE_MM = 20;
const SERVO_LIMIT_DECI = 200;
// Obstacle telemetry arrives at 2 Hz. A "current" mark allows three frame intervals.
const CURRENT_RETURN_MAX_AGE_MS = 1500;
const SURFACE_MIN_SLOTS = 3;
const SURFACE_MAX_RANGE_STEP_MM = 60;
const EDGE_MIN_SPAN_MM = 20;
const EDGE_MIN_NORMAL_COMPONENT = 0.5;

export type ScanMark = 'echo' | 'open' | 'bad' | 'none';

export type ScanReturn = {
  /** Positive is left of the car. */
  angleDeg: number;
  distMm: number;
};

export type ScanPoint = ScanReturn & {
  slot: number;
  echoClass: number;
  measured: boolean;
  mark: ScanMark;
};

export type ScanSurface = { points: ScanPoint[]; edges: ScanPoint[] };
type XY = { x: number; y: number };
export type MeasuredSurface = { firstSlot: number; points: XY[]; boundedStart: boolean; boundedEnd: boolean };
export type Survey = { timestamp: number; request: number; surfaces: MeasuredSurface[] };
// LOOK_SONAR_FWD_MM and OBSTACLE_SONAR_FWD_MM measure from the axle.
export const SONAR_FORWARD_MM = 70;

/** Legacy scan slots use the initial scan bearing, across the 0°, ±40°, and ±70° body windows. */
function scanPosition(point: ScanPoint): XY {
  const angle = point.angleDeg;
  const bodyDeg = Math.sign(angle) * (Math.abs(angle) <= 20 ? 0 : Math.abs(angle) <= 60 ? 40 : 70);
  const bearing = angle * Math.PI / 180;
  const body = bodyDeg * Math.PI / 180;
  return { x: -SONAR_FORWARD_MM * Math.sin(body) - point.distMm * Math.sin(bearing),
    y: SONAR_FORWARD_MM * Math.cos(body) + point.distMm * Math.cos(bearing) };
}

/** Servo-only sweep position from the forward sonar mount without legacy body-pivot jumps. */
function servoPosition(point: ScanPoint): XY {
  const bearing = point.angleDeg * Math.PI / 180;
  return {
    x: -point.distMm * Math.sin(bearing),
    y: SONAR_FORWARD_MM + point.distMm * Math.cos(bearing),
  };
}

export function profileHits(values: LinkPacket['values']): XY[] {
  const sparse = values.status === ENUMS.profileStatus.pilotSparse;
  const project = sparse ? servoPosition : scanPosition;
  return scanPoints(values)
    .filter((point) => isPlottedEcho(point))
    .map(project);
}

/** Cluster sparse pilot echoes by angular proximity and range step for visual inspection. */
export function sparseProfileSurfaces(values: LinkPacket['values']): MeasuredSurface[] {
  if (values.status !== ENUMS.profileStatus.pilotSparse) return [];
  const all = scanPoints(values);
  const bySlot = new Map(all.map((point) => [point.slot, point]));
  const echoes = all.filter((point) => isPlottedEcho(point));
  if (echoes.length < 2) return [];
  const surfaces: MeasuredSurface[] = [];
  let cluster: ScanPoint[] = [];
  const flush = () => {
    if (cluster.length >= 2) {
      const first = cluster[0]!;
      const last = cluster.at(-1)!;
      const before = bySlot.get(first.slot - 1);
      const after = bySlot.get(last.slot + 1);
      const boundedStart = before?.mark === 'open' || (before !== undefined && isPlottedEcho(before) && Math.abs(before.distMm - first.distMm) > 120);
      const boundedEnd = after?.mark === 'open' || (after !== undefined && isPlottedEcho(after) && Math.abs(after.distMm - last.distMm) > 120);
      surfaces.push({
        firstSlot: first.slot,
        points: cluster.map(servoPosition),
        boundedStart,
        boundedEnd,
      });
    }
    cluster = [];
  };
  for (const point of echoes) {
    const prev = cluster.at(-1);
    if (prev !== undefined && (point.angleDeg - prev.angleDeg > 15 || Math.abs(point.distMm - prev.distMm) > SURFACE_MAX_RANGE_STEP_MM)) {
      flush();
    }
    cluster.push(point);
  }
  flush();
  return surfaces;
}

export function stationaryContact(values: LinkPacket['values']): boolean {
  return values.status === ENUMS.profileStatus.pilotSparse && Boolean((values.flags ?? 0) & (1 << BIT_FIELDS.profileFlags.stationaryContact));
}

export function profileSurfaces(values: LinkPacket['values']): MeasuredSurface[] {
  const status = values.status;
  const contact = stationaryContact(values);
  if (!contact && status !== undefined && (status < ENUMS.profileStatus.ok || status > ENUMS.profileStatus.noSafePath)) return [];
  const points = scanPoints(values);
  const project = contact ? servoPosition : scanPosition;
  const sideFan = contact && Boolean((values.flags ?? 0) & (1 << BIT_FIELDS.profileFlags.probeSideValid));
  if (sideFan) {
    const bySlot = new Map(points.map((point) => [point.slot, point]));
    const surfaces: MeasuredSurface[] = [];
    let run: ScanPoint[] = [];
    const finish = () => {
      if (run.length >= 2) {
        const first = run[0]!;
        const last = run.at(-1)!;
        const prevSlot = bySlot.get(first.slot - 1);
        const nextSlot = bySlot.get(last.slot + 1);
        const boundedStart = run.length >= SURFACE_MIN_SLOTS
          ? supportedEdge(first, run[2]!, prevSlot, project)
          : prevSlot?.mark === 'open';
        const boundedEnd = run.length >= SURFACE_MIN_SLOTS
          ? supportedEdge(last, run.at(-3)!, nextSlot, project)
          : nextSlot?.mark === 'open';
        surfaces.push({ firstSlot: first.slot, points: run.map(project), boundedStart, boundedEnd });
      }
      run = [];
    };
    for (const point of points) {
      const previous = run.at(-1);
      if (!isPlottedEcho(point, SCAN_MAX_MM)) {
        finish();
        continue;
      }
      if (previous !== undefined && Math.abs(point.distMm - previous.distMm) > SURFACE_MAX_RANGE_STEP_MM) finish();
      run.push(point);
    }
    finish();
    return surfaces;
  }
  return scanSurfaces(points, SCAN_MAX_MM, undefined, project).map(({ points: run, edges }) => ({
    firstSlot: run[0]!.slot, points: run.map(project),
    boundedStart: edges.includes(run[0]!), boundedEnd: edges.includes(run.at(-1)!),
  }));
}

export function placeSurface(surface: MeasuredSurface, pose: XY, heading: number): MeasuredSurface {
  const bearing = heading * Math.PI / 180;
  const cos = Math.cos(bearing);
  const sin = Math.sin(bearing);
  return { ...surface, points: surface.points.map(({ x, y }) => ({
    x: pose.x + cos * x + sin * y, y: pose.y - sin * x + cos * y,
  })) };
}

function inRange(distMm: number | undefined, maxMm: number): distMm is number {
  return distMm !== undefined && distMm >= MIN_RANGE_MM && distMm <= maxMm;
}

export function isPlottedEcho(point: ScanPoint, maxMm = SCAN_MAX_MM): boolean {
  return point.mark === 'echo' && inRange(point.distMm, maxMm);
}

export function isRecentSonarPing(packet: LinkPacket | undefined, nowMs: number): packet is LinkPacket {
  if (packet === undefined) return false;
  const age = nowMs - packet.timestamp;
  return age >= 0 && age <= CURRENT_RETURN_MAX_AGE_MS;
}

export function currentScanReturn(packet: LinkPacket | undefined, nowMs: number, ready: boolean): ScanReturn | null {
  if (!ready || !isRecentSonarPing(packet, nowMs)) return null;
  const { dist_mm: distMm, echo_class: echoClass, servo_deci: servoDeci } = packet.values;
  if (echoClass !== ENUMS.echoClass.valid ||
      !inRange(distMm, SCAN_MAX_MM) || servoDeci === undefined || !(Math.abs(servoDeci) <= SERVO_LIMIT_DECI)) return null;
  return { angleDeg: servoDeci / 10, distMm };
}

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
    if (word === undefined || !Number.isInteger(word) || word < 0 || word > 0xffff) continue;
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

function supportedEdge(edge: ScanPoint, inner: ScanPoint, flank?: ScanPoint, project: (point: ScanPoint) => XY = scanPosition): boolean {
  if (flank?.mark !== 'open') return false;
  const a = project(edge);
  const b = project(inner);
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const span = Math.hypot(dx, dy);
  // The ray must have a useful normal component to the observed surface.
  const ray = edge.angleDeg * Math.PI / 180;
  const normal = Math.abs(Math.sin(ray) * dy + Math.cos(ray) * dx) / span;
  return span >= EDGE_MIN_SPAN_MM && normal >= EDGE_MIN_NORMAL_COMPONENT;
}

/** Join observed slots of a complete profile without bridging unknown space or depth jumps. */
export function scanSurfaces(
  points: ScanPoint[],
  maxMm = SCAN_MAX_MM,
  profileStatus?: number,
  project: (point: ScanPoint) => XY = scanPosition,
): ScanSurface[] {
  if (points.length !== SLOT_COUNT || profileStatus === ENUMS.profileStatus.pilotSparse) return [];
  const surfaces: ScanSurface[] = [];
  let run: ScanPoint[] = [];
  const finishRun = () => {
    if (run.length >= SURFACE_MIN_SLOTS) {
      const first = run[0]!;
      const last = run.at(-1)!;
      const edges: ScanPoint[] = [];
      if (supportedEdge(first, run[2]!, points[first.slot - 1], project)) edges.push(first);
      if (supportedEdge(last, run.at(-3)!, points[last.slot + 1], project)) edges.push(last);
      surfaces.push({ points: run, edges });
    }
    run = [];
  };
  for (const point of points) {
    const previous = run.at(-1);
    if (!isPlottedEcho(point, maxMm)) {
      finishRun();
      continue;
    }
    if (previous !== undefined && Math.abs(point.distMm - previous.distMm) > SURFACE_MAX_RANGE_STEP_MM) finishRun();
    run.push(point);
  }
  finishRun();
  return surfaces;
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
