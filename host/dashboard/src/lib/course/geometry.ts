import type { EventKind } from '../events';
import type { LinkPacket } from '../link';
import { odometerDiscontinuous } from '../odometer';
import type { MeasuredSurface } from '../scan';
import { hasSpeedFault } from '../schema';
import { BIT_FIELDS, ENUMS } from '../schema.generated';
import type { EventRow, TrailPoint } from '../telemetry-store';

export type CoursePoint = { x: number; y: number };

/** Orthogonal projection onto a finite segment, clamped to its endpoints. */
export function closestPointOnSegment(point: CoursePoint, a: CoursePoint, b: CoursePoint): CoursePoint {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSq = dx * dx + dy * dy;
  const fraction = lengthSq === 0 ? 0 : ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSq;
  const t = Math.max(0, Math.min(1, fraction));
  return { x: a.x + t * dx, y: a.y + t * dy };
}

export type CourseMarker = CoursePoint & {
  kind: EventKind;
  title: string;
  eventId: number;
  odometerMm: number;
  event: EventRow;
  speedMms: number | null;
};

export type CourseJunction = CoursePoint & {
  headingDeg: number;
  direction: 'left' | 'right' | 'uturn' | 'straight' | 'cross';
  corroborated?: boolean;
  timestamp: number;
  eventId: number;
};

export type CourseHump = CoursePoint & {
  headingDeg: number;
  lengthMm: number;
  widthMm: number;
  peakMm: number | null;
  timestamp: number;
};

export type Course = {
  points: CoursePoint[];
  samples: TrailPoint[];
  hasGap: boolean;
  positionKnown: boolean;
  headingReference: 'compass' | 'relative' | 'unavailable';
  bootTimestamp: number | null;
  markers: CourseMarker[];
  junctions: CourseJunction[];
  humps: CourseHump[];
  /** The obstacle the last hug found, null before one. */
  obstacle: (ObstacleContact & { timestamp: number }) | null;
  surfaces: (MeasuredSurface & { timestamp: number; request: number })[];
  forwardReturn: { rangeMm: number; beam: { from: CoursePoint; to: CoursePoint } | null } | null;
  /** Smallest box around the points, in mm. */
  bounds: { minX: number; maxX: number; minY: number; maxY: number };
};

export type CourseOrientation = 'north-up' | 'car-up';

/** Transform a world point into the car-up frame centered on `origin` facing `headingDeg`. */
export function rotateToCarUp(point: CoursePoint, origin: CoursePoint, headingDeg: number | null): CoursePoint {
  const dx = point.x - origin.x;
  const dy = point.y - origin.y;
  if (headingDeg === null || headingDeg === 0) return { x: dx, y: dy };
  const radians = (headingDeg * Math.PI) / 180;
  const cos = Math.cos(radians);
  const sin = Math.sin(radians);
  return { x: cos * dx - sin * dy, y: sin * dx + cos * dy };
}

/** View coordinates only. The history and contact pose remain in the world frame. */
export function orientCourse(course: Course, orientation: CourseOrientation): {
  course: Course;
  headingDeg: number | null;
  northDeg: number;
  origin: CoursePoint;
} {
  const heading = course.positionKnown ? course.samples.at(-1)?.headingDeg ?? null : null;
  const car = course.points.at(-1);
  if (orientation === 'north-up' || heading === null || car === undefined) {
    return { course, headingDeg: heading, northDeg: 0, origin: { x: 0, y: 0 } };
  }

  const project = (point: CoursePoint): CoursePoint => rotateToCarUp(point, car, heading);
  const points = course.points.map(project);
  const surfaces = course.surfaces.map((surface) => ({ ...surface, points: surface.points.map(project) }));
  const markers = course.markers.map((marker) => ({ ...marker, ...project(marker) }));
  const junctions = course.junctions.map((j) => ({ ...j, ...project(j), headingDeg: headingWrap(j.headingDeg - heading) }));
  const humps = course.humps.map((h) => ({ ...h, ...project(h), headingDeg: headingWrap(h.headingDeg - heading) }));
  const contact = course.obstacle;
  const obstacle = contact && { ...contact, ...project(contact), headingDeg: headingWrap(contact.headingDeg - heading) };
  const lastReturn = course.forwardReturn;
  const beam = lastReturn?.beam;
  const forwardReturn = lastReturn === null || beam == null ? lastReturn : {
    ...lastReturn,
    beam: { from: project(beam.from), to: project(beam.to) },
  };

  return {
    headingDeg: 0,
    northDeg: -heading,
    origin: project({ x: 0, y: 0 }),
    course: { ...course, points, markers, junctions, humps, obstacle, forwardReturn, surfaces, bounds: courseBounds(points, obstacle, surfaces, humps) },
  };
}

/** The firmware saturates the face, width and breadth words at 12 bits (mm). */
const WIRE_MAX_MM = 4095;
export const wireMm = (mm: number): number => (Number.isFinite(mm) ? Math.min(Math.max(mm, 0), WIRE_MAX_MM) : 0);

export type ObstacleContact = CoursePoint & { headingDeg: number };

/** Project only the front contact; edge endpoints come from located survey rays. */
export function placeContact(pose: ObstacleContact, faceMm: number): ObstacleContact {
  const rad = pose.headingDeg * RAD_PER_DEG;
  return { x: pose.x + wireMm(faceMm) * Math.sin(rad), y: pose.y + wireMm(faceMm) * Math.cos(rad),
    headingDeg: headingWrap(pose.headingDeg) };
}

export const RAD_PER_DEG = Math.PI / 180;
export const headingWrap = (degrees: number) => ((degrees % 360) + 360) % 360;

// The chassis wheel track matches CHASSIS_WHEEL_TRACK_MM.
export const WHEEL_TRACK_MM = 107;

function wheelYaw(wheels: NonNullable<TrailPoint['wheels']>, pivotScalePct?: number): number {
  const { leftMms, rightMms } = wheels;
  const scale = pivotScalePct !== undefined && leftMms * rightMms < 0 ? pivotScalePct / 100 : 1;
  return (leftMms - rightMms) * scale / WHEEL_TRACK_MM;
}

export function courseBounds(
  points: CoursePoint[],
  obstacle: ObstacleContact | null,
  surfaces: MeasuredSurface[],
  humps: readonly CourseHump[] = [],
): Course['bounds'] {
  const bounds = { minX: 0, maxX: 0, minY: 0, maxY: 0 };
  for (const { x, y } of [...points, ...(obstacle ? [obstacle] : []), ...surfaces.flatMap((face) => face.points), ...humps]) {
    bounds.minX = Math.min(bounds.minX, x);
    bounds.maxX = Math.max(bounds.maxX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.maxY = Math.max(bounds.maxY, y);
  }
  return bounds;
}

// Firmware status and paired motion are published at 5 Hz.
const STATUS_PERIOD_S = 0.2;
// Wheel reads and odometer are rounded on the wire and taken separately.
const ODOMETER_PAIR_SLACK_MM = 3;

type CourseStep = { distance: number; turn: number; wheelTurn: boolean };

function courseStep(a: TrailPoint, b: TrailPoint): CourseStep | null {
  const gap = a.seq === undefined || b.seq === undefined ? 1 : (b.seq - a.seq + 65536) % 65536;
  if (gap < 1 || gap > 2 || odometerDiscontinuous(a.odometerMm, b.odometerMm)) return null;
  const lateReceipt = b.timestamp - a.timestamp > SPEED_GAP_MS;
  const distance = b.odometerMm - a.odometerMm;
  const aw = a.wheels;
  const bw = b.wheels;
  if (aw !== undefined && bw !== undefined) {
    const left = (aw.leftMms + bw.leftMms) / 2;
    const right = (aw.rightMms + bw.rightMms) / 2;
    const dt = gap * STATUS_PERIOD_S;
    // Both reported wheels can be zero when the firmware lacks fresh wheel
    // feedback. Signed odometer travel rules out treating that as straight.
    const atOdometerFloor = a.odometerMm === 0 && b.odometerMm === 0;
    const aZero = aw.leftMms === 0 && aw.rightMms === 0;
    const bZero = bw.leftMms === 0 && bw.rightMms === 0;
    const credibleWheels = (!aZero || !a.speedMms) && (!bZero || !b.speedMms) && (!aZero || !bZero || distance === 0);
    const aSpeed = (aw.leftMms + aw.rightMms) / 2;
    const bSpeed = (bw.leftMms + bw.rightMms) / 2;
    const consistentTurn = Math.sign(aw.leftMms - aw.rightMms) === Math.sign(bw.leftMms - bw.rightMms);
    const consistentTravel = distance >= dt * Math.min(aSpeed, bSpeed) - ODOMETER_PAIR_SLACK_MM &&
      distance <= dt * Math.max(aSpeed, bSpeed) + ODOMETER_PAIR_SLACK_MM;
    if (credibleWheels && ((gap === 1 && !lateReceipt) || (consistentTurn && consistentTravel))) {
      // The pivot witness describes the current sample, so do not extend a
      // correction through a missing pass. Integrate the two endpoint rates.
      const turn = (wheelYaw(aw, gap === 1 ? a.pivotScalePct : undefined) +
        wheelYaw(bw, gap === 1 ? b.pivotScalePct : undefined)) * dt / 2;
      return {
        distance: atOdometerFloor ? Math.min(0, dt * (left + right) / 2) : distance,
        turn,
        wheelTurn: true,
      };
    }
  }
  if (gap > 1 || lateReceipt || a.headingDeg === null || b.headingDeg === null) return null;
  return { distance, turn: (headingWrap(b.headingDeg - a.headingDeg + 180) - 180) * RAD_PER_DEG, wheelTurn: false };
}

// Keep one world frame for the boot. An unsupported interval contributes no
// displacement. The next supported step starts at the last plotted location.
export function reconstruct(trail: TrailPoint[]) {
  const points: CoursePoint[] = [];
  const samples: TrailPoint[] = [];
  let point = { x: 0, y: 0 };
  let headingReference: Course['headingReference'] = 'unavailable';
  let lastHeading = 0;
  let hasGap = false;
  for (const [i, sample] of trail.entries()) {
    const previous = trail[i - 1];
    const checkpoint = sample.checkpoint;
    let headingDeg: number | null = null;
    if (checkpoint !== undefined) {
      point = { x: checkpoint.x, y: checkpoint.y };
      headingDeg = checkpoint.headingDeg;
      headingReference = checkpoint.reference;
      hasGap ||= checkpoint.hasGap;
    } else if (previous === undefined) {
      headingDeg = sample.headingDeg;
      if (headingDeg !== null) headingReference = 'compass';
    } else {
      const step = courseStep(previous, sample);
      if (step === null) {
        hasGap = true;
      } else {
        if (headingReference === 'unavailable') headingReference = previous.headingDeg === null ? 'relative' : 'compass';
        const startHeading = samples[i - 1]!.headingDeg ??
          (headingReference === 'compass' ? previous.headingDeg ?? lastHeading : lastHeading);
        // A first pair of wheel observations can establish the initial bearing.
        if (i === 1 && samples[0]!.headingDeg === null) samples[0] = { ...previous, headingDeg: startHeading };
        const next = startHeading * RAD_PER_DEG + step.turn;
        // A compass-only quarter turn has no observed radius. Place its
        // travel along the end bearing; ordinary adjacent samples use an arc.
        const halfTurn = step.wheelTurn || Math.abs(step.turn) < Math.PI / 2 ? step.turn / 2 : 0;
        const distance = halfTurn === 0 ? step.distance : step.distance * Math.sin(halfTurn) / halfTurn;
        point = { x: point.x + distance * Math.sin(next - halfTurn), y: point.y + distance * Math.cos(next - halfTurn) };
        headingDeg = headingWrap(next / RAD_PER_DEG);
      }
    }
    if (headingDeg !== null) lastHeading = headingDeg;
    points.push(point);
    samples.push(headingDeg === sample.headingDeg ? sample : { ...sample, headingDeg });
  }
  return { points, samples, headingReference, hasGap };
}

export type SpeedBand = 'slow' | 'medium' | 'fast' | 'unknown';
export type CourseStroke = { points: CoursePoint[]; band: SpeedBand; recent: boolean };
export type CourseTraceSegment = {
  points: CoursePoint[];
  band: SpeedBand;
  speedMms: number | null;
  pitchDeg: number | null;
  hump: boolean;
  slip: boolean;
  recent: boolean;
  opacity: number;
  overlapped: boolean;
  estimated?: boolean;
};


const RECENT_TRAVEL_MS = 5000;
export const SPEED_GAP_MS = 1000;
const MEDIUM_SPEED_MMS = 60;
const FAST_SPEED_MMS = 150;

export function sampleSpeed(sample: TrailPoint): number | null {
  if (sample.speedFault) return null;
  if (sample.speedMms !== null) return sample.speedMms;
  const wheels = sample.wheels;
  return wheels === undefined ? null : Math.trunc((wheels.leftMms + wheels.rightMms) / 2);
}

/** Status need not contain an odometer. Use wheel evidence only for the same pass. */
export function latestCourseSpeed(status: LinkPacket | undefined, last: TrailPoint | undefined): number | null {
  if (status === undefined) return last === undefined ? null : sampleSpeed(last);
  if (hasSpeedFault(status.values.faults)) return null;
  if (status.values.speed_mms !== undefined) return status.values.speed_mms;
  return last !== undefined && last.timestamp === status.timestamp && last.seq === status.values.seq ? sampleSpeed(last) : null;
}

/** Group adjacent segments with the same appearance to keep the drawing small. */
export function courseStrokes(course: Course): CourseStroke[] {
  const { points, samples } = course;
  const recentSince = (samples.at(-1)?.timestamp ?? 0) - RECENT_TRAVEL_MS;
  const strokes: CourseStroke[] = [];
  let active: CourseStroke | undefined;
  for (let i = 1; i < points.length; i += 1) {
    const from = points[i - 1]!;
    const to = points[i]!;
    if (from.x === to.x && from.y === to.y) continue;

    const sample = samples[i]!;
    const gap = sample.timestamp - samples[i - 1]!.timestamp;
    const recent = sample.timestamp >= recentSince;
    const band = speedBand(sampleSpeed(sample), gap);
    if (active === undefined || active.band !== band || active.recent !== recent) {
      active = { points: [from], band, recent };
      strokes.push(active);
    }
    active.points.push(to);
  }
  return strokes;
}

function speedBand(speedMms: number | null, gapMs: number): SpeedBand {
  if (speedMms === null || gapMs > SPEED_GAP_MS) return 'unknown';
  const speed = Math.abs(speedMms);
  if (speed >= FAST_SPEED_MMS) return 'fast';
  return speed >= MEDIUM_SPEED_MMS ? 'medium' : 'slow';
}

const STRAIGHT_STEP_MAX_RAD = 4.5 * (Math.PI / 180);
const STRAIGHT_RUN_MAX_RAD = 12 * (Math.PI / 180);
const STRAIGHT_MIN_STEPS = 3;
const STRAIGHT_BLEND = 0.75;
const GAP_ESTIMATE_MAX_MM = 600;
const GAP_ESTIMATE_MAX_MS = 3000;
const GAP_STRAIGHT_MAX_RAD = 35 * (Math.PI / 180);
const OVERLAP_CELL_MM = 45;
const SPEED_BUCKET_MMS = 10;
const IR_FORWARD_MM = 130;
const IR_HALF_WIDTH_MM = 22.5;
const TAPE_BREAK_JUMP_MM = 500;
const TAPE_BREAK_GAP_MS = 6000;
const SLIP_YAW_DIFF_DEG = 12;
const SLIP_WHEEL_DIFF_MMS = 35;

/** Detect wheel slip when encoder-implied yaw diverges from compass heading or one wheel spins without turning. */
export function isStepSlip(prev: TrailPoint, curr: TrailPoint): boolean {
  const wheels = curr.wheels;
  const dtMs = curr.timestamp - prev.timestamp;
  if (wheels === undefined || dtMs <= 0 || dtMs > SPEED_GAP_MS) return false;
  if (prev.headingDeg === null || curr.headingDeg === null) return false;
  const actualYawDeg = ((curr.headingDeg - prev.headingDeg + 540) % 360) - 180;
  const wheelYawDeg = (wheelYaw(wheels, curr.pivotScalePct) * (dtMs / 1000)) * (180 / Math.PI);
  if (Math.abs(wheelYawDeg - actualYawDeg) >= SLIP_YAW_DIFF_DEG) return true;
  const diff = Math.abs(wheels.leftMms - wheels.rightMms);
  const peak = Math.max(Math.abs(wheels.leftMms), Math.abs(wheels.rightMms));
  return diff >= SLIP_WHEEL_DIFF_MMS && peak >= 40 && Math.abs(actualYawDeg) <= 3;
}

function angleDiff(a: number, b: number): number {
  let d = (a - b) % (2 * Math.PI);
  if (d > Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

/**
 * Damp small oscillating encoder-heading jitter along straightaways and estimate
 * short data gaps as straight travel from odometer delta and surrounding headings.
 */
export function stabilizeStraightPoints(points: readonly CoursePoint[], samples: readonly TrailPoint[]): CoursePoint[] {
  if (points.length < 2) return points.slice();
  type Step = { dx: number; dy: number; dist: number; bearing: number; valid: boolean; gapEstimated: boolean };
  const steps: Step[] = [];
  let lastBearing: number | null = samples[0]?.headingDeg != null ? samples[0].headingDeg * RAD_PER_DEG : null;

  for (let i = 1; i < points.length; i += 1) {
    let dx = points[i]!.x - points[i - 1]!.x;
    let dy = points[i]!.y - points[i - 1]!.y;
    let dist = Math.hypot(dx, dy);
    const prev = samples[i - 1]!;
    const curr = samples[i]!;
    const gap = curr.timestamp - prev.timestamp;
    let gapEstimated = false;

    // Attenuate false forward advance during one-wheel slip only when heading isn't turning (not an intentional corner).
    if (dist > 2 && isStepSlip(prev, curr) && curr.wheels !== undefined &&
        prev.headingDeg !== null && curr.headingDeg !== null) {
      const actualYawDeg = ((curr.headingDeg - prev.headingDeg + 540) % 360) - 180;
      const slowMms = Math.min(Math.abs(curr.wheels.leftMms), Math.abs(curr.wheels.rightMms));
      const fastMms = Math.max(Math.abs(curr.wheels.leftMms), Math.abs(curr.wheels.rightMms));
      if (Math.abs(actualYawDeg) <= 4 && fastMms > 20 && slowMms < fastMms * 0.5) {
        const capDist = Math.max(1, slowMms * (gap / 1000) + 2);
        if (dist > capDist) {
          const factor = capDist / dist;
          dx *= factor;
          dy *= factor;
          dist = capDist;
        }
      }
    }

    // When a short packet gap collapsed displacement to zero, estimate straight travel from odometer delta.
    if (dist < 0.5 && gap > 0 && gap <= GAP_ESTIMATE_MAX_MS && !odometerDiscontinuous(prev.odometerMm, curr.odometerMm)) {
      const odoDelta = curr.odometerMm - prev.odometerMm;
      const startHead = prev.headingDeg !== null ? prev.headingDeg * RAD_PER_DEG : lastBearing;
      const endHead = curr.headingDeg !== null ? curr.headingDeg * RAD_PER_DEG : startHead;
      const straightEnough = endHead === null || startHead === null || Math.abs(angleDiff(endHead, startHead)) <= GAP_STRAIGHT_MAX_RAD;
      if (straightEnough && Math.abs(odoDelta) >= 2 && Math.abs(odoDelta) <= GAP_ESTIMATE_MAX_MM && startHead !== null) {
        const heading = endHead !== null ? startHead + angleDiff(endHead, startHead) / 2 : startHead;
        dx = odoDelta * Math.sin(heading);
        dy = odoDelta * Math.cos(heading);
        dist = Math.abs(odoDelta);
        gapEstimated = true;
      }
    }

    const bearing = dist >= 0.5 ? Math.atan2(dx, dy) : (curr.headingDeg !== null ? curr.headingDeg * RAD_PER_DEG : lastBearing ?? 0);
    if (dist >= 2 || curr.headingDeg !== null) lastBearing = bearing;
    steps.push({ dx, dy, dist, bearing, valid: dist >= 2 && (gap <= SPEED_GAP_MS || gapEstimated), gapEstimated });
  }

  const adjusted = steps.map((s) => ({ dx: s.dx, dy: s.dy }));
  let i = 0;
  while (i < steps.length) {
    if (!steps[i]!.valid) {
      i += 1;
      continue;
    }
    let j = i + 1;
    let sumSin = Math.sin(steps[i]!.bearing) * steps[i]!.dist;
    let sumCos = Math.cos(steps[i]!.bearing) * steps[i]!.dist;
    while (j < steps.length && steps[j]!.valid) {
      const mean = Math.atan2(sumSin, sumCos);
      if (Math.abs(angleDiff(steps[j]!.bearing, steps[j - 1]!.bearing)) > STRAIGHT_STEP_MAX_RAD ||
          Math.abs(angleDiff(steps[j]!.bearing, mean)) > STRAIGHT_RUN_MAX_RAD) break;
      sumSin += Math.sin(steps[j]!.bearing) * steps[j]!.dist;
      sumCos += Math.cos(steps[j]!.bearing) * steps[j]!.dist;
      j += 1;
    }
    if (j - i >= STRAIGHT_MIN_STEPS) {
      const mean = Math.atan2(sumSin, sumCos);
      for (let k = i; k < j; k += 1) {
        const step = steps[k]!;
        const blended = step.bearing + angleDiff(mean, step.bearing) * STRAIGHT_BLEND;
        adjusted[k] = { dx: step.dist * Math.sin(blended), dy: step.dist * Math.cos(blended) };
      }
    }
    i = j;
  }

  const out: CoursePoint[] = [{ ...points[0]! }];
  for (let k = 0; k < adjusted.length; k += 1) {
    const prev = out[k]!;
    out.push({ x: prev.x + adjusted[k]!.dx, y: prev.y + adjusted[k]!.dy });
  }
  return out;
}

/**
 * Build display segments with stabilized straightaways, continuous speed buckets
 * for gradient coloring, and attenuated opacity on overlapped historical passes.
 */
export function courseTraceSegments(course: Course): CourseTraceSegment[] {
  const { samples } = course;
  const points = stabilizeStraightPoints(course.points, samples);
  const count = points.length;
  if (count < 2) return [];

  const firstTs = samples[0]?.timestamp ?? 0;
  const lastTs = samples.at(-1)?.timestamp ?? 0;
  const spanTs = Math.max(1, lastTs - firstTs);
  const recentSince = lastTs - RECENT_TRAVEL_MS;

  // Walk newest-to-oldest to flag earlier segments whose spatial cell is revisited later.
  const seenCells = new Set<string>();
  const overlapped = new Array<boolean>(count).fill(false);
  for (let i = count - 1; i >= 1; i -= 1) {
    const midX = (points[i - 1]!.x + points[i]!.x) / 2;
    const midY = (points[i - 1]!.y + points[i]!.y) / 2;
    const cell = `${Math.floor(midX / OVERLAP_CELL_MM)},${Math.floor(midY / OVERLAP_CELL_MM)}`;
    if (seenCells.has(cell)) overlapped[i] = true;
    else seenCells.add(cell);
  }

  const segments: CourseTraceSegment[] = [];
  let active: CourseTraceSegment | undefined;
  let activeKey = '';

  for (let i = 1; i < count; i += 1) {
    const from = points[i - 1]!;
    const to = points[i]!;
    if (from.x === to.x && from.y === to.y) continue;

    const sample = samples[i]!;
    const rawDist = Math.hypot(course.points[i]!.x - course.points[i - 1]!.x, course.points[i]!.y - course.points[i - 1]!.y);
    const estimated = rawDist < 0.5 && Math.hypot(to.x - from.x, to.y - from.y) >= 1;
    const gap = sample.timestamp - samples[i - 1]!.timestamp;
    const recent = sample.timestamp >= recentSince;
    const rawSpeed = sampleSpeed(sample);
    const band = estimated ? 'unknown' : speedBand(rawSpeed, gap);
    const speedMms = band === 'unknown' || rawSpeed === null ? null : Math.round(Math.abs(rawSpeed) / SPEED_BUCKET_MMS) * SPEED_BUCKET_MMS;
    const pitchDeg = sample.pitchDeg === undefined ? null : Math.round(sample.pitchDeg / 2) * 2;
    const hump = sample.humpState !== undefined && sample.humpState !== ENUMS.humpState.flat;
    const slip = isStepSlip(samples[i - 1]!, sample);
    const age = (sample.timestamp - firstTs) / spanTs;
    const baseOpacity = recent ? 0.78 : 0.26 + 0.28 * age;
    const isOverlap = !recent && overlapped[i]!;
    const opacity = Math.round((isOverlap ? baseOpacity * 0.5 : baseOpacity) * 100) / 100;
    const key = `${band}:${speedMms ?? 'u'}:${pitchDeg ?? 'n'}:${hump ? 1 : 0}:${slip ? 1 : 0}:${recent ? 1 : 0}:${opacity}:${isOverlap ? 1 : 0}:${estimated ? 1 : 0}`;

    if (active === undefined || activeKey !== key) {
      active = { points: [from], band, speedMms, pitchDeg, hump, slip, recent, opacity, overlapped: isOverlap, ...(estimated && { estimated: true }) };
      activeKey = key;
      segments.push(active);
    }
    active.points.push(to);
  }

  return segments;
}

const LEFT_TAPE_BIT = 1 << BIT_FIELDS.lineBits.left;
const RIGHT_TAPE_BIT = 1 << BIT_FIELDS.lineBits.right;
const TAPE_MASK = LEFT_TAPE_BIT | RIGHT_TAPE_BIT;

/**
 * Project observed floor tape to the actual 2D front IR sensor position
 * (+130 mm ahead of the rear drive axle, ±22.5 mm lateral for single-sensor hits).
 * Keeps the tape continuous across brief straddles between sensors, only breaking on
 * large gaps or detours, and smooths alternating L/R hits onto the tape centerline.
 */
export function courseTapeRuns(course: Course): CoursePoint[][] {
  const { samples } = course;
  const points = stabilizeStraightPoints(course.points, samples);
  const rawRuns: CoursePoint[][] = [];
  let active: CoursePoint[] | undefined;
  let lastTapeTs = -Infinity;
  let lastRad: number | null = samples[0]?.headingDeg != null ? samples[0].headingDeg * RAD_PER_DEG : null;

  for (let i = 0; i < points.length; i += 1) {
    const sample = samples[i]!;
    const axle = points[i]!;
    if (sample.headingDeg !== null) {
      lastRad = sample.headingDeg * RAD_PER_DEG;
    } else if (i > 0) {
      const dx = axle.x - points[i - 1]!.x;
      const dy = axle.y - points[i - 1]!.y;
      if (Math.hypot(dx, dy) >= 2) lastRad = Math.atan2(dx, dy);
    }

    const bits = sample.lineBits ?? 0;
    const leftOn = (bits & LEFT_TAPE_BIT) !== 0;
    const rightOn = (bits & RIGHT_TAPE_BIT) !== 0;
    if ((!leftOn && !rightOn) || lastRad === null) continue;

    const fx = Math.sin(lastRad);
    const fy = Math.cos(lastRad);
    const rx = Math.cos(lastRad);
    const ry = -Math.sin(lastRad);
    const lat = leftOn && rightOn ? 0 : leftOn ? -IR_HALF_WIDTH_MM : IR_HALF_WIDTH_MM;
    const pt: CoursePoint = {
      x: axle.x + IR_FORWARD_MM * fx + lat * rx,
      y: axle.y + IR_FORWARD_MM * fy + lat * ry,
    };

    const prevPt = active?.at(-1);
    if (prevPt !== undefined) {
      const jump = Math.hypot(pt.x - prevPt.x, pt.y - prevPt.y);
      if (jump < 2) {
        lastTapeTs = sample.timestamp;
        continue;
      }
      if (jump > TAPE_BREAK_JUMP_MM || sample.timestamp - lastTapeTs > TAPE_BREAK_GAP_MS) {
        active = undefined;
      }
    }
    if (active === undefined) {
      active = [pt];
      rawRuns.push(active);
    } else {
      active.push(pt);
    }
    lastTapeTs = sample.timestamp;
  }

  // Gently smooth interior points so alternating L/R sensor grazes collapse onto the tape center.
  return rawRuns.map((run) => {
    if (run.length === 1) {
      const p = run[0]!;
      return [p, { x: p.x + 0.5, y: p.y + 0.5 }];
    }
    return run.map((pt, idx) => {
      if (idx === 0 || idx === run.length - 1) return pt;
      const a = run[idx - 1]!;
      const b = run[idx + 1]!;
      if (Math.hypot(b.x - a.x, b.y - a.y) > 220) return pt;
      return {
        x: 0.25 * a.x + 0.5 * pt.x + 0.25 * b.x,
        y: 0.25 * a.y + 0.5 * pt.y + 0.25 * b.y,
      };
    });
  });
}
