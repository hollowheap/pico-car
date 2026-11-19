import type { EventKind } from './events';
import type { EventRow, ObstacleStamp, TrailPoint } from './telemetry-store';

export type CoursePoint = { x: number; y: number };

export type CourseMarker = CoursePoint & {
  kind: EventKind;
  title: string;
  eventId: number;
  odometerMm: number;
};

export type Course = {
  points: CoursePoint[];
  markers: CourseMarker[];
  /** The obstacle the last hug found, null before one. */
  obstacle: ObstacleBox | null;
  /** Smallest box around the points, in mm. */
  bounds: { minX: number; maxX: number; minY: number; maxY: number };
};

/**
 * Drawn size of a side the firmware has not measured yet (mm). A small
 * round figure; the side is dashed so it reads as a guess.
 */
export const OBSTACLE_PLACEHOLDER_MM = 100;

/** The firmware saturates the face, width and breadth words at 12 bits (mm). */
const WIRE_MAX_MM = 4095;
const wireMm = (mm: number): number => (Number.isFinite(mm) ? Math.min(Math.max(mm, 0), WIRE_MAX_MM) : 0);

export type ObstacleBox = {
  /** Near left, near right, far right, far left, seen from the car. */
  corners: [CoursePoint, CoursePoint, CoursePoint, CoursePoint];
  /** The width is unknown: the two side edges are dashed. */
  widthDashed: boolean;
  /** The depth is unknown: the far edge is dashed. */
  depthDashed: boolean;
};

/**
 * The box a hug found. The near face is `faceMm` ahead of the pose along its
 * heading (the firmware measures it from the axle) and the box is centred on
 * the heading line: the frame carries no lateral offset. A size of 0, or one
 * that is not finite, means unknown; sizes above the wire range are clamped.
 */
export function placeObstacle(
  pose: CoursePoint & { headingDeg: number },
  faceMm: number,
  widthMm: number,
  breadthMm: number,
): ObstacleBox {
  const near = wireMm(faceMm);
  const measuredWidth = wireMm(widthMm);
  const measuredDepth = wireMm(breadthMm);
  const width = measuredWidth > 0 ? measuredWidth : OBSTACLE_PLACEHOLDER_MM;
  const depth = measuredDepth > 0 ? measuredDepth : OBSTACLE_PLACEHOLDER_MM;
  const rad = (pose.headingDeg * Math.PI) / 180;
  const ahead = { x: Math.sin(rad), y: Math.cos(rad) };
  const right = { x: Math.cos(rad), y: -Math.sin(rad) };
  const at = (along: number, across: number): CoursePoint => ({
    x: pose.x + ahead.x * along + right.x * across,
    y: pose.y + ahead.y * along + right.y * across,
  });
  const far = near + depth;
  return {
    corners: [at(near, -width / 2), at(near, width / 2), at(far, width / 2), at(far, -width / 2)],
    widthDashed: measuredWidth <= 0,
    depthDashed: measuredDepth <= 0,
  };
}

/** Accessibility text of the box: which sides are only a guess. */
export function obstacleLabel(box: ObstacleBox): string {
  const unmeasured = [box.widthDashed ? 'width' : null, box.depthDashed ? 'depth' : null].filter((side) => side !== null);
  return unmeasured.length === 0 ? 'obstacle estimate' : `obstacle estimate, ${unmeasured.join(' and ')} not measured`;
}

/** Events that do not belong on the trace (they say nothing about the place). */
const NOT_ON_MAP: EventKind[] = ['command', 'link', 'system'];

/**
 * Dead-reckon the course from the odometer and the compass heading. North is up,
 * x grows to the east. The first point is the origin. There is no GPS: the
 * trace drifts with the heading error.
 */
export function buildCourse(
  trail: TrailPoint[],
  events: EventRow[],
  stamp: ObstacleStamp | null = null,
  trailStart: number | null = null,
): Course {
  const points: CoursePoint[] = [];
  let x = 0;
  let y = 0;
  let heading: number | null = null;
  let previous: TrailPoint | undefined;
  let minOdometer = Infinity;
  let maxOdometer = -Infinity;
  for (const sample of trail) {
    minOdometer = Math.min(minOdometer, sample.odometerMm);
    maxOdometer = Math.max(maxOdometer, sample.odometerMm);
    if (sample.headingDeg !== null) heading = sample.headingDeg;
    if (previous !== undefined && heading !== null) {
      const step = sample.odometerMm - previous.odometerMm;
      const rad = (heading * Math.PI) / 180;
      x += step * Math.sin(rad);
      y += step * Math.cos(rad);
    }
    points.push({ x, y });
    previous = sample;
  }
  const markers: CourseMarker[] = [];
  for (const event of events) {
    if (NOT_ON_MAP.includes(event.text.kind)) continue;
    if (trailStart !== null && event.timestamp < trailStart) continue;
    if (event.odometerMm < minOdometer || event.odometerMm > maxOdometer) continue;
    const at = pointAtPacket(trail, points, event.timestamp, event.odometerMm);
    if (at !== null) {
      markers.push({ ...at, kind: event.text.kind, title: event.text.title, eventId: event.eventId, odometerMm: event.odometerMm });
    }
  }
  let obstacle: ObstacleBox | null = null;
  // No face reading yet: the box would sit on the axle and read as measured.
  if (stamp !== null && stamp.headingDeg !== null && wireMm(stamp.faceMm) > 0) {
    let index = firstAtOrAfter(trail, stamp.timestamp);
    while (trail[index + 1]?.timestamp === stamp.timestamp) index += 1;
    const at = trail[index]?.timestamp === stamp.timestamp ? (points[index] ?? null) : null;
    if (at !== null) obstacle = placeObstacle({ ...at, headingDeg: stamp.headingDeg }, stamp.faceMm, stamp.widthMm, stamp.breadthMm);
  }
  const xs = [...points, ...(obstacle?.corners ?? [])].map((point) => point.x);
  const ys = [...points, ...(obstacle?.corners ?? [])].map((point) => point.y);
  return {
    points,
    markers,
    obstacle,
    bounds: {
      minX: Math.min(0, ...xs),
      maxX: Math.max(0, ...xs),
      minY: Math.min(0, ...ys),
      maxY: Math.max(0, ...ys),
    },
  };
}

/** The first index whose timestamp is at or after `timestamp`; `trail.length` when none is. */
function firstAtOrAfter(trail: TrailPoint[], timestamp: number): number {
  let low = 0;
  let high = trail.length;
  while (low < high) {
    const mid = (low + high) >> 1;
    if ((trail[mid]?.timestamp ?? Infinity) < timestamp) low = mid + 1;
    else high = mid;
  }
  return low;
}

/**
 * The odometer falls in a reverse, so one value is reached on several passes. A
 * packet's timestamp says which: take the segment nearest in trail order that spans the
 * value; no segment spanning it means no place. An event older than the trail
 * lands on the first segment that spans the value.
 */
function pointAtPacket(trail: TrailPoint[], points: CoursePoint[], timestamp: number, odometerMm: number): CoursePoint | null {
  if (trail.length === 0) return null;
  const start = Math.min(Math.max(firstAtOrAfter(trail, timestamp), 1), trail.length - 1);
  for (let step = 0; step < trail.length; step += 1) {
    for (const k of step === 0 ? [start] : [start - step, start + step]) {
      const at = pointInSegment(trail, points, k, odometerMm);
      if (at !== null) return at;
    }
  }
  return null;
}

function pointInSegment(trail: TrailPoint[], points: CoursePoint[], k: number, odometerMm: number): CoursePoint | null {
  const a = trail[k - 1];
  const b = trail[k];
  const pa = points[k - 1];
  const pb = points[k];
  if (a === undefined || b === undefined || pa === undefined || pb === undefined) return null;
  if (odometerMm < Math.min(a.odometerMm, b.odometerMm) || odometerMm > Math.max(a.odometerMm, b.odometerMm)) return null;
  const span = b.odometerMm - a.odometerMm;
  const t = span === 0 ? 0 : (odometerMm - a.odometerMm) / span;
  return { x: pa.x + (pb.x - pa.x) * t, y: pa.y + (pb.y - pa.y) * t };
}
