import type { EventKind } from './events';
import type { EventRow, TrailPoint } from './telemetry-store';

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
  /** Smallest box around the points, in mm. */
  bounds: { minX: number; maxX: number; minY: number; maxY: number };
};

/** Events that do not belong on the trace (they say nothing about the place). */
const NOT_ON_MAP: EventKind[] = ['command', 'link', 'system'];

/**
 * Dead-reckon the course from the odometer and the compass heading. North is up,
 * x grows to the east. The first point is the origin. There is no GPS: the
 * trace drifts with the heading error.
 */
export function buildCourse(trail: TrailPoint[], events: EventRow[]): Course {
  const points: CoursePoint[] = [];
  let x = 0;
  let y = 0;
  let heading: number | null = null;
  let previous: TrailPoint | undefined;
  for (const sample of trail) {
    if (sample.headingDeg !== null) heading = sample.headingDeg;
    if (previous !== undefined && heading !== null) {
      const step = sample.odometerMm - previous.odometerMm;
      if (step > 0) {
        const rad = (heading * Math.PI) / 180;
        x += step * Math.sin(rad);
        y += step * Math.cos(rad);
      }
    }
    points.push({ x, y });
    previous = sample;
  }
  const markers: CourseMarker[] = [];
  for (const event of events) {
    if (NOT_ON_MAP.includes(event.text.kind)) continue;
    const at = pointAtOdometer(trail, points, event.odometerMm);
    if (at !== null) {
      markers.push({ ...at, kind: event.text.kind, title: event.text.title, eventId: event.eventId, odometerMm: event.odometerMm });
    }
  }
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  return {
    points,
    markers,
    bounds: {
      minX: Math.min(0, ...xs),
      maxX: Math.max(0, ...xs),
      minY: Math.min(0, ...ys),
      maxY: Math.max(0, ...ys),
    },
  };
}

function pointAtOdometer(trail: TrailPoint[], points: CoursePoint[], odometerMm: number): CoursePoint | null {
  if (trail.length === 0) return null;
  const first = trail[0];
  const last = trail[trail.length - 1];
  if (first === undefined || last === undefined) return null;
  if (odometerMm < first.odometerMm || odometerMm > last.odometerMm) return null;
  for (let i = 1; i < trail.length; i += 1) {
    const a = trail[i - 1];
    const b = trail[i];
    const pa = points[i - 1];
    const pb = points[i];
    if (a === undefined || b === undefined || pa === undefined || pb === undefined) continue;
    if (odometerMm <= b.odometerMm) {
      const span = b.odometerMm - a.odometerMm;
      const t = span === 0 ? 0 : (odometerMm - a.odometerMm) / span;
      return { x: pa.x + (pb.x - pa.x) * t, y: pa.y + (pb.y - pa.y) * t };
    }
  }
  return null;
}
