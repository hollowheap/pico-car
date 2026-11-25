import type { Course, CourseHump, CourseOrientation, CoursePoint, ObstacleContact } from '../../../bridge/src/console/course.js';
import type { MeasuredSurface } from '../../../bridge/src/console/scan.js';
import type { EventRow } from '../../../bridge/src/console/telemetry-store.js';
export type { Course, CourseMarker, CourseOrientation, CoursePoint } from '../../../bridge/src/console/course.js';

/** Orthogonal projection onto a finite segment, clamped to its endpoints. */
export function closestPointOnSegment(point: CoursePoint, a: CoursePoint, b: CoursePoint): CoursePoint {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSq = dx * dx + dy * dy;
  const fraction = lengthSq === 0 ? 0 : ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSq;
  const t = Math.max(0, Math.min(1, fraction));
  return { x: a.x + t * dx, y: a.y + t * dy };
}

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

export function eventKey(event: Pick<EventRow, 'eventId' | 'timestamp' | 'type'>): string {
  return `${event.eventId}:${event.timestamp}:${event.type}`;
}

const headingWrap = (degrees: number) => ((degrees % 360) + 360) % 360;

function courseBounds(
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
