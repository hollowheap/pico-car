import { closestPointOnSegment, type Course, type CoursePoint, type ObstacleContact } from './course';
import type { LinkPacket } from './link';
import { profileHits, profileSurfaces, sparseProfileSurfaces, type MeasuredSurface } from './scan';

export type ObstacleView = {
  contact: ObstacleContact | null;
  surfaces: MeasuredSurface[];
  hits?: CoursePoint[];
  car: CoursePoint;
  carLabel: 'Car' | 'Last fix' | 'At scan' | 'Origin';
  heading: number | null;
  lastMappedAt?: number;
};

/** A hug and a survey have independent origins. Show the latest shape that can be located. */
export function obstacleView(course: Course, profile: LinkPacket | undefined): ObstacleView {
  const survey = course.surfaces.reduce<Course['surfaces'][number] | undefined>(
    (last, face) => !last || face.timestamp > last.timestamp ? face : last, undefined);
  const contact = !survey ? course.obstacle : null;
  const mapped = course.surfaces;
  const stamp = contact?.timestamp ?? survey?.timestamp;
  const newer = profile && (stamp === undefined || profile.timestamp > stamp ||
    (profile.timestamp === stamp && !contact && profile.values.req_id !== survey?.request));
  if (newer) {
    const local = profileSurfaces(profile.values);
    const hits = profileHits(profile.values);
    if (local.length) return { contact: null, surfaces: local, hits, car: { x: 0, y: 0 }, carLabel: 'At scan', heading: 0 };
    if (stamp === undefined && hits.length) {
      return {
        contact: null,
        surfaces: sparseProfileSurfaces(profile.values),
        hits,
        car: { x: 0, y: 0 },
        carLabel: 'At scan',
        heading: 0,
      };
    }
  }
  return { contact, surfaces: mapped, car: course.points.at(-1) ?? { x: 0, y: 0 },
    carLabel: course.positionKnown ? 'Car' : contact || mapped.length ? 'Last fix' : 'Origin',
    heading: course.positionKnown ? course.samples.at(-1)?.headingDeg ?? null : null,
    ...(newer && stamp !== undefined && { lastMappedAt: stamp }) };
}

/** Project the axle onto the closest supported boundary segment, including its endpoints. */
export function nearestBoundary(car: CoursePoint, outlines: CoursePoint[][]): CoursePoint | undefined {
  let nearest: CoursePoint | undefined;
  let best = Infinity;
  for (const points of outlines) {
    for (let i = 1; i < points.length; i++) {
      const point = closestPointOnSegment(car, points[i - 1]!, points[i]!);
      const distance = Math.hypot(point.x - car.x, point.y - car.y);
      if (distance < best) {
        nearest = point;
        best = distance;
      }
    }
  }
  return nearest;
}
