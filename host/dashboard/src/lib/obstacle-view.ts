import { closestPointOnSegment, type CoursePoint } from './course';
export type { ObstacleView } from '../../../bridge/src/console/obstacle-view.js';

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
