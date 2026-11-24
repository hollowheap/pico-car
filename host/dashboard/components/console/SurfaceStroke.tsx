import { G, Line, Polyline } from 'react-native-svg';

import type { CoursePoint } from '../../lib/course';
import type { MeasuredSurface } from '../../lib/scan';

/** End ticks are perpendicular to the measured face. An open-ended stroke carries no bound. */
export function SurfaceStroke({ surface, project, color, pathId, startId, endId }: {
  surface: MeasuredSurface; project: (point: CoursePoint) => CoursePoint; color: string;
  pathId: string; startId: string; endId: string;
}) {
  const points = surface.points.map(project);
  const tick = (endpoint: CoursePoint, neighbor: CoursePoint, id: string) => {
    const length = Math.hypot(endpoint.x - neighbor.x, endpoint.y - neighbor.y);
    if (!length) return null;
    const dx = (endpoint.y - neighbor.y) * 4 / length;
    const dy = (neighbor.x - endpoint.x) * 4 / length;
    return <Line x1={endpoint.x - dx} x2={endpoint.x + dx} y1={endpoint.y - dy} y2={endpoint.y + dy}
      stroke={color} strokeWidth={2.5} testID={id} />;
  };
  return (
    <G>
      <Polyline fill="none" points={points.map((p) => `${p.x},${p.y}`).join(' ')} stroke={color}
        strokeLinejoin="round" strokeWidth={2.5} testID={pathId} />
      {surface.boundedStart ? tick(points[0]!, points[1]!, startId) : null}
      {surface.boundedEnd ? tick(points.at(-1)!, points.at(-2)!, endId) : null}
    </G>
  );
}
