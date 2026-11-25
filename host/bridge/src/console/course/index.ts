import { placeSurface, SONAR_FORWARD_MM } from '../scan.js';
import type { EventRow, ForwardReturnStamp, ObstacleStamp, TrailPoint } from '../telemetry-store.js';
import { placeCourseEvents } from './events.js';
import { RAD_PER_DEG, courseBounds, headingWrap, placeContact, reconstruct, wireMm, type Course, type CoursePoint } from './geometry.js';

export { eventKey } from './events.js';
export {
  closestPointOnSegment,
  courseStrokes,
  courseTapeRuns,
  courseTraceSegments,
  isStepSlip,
  latestCourseSpeed,
  orientCourse,
  placeContact,
  reconstruct,
  rotateToCarUp,
  stabilizeStraightPoints,
  type Course,
  type CourseHump,
  type CourseJunction,
  type CourseMarker,
  type CourseOrientation,
  type CoursePoint,
  type CourseStroke,
  type CourseTraceSegment,
  type ObstacleContact,
  type SpeedBand,
} from './geometry.js';

export function buildCourse(
  trail: TrailPoint[],
  events: EventRow[],
  stamp: ObstacleStamp | null = null,
  trailStart: number | null = null,
  forwardStamp: ForwardReturnStamp | null = null,
): Course {
  const sessionTrail = trailStart === null ? trail : trail.filter((sample) => sample.timestamp >= trailStart);
  const { points, samples, headingReference, hasGap } = reconstruct(sessionTrail);
  const positionKnown = samples.at(-1)?.headingDeg != null;
  let minOdometer = Infinity;
  let maxOdometer = -Infinity;
  const surfaces: Course['surfaces'] = [];
  const seen = new Set<string>();
  for (const [i, sample] of samples.entries()) {
    minOdometer = Math.min(minOdometer, sample.odometerMm);
    maxOdometer = Math.max(maxOdometer, sample.odometerMm);
    if (sample.headingDeg === null || !sample.surveys) continue;
    for (const survey of sample.surveys) {
      const id = `${survey.timestamp}/${survey.request}`;
      if (seen.has(id)) continue;
      seen.add(id);
      for (const face of survey.surfaces) surfaces.push({
        ...placeSurface(face, points[i]!, sample.headingDeg), timestamp: survey.timestamp, request: survey.request,
      });
    }
  }

  const { markers, junctions, humps } = placeCourseEvents(samples, points, events, trailStart, minOdometer, maxOdometer);
  let obstacle: Course['obstacle'] = null;
  // A missing face distance cannot locate the contact.
  if (stamp !== null && wireMm(stamp.faceMm) > 0) {
    const faceIndex = sessionTrail.indexOf(stamp.sample);
    const at = points[faceIndex];
    const headingIndex = sessionTrail.indexOf(stamp.headingSample ?? stamp.sample);
    const yaw = stamp.bodyYaw;
    const contact = yaw === undefined ? null : hugReportPose(samples, points, headingIndex, yaw.timestamp);
    // Encoder yaw is positive left from contact, while the mapped bearing increases clockwise.
    const headingDeg = yaw === undefined
      ? (stamp.headingSample === undefined ? stamp.headingDeg : null) ?? samples[headingIndex]?.headingDeg ?? null
      : contact === null ? null : headingWrap(contact.headingDeg + yaw.deci / 10);
    const axle = stamp.faceTime === undefined
      ? samples[faceIndex]?.headingDeg != null ? at : null
      : hugReportPose(samples, points, faceIndex, stamp.faceTime);
    if (axle != null && headingDeg !== null) {
      obstacle = { ...placeContact({ ...axle, headingDeg }, stamp.faceMm),
        timestamp: stamp.faceTime ?? stamp.sample.timestamp };
    }
  }
  const forwardReturn = obstacle === null ? placeForwardReturn(sessionTrail, samples, points, forwardStamp) : null;
  return { points, samples, hasGap, positionKnown, headingReference, bootTimestamp: trailStart,
    markers, junctions, humps, obstacle, forwardReturn, surfaces, bounds: courseBounds(points, obstacle, surfaces, humps) };
}

/** Interpolate bridge-timed hug reports between adjacent 5 Hz poses, never across a missing pass. */
function hugReportPose(samples: TrailPoint[], points: CoursePoint[], index: number, time: number) {
  const a = samples[index];
  const b = samples[index + 1];
  if (!a) return null;
  // The status and obstacle packets in one bridge batch can be stamped a few milliseconds apart.
  const nearest = b && Math.abs(b.timestamp - time) < Math.abs(a.timestamp - time) ? index + 1 : index;
  const near = samples[nearest]!;
  if (Math.abs(near.timestamp - time) <= 5) {
    return near.headingDeg === null ? null : { ...points[nearest]!, headingDeg: near.headingDeg };
  }
  if (a.headingDeg === null || !b || b.headingDeg === null || time <= a.timestamp || time >= b.timestamp ||
      b.timestamp - a.timestamp > 250 || (a.seq !== undefined && b.seq !== undefined && ((b.seq - a.seq) & 0xffff) !== 1)) return null;
  const fraction = (time - a.timestamp) / (b.timestamp - a.timestamp);
  const p = points[index]!;
  const q = points[index + 1]!;
  return { x: p.x + fraction * (q.x - p.x), y: p.y + fraction * (q.y - p.y),
    headingDeg: headingWrap(a.headingDeg + fraction * (headingWrap(b.headingDeg - a.headingDeg + 180) - 180)) };
}

function placeForwardReturn(source: TrailPoint[], samples: TrailPoint[], points: CoursePoint[], stamp: ForwardReturnStamp | null): Course['forwardReturn'] {
  if (stamp === null) return null;
  const { pose, rangeMm } = stamp;
  const index = pose === null ? -1 : source.indexOf(pose);
  const point = points[index];
  const headingDeg = samples[index]?.headingDeg ?? null;
  if (point === undefined || headingDeg === null) return { rangeMm, beam: null };
  const rad = headingDeg * RAD_PER_DEG;
  const ahead = (mm: number) => ({ x: point.x + mm * Math.sin(rad), y: point.y + mm * Math.cos(rad) });
  return { rangeMm, beam: { from: ahead(SONAR_FORWARD_MM), to: ahead(SONAR_FORWARD_MM + rangeMm) } };
}
