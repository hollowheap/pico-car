import { closestPointOnSegment, reconstruct, type CoursePoint } from './course.js';
import { placeSurface, type Survey } from './scan.js';
import type { TelemetryStore, TrailPoint } from './telemetry-store.js';

/** Squared error if a point is removed from its neighbors' polyline. */
function shortcutError(a: CoursePoint, b: CoursePoint, c: CoursePoint): number {
  const closest = closestPointOnSegment(b, a, c);
  return (b.x - closest.x) ** 2 + (b.y - closest.y) ** 2;
}

/** Simplify an old prefix in one batch, leaving recent raw status and wheel pairing intact. */
export function compactTrail(store: TelemetryStore): TelemetryStore {
  const { trail, obstacle, forwardReturn } = store;
  const prefix = trail.slice(0, Math.floor(trail.length / 2));
  const { points, samples, headingReference, hasGap } = reconstruct(prefix);
  const pins = new Set([obstacle?.sample, obstacle?.headingSample, forwardReturn?.pose, store.scanStart]);
  // A 2 Hz hug report may fall between two 5 Hz poses, both of which locate its measurement.
  for (const base of [obstacle?.bodyYaw && obstacle.headingSample, obstacle?.faceTime !== undefined && obstacle.sample]) {
    if (base) {
      const i = prefix.indexOf(base);
      if (i >= 0) pins.add(prefix[i + 1]);
    }
  }
  // An exceptionally survey-dense trail can still compress. Rebase any removed
  // observations onto the first located checkpoint rather than dropping them.
  const surveyOrigin = samples.findIndex((point) => point.headingDeg !== null);
  if (surveyOrigin >= 0) pins.add(prefix[surveyOrigin]);
  // Wheel-only travel after a gap resumes on the last resolved bearing.
  const lastBearing = samples.findLastIndex((sample) => sample.headingDeg !== null);
  if (lastBearing >= 0) pins.add(prefix[lastBearing]);
  const events = store.events.map((event) => event.timestamp).sort((a, b) => a - b);
  let event = 0;
  for (let i = 0; i < prefix.length && event < events.length; i++) {
    while (event < events.length && events[event]! <= prefix[i]!.timestamp) {
      pins.add(prefix[i]);
      pins.add(prefix[i - 1]);
      event++;
    }
  }

  // The endpoints anchor the boot origin and raw tail. Skip neighboring
  // removals so each shortcut is judged against the points that survive it.
  const candidates: { index: number; error: number }[] = [];
  for (let i = 1; i < prefix.length - 1; i++) {
    if (pins.has(prefix[i])) continue;
    candidates.push({ index: i, error: prefix[i]?.surveys ? Infinity : shortcutError(points[i - 1]!, points[i]!, points[i + 1]!) });
  }
  candidates.sort((a, b) => a.error - b.error);
  const removed = new Set<number>();
  const target = Math.floor(prefix.length / 3);
  for (const { index } of candidates) {
    if (!removed.has(index - 1) && !removed.has(index + 1)) removed.add(index);
    if (removed.size >= target) break;
  }

  const saved: TrailPoint[] = [];
  const replacements = new Map<TrailPoint, TrailPoint>();
  const transferred: Survey[] = [];
  const origin = points[surveyOrigin];
  const originBearing = samples[surveyOrigin]?.headingDeg;
  if (origin && originBearing != null) {
    const rad = originBearing * Math.PI / 180;
    const shift = { x: -origin.x * Math.cos(rad) + origin.y * Math.sin(rad),
      y: -origin.x * Math.sin(rad) - origin.y * Math.cos(rad) };
    for (const index of removed) {
      const heading = samples[index]?.headingDeg;
      if (heading == null) continue;
      for (const survey of prefix[index]?.surveys ?? []) transferred.push({ ...survey,
        surfaces: survey.surfaces.map((face) => placeSurface(placeSurface(face, points[index]!, heading), shift, -originBearing)) });
    }
  }
  let gap = false;
  for (let i = 0; i < prefix.length; i++) {
    const source = prefix[i]!;
    const headingDeg = samples[i]!.headingDeg;
    gap ||= source.checkpoint?.hasGap === true || (i > 0 && headingDeg === null);
    if (removed.has(i)) continue;
    let point = source.checkpoint === undefined ? {
      ...source,
      checkpoint: { ...points[i]!, headingDeg, reference: headingReference,
        hasGap: i === prefix.length - 1 ? hasGap : gap },
    } : source;
    if (i === surveyOrigin && transferred.length) point = { ...point, surveys: [...point.surveys ?? [], ...transferred] };
    saved.push(point);
    if (point !== source && pins.has(source)) replacements.set(source, point);
  }
  const replace = (source: TrailPoint) => replacements.get(source) ?? source;
  return {
    ...store,
    trail: saved.concat(trail.slice(prefix.length)),
    scanStart: store.scanStart === null ? null : replace(store.scanStart),
    obstacle: obstacle === null ? null : { ...obstacle, sample: replace(obstacle.sample),
      headingSample: obstacle.headingSample === undefined ? undefined : replace(obstacle.headingSample) },
    forwardReturn: forwardReturn?.pose == null ? forwardReturn : { ...forwardReturn, pose: replace(forwardReturn.pose) },
  };
}
