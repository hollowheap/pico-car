import { buildCourse } from './course.js';
import { packetOnTimeline, type LinkPacket } from './link.js';
import { emptyStore, reduceStore } from './telemetry-store.js';

export type CalibrationCheckpoint = {
  label: string;
  seq: number;
  xMm: number;
  yMm: number;
  headingDeg?: number;
};

type CheckpointScore = CalibrationCheckpoint & ({
  unavailable: string;
} | {
  estimatedXMm: number;
  estimatedYMm: number;
  estimatedHeadingDeg: number;
  positionErrorMm: number;
  headingErrorDeg?: number;
});

function wrapHeading(degrees: number): number {
  return ((degrees % 360) + 360) % 360;
}

export function scoreCourse(frames: readonly LinkPacket[], measured: readonly CalibrationCheckpoint[]) {
  const [origin, ...checkpoints] = measured;
  if (origin?.headingDeg === undefined || checkpoints.length === 0) {
    throw new Error('Provide a measured start pose and at least one later checkpoint.');
  }

  const store = frames.reduce((state, frame) => reduceStore(state, packetOnTimeline(frame)), emptyStore());
  const { points, samples } = buildCourse(store.trail, [], null, store.trailStart);
  const indices = new Map<number, number>();
  for (const [index, { seq }] of samples.entries()) {
    if (seq !== undefined) indices.set(seq, indices.has(seq) ? -1 : index);
  }

  const startIndex = indices.get(origin.seq);
  const startPoint = startIndex === undefined ? undefined : points[startIndex];
  const startHeading = startIndex === undefined ? undefined : samples[startIndex]?.headingDeg;
  if (startIndex === undefined || startPoint === undefined || startHeading == null) {
    throw new Error('The start sequence must identify one reconstructed pose with a heading.');
  }
  const firstGap = samples.findIndex((sample, index) => index > startIndex &&
    (sample.headingDeg === null || sample.checkpoint?.hasGap === true));

  // Apply only the transform measured at the start so later drift survives.
  const headingOffset = origin.headingDeg - startHeading;
  const radians = headingOffset * Math.PI / 180;
  const cos = Math.cos(radians);
  const sin = Math.sin(radians);

  const scores = checkpoints.map((checkpoint): CheckpointScore => {
    const index = indices.get(checkpoint.seq);
    if (index === undefined || index < 0) {
      return { ...checkpoint, unavailable: 'status sequence missing or ambiguous' };
    }
    if (index <= startIndex) return { ...checkpoint, unavailable: 'checkpoint is at or before the origin' };
    const point = points[index];
    const headingDeg = samples[index]?.headingDeg;
    if (point === undefined || headingDeg == null || (firstGap >= 0 && index >= firstGap)) {
      return { ...checkpoint, unavailable: 'trace gap after origin' };
    }

    const dx = point.x - startPoint.x;
    const dy = point.y - startPoint.y;
    const estimatedXMm = origin.xMm + cos * dx + sin * dy;
    const estimatedYMm = origin.yMm - sin * dx + cos * dy;
    const estimatedHeadingDeg = wrapHeading(headingDeg + headingOffset);
    return {
      ...checkpoint,
      estimatedXMm,
      estimatedYMm,
      estimatedHeadingDeg,
      positionErrorMm: Math.hypot(estimatedXMm - checkpoint.xMm, estimatedYMm - checkpoint.yMm),
      ...(checkpoint.headingDeg !== undefined && {
        headingErrorDeg: wrapHeading(estimatedHeadingDeg - checkpoint.headingDeg + 180) - 180,
      }),
    };
  });

  const errors = scores.flatMap(score => 'positionErrorMm' in score ? [score.positionErrorMm] : []);
  return {
    origin,
    checkpoints: scores,
    scored: errors.length,
    missing: scores.length - errors.length,
    positionRmseMm: errors.length ? Math.sqrt(errors.reduce((sum, error) => sum + error * error, 0) / errors.length) : null,
    maxPositionErrorMm: errors.length ? Math.max(...errors) : null,
  };
}
