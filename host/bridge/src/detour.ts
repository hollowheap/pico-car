/** All positions are millimetres in a surveyed frame with the start axle at
 * (0, 0), +x along the tape ahead and +y left. Positive yaw turns left. */
export type Bounds = { minX: number; maxX: number; minY: number; maxY: number };
export type Face = 'front' | 'back' | 'left' | 'right';

export type SonarObservation = {
  face: Face;
  capturedAtMs: number;
  pose: { xMm: number; yMm: number; yawDeg: number; capturedAtMs: number };
  bearingDeg: number;
  echo: 'valid' | 'open' | 'invalid';
  rangeMm: number;
};

export type DetourSurvey = {
  model: 'single-tape-aligned-rectangle' | 'unverified';
  /** Independently surveyed area containing just the one known rectangle.
   * A sonar OPEN result is insufficient to establish this clearance. */
  verifiedArea: Bounds;
  /** Nominal track limits in the survey frame; the bypass may leave the track. */
  strip: Pick<Bounds, 'minY' | 'maxY'>;
  /** Continuous supporting floor in the same frame, independently established
   * over the entire rectangle. Omit when ground support is unknown. */
  supportingFloor?: Bounds;
  footprint: { frontMm: number; rearMm: number; leftMm: number; rightMm: number };
  /** Maximum displacement of any chassis point from the route, including yaw error. */
  trackingErrorMm: number;
  /** Nominal endpoint alignment with one of the two surveyed tape sensors. */
  terminalTape?: { sensorOffsetMm: number; captureToleranceMm: number };
  clearanceMm: number;
  sonar: { forwardOffsetMm: number; beamHalfDeg: number; rangeErrorMm: number; poseErrorMm: number; headingErrorDeg: number };
  observations: readonly SonarObservation[];
};

export type DetourSegment = { lengthMm: number; turnDeci: number };
export type DetourPlan =
  | { kind: 'route'; box: Bounds; segments: DetourSegment[]; distanceMm: number; end: { xMm: number; yMm: number } }
  | { kind: 'hold'; reason: 'unverified survey' | 'incomplete contour' | 'no safe route' };

const DEG = Math.PI / 180;
const MAX_POSE_AGE_MS = 25;
const MAX_SCAN_HITS = 256;
const MIN_FACE_SPAN_MM = 20;
const MAX_INCIDENCE_DEG = 25;
const MAX_RANGE_MM = 4000;
// The pilot accepts at most seven primitives and runs them at 100 mm/s.
const MAX_STEPS = 7;
const MAX_LEG_MM = 1500;
const MAX_ROUTE_MM = 3500;
const MAX_STEP_TURN_DECI = 1800;
const MAX_TURN_DECI_PER_S = 500;
const FORWARD_MM_PER_S = 100;
const MIN_TURN_RADIUS_MM = 120;
const CHECK_STEP_MM = 10;

const FACES = {
  front: { axis: 'x', outward: -1, beam: 0 },
  back: { axis: 'x', outward: 1, beam: 180 },
  left: { axis: 'y', outward: 1, beam: -90 },
  right: { axis: 'y', outward: -1, beam: 90 },
} as const;

type Support = { low: number; high: number; tangent: number; tangentError: number };
type Pose = { x: number; y: number; yaw: number };

const positive = (n: number) => Number.isFinite(n) && n > 0;
const nonnegative = (n: number) => Number.isFinite(n) && n >= 0;
const angleDistance = (a: number, b: number) => Math.abs((((a - b + 180) % 360) + 360) % 360 - 180);

function validBounds(area: Bounds | undefined): area is Bounds {
  return area != null && [area.minX, area.maxX, area.minY, area.maxY].every(Number.isFinite) &&
    area.minX < area.maxX && area.minY < area.maxY;
}

function surveyed(survey: DetourSurvey): boolean {
  const { verifiedArea: area, strip, sonar, terminalTape } = survey;
  return survey.model === 'single-tape-aligned-rectangle' &&
    validBounds(area) && Object.values(strip).every(Number.isFinite) && strip.minY < strip.maxY &&
    Object.values(survey.footprint).every(positive) && positive(survey.clearanceMm) &&
    (terminalTape === undefined || (positive(terminalTape.sensorOffsetMm) &&
      positive(terminalTape.captureToleranceMm) && terminalTape.captureToleranceMm < terminalTape.sensorOffsetMm)) &&
    nonnegative(survey.trackingErrorMm) && Object.values(sonar).every(nonnegative) &&
    sonar.beamHalfDeg + sonar.headingErrorDeg < MAX_INCIDENCE_DEG &&
    survey.observations.length <= MAX_SCAN_HITS;
}

function supportOf(observation: SonarObservation, sonar: DetourSurvey['sonar']): Support | null {
  const { pose, capturedAtMs, bearingDeg, rangeMm } = observation;
  if (observation.echo !== 'valid' || !positive(rangeMm) || rangeMm > MAX_RANGE_MM || rangeMm <= sonar.rangeErrorMm ||
      ![capturedAtMs, pose.capturedAtMs, pose.xMm, pose.yMm, pose.yawDeg, bearingDeg].every(Number.isFinite) ||
      Math.abs(capturedAtMs - pose.capturedAtMs) > MAX_POSE_AGE_MS) return null;

  const { axis, outward, beam } = FACES[observation.face];
  const heading = pose.yawDeg * DEG;
  const ray = pose.yawDeg + bearingDeg;
  const incidence = angleDistance(ray, beam);
  const angularError = sonar.beamHalfDeg + sonar.headingErrorDeg;
  if (incidence + angularError > MAX_INCIDENCE_DEG) return null;

  const originX = pose.xMm + sonar.forwardOffsetMm * Math.cos(heading);
  const originY = pose.yMm + sonar.forwardOffsetMm * Math.sin(heading);
  const originError = sonar.poseErrorMm + 2 * sonar.forwardOffsetMm * Math.sin(sonar.headingErrorDeg * DEG / 2);
  const normal = outward * (axis === 'x' ? originX : originY);
  const near = (rangeMm - sonar.rangeErrorMm) * Math.cos((incidence + angularError) * DEG);
  const far = (rangeMm + sonar.rangeErrorMm) * Math.cos(Math.max(0, incidence - angularError) * DEG);
  return {
    low: normal - far - originError,
    high: normal - near + originError,
    tangent: axis === 'x' ? originY + rangeMm * Math.sin(ray * DEG) : originX + rangeMm * Math.cos(ray * DEG),
    tangentError: originError + sonar.rangeErrorMm + 2 * rangeMm * Math.sin(angularError * DEG / 2),
  };
}

function fitBox({ observations, sonar, verifiedArea }: DetourSurvey): Bounds | null {
  const faces: Face[] = ['front', 'back', 'left', 'right'];
  const samples: Record<Face, Support[]> = { front: [], back: [], left: [], right: [] };
  const captures = new Set<number>();
  for (const observation of observations) {
    if (!Object.hasOwn(FACES, observation.face) || captures.has(observation.capturedAtMs)) return null;
    captures.add(observation.capturedAtMs);
    const support = supportOf(observation, sonar);
    if (support !== null) samples[observation.face].push(support);
  }

  const outer: Record<Face, number> = { front: 0, back: 0, left: 0, right: 0 };
  for (const face of faces) {
    const supports = samples[face];
    if (supports.length < 3) return null;
    const tangents = supports.map(({ tangent }) => tangent);
    if (Math.max(...tangents) - Math.min(...tangents) < MIN_FACE_SPAN_MM) return null;

    // Intersect calibrated normal intervals. The outward end of the common
    // interval extends beyond any face position still consistent with all hits.
    const low = Math.max(...supports.map((sample) => sample.low));
    const high = Math.min(...supports.map((sample) => sample.high));
    if (low > high) return null;
    outer[face] = high;
  }

  const box = { minX: -outer.front, maxX: outer.back, minY: -outer.right, maxY: outer.left };
  if (!(box.minX < box.maxX && box.minY < box.maxY) || box.minX < verifiedArea.minX || box.maxX > verifiedArea.maxX ||
      box.minY < verifiedArea.minY || box.maxY > verifiedArea.maxY) return null;
  for (const face of faces) {
    const [min, max] = FACES[face].axis === 'x' ? [box.minY, box.maxY] : [box.minX, box.maxX];
    if (samples[face].some(({ tangent, tangentError }) => tangent + tangentError < min || tangent - tangentError > max)) return null;
  }
  return box;
}

function pointAlong(start: Pose, { lengthMm, turnDeci }: DetourSegment, fraction: number): Pose {
  const turn = turnDeci * DEG / 10;
  if (turn === 0) {
    const distance = fraction * lengthMm;
    return { x: start.x + distance * Math.cos(start.yaw), y: start.y + distance * Math.sin(start.yaw), yaw: start.yaw };
  }
  const radius = lengthMm / turn;
  const yaw = start.yaw + fraction * turn;
  return { x: start.x + radius * (Math.sin(yaw) - Math.sin(start.yaw)),
    y: start.y + radius * (Math.cos(start.yaw) - Math.cos(yaw)), yaw };
}

function bodyFits(pose: Pose, box: Bounds, survey: DetourSurvey, area: Bounds, samplePad: number): boolean {
  const { frontMm, rearMm, leftMm, rightMm } = survey.footprint;
  const u = { x: Math.cos(pose.yaw), y: Math.sin(pose.yaw) };
  const v = { x: -u.y, y: u.x };
  const bodyU = (frontMm + rearMm) / 2;
  const bodyV = (leftMm + rightMm) / 2;
  const x = pose.x + u.x * (frontMm - rearMm) / 2 + v.x * (leftMm - rightMm) / 2;
  const y = pose.y + u.y * (frontMm - rearMm) / 2 + v.y * (leftMm - rightMm) / 2;
  const bodyX = Math.abs(u.x) * bodyU + Math.abs(v.x) * bodyV;
  const bodyY = Math.abs(u.y) * bodyU + Math.abs(v.y) * bodyV;
  const fitPad = survey.trackingErrorMm + samplePad;
  if (x - bodyX - fitPad < area.minX || x + bodyX + fitPad > area.maxX ||
      y - bodyY - fitPad < area.minY || y + bodyY + fitPad > area.maxY) return false;

  // Separating axes of the oriented body and the obstacle, expanded for
  // stand-off and movement between footprint samples.
  const padding = fitPad + survey.clearanceMm;
  const boxX = (box.maxX - box.minX) / 2 + padding;
  const boxY = (box.maxY - box.minY) / 2 + padding;
  const dx = (box.maxX + box.minX) / 2 - x;
  const dy = (box.maxY + box.minY) / 2 - y;
  return Math.abs(dx) > bodyX + boxX || Math.abs(dy) > bodyY + boxY ||
    Math.abs(dx * u.x + dy * u.y) > bodyU + Math.abs(u.x) * boxX + Math.abs(u.y) * boxY ||
    Math.abs(dx * v.x + dy * v.y) > bodyV + Math.abs(v.x) * boxX + Math.abs(v.y) * boxY;
}

function safeEnd(segments: DetourSegment[], box: Bounds, survey: DetourSurvey, area: Bounds, bodyRadius: number): Pose | null {
  let pose = { x: 0, y: 0, yaw: 0 };
  for (const segment of segments) {
    const steps = Math.ceil(segment.lengthMm / CHECK_STEP_MM);
    const turn = Math.abs(segment.turnDeci * DEG / 10);
    // A chassis point moves at most axle travel plus its radius times yaw.
    // Every intervening pose is within this pad of the nearest checked pose.
    const samplePad = (segment.lengthMm + bodyRadius * turn) / (2 * steps);
    for (let i = 0; i <= steps; i++) {
      if (!bodyFits(pointAlong(pose, segment, i / steps), box, survey, area, samplePad)) return null;
    }
    pose = pointAlong(pose, segment, 1);
  }
  return pose;
}

function laneChange(box: Bounds, survey: DetourSurvey, bodyRadius: number, side: number,
  turn: number, earlierReturnMm: number): DetourSegment[] {
  const clearance = survey.clearanceMm + survey.trackingErrorMm + CHECK_STEP_MM;
  const offset = side > 0 ? box.maxY + survey.footprint.rightMm + clearance
    : -(box.minY - survey.footprint.leftMm - clearance);
  if (offset <= 0) return [{ lengthMm: Math.ceil(box.maxX + bodyRadius + clearance), turnDeci: 0 }];

  const angle = turn * DEG / 10;
  const terminalInset = survey.terminalTape?.sensorOffsetMm ?? 0;
  const shiftPerMm = 2 * (1 - Math.cos(angle)) / angle;
  const returnReduction = Math.round(terminalInset / shiftPerMm);
  const lengthMm = Math.max(Math.ceil(offset / shiftPerMm), Math.ceil(MIN_TURN_RADIUS_MM * angle) + returnReduction);
  const advance = 2 * lengthMm * Math.sin(angle) / angle;
  const approach = Math.floor(box.minX - bodyRadius - clearance - advance);
  if (approach < 0) return [];
  const pass = Math.ceil(box.maxX + bodyRadius + clearance - (approach + advance)) - earlierReturnMm;
  const bend = { lengthMm, turnDeci: side * turn };
  const counter = { lengthMm, turnDeci: -bend.turnDeci };
  const returnLength = lengthMm - returnReduction;
  const segments = [bend, counter, { lengthMm: pass, turnDeci: 0 },
    { lengthMm: returnLength, turnDeci: counter.turnDeci }, { lengthMm: returnLength, turnDeci: bend.turnDeci }];
  return approach === 0 ? segments : [{ lengthMm: approach, turnDeci: 0 }, ...segments];
}

function executable(segments: DetourSegment[], distanceMm: number): boolean {
  return segments.length > 0 && segments.length <= MAX_STEPS && distanceMm <= MAX_ROUTE_MM &&
    segments.reduce((sum, { turnDeci }) => sum + turnDeci, 0) % 3600 === 0 &&
    segments.every(({ lengthMm, turnDeci }) => {
      const turn = Math.abs(turnDeci);
      return Number.isInteger(lengthMm) && Number.isInteger(turnDeci) && lengthMm > 0 && lengthMm <= MAX_LEG_MM &&
        turn <= MAX_STEP_TURN_DECI && turn * FORWARD_MM_PER_S <= lengthMm * MAX_TURN_DECI_PER_S;
    });
}

/** Choose the shortest verified rounded route in the surveyed start frame.
 * Route admission, evidence age and motion safety remain the caller's responsibility. */
export function planDetour(survey: DetourSurvey): DetourPlan {
  const floor = survey.supportingFloor;
  if (!validBounds(floor) || !surveyed(survey)) return { kind: 'hold', reason: 'unverified survey' };
  const area = {
    minX: Math.max(floor.minX, survey.verifiedArea.minX), maxX: Math.min(floor.maxX, survey.verifiedArea.maxX),
    minY: Math.max(floor.minY, survey.verifiedArea.minY), maxY: Math.min(floor.maxY, survey.verifiedArea.maxY),
  };
  const box = fitBox(survey);
  if (box === null) return { kind: 'hold', reason: 'incomplete contour' };

  const { frontMm, rearMm, leftMm, rightMm } = survey.footprint;
  const terminalTape = survey.terminalTape;
  const bodyRadius = Math.hypot(Math.max(frontMm, rearMm), Math.max(leftMm, rightMm));
  let best: Extract<DetourPlan, { kind: 'route' }> | undefined;
  for (const side of [1, -1]) {
    for (const turn of [300, 450, 600, 900]) {
      // An earlier merge may cut the nominal stand-off corner. The whole
      // rounded sweep must still clear the inflated box and floor bounds.
      for (const earlierReturnMm of [150, 100, 50, 0]) {
        const segments = laneChange(box, survey, bodyRadius, side, turn, earlierReturnMm);
        const distanceMm = segments.reduce((sum, segment) => sum + segment.lengthMm, 0);
        if (!executable(segments, distanceMm)) continue;
        if (best !== undefined && distanceMm >= best.distanceMm) continue;
        const end = safeEnd(segments, box, survey, area, bodyRadius);
        if (end === null || (terminalTape !== undefined &&
          Math.abs(end.y - side * terminalTape.sensorOffsetMm) > terminalTape.captureToleranceMm)) continue;
        best = { kind: 'route', box, distanceMm, segments, end: { xMm: end.x, yMm: end.y } };
      }
    }
  }
  return best ?? { kind: 'hold', reason: 'no safe route' };
}
