import { readFileSync } from 'node:fs';

import { planDetour, type Bounds, type DetourSurvey } from '../host/bridge/src/detour.js';

type SimSurvey = {
  widthMm: number;
  offMm: number;
  seed: number;
  startXMm: number;
  observations: DetourSurvey['observations'];
  verifiedArea: Bounds;
  supportingFloor?: Bounds;
  strip: DetourSurvey['strip'];
};

const lines = readFileSync(0, 'utf8').trim().split('\n').filter(Boolean);
const records = lines.map((line) => JSON.parse(line) as SimSurvey);
if (records.length === 0) throw new Error('Expected surveyed simulated sonar observations on stdin');

const out = [`CONTOUR1 ${records.length}`];
for (const record of records) {
  const { widthMm, offMm, seed, startXMm, observations, verifiedArea, strip } = record;
  if (![widthMm, offMm, seed, startXMm].every(Number.isSafeInteger)) throw new Error('Invalid simulator case ID');
  const route = planDetour({
    model: 'single-tape-aligned-rectangle',
    verifiedArea, supportingFloor: record.supportingFloor, strip, observations,
    // Enclose the simulated 119 mm body disc as well as the real outline.
    footprint: { frontMm: 130, rearMm: 120, leftMm: 119, rightMm: 119 },
    // Allow 25 mm translation and corner displacement at up to 8° yaw error.
    trackingErrorMm: 25 + Math.ceil(Math.hypot(130, 119) * Math.PI * 8 / 180),
    // Offset the endpoint for the 25 mm sensor spacing and 15 mm simulated tape.
    terminalTape: { sensorOffsetMm: 12.5, captureToleranceMm: 2 },
    clearanceMm: 120,
    sonar: { forwardOffsetMm: 70, beamHalfDeg: 9.5, rangeErrorMm: 3, poseErrorMm: 0, headingErrorDeg: 0 },
  });
  if (route.kind === 'hold') throw new Error(`Case ${widthMm}/${offMm}/${seed} requires a hold: ${route.reason}`);
  out.push(`${widthMm} ${offMm} ${seed} ${startXMm} ${route.segments.length}`);
  for (const segment of route.segments) out.push(`${segment.lengthMm} ${segment.turnDeci}`);
}

process.stdout.write(`${out.join('\n')}\n`);
