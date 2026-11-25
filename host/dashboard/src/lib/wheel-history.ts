import type { SpeedPoint, WheelPlotSeries, WheelSample } from '../../../bridge/src/console/wheel-history.js';
export type { SpeedPoint, WheelPlotSeries, WheelSample } from '../../../bridge/src/console/wheel-history.js';

export const WHEEL_WINDOW_MS = 10000;

const MAX_GAP_MS = 600;

const MAX_INTERPOLATE_GAP_MS = 1000;

const MAX_INTERPOLATE_SEQ_GAP = 4;

const STEERING_MIN_WHEEL_MMS = 5;

/** Signed steering balance, with full deflection once the inner wheel stops or reverses. */
export function steeringFraction(left?: number, right?: number): number {
  if (left === undefined || right === undefined || !Number.isFinite(left) || !Number.isFinite(right)) return 0;
  if (Math.max(Math.abs(left), Math.abs(right)) < STEERING_MIN_WHEEL_MMS) return 0;
  const travel = Math.abs(left) + Math.abs(right);
  return (left - right) / travel;
}

export function wheelSegments(history: WheelSample[], now: number, side: 'left' | 'right') {
  const runs: SpeedPoint[][] = [];
  let previous: WheelSample | undefined;
  for (const p of history) {
    const value = p[side];
    if (p.at < now - WHEEL_WINDOW_MS || p.at > now || value === undefined || !Number.isFinite(value)) {
      previous = undefined;
      continue;
    }
    if (!previous || p.at - previous.at > MAX_GAP_MS ||
        (p.seq !== undefined && previous.seq !== undefined && ((previous.seq + 1) & 255) !== p.seq)) runs.push([]);
    runs[runs.length - 1]!.push({ at: p.at, value });
    previous = p;
  }
  return runs;
}

function clipSegment(a: SpeedPoint, b: SpeedPoint, start: number, end: number): [SpeedPoint, SpeedPoint] | null {
  if (b.at < start || a.at > end) return null;
  const dt = b.at - a.at;
  const lerp = (t: number): SpeedPoint => ({
    at: t,
    value: dt === 0 ? a.value : a.value + (b.value - a.value) * ((t - a.at) / dt),
  });
  return [a.at < start ? lerp(start) : a, b.at > end ? lerp(end) : b];
}

function clipRun(run: SpeedPoint[], start: number, end: number): SpeedPoint[] {
  if (run.length === 0) return [];
  if (run.length === 1) {
    const p = run[0]!;
    return p.at >= start && p.at <= end ? [p] : [];
  }
  const out: SpeedPoint[] = [];
  for (let i = 1; i < run.length; i += 1) {
    const clipped = clipSegment(run[i - 1]!, run[i]!, start, end);
    if (!clipped) continue;
    if (out.length === 0 || out.at(-1)!.at !== clipped[0].at) out.push(clipped[0]);
    out.push(clipped[1]);
  }
  return out;
}

/**
 * Build smoothly clipped wheel traces for [now - WHEEL_WINDOW_MS, now],
 * separating contiguous measured runs from dashed bridges across short packet losses.
 */
export function wheelPlotSeries(history: WheelSample[], now: number, side: 'left' | 'right'): WheelPlotSeries {
  const start = now - WHEEL_WINDOW_MS;
  const targetKey = side === 'left' ? 'leftTarget' : 'rightTarget';
  const rawMeasured: SpeedPoint[][] = [];
  const rawTargets: SpeedPoint[][] = [];
  const estimated: [SpeedPoint, SpeedPoint][] = [];
  let previous: WheelSample | undefined;
  let lastValid: SpeedPoint | undefined;
  let hasFuture = false;

  for (const p of history) {
    const value = p[side];
    if (value === undefined || !Number.isFinite(value)) {
      previous = undefined;
      continue;
    }
    if (p.at > now) {
      hasFuture = true;
    }
    const point: SpeedPoint = { at: p.at, value };
    const target = p[targetKey];
    const hasTarget = target !== undefined && Number.isFinite(target);

    if (!previous) {
      rawMeasured.push([point]);
      if (hasTarget) rawTargets.push([{ at: p.at, value: target }]);
    } else {
      const dt = p.at - previous.at;
      const seqDelta = p.seq !== undefined && previous.seq !== undefined ? (p.seq - previous.seq) & 255 : 1;
      const contiguous = dt >= 0 && dt <= MAX_GAP_MS && seqDelta === 1;
      const bridgeable = !contiguous && dt > 0 && dt <= MAX_INTERPOLATE_GAP_MS &&
        seqDelta > 0 && seqDelta <= MAX_INTERPOLATE_SEQ_GAP;

      if (contiguous) {
        rawMeasured[rawMeasured.length - 1]!.push(point);
        if (hasTarget) {
          if (rawTargets.length === 0) rawTargets.push([]);
          rawTargets[rawTargets.length - 1]!.push({ at: p.at, value: target });
        }
      } else {
        if (bridgeable && lastValid) {
          const clipped = clipSegment(lastValid, point, start, now);
          if (clipped) estimated.push(clipped);
        }
        rawMeasured.push([point]);
        if (hasTarget) rawTargets.push([{ at: p.at, value: target }]);
      }
    }
    previous = p;
    lastValid = point;
    if (p.at > now) break;
  }

  // Hold the last measured value up to `now` across normal inter-packet arrival jitter.
  if (!hasFuture && lastValid && lastValid.at < now && now - lastValid.at <= MAX_GAP_MS) {
    const clipped = clipSegment(lastValid, { at: now, value: lastValid.value }, start, now);
    if (clipped) estimated.push(clipped);
  }

  return {
    measured: rawMeasured.map((run) => clipRun(run, start, now)).filter((run) => run.length > 0),
    estimated,
    targets: rawTargets.map((run) => clipRun(run, start, now)).filter((run) => run.length > 0),
  };
}
