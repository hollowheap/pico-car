import type { EventRow, TrailPoint } from './telemetry-store';

export type RunSummary = {
  distanceMm: number;
  durationS: number;
  averageMms: number;
  turns: number;
  barcodesOk: number;
  barcodesRefused: number;
  obstacles: number;
  bypasses: number;
  humps: number;
  tallestHumpMm: number;
  lifts: number;
  lineLosses: number;
  faults: number;
  faultBits: number[];
  stops: number;
};

const REJECTED_ACTION = 5;
const MS_PER_S = 1000;

/**
 * Summarise the most recent run: from the last `runStart` event, or from the
 * first sample when there is none. There is no run id on the wire, so this is
 * one summary and never a list of runs.
 */
export function summarizeRun(trail: TrailPoint[], events: EventRow[]): RunSummary {
  let startIndex = -1;
  events.forEach((event, index) => {
    if (event.name === 'runStart') startIndex = index;
  });
  const startEvent = startIndex >= 0 ? events[startIndex] : undefined;
  const inRun = startIndex >= 0 ? events.slice(startIndex) : events;
  const samples = startEvent === undefined
    ? trail
    : trail.filter((sample) => sample.timestamp >= startEvent.timestamp);
  const first = samples[0];
  const last = samples[samples.length - 1];
  const distanceMm = first !== undefined && last !== undefined ? Math.max(0, last.odometerMm - first.odometerMm) : 0;
  const durationS = first !== undefined && last !== undefined ? (last.timestamp - first.timestamp) / MS_PER_S : 0;
  const count = (name: string) => inRun.filter((event) => event.name === name).length;
  const bits = new Set(inRun.filter((event) => event.name === 'faultSet').map((event) => event.p1));
  const humps = inRun.filter((event) => event.name === 'humpEnd' && event.p2 === 1);
  return {
    distanceMm,
    durationS,
    averageMms: durationS > 0 ? distanceMm / durationS : 0,
    turns: count('turnDone'),
    barcodesOk: inRun.filter((event) => event.name === 'barcode' && (event.p2 & 0x7) !== REJECTED_ACTION).length,
    barcodesRefused:
      count('barcodeBad') + inRun.filter((event) => event.name === 'barcode' && (event.p2 & 0x7) === REJECTED_ACTION).length,
    obstacles: count('obstacleSeen'),
    bypasses: count('bypassEnd'),
    humps: humps.length,
    tallestHumpMm: humps.reduce((tallest, event) => Math.max(tallest, event.p1), 0),
    lifts: count('liftStart'),
    lineLosses: count('lineLost'),
    faults: count('faultSet'),
    faultBits: [...bits].sort((a, b) => a - b),
    stops: count('runStop'),
  };
}
