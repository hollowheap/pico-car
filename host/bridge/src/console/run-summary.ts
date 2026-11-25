import { barcodeHasNoAction } from './events.js';
import { odometerDiscontinuous } from './odometer.js';
import type { EventRow, TrailPoint } from './telemetry-store.js';

export type RunSummary = {
  scope: 'This run' | 'Recorded';
  hasTravel: boolean;
  distanceMm: number;
  durationS: number;
  averageMms: number;
  turns: number;
  barcodesOk: number;
  barcodesNoAction: number;
  barcodesBad: number;
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

const MS_PER_S = 1000;

/** Longest completed recovery present in the retained event history. */
export function longestLineRecovery(events: EventRow[], since: number | null = null): number | null {
  let longest: number | null = null;
  for (const event of events) {
    if (event.name === 'lineFound' && event.timestamp >= (since ?? -Infinity)) longest = Math.max(longest ?? 0, event.p2);
  }
  return longest;
}

/**
 * Summarise the most recent run: from the last `runStart` event, or from the
 * first sample when there is none. There is no run id on the wire, so this is
 * one summary and never a list of runs.
 */
export function summarizeRun(trail: TrailPoint[], events: EventRow[], bootStart: number | null = null): RunSummary {
  let since = bootStart ?? -Infinity;
  let startIndex = -1;
  events.forEach((event, index) => {
    if (event.name === 'runStart' && event.timestamp >= since) {
      since = event.timestamp;
      startIndex = index;
    }
  });
  // Several events in a bridge drain may share a receipt timestamp.
  const inRun = events.filter((event, index) => event.timestamp > since || (event.timestamp === since && index >= startIndex));
  const samples = trail.filter((sample) => sample.timestamp >= since);
  const first = samples[0];
  const last = samples[samples.length - 1];
  let distanceMm = 0;
  for (let i = 1; i < samples.length; ++i) {
    const previous = samples[i - 1]!.odometerMm;
    const next = samples[i]!.odometerMm;
    if (!odometerDiscontinuous(previous, next)) distanceMm += Math.abs(next - previous);
  }
  const durationS = first !== undefined && last !== undefined ? (last.timestamp - first.timestamp) / MS_PER_S : 0;
  const count = (name: string) => inRun.filter((event) => event.name === name).length;
  const bits = new Set(inRun.filter((event) => event.name === 'faultSet').map((event) => event.p1));
  const humps = inRun.filter((event) => event.name === 'humpEnd' && event.p2 === 1);
  return {
    scope: startIndex < 0 ? 'Recorded' : 'This run',
    hasTravel: samples.length >= 2 && durationS > 0,
    distanceMm,
    durationS,
    averageMms: durationS > 0 ? distanceMm / durationS : 0,
    turns: count('turnDone'),
    barcodesOk: count('barcode'),
    barcodesNoAction: inRun.filter((event) => event.name === 'barcode' && barcodeHasNoAction(event.p2 & 0x7)).length,
    barcodesBad: count('barcodeBad'),
    obstacles: count('obstacleSeen'),
    bypasses: count('bypassEnd'),
    humps: humps.length,
    tallestHumpMm: humps.reduce((tallest, event) => Math.max(tallest, event.p1), 0),
    lifts: count('liftStart'),
    // Only p2 = 0 starts a search; a retry (0x8000 | n) and the stop (turn in degrees) also have p2 > 0.
    lineLosses: inRun.filter((event) => event.name === 'lineLost' && event.p2 === 0).length,
    faults: count('faultSet'),
    faultBits: [...bits].sort((a, b) => a - b),
    stops: count('runStop'),
  };
}
