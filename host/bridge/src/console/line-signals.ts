import type { LinkPacket } from './link.js';
import { BIT_FIELDS, ENUMS } from './schema.generated.js';

export const LINE_SIGNAL_WINDOW_MS = 10_000;
/** Ten seconds of 5 Hz status and room for closely spaced receipts. */
export const LINE_SIGNALS_MAX = 60;
const MAX_CONTIGUOUS_MS = 450;
const STATUS_SEQUENCE_MASK = 0xffff;
const BARCODE_ELEMENT_LABELS: Record<number, string> = {
  [ENUMS.barcodeElement.space]: 'Space',
  [ENUMS.barcodeElement.bar]: 'Bar',
  [ENUMS.barcodeElement.block]: 'Block',
};
const BARCODE_PHASE_LABELS: Partial<Record<number, string>> = {
  [ENUMS.barcodePhase.start]: 'Start',
  [ENUMS.barcodePhase.decoding]: 'Decoding',
  [ENUMS.barcodePhase.complete]: 'Complete',
  [ENUMS.barcodePhase.aborted]: 'Aborted',
};

export type LineSignalSample = { timestamp: number; seq?: number; bits: number | null };
export type LineSignalPoint = { at: number; on: boolean };
export type LineSignalSpan = { from: number; to: number; on: boolean };
export type LineSignalPlotLane = {
  measured: LineSignalSpan[];
  transitions: number[];
  estimated: LineSignalSpan[];
};

const MAX_INTERPOLATE_MS = 1000;
const MAX_INTERPOLATE_SEQ_GAP = 4;

export function barcodeElementLabel(bits: number | undefined): string {
  return BARCODE_ELEMENT_LABELS[((bits ?? 0) >> BIT_FIELDS.lineBits.barcodeElementLsb) & 3] ?? 'Unknown';
}

export function barcodePhaseLabel(code: number): string | undefined {
  return BARCODE_PHASE_LABELS[code];
}

export function barcodePhaseCode(bits: number | undefined): number {
  return ((bits ?? 0) >> BIT_FIELDS.lineBits.barcodePhaseLsb) & 7;
}

export function lineSignalRuns(samples: readonly LineSignalSample[], nowMs: number, bit: number): LineSignalPoint[][] {
  const runs: LineSignalPoint[][] = [];
  let previous: LineSignalSample | undefined;
  let run: LineSignalPoint[] = [];

  for (const sample of samples) {
    if (sample.timestamp < nowMs - LINE_SIGNAL_WINDOW_MS || sample.timestamp > nowMs) continue;
    if (sample.bits === null) {
      previous = undefined;
      continue;
    }
    const on = (sample.bits & (1 << bit)) !== 0;
    const elapsed = sample.timestamp - (previous?.timestamp ?? -Infinity);
    const nextSeq = previous?.seq === undefined || sample.seq === undefined ||
      ((sample.seq - previous.seq) & STATUS_SEQUENCE_MASK) === 1;
    if (previous === undefined || elapsed < 0 || elapsed > MAX_CONTIGUOUS_MS ||
        !nextSeq || run[0]?.on !== on) {
      run = [];
      runs.push(run);
    }
    run.push({ at: sample.timestamp, on });
    previous = sample;
  }
  return runs;
}


/** Use the capture clock for intervals, including when MQTT delivers several edges together. */
export function barcodeEdgeRuns(edges: readonly LinkPacket[], nowMs: number): LineSignalPoint[][] {
  const runs: LineSignalPoint[][] = [];
  let next: LinkPacket | undefined;
  let at = 0;
  for (let i = edges.length - 1; i >= 0; i--) {
    const edge = edges[i]!;
    const v = edge.values;
    const deltaUs = next?.values.edge_us !== undefined && v.edge_us !== undefined
      ? (next.values.edge_us - v.edge_us) >>> 0 : 0;
    const continuous = next?.values.seq !== undefined && v.seq !== undefined && ((next.values.seq - v.seq) & 255) === 1 &&
      next.values.capture_loss === 0 && next.values.level_high !== v.level_high &&
      deltaUs === next.values.interval_us && deltaUs > 0;
    at = continuous ? at - deltaUs / 1000 : edge.timestamp;
    next = edge;
    if (at < nowMs - LINE_SIGNAL_WINDOW_MS || at > nowMs || v.capture_loss !== 0 ||
        (v.interval_us === undefined || v.interval_us <= 0) || (v.level_high !== 0 && v.level_high !== 1)) continue;
    // The new level ends the interval spent at the opposite level.
    const on = v.level_high === 0;
    runs.push([{ at: Math.max(nowMs - LINE_SIGNAL_WINDOW_MS, at - v.interval_us / 1000), on }, { at, on }]);
  }
  return runs.reverse();
}

function clampSpan(from: number, to: number, on: boolean, start: number, end: number): LineSignalSpan | null {
  const a = Math.max(start, from);
  const b = Math.min(end, to);
  return b >= a ? { from: a, to: b, on } : null;
}

/**
 * Build smoothly clipped digital waveform spans for [nowMs - LINE_SIGNAL_WINDOW_MS, nowMs],
 * separating contiguous measured spans from dashed step-hold estimates across short packet losses.
 */
export function lineSignalPlotData(
  samples: readonly LineSignalSample[],
  edges: readonly LinkPacket[],
  nowMs: number,
  bit: number,
): LineSignalPlotLane {
  const start = nowMs - LINE_SIGNAL_WINDOW_MS;
  const measuredEdges = bit === BIT_FIELDS.lineBits.barcode ? barcodeEdgeRuns(edges, nowMs) : [];
  if (measuredEdges.length > 0) {
    const measured: LineSignalSpan[] = [];
    const transitions: number[] = [];
    let prev: LineSignalSpan | undefined;
    for (const run of measuredEdges) {
      const first = run[0]!;
      const last = run[run.length - 1]!;
      const span = clampSpan(first.at, last.at, first.on, start, nowMs);
      if (!span) continue;
      if (prev && prev.on !== span.on && Math.abs(span.from - prev.to) <= 5) {
        transitions.push(span.from);
      }
      measured.push(span);
      prev = span;
    }
    return { measured, transitions, estimated: [] };
  }

  const measured: LineSignalSpan[] = [];
  const transitions: number[] = [];
  const estimated: LineSignalSpan[] = [];
  let previous: { timestamp: number; seq?: number; on: boolean } | undefined;
  let hasFuture = false;

  for (const sample of samples) {
    if (sample.bits === null) {
      previous = undefined;
      continue;
    }
    const on = (sample.bits & (1 << bit)) !== 0;
    if (sample.timestamp > nowMs) {
      hasFuture = true;
    }

    if (previous !== undefined) {
      const elapsed = sample.timestamp - previous.timestamp;
      const seqDelta = previous.seq === undefined || sample.seq === undefined
        ? 1
        : (sample.seq - previous.seq) & STATUS_SEQUENCE_MASK;
      const contiguous = elapsed >= 0 && elapsed <= MAX_CONTIGUOUS_MS && seqDelta === 1;
      const bridgeable = !contiguous && elapsed > 0 && elapsed <= MAX_INTERPOLATE_MS &&
        seqDelta > 0 && seqDelta <= MAX_INTERPOLATE_SEQ_GAP;

      if (contiguous) {
        const span = clampSpan(previous.timestamp, sample.timestamp, previous.on, start, nowMs);
        if (span) measured.push(span);
        if (previous.on !== on && sample.timestamp >= start && sample.timestamp <= nowMs) {
          transitions.push(sample.timestamp);
        }
      } else if (bridgeable) {
        const span = clampSpan(previous.timestamp, sample.timestamp, previous.on, start, nowMs);
        if (span) estimated.push(span);
      }
    }

    previous = { timestamp: sample.timestamp, seq: sample.seq, on };
    if (sample.timestamp > nowMs) break;
  }

  if (!hasFuture && previous !== undefined && previous.timestamp <= nowMs && nowMs - previous.timestamp <= MAX_CONTIGUOUS_MS) {
    const span = clampSpan(previous.timestamp, nowMs, previous.on, start, nowMs);
    if (span) estimated.push(span);
  } else if (measured.length === 0 && estimated.length === 0 && previous !== undefined &&
      previous.timestamp >= start && previous.timestamp <= nowMs) {
    measured.push({ from: previous.timestamp, to: previous.timestamp, on: previous.on });
  }

  return { measured, transitions, estimated };
}
