const UINT32_MAX = 0xffffffff;
const UINT16_MAX = 0xffff;
const INT16_MIN = -0x8000;
const INT16_MAX = 0x7fff;
// A full 192-row v1 capture uses under 37 KiB before external references.
export const BENCH_TRACE_MAX_BYTES = 128 * 1024;

const markers = ['release', 'capture', 'complete', 'contention'] as const;
const echoClasses = ['valid', 'open', 'too_close', 'no_echo', 'glitch', 'stuck'] as const;

type Row = Record<string, unknown>;
type DeviceSample = {
  seq: number;
  timeUs: number;
  cue?: string;
} & (
  | { kind: 'servo'; requestedDeci: number; effectiveDeci: number; pwmUs: number }
  | { kind: 'trigger'; requestedDeci: number; effectiveDeci: number }
  | { kind: 'reply'; requestedDeci: number; effectiveDeci: number; echo: typeof echoClasses[number]; rangeMm: number }
  | { kind: 'scheduler'; marker: typeof markers[number] }
);

export type BenchTraceReport = {
  header: { version: 1; build: string; fixture: string; trial: string };
  samples: (DeviceSample & { measurement: { source: string; angleDeci: number } | null })[];
  dropped: number;
  missing: number;
  incomplete: boolean;
};

function readRow(line: string): Row {
  let value: unknown;
  try {
    value = JSON.parse(line);
  } catch {
    throw new Error('Invalid JSON in bench trace.');
  }
  if (value === null || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error('Expected a bench trace record.');
  }
  return value as Row;
}

function identifier(row: Row, field: string): string {
  const value = row[field];
  if (typeof value !== 'string' || value.length === 0 || value.length > 32 || /[^a-zA-Z0-9._+-]/.test(value)) {
    throw new Error(`Invalid bench trace ${field}.`);
  }
  return value;
}

function integer(row: Row, field: string, min: number, max: number): number {
  const value = row[field];
  if (typeof value !== 'number' || !Number.isInteger(value) || value < min || value > max) {
    throw new Error(`Invalid bench trace ${field}.`);
  }
  return value;
}

function oneOf<T extends string>(row: Row, field: string, options: readonly T[]): T {
  const value = options.find(option => option === row[field]);
  if (value === undefined) throw new Error(`Invalid bench trace ${field}.`);
  return value;
}

function readSample(row: Row): DeviceSample {
  const common = {
    seq: integer(row, 'seq', 0, UINT32_MAX),
    timeUs: integer(row, 'timeUs', 0, UINT32_MAX),
    ...(row.cue !== undefined && { cue: identifier(row, 'cue') }),
  };
  const kind = row.kind;
  if (kind === 'scheduler') {
    return { ...common, kind, marker: oneOf(row, 'marker', markers) };
  }
  if (kind !== 'servo' && kind !== 'trigger' && kind !== 'reply') {
    throw new Error('Unknown bench trace record kind.');
  }

  const command = {
    ...common,
    requestedDeci: integer(row, 'requestedDeci', INT16_MIN, INT16_MAX),
    effectiveDeci: integer(row, 'effectiveDeci', INT16_MIN, INT16_MAX),
  };
  if (kind === 'servo') {
    return { ...command, kind, pwmUs: integer(row, 'pwmUs', 0, UINT16_MAX) };
  }
  if (kind === 'reply') {
    return { ...command, kind, echo: oneOf(row, 'echo', echoClasses), rangeMm: integer(row, 'rangeMm', 0, UINT16_MAX) };
  }
  return { ...command, kind };
}

export function parseBenchTrace(text: string): BenchTraceReport {
  if (text.length > BENCH_TRACE_MAX_BYTES || new TextEncoder().encode(text).length > BENCH_TRACE_MAX_BYTES) {
    throw new Error(`Bench traces may be at most ${BENCH_TRACE_MAX_BYTES / 1024} KiB.`);
  }

  const lines = text.trim().split('\n');
  const first = readRow(lines[0] ?? '');
  if (first.kind !== 'header') throw new Error('Missing bench trace header.');
  if (first.version !== 1) throw new Error('Unsupported bench trace version.');
  const header = {
    version: first.version,
    build: identifier(first, 'build'),
    fixture: identifier(first, 'fixture'),
    trial: identifier(first, 'trial'),
  } as const;

  const last = readRow(lines[lines.length - 1] ?? '');
  if (last.kind !== 'end') throw new Error('Missing bench trace trailer.');
  const dropped = integer(last, 'dropped', 0, UINT32_MAX);
  const samples: DeviceSample[] = [];
  const measurements = new Map<string, { source: string; angleDeci: number }>();
  let missing = 0;
  let previousSeq = -1;
  let previousTimeUs = 0;

  for (const line of lines.slice(1, -1)) {
    const row = readRow(line);
    if (row.kind === 'reference') {
      const trial = identifier(row, 'trial');
      const cue = identifier(row, 'cue');
      const source = row.source;
      if (typeof source !== 'string' || source.trim().length === 0) {
        throw new Error('Invalid bench trace reference source.');
      }
      const angleDeci = integer(row, 'angleDeci', INT16_MIN, INT16_MAX);
      if (trial === header.trial) {
        if (measurements.has(cue)) throw new Error('Ambiguous bench trace reference cue.');
        measurements.set(cue, { source, angleDeci });
      }
      continue;
    }

    const sample = readSample(row);
    if (sample.seq <= previousSeq || sample.timeUs < previousTimeUs) {
      throw new Error('Bench samples are out of device sequence or time order.');
    }
    missing += sample.seq - previousSeq - 1;
    previousSeq = sample.seq;
    previousTimeUs = sample.timeUs;
    samples.push(sample);
  }

  return {
    header,
    samples: samples.map(sample => ({
      ...sample,
      measurement: sample.cue === undefined ? null : measurements.get(sample.cue) ?? null,
    })),
    dropped,
    missing,
    incomplete: dropped > 0 || missing > 0,
  };
}
