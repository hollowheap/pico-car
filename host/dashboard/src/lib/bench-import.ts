import { BENCH_TRACE_MAX_BYTES, parseBenchTrace, type BenchTraceReport } from './bench-trace';

export type BenchTraceFile = {
  name: string;
  size: number;
  text: () => Promise<string>;
};

export type BenchImportResult =
  | { ok: true; fileName: string; report: BenchTraceReport }
  | { ok: false; fileName: string; error: string };

export type BenchViewState = {
  fileName: string | null;
  report: BenchTraceReport | null;
  error: string | null;
};

export const EMPTY_BENCH_STATE: BenchViewState = {
  fileName: null,
  report: null,
  error: null,
};

export const BENCH_TRACE_SIZE_ERROR = `Bench traces may be at most ${BENCH_TRACE_MAX_BYTES / 1024} KiB.`;

export async function importBenchTraceFile(file: BenchTraceFile): Promise<BenchImportResult> {
  if (!Number.isFinite(file.size) || file.size > BENCH_TRACE_MAX_BYTES) {
    return { ok: false, fileName: file.name, error: BENCH_TRACE_SIZE_ERROR };
  }

  let text: string;
  try {
    text = await file.text();
  } catch {
    return { ok: false, fileName: file.name, error: 'Could not read the selected bench trace file.' };
  }

  try {
    return { ok: true, fileName: file.name, report: parseBenchTrace(text) };
  } catch (err) {
    return {
      ok: false,
      fileName: file.name,
      error: err instanceof Error ? err.message : 'Invalid bench trace.',
    };
  }
}

export function reduceBenchImport(state: BenchViewState, result: BenchImportResult): BenchViewState {
  if (result.ok) {
    return { fileName: result.fileName, report: result.report, error: null };
  }
  return { ...state, error: result.error };
}

export function formatDeciDegrees(deci: number): string {
  return `${(deci / 10).toFixed(1)}°`;
}

export function formatMicros(us: number): string {
  return `${us} µs`;
}

export function formatEchoClass(echo: string): string {
  return echo.replace(/_/g, ' ');
}

export function incompleteSummary(dropped: number, missing: number): string {
  const parts: string[] = [];
  if (dropped > 0) parts.push(`${dropped} dropped ${dropped === 1 ? 'record' : 'records'}`);
  if (missing > 0) parts.push(`${missing} missing sequence ${missing === 1 ? 'entry' : 'entries'}`);
  return parts.length === 0 ? 'Complete capture' : parts.join(', ');
}
