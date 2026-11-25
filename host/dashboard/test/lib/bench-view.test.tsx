/// <reference types="node" />

import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import type { ReactElement, ReactNode } from 'react';
import { describe, expect, it, vi } from 'vitest';

import BenchScreen, { BenchTraceView } from '../../src/app/bench';
import {
  BENCH_TRACE_SIZE_ERROR,
  EMPTY_BENCH_STATE,
  importBenchTraceFile,
  reduceBenchImport,
} from '../../src/lib/bench-import';
import { BENCH_TRACE_MAX_BYTES, parseBenchTrace } from '../../src/lib/bench-trace';
import { themeFor } from '../../src/lib/theme';

const { renderToStaticMarkup } = createRequire(import.meta.url)('react-dom/server') as {
  renderToStaticMarkup: (element: ReactElement) => string;
};

type ViewProps = {
  accessibilityLabel?: string;
  accessibilityRole?: string;
  children?: ReactNode;
  role?: string;
  testID?: string;
};

type TextProps = {
  children?: ReactNode;
  nativeID?: string;
  testID?: string;
};

vi.mock('expo-router', () => ({
  Link: ({ children }: { children?: ReactNode }) => children,
}));
vi.mock('react-native', () => ({
  Platform: { OS: 'web' },
  StyleSheet: { create: (styles: unknown) => styles, absoluteFill: {} },
  useWindowDimensions: () => ({ width: 1280, height: 800 }),
  ScrollView: ({ children, testID }: ViewProps) => <div data-testid={testID}>{children}</div>,
  Pressable: ({ children, accessibilityLabel, accessibilityRole, testID }: ViewProps) => (
    <a aria-label={accessibilityLabel} data-testid={testID} role={accessibilityRole}>
      {children}
    </a>
  ),
  View: ({ children, accessibilityLabel, accessibilityRole, role, testID }: ViewProps) => (
    <div
      aria-label={accessibilityLabel}
      data-testid={testID}
      role={role ?? (accessibilityRole === 'summary' ? 'region' : undefined)}
    >
      {children}
    </div>
  ),
  Text: ({ children, nativeID, testID }: TextProps) => (
    <span data-testid={testID} id={nativeID}>
      {children}
    </span>
  ),
}));
vi.mock('../../src/lib/theme-provider', () => ({ useTheme: () => themeFor('dark') }));
vi.mock('../../src/components/console/PanelIcon', () => ({ PanelIcon: () => null }));

const sharedFixture = readFileSync(new URL('../../../../tests/fixtures/bench-trace.jsonl', import.meta.url), 'utf8');

describe('bench trace file import', () => {
  it('rejects an oversized file before calling file.text()', async () => {
    const text = vi.fn(async () => sharedFixture);
    const result = await importBenchTraceFile({
      name: 'oversized.jsonl',
      size: BENCH_TRACE_MAX_BYTES + 1,
      text,
    });

    expect(text).not.toHaveBeenCalled();
    expect(result).toEqual({
      ok: false,
      fileName: 'oversized.jsonl',
      error: BENCH_TRACE_SIZE_ERROR,
    });
  });

  it('rejects a non-finite file size before calling file.text()', async () => {
    const text = vi.fn(async () => sharedFixture);
    const result = await importBenchTraceFile({
      name: 'unknown-size.jsonl',
      size: Number.NaN,
      text,
    });

    expect(text).not.toHaveBeenCalled();
    expect(result.ok).toBe(false);
  });

  it('reads and parses a valid file at the exact size limit', async () => {
    const padded = sharedFixture.padEnd(BENCH_TRACE_MAX_BYTES, ' ');
    const text = vi.fn(async () => padded);
    const result = await importBenchTraceFile({
      name: 'boundary.jsonl',
      size: BENCH_TRACE_MAX_BYTES,
      text,
    });

    expect(text).toHaveBeenCalledTimes(1);
    expect(result.ok).toBe(true);
    if (result.ok) {
      expect(result.report.header.trial).toBe('center-return');
      expect(result.report.samples).toHaveLength(2);
    }
  });

  it('preserves the prior valid report when a later import is oversized or invalid', async () => {
    const loaded = reduceBenchImport(
      EMPTY_BENCH_STATE,
      await importBenchTraceFile({
        name: 'bench-trace.jsonl',
        size: Buffer.byteLength(sharedFixture, 'utf8'),
        text: async () => sharedFixture,
      }),
    );

    expect(loaded.report?.header.trial).toBe('center-return');
    expect(loaded.error).toBeNull();

    const textSpy = vi.fn(async () => 'should not be read');
    const afterOversized = reduceBenchImport(
      loaded,
      await importBenchTraceFile({
        name: 'huge.jsonl',
        size: BENCH_TRACE_MAX_BYTES + 1024,
        text: textSpy,
      }),
    );

    expect(textSpy).not.toHaveBeenCalled();
    expect(afterOversized.report).toBe(loaded.report);
    expect(afterOversized.fileName).toBe('bench-trace.jsonl');
    expect(afterOversized.error).toMatch(/128 KiB/i);

    const afterUnsupported = reduceBenchImport(
      afterOversized,
      await importBenchTraceFile({
        name: 'v99.jsonl',
        size: 128,
        text: async () => sharedFixture.replace('"version":1', '"version":99'),
      }),
    );

    expect(afterUnsupported.report).toBe(loaded.report);
    expect(afterUnsupported.fileName).toBe('bench-trace.jsonl');
    expect(afterUnsupported.error).toMatch(/version/i);
  });
});

describe('bench replay presentation', () => {
  it('renders an accessible file input, bench trace region and zero vehicle commands', () => {
    const markup = renderToStaticMarkup(<BenchScreen />);

    expect(markup).toContain('<label for="bench-trace-input"');
    expect(markup).toContain('>Import bench trace</label>');
    expect(markup).toContain('id="bench-trace-input"');
    expect(markup).toContain('type="file"');
    expect(markup).toContain('aria-label="Bench trace"');
    expect(markup).toContain('role="region"');
    for (const command of ['Start', 'Stop', 'E-stop', 'Reset', 'Left', 'Right', 'U-turn', 'Straight', 'Recalibrate']) {
      expect(markup).not.toContain(`>${command}<`);
    }
  });

  it('displays provenance, device timing, separate physical source, unavailable status and preserved report on error', () => {
    const report = parseBenchTrace(sharedFixture);
    const markup = renderToStaticMarkup(
      <BenchTraceView
        state={{ fileName: 'bench-trace.jsonl', report, error: BENCH_TRACE_SIZE_ERROR }}
        wide
      />,
    );

    expect(markup).toContain('role="alert"');
    expect(markup).toContain('Bench traces may be at most 128 KiB.');
    expect(markup).toContain('>abc123<');
    expect(markup).toContain('>servo<');
    expect(markup).toContain('>center-return<');
    expect(markup).toContain('>v1<');
    expect(markup).toContain('>Device (µs)<');
    expect(markup).toContain('>100 µs<');
    expect(markup).toContain('>1494 µs<');
    expect(markup).toContain('>0.0°<');
    expect(markup).toContain('>0.7°<');
    expect(markup).toContain('>angle gauge<');
    expect(markup).toContain('>Measured bearing unavailable<');
    expect(markup).toContain('>Complete<');
  });

  it('displays incomplete capture warnings and scheduler/trigger/reply details', () => {
    const exported = [
      '{"kind":"header","version":1,"build":"abc1234+dirty","fixture":"bench-timing","trial":"timing-01"}',
      '{"kind":"scheduler","seq":0,"timeUs":29,"marker":"release"}',
      '{"kind":"trigger","seq":2,"timeUs":40,"requestedDeci":-350,"effectiveDeci":-300}',
      '{"kind":"reply","seq":3,"timeUs":85,"requestedDeci":-350,"effectiveDeci":-300,"echo":"too_close","rangeMm":42}',
      '{"kind":"end","dropped":3}',
    ].join('\n');
    const report = parseBenchTrace(exported);
    const markup = renderToStaticMarkup(
      <BenchTraceView state={{ fileName: 'timing.jsonl', report, error: null }} wide={false} />,
    );

    expect(markup).toContain('>Incomplete<');
    expect(markup).toContain('>Incomplete capture<');
    expect(markup).toContain('3 dropped records, 1 missing sequence entry.');
    expect(markup).toContain('>release<');
    expect(markup).toContain('>-35.0°<');
    expect(markup).toContain('>-30.0°<');
    expect(markup).toContain('>too close<');
    expect(markup).toContain('>42 mm<');
  });

  it('does not import live telemetry store or command modules in app/bench.tsx', () => {
    const source = readFileSync(new URL('../../src/app/bench.tsx', import.meta.url), 'utf8');
    expect(source).not.toMatch(/telemetry-context|telemetry-store|useCommand|ControlsPanel/);
  });
});
