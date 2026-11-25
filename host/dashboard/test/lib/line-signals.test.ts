import { expect, it } from 'vitest';

import { describeEvent } from '../../src/lib/events';
import { barcodeEdgeRuns, barcodeElementLabel, barcodePhaseCode, barcodePhaseLabel, lineSignalPlotData, lineSignalRuns, type LineSignalSample } from '../../src/lib/line-signals';
import { EVENTS } from '../../src/lib/schema.generated';
import { emptyStore, reduceStore } from '../../src/lib/telemetry-store';

it('retains bounded status samples with unchanged odometry and resets them on boot', () => {
  const store = Array.from({ length: 100 }, (_, i) => ({
    packet: 'status', timestamp: i * 200, values: { seq: i, odometer_mm: 50, speed_mms: 0, line_bits: i % 8 },
  } as const)).reduce(reduceStore, emptyStore());
  expect(store.lineSignals).toHaveLength(60);
  expect(store.lineSignals[0]).toEqual({ timestamp: 8000, seq: 40, bits: 0 });
  expect(store.lineSignals.at(-1)).toEqual({ timestamp: 19800, seq: 99, bits: 3 });
  expect(reduceStore(store, { packet: 'event', timestamp: 20000, values: { event_type: 1 } }).lineSignals).toEqual([]);
});

it('recovers a short IR pulse from a reordered status batch across sequence rollover', () => {
  const frames = [65535, 1, 0].map((seq, i) => ({
    packet: 'status' as const, timestamp: 10000 + i, values: { seq, odometer_mm: 40, line_bits: seq === 0 ? 1 : 0 },
  }));
  const store = frames.reduce(reduceStore, emptyStore());
  expect(store.lineSignals.map(s => [s.seq, s.bits])).toEqual([[65535, 0], [0, 1], [1, 0]]);
  expect(store.lineSignals[1]!.timestamp).toBeLessThanOrEqual(store.lineSignals[2]!.timestamp);
  expect(store.latest.status?.values.seq).toBe(1);
});

it('shows observed changes without drawing through missed, invalid, or delayed samples', () => {
  const samples: LineSignalSample[] = [
    { timestamp: 0, seq: 65535, bits: 1 },
    { timestamp: 200, seq: 0, bits: 1 },
    { timestamp: 400, seq: 1, bits: 0 },
    { timestamp: 600, seq: 3, bits: 0 },
    { timestamp: 800, seq: 4, bits: null },
    { timestamp: 1000, seq: 5, bits: 0 },
    { timestamp: 2200, seq: 6, bits: 0 },
  ];
  expect(lineSignalRuns(samples, 2200, 0)).toEqual([
    [{ at: 0, on: true }, { at: 200, on: true }],
    [{ at: 400, on: false }], [{ at: 600, on: false }], [{ at: 1000, on: false }], [{ at: 2200, on: false }],
  ]);
  expect(lineSignalRuns(samples, 12_201, 0)).toEqual([]);
});

it.each([[undefined, 'Unknown'], [7, 'Unknown'], [8 | 1, 'Space'], [16 | 2, 'Bar'], [24 | 4, 'Block']])(
  'decodes the last completed barcode element locally from status bits %s', (bits, expected) => {
    expect(barcodeElementLabel(bits)).toBe(expected);
  },
);

it('uses snapshot phase for catch-up and preserves orientation and reject details on observed events', () => {
  expect(barcodePhaseLabel(barcodePhaseCode(7))).toBeUndefined();
  expect(barcodePhaseLabel(barcodePhaseCode(7 << 5))).toBeUndefined();
  expect(barcodePhaseLabel(barcodePhaseCode((2 << 5) | (3 << 3) | 7))).toBe('Decoding');
  expect(describeEvent(EVENTS.barcodePhase, 1, 0)).toEqual({
    kind: 'barcode', title: 'Barcode start', detail: '',
  });
  expect(describeEvent(EVENTS.barcodePhase, 4, (2 << 8) | 1)).toEqual({
    kind: 'barcode', title: 'Barcode aborted', detail: 'Read in reverse, Parity check failed.',
  });
});

it('shows both dark stripes between two light status samples at their device-measured widths', () => {
  const edges = [0, 12000, 32000, 38000].map((offset, i) => ({
    timestamp: 900, packet: 'barcode_sample', values: {
      seq: (254 + i) & 255, edge_us: (0xfffff000 + offset) >>> 0, level_high: 1 - i % 2,
      interval_us: [0, 12000, 20000, 6000][i]!, capture_loss: 0, width_valid: 0,
    },
  }));
  const store = edges.reduce(reduceStore, emptyStore());
  const runs = barcodeEdgeRuns(store.barcodeEdges, 1000);
  expect(runs.filter(r => r[0]!.on).map(r => r.at(-1)!.at - r[0]!.at)).toEqual([12, 6]);
  edges[3]!.values.capture_loss = 1;
  expect(barcodeEdgeRuns(edges, 1000).filter(r => r[0]!.on)).toHaveLength(1);
  expect(reduceStore(store, { timestamp: 1000, packet: 'event', values: { event_type: 1 } }).barcodeEdges).toEqual([]);
});

it('clamps digital IR spans at the left edge, emits transitions, and bridges short packet losses', () => {
  const samples: LineSignalSample[] = [
    { timestamp: 0, seq: 1, bits: 1 },
    { timestamp: 200, seq: 2, bits: 0 },
    // seq 3 dropped (400 ms gap -> estimated bridge)
    { timestamp: 600, seq: 4, bits: 0 },
  ];
  const plot = lineSignalPlotData(samples, [], 800, 0);
  expect(plot.measured).toEqual([{ from: 0, to: 200, on: true }]);
  expect(plot.transitions).toEqual([200]);
  expect(plot.estimated).toEqual([
    { from: 200, to: 600, on: false },
    { from: 600, to: 800, on: false },
  ]);

  // Left-edge clamp at now=10100 (window [100, 10100]) keeps 100..200 smoothly.
  const shifted = lineSignalPlotData(samples, [], 10_100, 0);
  expect(shifted.measured).toEqual([{ from: 100, to: 200, on: true }]);
});
