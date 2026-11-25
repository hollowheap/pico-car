import { useMemo, useState } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';
import Svg, { Path } from 'react-native-svg';

import type { ConsoleView } from '../../lib/console-view';
import { barcodeElementLabel, barcodePhaseLabel, LINE_SIGNAL_WINDOW_MS, lineSignalPlotData, type LineSignalSample } from '../../lib/line-signals';
import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import type { RunSummary } from '../../../../bridge/src/console/run-summary.js';
import { BIT_FIELDS } from '../../lib/schema.generated';
import type { DiagRates } from '../../../../bridge/src/console/telemetry-store.js';
import { useTheme } from '../../lib/theme-provider';
import { clampX, timeToX } from '../../lib/time-plot';
import { usePlotNow } from '../../lib/useNow';
import { formatTime } from './format';
import { MetricCell } from './MetricCell';
import { Panel, useDense, useExpanded } from './Panel';
import { EventCounts } from './EventCounts';
import { SectionPicker } from './SectionPicker';

type Props = {
  summary?: RunSummary;
  statusPacket: LinkPacket | undefined;
  diag: LinkPacket | undefined;
  rates: DiagRates;
  barcode: ConsoleView['barcode'];
  junction: ConsoleView['junction'];
  longestRecoveryMm: ConsoleView['longestRecoveryMm'];
  lineSignals: readonly LineSignalSample[];
  barcodeEdges?: readonly LinkPacket[];
  nowMs: number;
  clockOffsetMs?: number;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
  summaryOnly?: boolean;
};

const SIGNALS = [
  { label: 'Left', bit: BIT_FIELDS.lineBits.left, high: 'On', low: 'Off' },
  { label: 'Right', bit: BIT_FIELDS.lineBits.right, high: 'On', low: 'Off' },
  { label: 'Barcode IR', bit: BIT_FIELDS.lineBits.barcode, high: 'Dark', low: 'Light' },
];
const COL_WIDTH = 120;
const COL_HEIGHT = 36;
const HIGH_Y = 8;
const LOW_Y = 28;

function LineSignalPlot({ bits, samples, edges, junctionMarkers, nowMs, clockOffsetMs, live }: {
  edges: readonly LinkPacket[];
  bits: number | undefined;
  samples: readonly LineSignalSample[];
  junctionMarkers: Readonly<ConsoleView['junction']['markers']>;
  nowMs: number;
  clockOffsetMs?: number;
  live: boolean;
}) {
  const theme = useTheme();
  const plotNow = usePlotNow(nowMs, live, clockOffsetMs);
  const { lanes, junctionPath } = useMemo(() => {
    const x = (at: number) => timeToX(at, plotNow, COL_WIDTH, LINE_SIGNAL_WINDOW_MS);
    const yOf = (on: boolean) => (on ? HIGH_Y : LOW_Y);
    const builtLanes = SIGNALS.map(({ bit }) => {
      const data = lineSignalPlotData(samples, edges, plotNow, bit);
      const solid = data.measured.map((span) => {
        const x1 = clampX(x(span.from) - (span.from === span.to ? 1 : 0), COL_WIDTH);
        const x2 = clampX(x(span.to) + (span.from === span.to ? 1 : 0), COL_WIDTH);
        return `M${x1.toFixed(1)} ${yOf(span.on)} H${x2.toFixed(1)}`;
      }).join(' ');
      const transitions = data.transitions.map((at) => {
        const px = clampX(x(at), COL_WIDTH);
        return `M${px.toFixed(1)} ${HIGH_Y} V${LOW_Y}`;
      }).join(' ');
      const estimated = data.estimated.map((span) => {
        const x1 = clampX(x(span.from), COL_WIDTH);
        const x2 = clampX(x(span.to), COL_WIDTH);
        return `M${x1.toFixed(1)} ${yOf(span.on)} H${x2.toFixed(1)}`;
      }).join(' ');
      return { solid, transitions, estimated };
    });
    const jPath = junctionMarkers
      .filter((m) => m.timestamp >= plotNow - LINE_SIGNAL_WINDOW_MS && m.timestamp <= plotNow)
      .map((m) => `M${x(m.timestamp).toFixed(1)} 0 V${COL_HEIGHT}`)
      .join(' ');
    return { lanes: builtLanes, junctionPath: jPath };
  }, [samples, edges, junctionMarkers, plotNow]);
  const caption = [theme.typography.caption, { color: theme.colors.muted }];

  return (
    <View testID="line-signal-chart" style={styles.signalColumns}>
      {SIGNALS.map(({ label, bit, high, low }, i) => {
        const active = bits !== undefined && (bits & (1 << bit)) !== 0;
        const lane = lanes[i]!;
        return (
          <View key={bit} style={[styles.signalColumn, i > 0 && [styles.colDivider, { borderLeftColor: theme.colors.border }]]}>
            <View style={styles.signalHeader}>
              <Text style={caption}>{label}</Text>
              <Text style={[theme.typography.caption, { color: active ? theme.colors.accent : theme.colors.text }]}>
                {bits === undefined ? '--' : active ? high : low}
              </Text>
            </View>
            <Svg accessibilityLabel={`${label} IR signal history, last ten seconds`}
              accessibilityRole="image" height={COL_HEIGHT} preserveAspectRatio="none"
              viewBox={`0 0 ${COL_WIDTH} ${COL_HEIGHT}`} width="100%">
              <Path d={`M0 ${HIGH_Y} H${COL_WIDTH} M0 ${LOW_Y} H${COL_WIDTH}`}
                fill="none" stroke={theme.colors.border} strokeDasharray="2 3" />
              {junctionPath ? <Path d={junctionPath} fill="none" stroke={theme.colors.accent}
                strokeDasharray="3 2" strokeWidth={1.2} opacity={0.75} /> : null}
              {lane.estimated ? <Path d={lane.estimated} fill="none" stroke={theme.colors.text}
                strokeDasharray="3 3" strokeLinecap="round" strokeWidth={1.4} opacity={0.45} /> : null}
              {lane.transitions ? <Path d={lane.transitions} fill="none" stroke={theme.colors.text}
                strokeWidth={1.1} opacity={0.35} /> : null}
              {lane.solid ? <Path d={lane.solid} fill="none" stroke={theme.colors.text}
                strokeLinecap="round" strokeWidth={1.8} /> : null}
            </Svg>
            <View style={styles.axis}>
              <Text style={caption}>−10 s</Text>
              <Text style={caption}>Now</Text>
            </View>
          </View>
        );
      })}
    </View>
  );
}

const VIEWS = { live: 'Live', history: 'History' } as const;

function BarcodeStrip({ edges, bits }: { edges: readonly LinkPacket[]; bits: number | undefined }) {
  const theme = useTheme();
  const recent = edges.slice(-32).filter((e) => (e.values.width_um ?? 0) > 0 || (e.values.interval_us ?? 0) > 0);
  const leftOn = bits !== undefined && (bits & (1 << BIT_FIELDS.lineBits.left)) !== 0;
  const rightOn = bits !== undefined && (bits & (1 << BIT_FIELDS.lineBits.right)) !== 0;
  const barOn = bits !== undefined && (bits & (1 << BIT_FIELDS.lineBits.barcode)) !== 0;
  const barState = bits === undefined ? '--' : barOn ? 'Dark' : 'Light';
  const widths = recent.map((e) => (e.values.width_valid ? e.values.width_um : Math.round((e.values.interval_us ?? 0) / 100)) || 1);
  const total = widths.reduce((sum, w) => sum + w, 0) || 1;
  return (
    <View style={styles.scannerRow} testID="barcode-spatial-strip">
      {/* Live 3-pod optical sensor array under the nose */}
      <View style={styles.podArray}>
        <View style={styles.podHead}>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>IR PODS</Text>
          <Text style={[theme.typography.caption, { color: barOn ? theme.colors.warning : theme.colors.muted }]}>{barState}</Text>
        </View>
        <View style={[styles.pods, { backgroundColor: theme.colors.elevated, borderColor: theme.colors.border }]}>
          {[
            ['L', leftOn, theme.colors.info],
            ['R', rightOn, theme.colors.info],
            ['B', barOn, theme.colors.warning],
          ].map(([tag, active, color]) => (
            <View key={tag as string} style={[styles.pod, {
              borderColor: active ? (color as string) : theme.colors.border,
              backgroundColor: active ? (color as string) : theme.colors.card,
            }]}>
              <Text style={[theme.typography.caption, { fontWeight: '600', color: active ? theme.colors.canvas : theme.colors.muted }]}>
                {tag as string}
              </Text>
            </View>
          ))}
        </View>
      </View>
      {/* Optical barcode stripe scanner with read-head crosshair */}
      <View style={[styles.stripWrap, styles.colDivider, { borderLeftColor: theme.colors.border }]}>
        <View style={styles.podHead}>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>STRIPES</Text>
          <Text style={[theme.typography.caption, styles.tabular, { color: theme.colors.muted }]}>
            {recent.length > 0 ? `${recent.length} · ${Math.round(total / 1000)} mm` : '--'}
          </Text>
        </View>
        {recent.length === 0 ? (
          <View style={[styles.stripEmpty, { borderColor: theme.colors.border }]}>
            <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>Waiting for barcode stripes</Text>
          </View>
        ) : (
          <View style={[styles.stripBar, { borderColor: theme.colors.border }]}>
            {recent.map((edge, i) => {
              const dark = edge.values.level_high === 0;
              const loss = (edge.values.capture_loss ?? 0) !== 0;
              return (
                <View
                  key={`${edge.values.seq ?? i}-${edge.values.edge_us ?? i}`}
                  style={{
                    flex: Math.max(1, widths[i]!),
                    backgroundColor: loss ? theme.colors.danger : dark ? theme.colors.text : 'transparent',
                    opacity: loss ? 0.85 : dark ? 0.92 : 0.12,
                  }}
                />
              );
            })}
            <View style={[styles.readHead, { backgroundColor: theme.colors.accent }]} />
          </View>
        )}
      </View>
    </View>
  );
}

/**
 * What the line sensors see now, how the follower copes (edges per second,
 * lost count), and the last barcodes with the action each one caused.
 */
export function LineBarcodePanel({ statusPacket, diag, rates, barcode, junction, longestRecoveryMm,
  lineSignals, barcodeEdges = [], nowMs, clockOffsetMs, status, style, summary, summaryOnly = false }: Props) {
  const theme = useTheme();
  const [view, setView] = useState<keyof typeof VIEWS>('live');
  const bits = statusPacket?.values.line_bits;
  const d = diag?.values;
  const { actionLabel, code, earlierPhases, letter, phase } = barcode;
  const dense = useDense();
  const expanded = useExpanded();
  const text = dense ? theme.typography.caption : theme.typography.body;
  const caption = [theme.typography.caption, { color: theme.colors.muted }];
  const junctionText = junction.active
    ? junction.active.direction === 'Straight' ? 'Going straight'
      : junction.active.direction === 'U-turn' ? 'U-turn' : `Turning ${junction.active.direction.toLowerCase()}`
    : junction.armed
      ? `${junction.armed.direction} armed${junction.armed.letter ? ` (${junction.armed.letter})` : ''}${junction.armed.probeNote ? `, ${junction.armed.probeNote}` : ''}`
      : junction.last
        ? `${junction.last.direction} ${junction.last.outcome === 'aborted' ? 'aborted' : 'done'} (${junction.last.durationMs} ms)`
        : junction.lastStripe
          ? `${junction.lastStripe.shape}${junction.lastStripe.corroborated ? '' : '?'}`
          : 'None';
  const firmwareCounts = [
    d?.barcode_ok === undefined ? null : `Decoded ${d.barcode_ok}`,
    d?.barcode_bad === undefined ? null : `read issues ${d.barcode_bad}`,
    d?.barcode_edge_drops === undefined ? null : `edge drops ${d.barcode_edge_drops}`,
  ].filter((field) => field !== null).join(', ');

  return (
    <Panel fill icon="line" status={status} style={style} testID="line-panel" title="Line and barcode"
      bodyStyle={styles.spread} empty={!summaryOnly && statusPacket === undefined && diag === undefined}
      footer={summaryOnly ? null : <>
        {firmwareCounts || d?.line_lost_count !== undefined ? <View style={styles.counts}>
          <Text style={caption}>Since boot</Text>
          {firmwareCounts ? <Text style={[text, { color: theme.colors.text }]}>{firmwareCounts}</Text> : null}
          {d?.line_lost_count === undefined ? null : <Text style={[text, { color: theme.colors.text }]}>
            {d.line_lost_count} line {d.line_lost_count === 1 ? 'loss' : 'losses'}
          </Text>}
        </View> : null}
        {summary && <EventCounts testID="line-events" scope={summary.scope} counts={[
          [d?.barcode_ok === undefined ? summary.barcodesOk : undefined, 'decode'],
          [summary.barcodesNoAction, 'no action', 'no action'],
          [d?.barcode_bad === undefined ? summary.barcodesBad : undefined, 'read issue'],
          [d?.line_lost_count === undefined ? summary.lineLosses : undefined, 'line loss', 'line losses'],
        ]} />}
      </>}
      right={summaryOnly || expanded ? null : <SectionPicker id="line-view" label="Line views" options={VIEWS} value={view} onChange={setView} />}>
      {summaryOnly ? null : (expanded || view === 'live') ? (
        <BarcodeStrip edges={barcodeEdges} bits={status?.kind === 'stale' ? undefined : bits} />
      ) : null}
      {summaryOnly ? null : (expanded || view === 'history') ? (
        <View style={expanded ? [styles.foldSection, { borderTopColor: theme.colors.border }] : undefined}>
          <LineSignalPlot bits={status?.kind === 'stale' ? undefined : bits} samples={lineSignals}
            edges={barcodeEdges} junctionMarkers={junction.markers} nowMs={nowMs} clockOffsetMs={clockOffsetMs}
            live={status?.kind === 'ready'} />
        </View>
      ) : null}
      <View style={styles.decoder}>
        <MetricCell label="Element" value={barcodeElementLabel(bits)} testID="last-barcode-element" />
        <MetricCell
          label="Junction"
          value={junctionText}
          tone={junction.active ? 'warning' : junction.armed ? 'info' : 'default'}
          testID="junction-status"
          style={[styles.colDivider, { borderLeftColor: theme.colors.border }]}
        />
        <MetricCell label="Phase" value={phase ?? 'None'} testID="barcode-phase"
          style={[styles.colDivider, { borderLeftColor: theme.colors.border }]} />
        {summaryOnly ? null : (
          <MetricCell
            label="Edges"
            value={`${rates.line_edges_per_s ?? '--'}/s (${rates.line_raw_edges_per_s ?? '--'})`}
            style={[styles.colDivider, { borderLeftColor: theme.colors.border }]}
          />
        )}
        <MetricCell
          label="Barcode"
          value={letter === null ? (code === 0 ? 'None' : 'Unknown') : `${letter} (${actionLabel})`}
          tone={letter !== null ? 'success' : 'default'}
          testID="decoded-barcode"
          style={[styles.colDivider, { borderLeftColor: theme.colors.border }]}
        />
      </View>
      {summaryOnly || earlierPhases.length === 0 ? null : (
        <View style={styles.phases} testID="barcode-phases">
          {earlierPhases.map((event) => (
            <View key={`${event.eventId}-${event.timestamp}`} style={styles.phase}>
              <Text style={caption}>{formatTime(event.receivedAt ?? event.timestamp)}</Text>
              <Text style={[theme.typography.caption, { color: theme.colors.text }]}>{barcodePhaseLabel(event.p1)}</Text>
            </View>
          ))}
        </View>
      )}
      {summaryOnly || longestRecoveryMm === null ? null : (
        <Text style={caption}>Longest recovery {longestRecoveryMm} mm</Text>
      )}
    </Panel>
  );
}

const styles = StyleSheet.create({
  headerRight: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  decoded: { alignItems: 'flex-end' },
  result: { alignItems: 'center', flexDirection: 'row', gap: 6 },
  scannerRow: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 0 },
  podArray: { gap: 2 },
  podHead: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 6 },
  pods: { flexDirection: 'row', gap: 4, paddingHorizontal: 5, paddingVertical: 3, borderRadius: 14, borderWidth: 1 },
  pod: { width: 22, height: 22, borderRadius: 11, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
  stripWrap: { flex: 1, minWidth: 0, gap: 2 },
  stripBar: { flexDirection: 'row', height: 26, borderRadius: 4, borderWidth: 1, overflow: 'hidden', position: 'relative' },
  readHead: { position: 'absolute', right: 0, top: 0, bottom: 0, width: 2 },
  stripEmpty: { height: 26, borderRadius: 6, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  signalColumns: { flexDirection: 'row', gap: 8 },
  signalColumn: { flex: 1, minWidth: 0, gap: 2 },
  colDivider: { borderLeftWidth: 1, paddingLeft: 8 },
  foldSection: { borderTopWidth: 1, paddingTop: 6 },
  signalHeader: { alignItems: 'center', flexDirection: 'row', justifyContent: 'space-between', gap: 4 },
  axis: { flexDirection: 'row', justifyContent: 'space-between' },
  decoder: { flexDirection: 'row', gap: 8 },
  spread: { justifyContent: 'space-evenly' },
  counts: { flexDirection: 'row', flexWrap: 'wrap', columnGap: 6 },
  metric: { flex: 1, minWidth: 0 },
  tabular: { fontVariant: ['tabular-nums'] },
  phases: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  phase: { minWidth: 58 },
});
