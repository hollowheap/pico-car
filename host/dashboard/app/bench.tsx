import { Link } from 'expo-router';
import { useRef, useState, type ChangeEvent } from 'react';
import {
  Platform,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  View,
  useWindowDimensions,
  type ViewStyle,
} from 'react-native';

import { Panel } from '../components/console/Panel';
import { StatusPill } from '../components/console/StatusPill';
import {
  EMPTY_BENCH_STATE,
  formatDeciDegrees,
  formatEchoClass,
  formatMicros,
  importBenchTraceFile,
  incompleteSummary,
  reduceBenchImport,
  type BenchViewState,
} from '../lib/bench-import';
import { BENCH_TRACE_MAX_BYTES, type BenchTraceReport } from '../lib/bench-trace';
import { consoleFrame } from '../lib/console-frame';
import { useTheme } from '../lib/theme-provider';

const INPUT_ID = 'bench-trace-input';
const HINT_ID = 'bench-trace-hint';

type Sample = BenchTraceReport['samples'][number];

function BackToConsoleLink() {
  const theme = useTheme();
  const [focused, setFocused] = useState(false);
  const [pressed, setPressed] = useState(false);
  const focusRing: ViewStyle | undefined =
    Platform.OS === 'web'
      ? ({
          outlineStyle: 'solid',
          outlineWidth: 2,
          outlineColor: theme.colors.accent,
          outlineOffset: 2,
        } as ViewStyle)
      : undefined;

  return (
    <Link asChild dismissTo href="/">
      <Pressable
        accessibilityLabel="Back to console"
        accessibilityRole="link"
        onBlur={() => setFocused(false)}
        onFocus={() => setFocused(true)}
        onPressIn={() => setPressed(true)}
        onPressOut={() => setPressed(false)}
        style={{
          ...(focused ? focusRing : undefined),
          alignItems: 'center',
          backgroundColor: pressed ? theme.colors.elevated : theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.control,
          borderWidth: 1,
          justifyContent: 'center',
          minHeight: 44,
          minWidth: 44,
          opacity: pressed ? 0.8 : 1,
          paddingHorizontal: 12,
        }}
        testID="bench-back-link"
      >
        <Text style={[theme.typography.label, { color: theme.colors.text }]}>Back to console</Text>
      </Pressable>
    </Link>
  );
}

function MetadataItem({ label, value, testID }: { label: string; value: string; testID?: string }) {
  const theme = useTheme();
  return (
    <View style={[styles.metaCell, { borderColor: theme.colors.border, backgroundColor: theme.colors.elevated, borderRadius: theme.radii.control }]}>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>{label}</Text>
      <Text style={[theme.typography.label, styles.tabular, { color: theme.colors.text }]} testID={testID}>
        {value}
      </Text>
    </View>
  );
}

function CommandedDetail({ sample }: { sample: Sample }) {
  const theme = useTheme();
  const valueStyle = [theme.typography.caption, styles.tabular, { color: theme.colors.text }];
  const labelStyle = [theme.typography.caption, { color: theme.colors.muted }];

  if (sample.kind === 'scheduler') {
    return (
      <View style={styles.detailGroup}>
        <Text style={labelStyle}>Marker</Text>
        <Text style={valueStyle}>{sample.marker}</Text>
      </View>
    );
  }

  return (
    <View style={styles.detailWrap}>
      <View style={styles.detailGroup}>
        <Text style={labelStyle}>Requested</Text>
        <Text style={valueStyle}>{formatDeciDegrees(sample.requestedDeci)}</Text>
      </View>
      <View style={styles.detailGroup}>
        <Text style={labelStyle}>Effective</Text>
        <Text style={valueStyle}>{formatDeciDegrees(sample.effectiveDeci)}</Text>
      </View>
      {sample.kind === 'servo' ? (
        <View style={styles.detailGroup}>
          <Text style={labelStyle}>PWM</Text>
          <Text style={valueStyle}>{formatMicros(sample.pwmUs)}</Text>
        </View>
      ) : null}
      {sample.kind === 'reply' ? (
        <>
          <View style={styles.detailGroup}>
            <Text style={labelStyle}>Echo</Text>
            <Text style={valueStyle}>{formatEchoClass(sample.echo)}</Text>
          </View>
          <View style={styles.detailGroup}>
            <Text style={labelStyle}>Range</Text>
            <Text style={valueStyle}>{`${sample.rangeMm} mm`}</Text>
          </View>
        </>
      ) : null}
    </View>
  );
}

function SampleRow({ sample, wide }: { sample: Sample; wide: boolean }) {
  const theme = useTheme();
  const caption = [theme.typography.caption, { color: theme.colors.muted }];
  const tabular = [theme.typography.caption, styles.tabular, { color: theme.colors.text }];

  return (
    <View
      style={[
        styles.sampleRow,
        wide ? styles.sampleRowWide : styles.sampleRowStack,
        { borderColor: theme.colors.border, backgroundColor: theme.colors.canvas, borderRadius: theme.radii.control },
      ]}
      testID={`bench-sample-${sample.seq}`}
    >
      <View style={wide ? styles.colSeq : styles.fieldPair}>
        {wide ? null : <Text style={caption}>Seq</Text>}
        <Text style={tabular}>{String(sample.seq)}</Text>
      </View>
      <View style={wide ? styles.colTime : styles.fieldPair}>
        {wide ? null : <Text style={caption}>Device time</Text>}
        <Text style={tabular}>{formatMicros(sample.timeUs)}</Text>
      </View>
      <View style={wide ? styles.colRecord : styles.fieldPair}>
        {wide ? null : <Text style={caption}>Record</Text>}
        <View style={styles.recordMeta}>
          <Text style={[theme.typography.label, { color: theme.colors.text }]}>{sample.kind}</Text>
          {sample.cue !== undefined ? (
            <View style={styles.cuePair}>
              <Text style={caption}>cue</Text>
              <Text style={[theme.typography.caption, { color: theme.colors.text }]}>{sample.cue}</Text>
            </View>
          ) : null}
        </View>
      </View>
      <View style={wide ? styles.colCommanded : styles.fieldBlock}>
        {wide ? null : <Text style={caption}>Commanded / device</Text>}
        <CommandedDetail sample={sample} />
      </View>
      <View style={wide ? styles.colMeasured : styles.fieldBlock}>
        {wide ? null : <Text style={caption}>Physical measurement</Text>}
        {sample.measurement === null ? (
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Measured bearing unavailable
          </Text>
        ) : (
          <View style={styles.measurementCell}>
            <Text style={[theme.typography.label, styles.tabular, { color: theme.colors.text }]}>
              {formatDeciDegrees(sample.measurement.angleDeci)}
            </Text>
            <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
              {sample.measurement.source}
            </Text>
          </View>
        )}
      </View>
    </View>
  );
}

export function BenchTraceView({
  state,
  onFileChange,
  wide,
}: {
  state: BenchViewState;
  onFileChange?: (event: ChangeEvent<HTMLInputElement>) => void;
  wide: boolean;
}) {
  const theme = useTheme();
  const { report, error, fileName } = state;

  return (
    <View style={styles.stack}>
      <Panel icon="trace" right={<BackToConsoleLink />} testID="bench-import-panel" title="Bench replay">
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
          Read-only replay of exported device bench traces and joined physical measurements. Live course history and vehicle controls remain untouched.
        </Text>
        {Platform.OS === 'web' ? (
          <View style={styles.uploadGroup}>
            <label
              htmlFor={INPUT_ID}
              style={{
                color: theme.colors.text,
                fontSize: theme.typography.label.fontSize,
                fontWeight: 600,
                letterSpacing: theme.typography.label.letterSpacing,
                lineHeight: `${theme.typography.label.lineHeight}px`,
              }}
            >
              Import bench trace
            </label>
            <input
              accept=".jsonl,.ndjson,application/x-ndjson,text/plain"
              aria-describedby={HINT_ID}
              id={INPUT_ID}
              onChange={onFileChange}
              style={{
                backgroundColor: theme.colors.elevated,
                border: `1px solid ${theme.colors.border}`,
                borderRadius: theme.radii.control,
                boxSizing: 'border-box',
                color: theme.colors.text,
                colorScheme: theme.mode,
                fontSize: 14,
                maxWidth: '100%',
                minHeight: 44,
                padding: '8px 10px',
                width: wide ? 420 : '100%',
              }}
              type="file"
            />
            <Text nativeID={HINT_ID} style={[theme.typography.caption, { color: theme.colors.muted }]}>
              {`JSONL export, up to ${BENCH_TRACE_MAX_BYTES / 1024} KiB. Uses the device microsecond timebase.`}
            </Text>
          </View>
        ) : (
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Bench trace file import is available in the web dashboard.
          </Text>
        )}
        {error !== null ? (
          <View
            accessibilityLiveRegion="assertive"
            role="alert"
            style={[
              styles.banner,
              {
                backgroundColor: theme.colors.dangerSurface,
                borderColor: theme.colors.danger,
                borderRadius: theme.radii.control,
              },
            ]}
            testID="bench-import-error"
          >
            <Text style={[theme.typography.label, { color: theme.colors.danger }]}>Import rejected</Text>
            <Text style={[theme.typography.caption, { color: theme.colors.danger }]}>{error}</Text>
          </View>
        ) : null}
      </Panel>

      <Panel
        right={
          report === null ? undefined : (
            <StatusPill
              label={report.incomplete ? 'Incomplete' : 'Complete'}
              tone={report.incomplete ? 'warning' : 'success'}
            />
          )
        }
        testID="bench-trace-region"
        title="Bench trace"
      >
        {report === null ? (
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]} testID="bench-empty">
            No bench trace loaded. Import a JSONL bench trace to inspect device timing, commanded output and physical measurements.
          </Text>
        ) : (
          <View style={styles.reportBody}>
            {report.incomplete ? (
              <View
                accessibilityLiveRegion="polite"
                role="status"
                style={[
                  styles.banner,
                  {
                    backgroundColor: theme.colors.warningSurface,
                    borderColor: theme.colors.warning,
                    borderRadius: theme.radii.control,
                  },
                ]}
                testID="bench-incomplete-warning"
              >
                <Text style={[theme.typography.label, { color: theme.colors.warning }]}>Incomplete capture</Text>
                <Text style={[theme.typography.caption, { color: theme.colors.warning }]}>
                  {`${incompleteSummary(report.dropped, report.missing)}.`}
                </Text>
              </View>
            ) : null}

            <View style={styles.metaGrid} testID="bench-provenance">
              <MetadataItem label="Build" testID="bench-build" value={report.header.build} />
              <MetadataItem label="Fixture" testID="bench-fixture" value={report.header.fixture} />
              <MetadataItem label="Trial" testID="bench-trial" value={report.header.trial} />
              <MetadataItem label="Format" testID="bench-version" value={`v${report.header.version}`} />
              <MetadataItem label="Timebase" value="Device (µs)" />
              <MetadataItem label="Dropped" testID="bench-dropped" value={String(report.dropped)} />
              <MetadataItem label="Missing seq" testID="bench-missing" value={String(report.missing)} />
              <MetadataItem label="Records" value={String(report.samples.length)} />
              {fileName !== null ? <MetadataItem label="File" value={fileName} /> : null}
            </View>

            {report.samples.length === 0 ? (
              <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
                No device records in this trace.
              </Text>
            ) : (
              <View style={styles.samplesSection}>
                {wide ? (
                  <View style={[styles.tableHeader, { borderBottomColor: theme.colors.border }]}>
                    <Text style={[theme.typography.label, styles.colSeq, { color: theme.colors.muted }]}>Seq</Text>
                    <Text style={[theme.typography.label, styles.colTime, { color: theme.colors.muted }]}>Device time</Text>
                    <Text style={[theme.typography.label, styles.colRecord, { color: theme.colors.muted }]}>Record</Text>
                    <Text style={[theme.typography.label, styles.colCommanded, { color: theme.colors.muted }]}>Commanded / device</Text>
                    <Text style={[theme.typography.label, styles.colMeasured, { color: theme.colors.muted }]}>Physical measurement</Text>
                  </View>
                ) : null}
                <View style={styles.sampleList}>
                  {report.samples.map((sample) => (
                    <SampleRow key={sample.seq} sample={sample} wide={wide} />
                  ))}
                </View>
              </View>
            )}
          </View>
        )}
      </Panel>
    </View>
  );
}

export default function BenchScreen() {
  const { width, height } = useWindowDimensions();
  const { inset, maxWidth } = consoleFrame(width, height);
  const [state, setState] = useState<BenchViewState>(EMPTY_BENCH_STATE);
  const importSeq = useRef(0);

  const handleFileChange = (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.currentTarget.files?.[0];
    event.currentTarget.value = '';
    if (!file) return;

    const seq = ++importSeq.current;
    void importBenchTraceFile(file).then((result) => {
      if (seq !== importSeq.current) return;
      setState((previous) => reduceBenchImport(previous, result));
    });
  };

  return (
    <ScrollView contentContainerStyle={[styles.page, { padding: inset, maxWidth }]} testID="bench-page">
      <BenchTraceView onFileChange={handleFileChange} state={state} wide={width >= 760} />
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { alignSelf: 'center', width: '100%' },
  stack: { gap: 12 },
  uploadGroup: { alignItems: 'flex-start', gap: 8, width: '100%' },
  banner: { borderWidth: 1, gap: 4, paddingHorizontal: 12, paddingVertical: 10 },
  reportBody: { gap: 12 },
  metaGrid: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  metaCell: { borderWidth: 1, gap: 2, minWidth: 112, paddingHorizontal: 10, paddingVertical: 8 },
  tabular: { fontVariant: ['tabular-nums'] },
  samplesSection: { gap: 6 },
  tableHeader: {
    borderBottomWidth: 1,
    flexDirection: 'row',
    gap: 12,
    paddingBottom: 6,
    paddingHorizontal: 10,
  },
  sampleList: { gap: 6 },
  sampleRow: { borderWidth: 1, paddingHorizontal: 10, paddingVertical: 8 },
  sampleRowWide: { alignItems: 'flex-start', flexDirection: 'row', gap: 12 },
  sampleRowStack: { gap: 6 },
  colSeq: { width: 48 },
  colTime: { width: 116 },
  colRecord: { width: 132 },
  colCommanded: { flex: 1.4, minWidth: 0 },
  colMeasured: { flex: 1, minWidth: 0 },
  fieldPair: { alignItems: 'center', flexDirection: 'row', justifyContent: 'space-between', gap: 8 },
  fieldBlock: { gap: 2 },
  recordMeta: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 6 },
  cuePair: { alignItems: 'center', flexDirection: 'row', gap: 4 },
  detailWrap: { flexDirection: 'row', flexWrap: 'wrap', gap: 10 },
  detailGroup: { alignItems: 'baseline', flexDirection: 'row', gap: 4 },
  measurementCell: { alignItems: 'baseline', flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
});
