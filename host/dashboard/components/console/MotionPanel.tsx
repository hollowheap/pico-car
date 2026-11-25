import { useRef, useState } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import Svg, { Circle, G, Line, Path, Polygon, Rect } from 'react-native-svg';
import { steeringFraction, wheelPlotSeries, WHEEL_WINDOW_MS, type SpeedPoint, type WheelPlotSeries, type WheelSample } from '../../lib/wheel-history';
import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import type { RunSummary } from '../../lib/run-summary';
import { enumLabel } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import { timeToX } from '../../lib/time-plot';
import { usePlotNow } from '../../lib/useNow';
import { withUnit } from './format';
import { MetricCell } from './MetricCell';
import { Panel, useExpanded } from './Panel';
import { SectionPicker } from './SectionPicker';
import { StatusPill } from './StatusPill';
import { EventCounts } from './EventCounts';

type Props = {
  summary?: RunSummary;
  motion: LinkPacket | undefined;
  history: WheelSample[];
  nowMs: number;
  clockOffsetMs?: number;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

const STRAIGHT_BAND_MMS = 10;
const WIDTH = 140;
const HEIGHT = 42;
const LIMIT_STEP_MMS = 50;
const LIMIT_MIN_MMS = 100;
const VIEWS = { live: 'Live', history: 'History' } as const;
const STEER_X = 70;
const STEER_SPAN = 32;
const STEER_EDGE_Y = 14;
const STEER_CONTROL_Y = 3;
const STEER_CENTER_Y = (STEER_EDGE_Y + STEER_CONTROL_Y) / 2;

function ChassisDriveViz({ leftMms, rightMms, leftTarget, rightTarget, leftDuty = 0, rightDuty = 0, slip }: {
  leftMms?: number;
  rightMms?: number;
  leftTarget?: number;
  rightTarget?: number;
  leftDuty?: number;
  rightDuty?: number;
  slip: number | null;
}) {
  const theme = useTheme();
  const lSpeed = leftMms ?? 0;
  const rSpeed = rightMms ?? 0;
  const arrow = (cx: number, duty: number) => {
    const frac = Math.max(-1, Math.min(1, duty / 1000));
    const mag = Math.abs(frac);
    const len = mag * 22;
    const color = mag >= 0.9 ? theme.colors.danger : mag >= 0.75 ? theme.colors.warning : theme.colors.info;
    const cy = 36;
    const tipY = frac >= 0 ? cy - len : cy + len;
    if (len < 2) return null;
    const dir = frac >= 0 ? -1 : 1;
    return (
      <G>
        <Line x1={cx} y1={cy} x2={cx} y2={tipY} stroke={color} strokeWidth={3} strokeLinecap="round" />
        <Polygon
          fill={color}
          points={`${cx},${tipY + dir * 4} ${cx - 4},${tipY - dir * 2} ${cx + 4},${tipY - dir * 2}`}
        />
      </G>
    );
  };
  const steer = steeringFraction(leftMms, rightMms);
  const steerX = STEER_X + steer * STEER_SPAN;
  const steerY = STEER_CENTER_Y + (STEER_EDGE_Y - STEER_CENTER_Y) * steer ** 2;
  const steerColor = Math.abs(steer) > 0.15 ? theme.colors.warning : theme.colors.success;
  const slipWarn = slip !== null && slip >= 20;
  const fmtWheel = (speed?: number, target?: number, duty?: number) =>
    `${speed ?? '--'}/${target ?? '--'} · ${duty === undefined ? '--' : `${Math.round(duty / 10)}%`}`;

  return (
    <View style={styles.chassisRow} testID="motion-effort-chart">
      <View style={styles.chassisSvgWrap}>
        <Svg accessibilityRole="image" accessibilityLabel="Top-down chassis drive vectors and differential steering"
          style={StyleSheet.absoluteFill} width="100%" height="100%" preserveAspectRatio="xMidYMid meet" viewBox="8 2 124 70">
          {/* Steering arc ahead of nose */}
          <Path d={`M ${STEER_X - STEER_SPAN} ${STEER_EDGE_Y} Q ${STEER_X} ${STEER_CONTROL_Y} ${STEER_X + STEER_SPAN} ${STEER_EDGE_Y}`}
            fill="none" stroke={theme.colors.border} strokeWidth={1.2} strokeDasharray="3 3" />
          <Path
            d={`M ${STEER_X} 19 Q ${STEER_X} 10 ${steerX} ${steerY}`}
            fill="none"
            stroke={steerColor}
            strokeWidth={2.4}
            strokeLinecap="round"
          />
          <Circle cx={steerX} cy={steerY} r={5} fill={steerColor} opacity={0.22} />
          <Circle cx={steerX} cy={steerY} r={2.6} fill={steerColor} />

          {/* Thrust vector guide tracks */}
          <Line x1={22} y1={14} x2={22} y2={58} stroke={theme.colors.border} strokeWidth={1.2} strokeDasharray="2 3" opacity={0.7} />
          <Line x1={118} y1={14} x2={118} y2={58} stroke={theme.colors.border} strokeWidth={1.2} strokeDasharray="2 3" opacity={0.7} />

          {/* Measured 2WD rover top-down silhouette (0.24 px/mm: rear axle y=52, tail -50 mm -> y=64, nose +110 mm -> y=25.6, IR +130 mm -> y=20.8) */}
          <Rect x={64.6} y={20.8} width={10.8} height={4.8} rx={1.2} fill={theme.colors.warning} opacity={0.9} />
          <Circle cx={67.5} cy={23.2} r={1} fill={theme.colors.card} />
          <Circle cx={72.5} cy={23.2} r={1} fill={theme.colors.card} />
          {/* Asymmetric left barcode outrigger (+45 mm ahead of rear axle -> y=41.2, 107 mm left -> x=44.3) */}
          <Rect x={44.5} y={40.4} width={15} height={1.8} rx={0.8} fill={theme.colors.warning} opacity={0.85} />
          <Rect x={43} y={39.1} width={4.6} height={4.4} rx={1} fill={theme.colors.warning} />
          {/* Rear drive axle */}
          <Line x1={53} y1={52} x2={87} y2={52} stroke={theme.colors.muted} strokeWidth={1.2} opacity={0.6} />
          {/* Filled chassis deck: long front (+110 mm), short rear (-50 mm) */}
          <Path
            d="M 59.2 25.6 L 80.8 25.6 L 82 43 L 78.5 44.5 L 78.5 59.5 L 81 61 L 78.5 64 L 61.5 64 L 59 61 L 61.5 59.5 L 61.5 44.5 L 58 43 Z"
            fill={theme.colors.elevated}
            stroke={slipWarn ? theme.colors.warning : theme.colors.border}
            strokeWidth={0.8}
            strokeLinejoin="round"
          />
          {/* Front caster under nose */}
          <Circle cx={70} cy={28.5} r={2} fill={theme.colors.muted} opacity={0.65} />
          {/* HC-SR04 sonar at +70 mm ahead of rear axle (y=35.2) */}
          <Rect x={62.5} y={33} width={15} height={4.4} rx={1.2} fill={theme.colors.info} opacity={0.9} />
          <Circle cx={66.2} cy={35.2} r={1.4} fill={theme.colors.card} />
          <Circle cx={73.8} cy={35.2} r={1.4} fill={theme.colors.card} />
          {/* Pico W + IMU deck near rear */}
          <Rect x={63.5} y={44} width={13} height={16} rx={2} fill={theme.colors.success} opacity={0.78} />
          <Circle cx={66.8} cy={47.2} r={1.1} fill={theme.colors.card} />

          {/* Rear 65x19 mm drive wheels (15.6 x 4.6 px) */}
          <Rect x={53.8} y={44.2} width={4.8} height={15.6} rx={1.5} fill={theme.colors.info} opacity={0.9} />
          <Rect x={81.4} y={44.2} width={4.8} height={15.6} rx={1.5} fill={theme.colors.text} opacity={0.9} />

          {/* Thrust vectors */}
          {arrow(22, leftDuty)}
          {arrow(118, rightDuty)}
        </Svg>
      </View>
      <View style={styles.effortGrid}>
        <View style={styles.effortPair}>
          <MetricCell label="Left (mm/s)" value={fmtWheel(leftMms, leftTarget, leftDuty)} tone="info" />
          <MetricCell label="Right (mm/s)" value={fmtWheel(rightMms, rightTarget, rightDuty)}
            style={[styles.colDivider, { borderLeftColor: theme.colors.border }]} />
        </View>
        <View style={styles.effortPair}>
          <MetricCell label="Slip" value={slip === null ? '--' : `${slip} mm/s`} tone={slipWarn ? 'warning' : 'success'} />
          <MetricCell label="Delta" value={`${lSpeed - rSpeed >= 0 ? '+' : ''}${lSpeed - rSpeed} mm/s`}
            style={[styles.colDivider, { borderLeftColor: theme.colors.border }]} />
        </View>
      </View>
    </View>
  );
}

function pathForRuns(runs: SpeedPoint[][], x: (at: number) => number, y: (value: number) => number): string {
  return runs.map((run) => run.map((p, k) =>
    `${k === 0 ? 'M' : 'L'}${x(p.at).toFixed(1)} ${y(p.value).toFixed(1)}${run.length === 1 ? 'h0.5' : ''}`).join(' ')).join(' ');
}

function WheelChart({ series, name, limit, now }: {
  series: WheelPlotSeries; name: string; limit: number; now: number;
}) {
  const theme = useTheme();
  const x = (at: number) => timeToX(at, now, WIDTH, WHEEL_WINDOW_MS);
  const y = (value: number) => HEIGHT / 2 - (HEIGHT / 2 - 3) * Math.max(-1, Math.min(1, value / limit));
  const caption = [theme.typography.caption, { color: theme.colors.muted }];
  const targetPath = pathForRuns(series.targets, x, y);
  const estimatedPath = series.estimated
    .map(([a, b]) => `M${x(a.at).toFixed(1)} ${y(a.value).toFixed(1)} L${x(b.at).toFixed(1)} ${y(b.value).toFixed(1)}`)
    .join(' ');
  const measuredPath = pathForRuns(series.measured, x, y);

  return (
    <View>
      <View style={styles.chart}>
        <View style={styles.scale}><Text style={caption}>+{limit}</Text><Text style={caption}>0</Text><Text style={caption}>−{limit}</Text></View>
        <View style={styles.plot}>
          <Svg accessibilityRole="image" accessibilityLabel={`Measured ${name.toLowerCase()} wheel speed in mm/s, last ten seconds`}
            width="100%" height={HEIGHT} viewBox={`0 0 ${WIDTH} ${HEIGHT}`} preserveAspectRatio="none" testID={`wheel-speed-${name.toLowerCase()}`}>
            <Path d={`M0 3 H${WIDTH} M0 ${HEIGHT / 2} H${WIDTH} M0 ${HEIGHT - 3} H${WIDTH}`} stroke={theme.colors.border} fill="none" />
            {targetPath ? <Path d={targetPath} fill="none" stroke={theme.colors.muted} strokeWidth={1.2}
              strokeDasharray="2 3" opacity={0.65} /> : null}
            {estimatedPath ? <Path d={estimatedPath} fill="none" stroke={theme.colors.accent} strokeWidth={1.6}
              strokeDasharray="3 3" strokeLinecap="round" opacity={0.55} /> : null}
            {measuredPath ? <Path d={measuredPath} fill="none" stroke={theme.colors.accent} strokeWidth={2}
              strokeLinecap="round" /> : null}
          </Svg>
          <View style={styles.traceHead}><Text style={caption}>−10 s</Text><Text style={caption}>Now</Text></View>
        </View>
      </View>
    </View>
  );
}

function DutyBar({ duty }: { duty?: number }) {
  const theme = useTheme();
  const fraction = duty === undefined || !Number.isFinite(duty) ? 0 : Math.max(-1, Math.min(1, duty / 1000));
  const widthPct = `${(Math.abs(fraction) * 50).toFixed(1)}%` as const;
  const leftPct = fraction >= 0 ? '50%' : `${(50 - Math.abs(fraction) * 50).toFixed(1)}%`;
  const abs = Math.abs(fraction);
  const fillColor = abs >= 0.9 ? theme.colors.danger : abs >= 0.75 ? theme.colors.warning : theme.colors.info;
  return (
    <View style={[styles.dutyTrack, { backgroundColor: theme.colors.border }]}>
      <View style={[styles.dutyCenter, { backgroundColor: theme.colors.muted }]} />
      {duty !== undefined && abs > 0.01 ? (
        <View style={[styles.dutyFill, {
          left: leftPct as `${number}%`,
          width: widthPct,
          backgroundColor: fillColor,
        }]} />
      ) : null}
    </View>
  );
}

type WheelProps = { name: string; speed?: number; target?: number; duty?: number; divided?: boolean; children: React.ReactNode };
function Wheel({ name, speed, target, duty, divided = false, children }: WheelProps) {
  const theme = useTheme();
  return <View style={[styles.wheel, divided && [styles.colDivider, { borderLeftColor: theme.colors.border }]]}>
    <View style={styles.wheelHead}>
      <Text style={[theme.typography.label, { color: name === 'Left' ? theme.colors.accent : theme.colors.text }]}>
        {name}
      </Text>
      <Text numberOfLines={1} style={[theme.typography.caption, styles.value, { color: theme.colors.text }]}>
        {speed ?? '--'}/{withUnit(target, 'mm/s')} · {duty === undefined ? '--' : Math.round(duty / 10) + '%'}
      </Text>
    </View>
    <DutyBar duty={duty} />
    {children}
  </View>;
}

/**
 * Left and right wheel speed against the target, the duty the controller
 * applied, and the speed trace. A speed difference on a straight run is slip.
 */
export function MotionPanel({ motion, history, nowMs, clockOffsetMs, status, style, summary }: Props) {
  const theme = useTheme();
  const expanded = useExpanded();
  const [view, setView] = useState<keyof typeof VIEWS>('live');
  const plotNow = usePlotNow(nowMs, status?.kind === 'ready', clockOffsetMs);

  const series = (['left', 'right'] as const).map(side => wheelPlotSeries(history, plotNow, side));
  const rawPeak = Math.max(
    LIMIT_MIN_MMS,
    ...series.flatMap((s) => [
      ...s.measured.flat(),
      ...s.targets.flat(),
      ...s.estimated.flat(),
    ]).map(p => Math.ceil(Math.abs(p.value) / LIMIT_STEP_MMS) * LIMIT_STEP_MMS),
  );
  // Hold the vertical scale steady unless the peak grows or stays well below the ceiling.
  const heldLimit = useRef(rawPeak);
  if (rawPeak > heldLimit.current || rawPeak <= heldLimit.current - LIMIT_STEP_MMS * 2) {
    heldLimit.current = rawPeak;
  }
  const limit = heldLimit.current;
  const v = motion?.values;
  const straight =
    v?.left_target_mms !== undefined && v.right_target_mms !== undefined &&
    Math.abs(v.left_target_mms - v.right_target_mms) <= STRAIGHT_BAND_MMS && Math.abs(v.left_target_mms) > STRAIGHT_BAND_MMS;
  const slip = straight && v?.left_mms !== undefined && v.right_mms !== undefined ? Math.abs(v.left_mms - v.right_mms) : null;
  const fault = v === undefined ? undefined : enumLabel('motionFault', v.motion_fault);
  const faulty = Boolean(v?.motion_fault);
  const coasting = v?.motion_state === 0 && v.left_target_mms === 0 && v.right_target_mms === 0 &&
    (Boolean(v.left_mms) || Boolean(v.right_mms));
  const label = v === undefined ? 'No data' : coasting ? 'Zero target' : enumLabel('motionState', v.motion_state);

  return (
    <Panel
      fill
      empty={motion === undefined}
      right={
        <View style={styles.headerRight}>
          <StatusPill
            label={faulty ? `${label}, ${fault}` : label}
            tone={v === undefined ? 'muted' : faulty ? 'danger' : v.motion_state === 1 ? 'success' : v.motion_state === 2 ? 'info' : 'neutral'}
          />
          {expanded ? null : <SectionPicker id="motion-view" label="Motion views" options={VIEWS} value={view} onChange={setView} />}
        </View>
      }
      icon="motion"
      footer={
        <View style={styles.footerRow}>
          {summary ? <EventCounts testID="motion-events" scope={summary.scope} counts={[[summary.turns, 'turn'], [summary.stops, 'stop']]} /> : <View />}
          {expanded || view === 'history' ? (
            <Text numberOfLines={1} style={[theme.typography.caption, { color: theme.colors.muted }]}>
              Solid: meas · Dashed: target/est
            </Text>
          ) : null}
        </View>
      }
      status={status}
      style={style}
      testID="motion-panel"
      title="Motion"
    >
      {expanded || view === 'live' ? (
        <ChassisDriveViz
          leftMms={v?.left_mms}
          rightMms={v?.right_mms}
          leftTarget={v?.left_target_mms}
          rightTarget={v?.right_target_mms}
          leftDuty={v?.left_duty_pm}
          rightDuty={v?.right_duty_pm}
          slip={slip}
        />
      ) : null}
      {expanded || view === 'history' ? (
        <View style={[styles.wheels, expanded && [styles.foldSection, { borderTopColor: theme.colors.border }]]}>
          <Wheel duty={v?.left_duty_pm} name="Left" speed={v?.left_mms} target={v?.left_target_mms}>
            <WheelChart name="Left" series={series[0]!} limit={limit} now={plotNow} />
          </Wheel>
          <Wheel divided duty={v?.right_duty_pm} name="Right" speed={v?.right_mms} target={v?.right_target_mms}>
            <WheelChart name="Right" series={series[1]!} limit={limit} now={plotNow} />
          </Wheel>
        </View>
      ) : null}
    </Panel>
  );
}

const styles = StyleSheet.create({
  headerRight: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  wheels: { flexDirection: 'row', gap: 10 },
  wheel: { flex: 1, minWidth: 0, gap: 2 },
  wheelHead: { flexDirection: 'row', alignItems: 'baseline', justifyContent: 'space-between', gap: 4 },
  colDivider: { borderLeftWidth: 1, paddingLeft: 10 },
  footerRow: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 8 },
  chassisRow: { flex: 1, flexDirection: 'row', alignItems: 'center', gap: 10, minHeight: 0 },
  chassisSvgWrap: { width: '42%', minWidth: 110, maxWidth: 260, alignSelf: 'stretch', position: 'relative', minHeight: 68 },
  foldSection: { borderTopWidth: 1, paddingTop: 8 },
  effortGrid: { flex: 1, minWidth: 0, gap: 8, alignSelf: 'stretch', justifyContent: 'space-evenly' },
  effortPair: { flexDirection: 'row', gap: 8 },
  chart: { flexDirection: 'row', gap: 6 },
  scale: { height: HEIGHT, justifyContent: 'space-between', alignItems: 'flex-end' },
  plot: { flex: 1, minWidth: 0 },
  value: { fontVariant: ['tabular-nums'] },
  traceHead: { alignItems: 'center', flexDirection: 'row', justifyContent: 'space-between' },
  dutyTrack: { height: 4, borderRadius: 2, overflow: 'hidden', position: 'relative' },
  dutyCenter: { position: 'absolute', left: '50%', top: 0, bottom: 0, width: 1, opacity: 0.6 },
  dutyFill: { position: 'absolute', top: 0, bottom: 0, borderRadius: 2 },
});
