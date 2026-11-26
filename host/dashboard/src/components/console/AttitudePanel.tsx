import { useId, useState } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';
import Svg, { Circle, Defs, G, Line, LinearGradient, Path, Rect, Stop } from 'react-native-svg';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import type { RunSummary } from '../../../../bridge/src/console/run-summary.js';
import { enumLabel, headingDegrees } from '../../lib/schema';
import { BIT_FIELDS, ENUMS } from '../../lib/schema.generated';
import type { PitchSample } from '../../../../bridge/src/console/telemetry-store.js';
import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';
import { DecodedFrameFields } from './FrameLogPanel';
import { bearingName } from './format';
import { Compass } from './Compass';
import { LevelGauge } from './LevelGauge';
import { EventCounts } from './EventCounts';
import { MetricCell } from './MetricCell';
import { Panel, useDense } from './Panel';
import { Sparkline } from './Sparkline';
import { StatusPill } from './StatusPill';

type Props = {
  summary?: RunSummary;
  /** Status packet: the heading and the mode of the car. */
  statusPacket: LinkPacket | undefined;
  physics: LinkPacket | undefined;
  pitchHistory?: readonly PitchSample[];
  recorded?: { packet: LinkPacket; returnToLive: () => void };
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** The tilt that counts as a lift, as in app_program/imu/attitude.c. */
const TILT_ENVELOPE_DEG = 20;
const CDEG = 100;
const COMPACT_COMPASS_SIZE = 112;
const COMPACT_LEVEL_HEIGHT = 96;

const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));
/** Size an instrument to the space its row was given; fall back to the minimum until measured. */
const fitTo = (rowHeight: number, spare: number, min: number, max: number) =>
  rowHeight > 0 ? clamp(rowHeight - spare, min, max) : min;

function GForceReticle({ aFwdMg = 0, yawRateDps = 0, azMg, size = 92 }: { aFwdMg?: number; yawRateDps?: number; azMg?: number; size?: number }) {
  const theme = useTheme();
  const cx = 46;
  const cy = 46;
  const r = 38;
  // Scale: ±500 mg forward accel vertically, ±120 deg/s lateral yaw horizontally.
  const nx = Math.max(-1, Math.min(1, -yawRateDps / 120));
  const ny = Math.max(-1, Math.min(1, -aFwdMg / 500));
  const mag = Math.min(1, Math.hypot(nx, ny));
  const scale = mag > 1 ? 1 / mag : 1;
  const dotX = cx + nx * scale * (r - 4);
  const dotY = cy + ny * scale * (r - 4);
  const dotColor = mag >= 0.8 ? theme.colors.danger : mag >= 0.45 ? theme.colors.warning : theme.colors.success;

  return (
    <View style={[styles.reticleBox, { width: size + 6 }]} testID="physics-dynamics-chart">
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>G-FORCE</Text>
      <Svg accessibilityRole="image" accessibilityLabel="Forward acceleration and turn-rate G-force reticle"
        width={size} height={size} viewBox="0 0 92 92">
        <Circle cx={cx} cy={cy} r={r} fill={theme.colors.elevated} opacity={0.45} />
        <Circle cx={cx} cy={cy} r={r} fill="none" stroke={theme.colors.border} strokeWidth={1.2} />
        <Circle cx={cx} cy={cy} r={r * 0.62} fill="none" stroke={theme.colors.border} strokeWidth={1} strokeDasharray="2 3" />
        <Circle cx={cx} cy={cy} r={r * 0.28} fill="none" stroke={theme.colors.border} strokeWidth={1} />
        <Line x1={cx - r} y1={cy} x2={cx + r} y2={cy} stroke={theme.colors.border} strokeWidth={1} />
        <Line x1={cx} y1={cy - r} x2={cx} y2={cy + r} stroke={theme.colors.border} strokeWidth={1} />
        <Circle cx={cx} cy={cy} r={1.8} fill={theme.colors.muted} />
        <Line x1={cx} y1={cy} x2={dotX} y2={dotY} stroke={dotColor} strokeWidth={2.2} opacity={0.75} strokeLinecap="round" />
        <Circle cx={dotX} cy={dotY} r={8.5} fill={dotColor} opacity={0.22} />
        <Circle cx={dotX} cy={dotY} r={4.5} fill={dotColor} stroke={theme.colors.card} strokeWidth={1.2} />
      </Svg>
      <Text style={[theme.typography.caption, styles.tabular, { color: theme.colors.muted }]}>
        Az {azMg === undefined ? '--' : `${(azMg / 1000).toFixed(2)}g`}
      </Text>
    </View>
  );
}

function HumpTerrainViz({ hump, pitchDeg, currentPeak, runPeak, heightLabel, svgHeight = 68 }: {
  hump?: number;
  pitchDeg: number | null;
  currentPeak?: number;
  runPeak: number;
  heightLabel: string;
  svgHeight?: number;
}) {
  const theme = useTheme();
  const gradId = 'ramp-fill-' + useId().replace(/[^a-zA-Z0-9_-]/g, '');
  const livePitch = pitchDeg ?? 0;
  const clampedPitch = Math.max(-28, Math.min(28, livePitch));
  const active = hump !== undefined && hump !== ENUMS.humpState.flat;
  // Scale ramp height dynamically from live pitch or measured peak (10..30 mm over 100 mm rise/fall).
  const inferredMm = Math.abs(clampedPitch) >= 2 ? Math.tan((Math.abs(clampedPitch) * Math.PI) / 180) * 100 : 0;
  const peakMm = Math.max(10, Math.min(30, currentPeak && currentPeak > 0 ? currentPeak : inferredMm > 10 ? inferredMm : runPeak > 0 ? runPeak : 20));
  const peakPx = peakMm * 0.34;
  const groundY = 56;
  const crestY = groundY - peakPx;
  const carPos = hump === ENUMS.humpState.climb
    ? { x: 58, y: groundY - peakPx * 0.52 }
    : hump === ENUMS.humpState.crest
      ? { x: 75, y: crestY }
      : hump === ENUMS.humpState.descend
        ? { x: 92, y: groundY - peakPx * 0.52 }
        : { x: 34, y: groundY };
  // Use the actual live IMU pitch angle to tilt the rover (positive pitch = nose up).
  const carRot = pitchDeg !== null ? -clampedPitch : hump === ENUMS.humpState.climb ? -14 : hump === ENUMS.humpState.descend ? 14 : 0;
  const stroke = active || Math.abs(livePitch) >= 4 ? theme.colors.warning : theme.colors.info;
  // Live tangent slope line beneath the rover matching the actual pitch angle.
  const rad = (carRot * Math.PI) / 180;
  const tx = Math.cos(rad) * 26;
  const ty = Math.sin(rad) * 26;

  return (
    <View style={styles.humpBox}>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>RAMP & GRADE</Text>
      <Svg accessibilityRole="image" accessibilityLabel="Dynamic course ramp and live rover grade angle"
        width="100%" height={svgHeight} viewBox="0 0 150 64">
        <Defs>
          <LinearGradient id={gradId} x1="0%" y1="0%" x2="0%" y2="100%">
            <Stop offset="0%" stopColor={stroke} stopOpacity={active ? '0.32' : '0.18'} />
            <Stop offset="100%" stopColor={stroke} stopOpacity="0.03" />
          </LinearGradient>
        </Defs>
        {/* Ground floor baseline */}
        <Line x1={4} y1={groundY} x2={146} y2={groundY} stroke={theme.colors.border} strokeWidth={1} />
        {/* Symmetric 100 mm rise + 100 mm fall ramp */}
        <Path d={`M 45 ${groundY} L 75 ${crestY.toFixed(1)} L 105 ${groundY} Z`} fill={`url(#${gradId})`} />
        <Line x1={75} y1={crestY} x2={75} y2={groundY} stroke={theme.colors.border} strokeDasharray="2 2" strokeWidth={0.9} />
        <Path d={`M 6 ${groundY} L 45 ${groundY} L 75 ${crestY.toFixed(1)} L 105 ${groundY} L 144 ${groundY}`}
          fill="none" stroke={stroke} strokeWidth={1.1} strokeLinejoin="round" strokeLinecap="round" />
        {/* Live pitch grade tangent line under the rover */}
        {pitchDeg !== null && Math.abs(clampedPitch) >= 1 ? (
          <Line
            x1={(carPos.x - tx).toFixed(1)}
            y1={(carPos.y - ty).toFixed(1)}
            x2={(carPos.x + tx).toFixed(1)}
            y2={(carPos.y + ty).toFixed(1)}
            stroke={theme.colors.accent}
            strokeWidth={1.4}
            strokeDasharray="3 2"
            opacity={0.85}
          />
        ) : null}
        <Circle cx={75} cy={crestY} r={3.5} fill={stroke} opacity={0.22} />
        <Circle cx={75} cy={crestY} r={1.8} fill={stroke} />
        {/* True-scale 2WD rover scaled up 1.22x for legibility while preserving chassis.h proportions */}
        <G transform={`translate(${carPos.x} ${carPos.y}) rotate(${carRot.toFixed(1)}) scale(1.22) translate(${-carPos.x} ${-carPos.y})`}>
          <Rect x={carPos.x + 17} y={carPos.y - 6.5} width={5} height={2.4} rx={0.8} fill={theme.colors.warning} />
          <Rect x={carPos.x - 19} y={carPos.y - 10} width={36} height={3.2} rx={1}
            fill={active ? theme.colors.warning : theme.colors.muted} opacity={0.85} />
          <Rect x={carPos.x - 16} y={carPos.y - 19} width={24} height={3.2} rx={1}
            fill={active ? theme.colors.warning : theme.colors.success} opacity={0.85} />
          <Line x1={carPos.x - 13} y1={carPos.y - 10} x2={carPos.x - 13} y2={carPos.y - 16} stroke={theme.colors.border} strokeWidth={0.9} />
          <Line x1={carPos.x + 5} y1={carPos.y - 10} x2={carPos.x + 5} y2={carPos.y - 16} stroke={theme.colors.border} strokeWidth={0.9} />
          <Rect x={carPos.x + 7} y={carPos.y - 21} width={5.5} height={6} rx={1} fill={theme.colors.info} opacity={0.9} />
          {/* Front caster */}
          <Circle cx={carPos.x + 14} cy={carPos.y - 2.8} r={2.8} fill={theme.colors.muted} opacity={0.9} />
          {/* Rear 65 mm drive wheel */}
          <Circle cx={carPos.x - 9} cy={carPos.y - 7.5} r={7.5} fill={theme.colors.text} opacity={0.92} />
          <Circle cx={carPos.x - 9} cy={carPos.y - 7.5} r={4.2} fill={theme.colors.card} />
          <Circle cx={carPos.x - 9} cy={carPos.y - 7.5} r={2} fill={theme.colors.info} />
        </G>
      </Svg>
      <View style={styles.humpMetrics}>
        <MetricCell label="Grade" value={signed(pitchDeg, 1)} tone={Math.abs(livePitch) >= 4 ? 'warning' : 'default'} />
        <MetricCell label="Hump height" value={heightLabel} tone={heightLabel === 'Invalid' ? 'warning' : 'default'}
          style={[styles.colDivider, { borderLeftColor: theme.colors.border }]} />
      </View>
    </View>
  );
}

const signed = (value: number | null, digits = 1) =>
  value === null ? '--' : `${value >= 0 ? '+' : ''}${value.toFixed(digits)}°`;

/**
 * Heading compass, pitch and roll gauges, and the lift state. The heading is
 * the magnetometer heading and it is not valid until the IMU has data.
 */
export function AttitudePanel({ statusPacket, physics, pitchHistory, status, style, recorded, summary }: Props) {
  const theme = useTheme();
  const heading = headingDegrees(physics?.values.heading_ddeg ?? statusPacket?.values.heading_ddeg);
  const calibrated = ((physics?.values.flags ?? 0) & (1 << BIT_FIELDS.physicsFlags.calibrated)) !== 0;
  const pitch = calibrated && physics?.values.pitch_cdeg !== undefined ? physics.values.pitch_cdeg / CDEG : null;
  const roll = calibrated && physics?.values.roll_cdeg !== undefined ? physics.values.roll_cdeg / CDEG : null;
  const yawRate = physics?.values.yaw_rate_cdps === undefined ? 0 : physics.values.yaw_rate_cdps / CDEG;
  const attitude = calibrated ? physics?.values.attitude_state : undefined;
  const hump = physics?.values.hump_state;
  const motion = physics?.values.motion_class;
  const runPeak = Math.max(physics?.values.run_peak_mm ?? 0, summary?.tallestHumpMm ?? 0);
  // The firmware saturates its seven-bit run peak at 127 mm.
  const peakLabel = runPeak === 127 ? 'peak ≥127 mm' : `peak ${runPeak} mm`;
  const repeatsSlope = (hump === ENUMS.humpState.climb && motion === ENUMS.motionClass.climbing) ||
    (hump === ENUMS.humpState.descend && motion === ENUMS.motionClass.descending);
  const alert = attitude === 2 || attitude === 3;
  const dense = useDense();
  const [rowHeight, setRowHeight] = useState(0);
  const compassSize = fitTo(rowHeight, 24, COMPACT_COMPASS_SIZE, 200);
  const levelHeight = fitTo(rowHeight, 34, COMPACT_LEVEL_HEIGHT, 190);

  if (recorded !== undefined) {
    return (
      <Panel icon="attitude" title="Physics" style={style} testID="attitude-panel"
        right={<Button compact variant="secondary" onPress={recorded.returnToLive}><Button.Text>Return to live</Button.Text></Button>}>
        <DecodedFrameFields packet={recorded.packet} recordedPhysics />
      </Panel>
    );
  }

  const pv = physics?.values;

  return (
    <Panel
      fill={dense}
      empty={physics === undefined}
      right={
        <StatusPill
          label={attitude === undefined ? 'Unknown' : enumLabel('attitudeState', attitude)}
          tone={attitude === undefined ? 'muted' : alert ? 'danger' : attitude === 1 ? 'warning' : 'success'}
        />
      }
      icon="attitude"
      footer={summary && <EventCounts testID="physics-events" scope={summary.scope} counts={[[summary.humps, 'hump'], [summary.lifts, 'lift']]}
        detail={runPeak > 0 ? peakLabel : undefined} />}
      status={status}
      style={style}
      testID="attitude-panel"
      title="Physics"
    >
      <View onLayout={(e) => setRowHeight(Math.round(e.nativeEvent.layout.height))}
        style={dense ? styles.instrumentRow : styles.instruments}>
        <View style={[styles.compass, dense ? { width: compassSize } : null]}>
          <Compass available={heading !== null} headingDeg={heading ?? 0} size={dense ? compassSize : undefined} yawRateDps={yawRate} />
          <Text style={[theme.typography.caption, styles.tabular, { color: heading === null ? theme.colors.muted : theme.colors.text }]}>
            {heading === null ? 'Heading --' : `${heading.toFixed(0)}° ${bearingName(heading)}`}
          </Text>
        </View>
        <View style={dense ? [styles.compactLevel, styles.colDivider, { borderLeftColor: theme.colors.border }] : null}>
          <LevelGauge envelopeDeg={TILT_ENVELOPE_DEG} pitchDeg={pitch} rollDeg={roll}
            plotHeight={dense ? levelHeight : undefined} />
        </View>
      </View>
      <View style={styles.metaRow}>
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
          {motion === undefined || repeatsSlope || (alert && motion === ENUMS.motionClass.lifted) ? 'IMU --' : `IMU ${enumLabel('motionClass', motion).toLowerCase()}`}
        </Text>
        <Text style={[theme.typography.caption, styles.tabular, { color: theme.colors.text }]}>
          Turn {signed(pv?.yaw_rate_cdps === undefined ? null : yawRate, 0)}/s
        </Text>
      </View>
      {pitchHistory !== undefined && pitchHistory.length >= 2 ? (
        <Sparkline
          color={theme.colors.info}
          height={28}
          label="Pitch history"
          range={[-25, 25]}
          values={pitchHistory.map((sample) => sample.pitchDeg)}
        />
      ) : null}
    </Panel>
  );
}

/** Dedicated card for live G-force reticle and pitch-driven ramp grade profile. */
export function DynamicsPanel({ physics, status, style, summary }: Pick<Props, 'physics' | 'status' | 'style' | 'summary'>) {
  const theme = useTheme();
  const pv = physics?.values;
  const calibrated = ((pv?.flags ?? 0) & (1 << BIT_FIELDS.physicsFlags.calibrated)) !== 0;
  const pitch = calibrated && pv?.pitch_cdeg !== undefined ? pv.pitch_cdeg / CDEG : null;
  const yawRate = pv?.yaw_rate_cdps === undefined ? 0 : pv.yaw_rate_cdps / CDEG;
  const hump = pv?.hump_state;
  const lastHump = summary?.lastHump;
  const sampledAt = physics?.timestamp ?? -Infinity;
  const active = hump !== undefined && hump !== ENUMS.humpState.flat;
  const measuring = active && (lastHump === undefined ||
    (lastHump.state === 'measuring' && sampledAt >= lastHump.timestamp));
  const heightValid = pv?.flags === undefined || (pv.flags & (1 << BIT_FIELDS.physicsFlags.heightValid)) !== 0;
  const currentPeak = measuring && heightValid ? pv?.hump_peak_mm : undefined;
  const runPeak = Math.max(pv?.run_peak_mm ?? 0, summary?.tallestHumpMm ?? 0);
  let heightLabel = 'Unmeasured';
  if (measuring) {
    heightLabel = !heightValid ? 'Invalid' : (currentPeak ?? 0) > 0 ? `${currentPeak} mm` : 'Measuring';
  } else if (lastHump) {
    heightLabel = lastHump.state === 'measured' ? `${lastHump.mm} mm`
      : lastHump.state === 'invalid' ? 'Invalid' : 'Measuring';
  } else if (runPeak > 0) {
    heightLabel = runPeak === 127 ? '≥127 mm' : `${runPeak} mm`;
  }
  const [rowHeight, setRowHeight] = useState(0);
  const reticleSize = fitTo(rowHeight, 36, 92, 180);
  const rampHeight = fitTo(rowHeight, 58, 68, 170);

  return (
    <Panel
      fill
      empty={physics === undefined}
      icon="dynamics"
      right={
        <StatusPill
          label={hump === undefined ? 'No data' : enumLabel('humpState', hump)}
          tone={hump === undefined ? 'muted' : active ? 'warning' : 'success'}
        />
      }
      status={status}
      style={style}
      testID="dynamics-panel"
      title="Dynamics"
    >
      <View onLayout={(e) => setRowHeight(Math.round(e.nativeEvent.layout.height))} style={styles.dynamicsRow}>
        <GForceReticle aFwdMg={pv?.a_fwd_mg} yawRateDps={yawRate} azMg={pv?.az_mg} size={reticleSize} />
        <View style={[styles.humpWrap, styles.colDivider, { borderLeftColor: theme.colors.border }]}>
          <HumpTerrainViz hump={hump} pitchDeg={pitch} currentPeak={currentPeak} runPeak={runPeak}
            heightLabel={heightLabel} svgHeight={rampHeight} />
        </View>
      </View>
    </Panel>
  );
}

const styles = StyleSheet.create({
  headerRight: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  instruments: { gap: 10 },
  instrumentRow: { flex: 1, minHeight: 0, alignItems: 'center', flexDirection: 'row', gap: 12, justifyContent: 'space-between' },
  compass: { alignItems: 'center', gap: 4 },
  compactLevel: { flex: 1, minWidth: 0 },
  colDivider: { borderLeftWidth: 1, paddingLeft: 10 },
  metaRow: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 8 },
  tabular: { fontVariant: ['tabular-nums'] },
  dynamicsRow: { flex: 1, minHeight: 0, flexDirection: 'row', alignItems: 'center', gap: 10 },
  foldSection: { borderTopWidth: 1, paddingTop: 8 },
  reticleBox: { alignItems: 'center', gap: 2 },
  humpWrap: { flex: 1, minWidth: 0 },
  humpBox: { gap: 2 },
  humpMetrics: { flexDirection: 'row', gap: 8 },
});
