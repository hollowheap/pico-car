import { useId } from 'react';
import { StyleSheet, Text, View } from 'react-native';
import Svg, { Circle, Defs, G, Line, Path, RadialGradient, Stop } from 'react-native-svg';

import { polarToXY, rangeRings, SCAN_MAX_MM, type ScanPoint } from '../../lib/scan';
import { enumLabel } from '../../lib/schema';
import { ENUMS } from '../../lib/schema.generated';
import { useTheme } from '../../lib/theme-provider';
import { SvgLabel } from './SvgLabel';

const WIDTH = 324;
const HEIGHT = 174;
const CX = WIDTH / 2;
const CY = 150;
const RADIUS = 124;
// OBSTACLE_BEAM_HALF_DECI, OBSTACLE_DETECT_MM and the bounded pilot's COURSE_STOP_MM.
const BEAM_HALF_DEG = 9.5;
const LEGACY_DETECT_MM = 250;
const PILOT_STOP_MM = 100;

export type SonarBearing = Pick<ScanPoint, 'angleDeg' | 'echoClass' | 'distMm'> & { slot: number | 'live' };
type Sweep = { points: SonarBearing[]; status?: number; movingBypass: boolean };

function hasRange(p: SonarBearing) {
  return p.echoClass === ENUMS.echoClass.valid && Number.isFinite(p.distMm) && p.distMm >= 20;
}

export function resultName(p: SonarBearing) {
  if (hasRange(p)) return `${p.distMm} mm`;
  return p.echoClass === ENUMS.echoClass.valid ? 'range unavailable' : enumLabel('echoClass', p.echoClass).toLowerCase();
}

export const bearingName = (angle: number) => angle === 0 ? 'Ahead' : `${Math.abs(angle)}° ${angle > 0 ? 'left' : 'right'}`;

function guides({ points, status, movingBypass }: Sweep) {
  const ranges = points.filter(hasRange);
  const outcomes = points.filter(p => !hasRange(p));
  const maxMm = Math.ceil(Math.max(SCAN_MAX_MM, ...ranges.map(p => p.distMm)) / SCAN_MAX_MM) * SCAN_MAX_MM;
  const legacy = status !== undefined && status >= ENUMS.profileStatus.ok && status <= ENUMS.profileStatus.failed;
  return {
    ranges, outcomes, maxMm,
    showDetect: status !== ENUMS.profileStatus.pilotSparse || movingBypass,
    showStop: !legacy && !movingBypass,
  };
}

export function SonarLegend(sweep: Sweep) {
  const { colors, typography } = useTheme();
  const { outcomes, maxMm, showDetect, showStop } = guides(sweep);
  const caption = [typography.caption, { color: colors.muted }];
  return <View style={styles.keys} testID="sonar-legend">
    <Text style={[caption, styles.tabular]}>{maxMm} mm · {maxMm / 6} mm/ring</Text>
    {showStop ? <View style={styles.key}><View style={[styles.sample, { borderColor: colors.muted }]} />
      <Text style={caption}>Stop {PILOT_STOP_MM} mm</Text></View> : null}
    {showDetect ? <View style={styles.key}><View style={[styles.sample, { borderColor: colors.muted, borderStyle: 'dashed' }]} />
      <Text style={caption}>{sweep.movingBypass ? 'Contact' : 'Detect'} {LEGACY_DETECT_MM} mm</Text></View> : null}
    {outcomes.some(p => p.echoClass === ENUMS.echoClass.open) ? <Text style={caption}>Outer arcs open</Text> : null}
    {outcomes.some(p => p.echoClass !== ENUMS.echoClass.open) ? <Text style={caption}>Outer ticks unusable</Text> : null}
  </View>;
}

/** Rays and echoes use sensor range. A sweep's bearings are relative to its starting heading. */
function arc(radius: number, from = -90, to = 90, cx = CX, cy = CY) {
  const a = polarToXY(from, radius, 1, cx, cy);
  const b = polarToXY(to, radius, 1, cx, cy);
  return `M ${a.x} ${a.y} A ${radius} ${radius} 0 0 0 ${b.x} ${b.y}`;
}

function wedge(radius: number, fromDeg: number, toDeg: number): string {
  const a = polarToXY(fromDeg, radius, 1, CX, CY);
  const b = polarToXY(toDeg, radius, 1, CX, CY);
  const large = Math.abs(toDeg - fromDeg) > 180 ? 1 : 0;
  return `M ${CX} ${CY} L ${a.x.toFixed(1)} ${a.y.toFixed(1)} A ${radius} ${radius} 0 ${large} 1 ${b.x.toFixed(1)} ${b.y.toFixed(1)} Z`;
}

function LiveAim({ angle, inset }: { angle: number; inset: boolean }) {
  const { colors } = useTheme();
  const cx = inset ? 61 : CX;
  const cy = inset ? 44 : CY;
  const r = inset ? 20 : RADIUS;
  const label = `Live aim ${bearingName(angle)}, current car frame`;
  return <G testID="sonar-live-frame" aria-label={label}>
    {inset ? <>
      <SvgLabel x={cx} y={14} fontSize={12} textAnchor="middle" fill={colors.text}>Live aim</SvgLabel>
      <Path d={`${arc(r, -90, 90, cx, cy)} Z`} fill="none" stroke={colors.muted} />
      <Line x1={cx} x2={cx} y1={cy} y2={cy - r} stroke={colors.border} />
      <SvgLabel x={cx} y={58} fontSize={12} textAnchor="middle" fill={colors.muted}>{bearingName(angle)}</SvgLabel>
    </> : null}
    <Path d={`${arc(r, angle - BEAM_HALF_DEG, angle + BEAM_HALF_DEG, cx, cy)} L ${cx} ${cy} Z`}
      fill={colors.accent} fillOpacity={0.15} stroke={colors.accent} strokeOpacity={0.8}
      strokeWidth={inset ? 1.3 : 1} testID="sonar-live-beam" />
  </G>;
}

export function SonarPlot({ recorded, aim, ...sweep }: Sweep & { recorded: boolean; aim?: number }) {
  const { colors, typography } = useTheme();
  const fanId = 'sonar-fan-' + useId().replace(/[^a-zA-Z0-9_-]/g, '');
  const { ranges, outcomes, maxMm, showDetect, showStop } = guides(sweep);
  const step = maxMm / 6;
  const mmPerUnit = maxMm / RADIUS;
  const frame = recorded ? 'Recorded sweep' : 'Current sonar';
  const caption = [typography.caption, { color: colors.muted }];
  return <View style={styles.plot}>
    <Text style={[typography.label, styles.forward, { color: colors.text }]}>Ahead</Text>
    <View style={styles.chart}><Svg testID="obstacle-sonar" accessibilityRole="image"
      accessibilityLabel={`${frame}, ${ranges.length} measured ranges. Forward is zero degrees. Rings every ${step} millimetres.`}
      style={StyleSheet.absoluteFill} width="100%" height="100%" preserveAspectRatio="xMidYMid meet" viewBox="24 16 276 144">
      <Defs>
        <RadialGradient id={fanId} cx="50%" cy="100%" r="100%">
          <Stop offset="0%" stopColor={colors.accent} stopOpacity="0.16" />
          <Stop offset="65%" stopColor={colors.info} stopOpacity="0.06" />
          <Stop offset="100%" stopColor={colors.info} stopOpacity="0.01" />
        </RadialGradient>
      </Defs>
      <Path d={wedge(RADIUS, -90, 90)} fill={`url(#${fanId})`} />
      <G testID="sonar-fan-reference">
        {rangeRings(maxMm, step).map(mm => <Path key={mm} testID={`sonar-ring-${mm}`}
          d={arc(mm / mmPerUnit)} stroke={mm === maxMm ? colors.muted : colors.border} fill="none" />)}
        <Line x1={CX - RADIUS} x2={CX + RADIUS} y1={CY} y2={CY} stroke={colors.muted} />
        {[-60, -30, 0, 30, 60].map(angle => {
          const end = polarToXY(angle, RADIUS, 1, CX, CY);
          return <Line key={angle} x1={CX} y1={CY} x2={end.x} y2={end.y} stroke={colors.border}
            testID={angle === 0 ? 'sonar-forward-reference' : `sonar-spoke-${angle}`} />;
        })}
      </G>
      {showDetect ? <Path testID="sonar-detect-250" d={arc(LEGACY_DETECT_MM / mmPerUnit)}
        fill="none" stroke={colors.muted} strokeWidth={1.4} strokeDasharray="4 3" /> : null}
      {showStop ? <Path testID="sonar-detect-100" d={arc(PILOT_STOP_MM / mmPerUnit, -BEAM_HALF_DEG, BEAM_HALF_DEG)}
        fill="none" stroke={colors.muted} strokeWidth={2.5} /> : null}
      {aim !== undefined ? <LiveAim angle={aim} inset={recorded} /> : null}
      {ranges.map(({ slot: key, angleDeg: angle, distMm: mm }) => {
        const start = polarToXY(angle, Math.max(mm / mmPerUnit - 4, 0), 1, CX, CY);
        const end = polarToXY(angle, mm, mmPerUnit, CX, CY);
        return (
          <G key={key}>
            <Line testID={`sonar-range-${key}`} aria-label={`${bearingName(angle)}, ${mm} mm`}
              x1={start.x} y1={start.y} x2={end.x} y2={end.y} stroke={colors.accent} strokeWidth={3} strokeLinecap="round" />
            <Circle cx={end.x} cy={end.y} r={6} fill={colors.accent} opacity={0.22} />
            <Circle cx={end.x} cy={end.y} r={2.8} fill={colors.accent} />
          </G>
        );
      })}
      {outcomes.map(p => {
        const label = `${bearingName(p.angleDeg)}, ${resultName(p)}`;
        const start = polarToXY(p.angleDeg, RADIUS + 3, 1, CX, CY);
        const end = polarToXY(p.angleDeg, RADIUS + 8, 1, CX, CY);
        return <Path key={p.slot} testID={`sonar-outcome-${p.slot}`} aria-label={label} fill="none" strokeWidth={2}
          stroke={p.echoClass === ENUMS.echoClass.open ? colors.muted : colors.danger}
          d={p.echoClass === ENUMS.echoClass.open ? arc(RADIUS + 4, p.angleDeg - 1.4, p.angleDeg + 1.4) : `M ${start.x} ${start.y} L ${end.x} ${end.y}`} />;
      })}
      <Path d={`M${CX} ${CY - 5} l-4 7 h8 Z`} fill={colors.text} />
    </Svg></View>
    <View style={styles.bottomLabels} pointerEvents="none">
      <Text style={caption}>90° L</Text>
      <Text style={caption}>90° R</Text>
    </View>
  </View>;
}

const styles = StyleSheet.create({
  plot: { flex: 1, gap: 2, minHeight: 0, minWidth: 0 },
  forward: { alignSelf: 'center' },
  bottomLabels: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' },
  tabular: { fontVariant: ['tabular-nums'] },
  chart: { flex: 1, minHeight: 0, position: 'relative', width: '100%', minWidth: 0 },
  keys: { flexDirection: 'row', flexWrap: 'wrap', columnGap: 10, rowGap: 2 },
  key: { flexDirection: 'row', alignItems: 'center', gap: 4 },
  sample: { width: 12, borderTopWidth: 2 },
});
