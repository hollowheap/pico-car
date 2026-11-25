import { StyleSheet, Text, View } from 'react-native';
import Svg, { Circle, G, Line, Path, Rect } from 'react-native-svg';

import { useTheme } from '../../lib/theme-provider';

type Props = {
  /** Pitch in degrees, positive nose up. */
  pitchDeg: number | null;
  /** Roll in degrees, positive left side down. */
  rollDeg: number | null;
  /** The tilt that counts as a lift. Drawn as two ticks on each gauge. */
  envelopeDeg: number;
  plotHeight?: number;
};

const WIDTH = 120;
const HEIGHT = 92;
const GROUND = 74;
const PIVOT_X = 60;
const ARC_R = 54;
const ARC_MAX_DEG = 40;

type ViewProps = { angle: number | null; envelopeDeg: number; label: string; rear: boolean; plotHeight: number };

function polar(deg: number, r: number) {
  const rad = (deg * Math.PI) / 180;
  return { x: PIVOT_X + r * Math.sin(rad), y: GROUND - r * Math.cos(rad) };
}

function arcPath(fromDeg: number, toDeg: number, r: number): string {
  const a = polar(fromDeg, r);
  const b = polar(toDeg, r);
  return `M ${a.x.toFixed(1)} ${a.y.toFixed(1)} A ${r} ${r} 0 0 1 ${b.x.toFixed(1)} ${b.y.toFixed(1)}`;
}

/** One gauge: the car tilts about its wheels inside a calibrated inclinometer arc. */
function Gauge({ angle, envelopeDeg, label, rear, plotHeight }: ViewProps) {
  const theme = useTheme();
  const tilt = angle ?? 0;
  const clamped = Math.max(-ARC_MAX_DEG, Math.min(ARC_MAX_DEG, tilt));
  const exceeded = angle !== null && Math.abs(angle) >= envelopeDeg;
  const tone = exceeded ? theme.colors.danger : angle !== null && Math.abs(angle) >= envelopeDeg / 2
    ? theme.colors.warning : theme.colors.success;
  const pointerBase = polar(-clamped, ARC_R - 7);
  const pointerTip = polar(-clamped, ARC_R + 1);

  return (
    <View style={styles.gauge}>
      <Text style={[theme.typography.label, styles.label, { color: theme.colors.muted }]}>{label}</Text>
      <Svg
        accessibilityLabel={angle === null ? `${label} unavailable` : `${label} ${angle.toFixed(1)} degrees`}
        accessibilityRole="image"
        height={plotHeight}
        preserveAspectRatio="xMidYMid meet"
        viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
        width="100%"
      >
        {/* Subtle inclinometer scale arc and 0° plumb reference */}
        <Path d={arcPath(-ARC_MAX_DEG, ARC_MAX_DEG, ARC_R)} fill="none" stroke={theme.colors.border} strokeWidth="1.2" />
        <Path
          d={arcPath(-envelopeDeg, envelopeDeg, ARC_R)}
          fill="none"
          stroke={exceeded ? theme.colors.danger : theme.colors.muted}
          strokeWidth="1.8"
          opacity={0.7}
        />
        <Line
          stroke={theme.colors.border}
          strokeDasharray="2 3"
          strokeWidth="1"
          x1={PIVOT_X}
          x2={PIVOT_X}
          y1={GROUND - ARC_R}
          y2={GROUND}
        />

        {/* Tick marks at ±envelopeDeg and 0° */}
        {[-envelopeDeg, 0, envelopeDeg].map((deg) => {
          const inner = polar(deg, ARC_R - (deg === 0 ? 5 : 4));
          const outer = polar(deg, ARC_R + (deg === 0 ? 3 : 5));
          return (
            <Line
              key={deg}
              stroke={deg === 0 ? theme.colors.muted : exceeded ? theme.colors.danger : theme.colors.muted}
              strokeWidth={deg === 0 ? '1.2' : '1.6'}
              x1={inner.x}
              x2={outer.x}
              y1={inner.y}
              y2={outer.y}
            />
          );
        })}

        {/* Ground horizon */}
        <Rect fill={theme.colors.border} height="12" opacity={0.18} rx="2" width={WIDTH - 12} x="6" y={GROUND} />
        <Line stroke={theme.colors.muted} strokeWidth="1.2" x1="6" x2={WIDTH - 6} y1={GROUND} y2={GROUND} opacity={0.65} />

        {/* Live tilt pointer on the arc */}
        {angle !== null ? (
          <Line
            stroke={tone}
            strokeLinecap="round"
            strokeWidth="2.4"
            x1={pointerBase.x}
            x2={pointerTip.x}
            y1={pointerBase.y}
            y2={pointerTip.y}
          />
        ) : null}

        {/* Tilted 2WD Pico W rover silhouette scaled up 1.28x about ground pivot while keeping exact chassis.h proportions */}
        <G opacity={angle === null ? 0.35 : 1} transform={`translate(${PIVOT_X} ${GROUND}) rotate(${-tilt}) scale(1.28) translate(${-PIVOT_X} ${-GROUND})`}>
          {rear ? (
            <>
              {/* Rear view: filled solid shapes with hairline accents */}
              <Rect fill={theme.colors.warning} height="2" rx="1" width="12" x={PIVOT_X - 27} y={GROUND - 8} opacity={0.85} />
              <Rect fill={theme.colors.warning} height="3.5" rx="1" width="4" x={PIVOT_X - 28} y={GROUND - 6.5} />
              <Line stroke={theme.colors.muted} strokeWidth="1" x1={PIVOT_X - 13} x2={PIVOT_X + 13} y1={GROUND - 8.1} y2={GROUND - 8.1} />
              {/* Lower acrylic deck */}
              <Rect fill={exceeded ? theme.colors.danger : theme.colors.muted} height="3.8" rx="1.2" opacity={0.85}
                width="22" x={PIVOT_X - 11} y={GROUND - 11.5} />
              {/* Standoffs */}
              <Line stroke={theme.colors.border} strokeWidth="1" x1={PIVOT_X - 7.5} x2={PIVOT_X - 7.5} y1={GROUND - 11.5} y2={GROUND - 19} />
              <Line stroke={theme.colors.border} strokeWidth="1" x1={PIVOT_X + 7.5} x2={PIVOT_X + 7.5} y1={GROUND - 11.5} y2={GROUND - 19} />
              {/* Upper Pico W + IMU deck at 80 mm */}
              <Rect fill={exceeded ? theme.colors.danger : theme.colors.success} height="3.6" rx="1.2" opacity={0.85}
                width="20" x={PIVOT_X - 10} y={GROUND - 21.5} />
              <Circle cx={PIVOT_X} cy={GROUND - 19.7} r="1.2" fill={theme.colors.card} />
              {/* HC-SR04 twin ultrasonic eyes */}
              <Rect fill={theme.colors.info} height="6.5" rx="1.6" opacity={0.9}
                width="14" x={PIVOT_X - 7} y={GROUND - 28.5} />
              <Circle cx={PIVOT_X - 3.3} cy={GROUND - 25.2} r="2" fill={theme.colors.card} />
              <Circle cx={PIVOT_X + 3.3} cy={GROUND - 25.2} r="2" fill={theme.colors.card} />
              {/* Left & Right 65x19 mm drive wheels */}
              <Rect fill={theme.colors.text} height="16.2" rx="1.8" width="4.8" x={PIVOT_X - 15.8} y={GROUND - 16.2} opacity={0.9} />
              <Rect fill={theme.colors.text} height="16.2" rx="1.8" width="4.8" x={PIVOT_X + 11} y={GROUND - 16.2} opacity={0.9} />
            </>
          ) : (
            <>
              {/* Side view (facing right): rear drive axle (tail -50 mm), front deck (+110 mm), front caster (+105 mm), sonar (+70 mm), IR tip (+130 mm) */}
              {/* Under-nose IR extension (+110..+130 mm) */}
              <Rect fill={theme.colors.warning} height="2.8" rx="0.8" width="5.5" x={PIVOT_X + 20} y={GROUND - 7.5} />
              <Line stroke={theme.colors.warning} strokeDasharray="1.5 1.5" strokeWidth="0.9" opacity={0.75}
                x1={PIVOT_X + 23.5} x2={PIVOT_X + 23.5} y1={GROUND - 4.7} y2={GROUND - 0.5} />
              {/* Lower chassis deck (-50..+110 mm = 40 px) */}
              <Rect fill={exceeded ? theme.colors.danger : theme.colors.muted} height="3.8" rx="1.2" opacity={0.85}
                width="42" x={PIVOT_X - 21} y={GROUND - 11.5} />
              {/* Standoffs */}
              <Line stroke={theme.colors.border} strokeWidth="1" x1={PIVOT_X - 15} x2={PIVOT_X - 15} y1={GROUND - 11.5} y2={GROUND - 19} />
              <Line stroke={theme.colors.border} strokeWidth="1" x1={PIVOT_X + 8} x2={PIVOT_X + 8} y1={GROUND - 11.5} y2={GROUND - 19} />
              {/* Upper Pico W / IMU deck at 80 mm */}
              <Rect fill={exceeded ? theme.colors.danger : theme.colors.success} height="3.6" rx="1.2" opacity={0.85}
                width="30" x={PIVOT_X - 18} y={GROUND - 21.5} />
              <Circle cx={PIVOT_X - 12} cy={GROUND - 19.7} r="1.2" fill={theme.colors.card} />
              {/* Sonar turret at +70 mm ahead of rear axle */}
              <Rect fill={theme.colors.muted} height="5" rx="0.8" width="3.8" x={PIVOT_X + 9} y={GROUND - 16.5} opacity={0.8} />
              <Rect fill={theme.colors.info} height="7" rx="1.5" width="6.5" x={PIVOT_X + 8} y={GROUND - 23.5} opacity={0.92} />
              <Line stroke={tone} strokeLinecap="round" strokeWidth="1.2" x1={PIVOT_X + 15} x2={PIVOT_X + 22} y1={GROUND - 20} y2={GROUND - 20} />
              {/* Front ball caster under nose */}
              <Circle cx={PIVOT_X + 16} cy={GROUND - 3.2} fill={theme.colors.muted} r="3.2" opacity={0.9} />
              <Circle cx={PIVOT_X + 16} cy={GROUND - 3.2} fill={theme.colors.card} r="1.1" />
              {/* Rear 65 mm drive wheel (only ~50 mm overhang behind axle) */}
              <Circle cx={PIVOT_X - 10} cy={GROUND - 8.1} fill={theme.colors.text} r="8.1" opacity={0.92} />
              <Circle cx={PIVOT_X - 10} cy={GROUND - 8.1} fill={theme.colors.card} r="4.6" />
              <Circle cx={PIVOT_X - 10} cy={GROUND - 8.1} fill={theme.colors.info} r="2.2" />
            </>
          )}
        </G>
      </Svg>
      <Text style={[theme.typography.caption, styles.reading, { color: exceeded ? theme.colors.danger : theme.colors.text }]}>
        {angle === null ? '--' : `${angle >= 0 ? '+' : ''}${angle.toFixed(1)}°`}
      </Text>
    </View>
  );
}

/** Pitch (side view) and roll (rear view) side by side. */
export function LevelGauge({ pitchDeg, rollDeg, envelopeDeg, plotHeight = HEIGHT * 1.4 }: Props) {
  const theme = useTheme();
  return (
    <View style={styles.row}>
      <View style={styles.cell}>
        <Gauge angle={pitchDeg} envelopeDeg={envelopeDeg} label="PITCH" plotHeight={plotHeight} rear={false} />
      </View>
      <View style={[styles.cell, styles.divider, { borderLeftColor: theme.colors.border }]}>
        <Gauge angle={rollDeg} envelopeDeg={envelopeDeg} label="ROLL" plotHeight={plotHeight} rear />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', gap: 10 },
  cell: { flex: 1 },
  divider: { borderLeftWidth: 1, paddingLeft: 10 },
  gauge: { alignSelf: 'center', gap: 3, maxWidth: 220, width: '100%' },
  label: { textAlign: 'center' },
  reading: { textAlign: 'center', fontVariant: ['tabular-nums'] },
});
