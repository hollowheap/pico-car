import { StyleSheet, Text, View } from 'react-native';
import Svg, { G, Line, Path, Rect } from 'react-native-svg';

import { useTheme } from '../../lib/theme-provider';

type Props = {
  /** Pitch in degrees, positive nose up. */
  pitchDeg: number | null;
  /** Roll in degrees, positive left side down. */
  rollDeg: number | null;
  /** The tilt that counts as a lift. Drawn as two ticks on each gauge. */
  envelopeDeg: number;
  compact?: boolean;
};

const WIDTH = 120;
const HEIGHT = 70;
const GROUND = 56;
const PIVOT_X = 60;

type ViewProps = { angle: number | null; envelopeDeg: number; label: string; rear: boolean; compact: boolean };

/** One gauge: the car tilts about its wheels on a ground line. */
function Gauge({ angle, envelopeDeg, label, rear, compact }: ViewProps) {
  const theme = useTheme();
  const tilt = angle ?? 0;
  const pivotY = GROUND;
  const mark = (deg: number) => {
    const r = 46;
    const rad = (deg * Math.PI) / 180;
    return { x: PIVOT_X + r * Math.sin(rad), y: pivotY - r * Math.cos(rad) };
  };
  const edge = (sign: number) => {
    const inner = mark(sign * envelopeDeg);
    const outer = { x: PIVOT_X + 54 * Math.sin((sign * envelopeDeg * Math.PI) / 180), y: pivotY - 54 * Math.cos((sign * envelopeDeg * Math.PI) / 180) };
    return { inner, outer };
  };
  const left = edge(-1);
  const right = edge(1);
  return (
    <View style={styles.gauge}>
      <Text style={[theme.typography.label, { color: theme.colors.muted }]}>{label}</Text>
      <Svg
      accessibilityLabel={angle === null ? `${label} unavailable` : `${label} ${angle.toFixed(1)} degrees`}
      accessibilityRole="image"
      height={compact ? 48 : HEIGHT * 1.6}
      preserveAspectRatio="xMidYMid meet"
      viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
      width="100%"
    >
      <Line stroke={theme.colors.border} strokeWidth="1" x1="6" x2={WIDTH - 6} y1={GROUND} y2={GROUND} />
      <Line stroke={theme.colors.muted} strokeWidth="1" x1={left.inner.x} x2={left.outer.x} y1={left.inner.y} y2={left.outer.y} />
      <Line stroke={theme.colors.muted} strokeWidth="1" x1={right.inner.x} x2={right.outer.x} y1={right.inner.y} y2={right.outer.y} />
      <G opacity={angle === null ? 0.35 : 1} transform={`rotate(${-tilt} ${PIVOT_X} ${pivotY})`}>
        {rear ? (
          <>
            <Rect fill={theme.colors.card} height="22" rx="3" stroke={theme.colors.text} strokeWidth="1.5" width="36" x={PIVOT_X - 18} y={GROUND - 28} />
            <Rect fill={theme.colors.text} height="10" width="5" x={PIVOT_X - 26} y={GROUND - 10} />
            <Rect fill={theme.colors.text} height="10" width="5" x={PIVOT_X + 21} y={GROUND - 10} />
          </>
        ) : (
          <>
            <Path
              d={`M ${PIVOT_X - 30} ${GROUND - 8} L ${PIVOT_X - 30} ${GROUND - 20} L ${PIVOT_X - 12} ${GROUND - 20} L ${PIVOT_X - 6} ${GROUND - 30} L ${PIVOT_X + 12} ${GROUND - 30} L ${PIVOT_X + 18} ${GROUND - 20} L ${PIVOT_X + 32} ${GROUND - 20} L ${PIVOT_X + 32} ${GROUND - 8} Z`}
              fill={theme.colors.card}
              stroke={theme.colors.text}
              strokeLinejoin="round"
              strokeWidth="1.5"
            />
            <Rect fill={theme.colors.text} height="8" rx="2" width="12" x={PIVOT_X - 24} y={GROUND - 8} />
            <Rect fill={theme.colors.text} height="8" rx="2" width="12" x={PIVOT_X + 12} y={GROUND - 8} />
            <Path d={`M ${PIVOT_X + 32} ${GROUND - 14} l 6 0`} stroke={theme.colors.accent} strokeWidth="2" />
          </>
        )}
      </G>
      </Svg>
    </View>
  );
}

/** Pitch (side view) and roll (rear view) side by side. */
export function LevelGauge({ pitchDeg, rollDeg, envelopeDeg, compact = false }: Props) {
  return (
    <View style={styles.row}>
      <View style={styles.cell}>
        <Gauge compact={compact} angle={pitchDeg} envelopeDeg={envelopeDeg} label="PITCH" rear={false} />
      </View>
      <View style={styles.cell}>
        <Gauge compact={compact} angle={rollDeg} envelopeDeg={envelopeDeg} label="ROLL" rear />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', gap: 8 },
  cell: { flex: 1 },
  gauge: { alignSelf: 'center', gap: 4, maxWidth: 220, width: '100%' },
});
