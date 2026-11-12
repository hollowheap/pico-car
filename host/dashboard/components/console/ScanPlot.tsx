import Svg, { Circle, G, Line, Path, Polygon } from 'react-native-svg';

import { polarToXY, rangeRings, type ScanPoint } from '../../lib/scan';
import { useTheme } from '../../lib/theme-provider';

type Props = {
  points: ScanPoint[];
  /** Sonar servo angle in degrees, positive left. null hides the beam. */
  servoDeg: number | null;
  /** Half of the beam width in degrees (OBSTACLE_BEAM_HALF_DECI / 10). */
  beamHalfDeg: number;
  /** The range where the car stops for an obstacle, mm. */
  detectMm: number;
  maxMm?: number;
};

const WIDTH = 300;
const HEIGHT = 190;
const CAR_X = WIDTH / 2;
const CAR_Y = HEIGHT - 14;
const RING_STEP_MM = 100;
const FAN_DEG = 90;
const SPOKES = [-60, -30, 0, 30, 60];

/**
 * Half-disc scan plot with the car at the bottom. Rings every 100 mm. A filled
 * dot is an echo, a ring on the edge is open space, a cross on the edge is a bad
 * echo (no echo, glitch, stuck, too close). The wedge is the live sonar beam.
 */
export function ScanPlot({ points, servoDeg, beamHalfDeg, detectMm, maxMm = 600 }: Props) {
  const theme = useTheme();
  const mmPerUnit = maxMm / (CAR_Y - 20);
  const at = (angle: number, mm: number) => polarToXY(angle, mm, mmPerUnit, CAR_X, CAR_Y);
  const arc = (mm: number) => {
    const a = at(-FAN_DEG, mm);
    const b = at(FAN_DEG, mm);
    const r = mm / mmPerUnit;
    return `M ${a.x} ${a.y} A ${r} ${r} 0 0 0 ${b.x} ${b.y}`;
  };
  const beam =
    servoDeg === null
      ? null
      : [at(servoDeg - beamHalfDeg, maxMm), at(servoDeg + beamHalfDeg, maxMm)];

  return (
    <Svg
      accessibilityLabel={`Scan plot, ${points.filter((p) => p.mark === 'echo').length} echoes`}
      accessibilityRole="image"
      height={HEIGHT * 1.2}
      preserveAspectRatio="xMidYMid meet"
      viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
      width="100%"
    >
      {rangeRings(maxMm, RING_STEP_MM).map((mm) => (
        <G key={mm}>
          <Path d={arc(mm)} fill="none" stroke={theme.colors.border} strokeWidth="1" />
        </G>
      ))}
      {SPOKES.map((angle) => {
        const end = at(angle, maxMm);
        return <Line key={angle} stroke={theme.colors.border} strokeWidth="1" x1={CAR_X} x2={end.x} y1={CAR_Y} y2={end.y} />;
      })}
      <Path d={arc(detectMm)} fill="none" stroke={theme.colors.muted} strokeDasharray="4 3" strokeWidth="1" />
      {beam === null ? null : (
        <Polygon
          fill={theme.colors.accent}
          opacity={0.28}
          points={`${CAR_X},${CAR_Y} ${beam[0]?.x},${beam[0]?.y} ${beam[1]?.x},${beam[1]?.y}`}
        />
      )}
      {points.map((point) => {
        if (point.mark === 'none') return null;
        if (point.mark === 'echo') {
          const p = at(point.angleDeg, Math.min(point.distMm, maxMm));
          return <Circle key={point.slot} cx={p.x} cy={p.y} fill={theme.colors.text} r="3" />;
        }
        const rim = at(point.angleDeg, maxMm + 18);
        if (point.mark === 'open') {
          return <Circle key={point.slot} cx={rim.x} cy={rim.y} fill="none" r="2.5" stroke={theme.colors.muted} strokeWidth="1" />;
        }
        return (
          <G key={point.slot}>
            <Line stroke={theme.colors.danger} strokeWidth="1.5" x1={rim.x - 2.5} x2={rim.x + 2.5} y1={rim.y - 2.5} y2={rim.y + 2.5} />
            <Line stroke={theme.colors.danger} strokeWidth="1.5" x1={rim.x - 2.5} x2={rim.x + 2.5} y1={rim.y + 2.5} y2={rim.y - 2.5} />
          </G>
        );
      })}
      <Polygon fill={theme.colors.text} points={`${CAR_X},${CAR_Y - 9} ${CAR_X - 7},${CAR_Y + 7} ${CAR_X + 7},${CAR_Y + 7}`} />
    </Svg>
  );
}
