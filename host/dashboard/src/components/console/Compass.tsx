import Svg, { Circle, G, Line, Path, Polygon, Rect } from 'react-native-svg';

import { SvgLabel } from './SvgLabel';

import { useTheme } from '../../lib/theme-provider';

type Props = {
  /** Compass heading in degrees, clockwise from north. */
  headingDeg: number;
  /** False while the heading is invalid: the card stops and dims. */
  available?: boolean;
  /** Turn rate in degrees per second, positive to the left. Draws the turn arc. */
  yawRateDps?: number;
  size?: number;
};

const CENTRE = 100;
const RING = 90;
const MINOR_LEN = 5;
const MAJOR_LEN = 10;
const LABEL_R = 70;
const ARC_R = 46;
/** The lubber mark sits between the turn-rate arc and the card letters, so a letter never covers it. */
const LUBBER_TIP_R = 60;
const LUBBER_BASE_R = 50;
const ARC_FULL_DPS = 90;
const ARC_MAX_DEG = 80;
const ARC_MIN_DPS = 3;
const CARDINALS: Record<number, string> = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };

const rad = (deg: number) => (deg * Math.PI) / 180;
const onRing = (deg: number, r: number) => ({
  x: CENTRE + r * Math.sin(rad(deg)),
  y: CENTRE - r * Math.cos(rad(deg)),
});

/** The turn-rate arc from 12 o'clock, to the left for a positive rate. */
function turnArc(yawRateDps: number): string | null {
  if (Math.abs(yawRateDps) < ARC_MIN_DPS) return null;
  const sweep = Math.min(ARC_MAX_DEG, (Math.abs(yawRateDps) / ARC_FULL_DPS) * ARC_MAX_DEG);
  const end = onRing(yawRateDps > 0 ? -sweep : sweep, ARC_R);
  const start = onRing(0, ARC_R);
  return `M ${start.x} ${start.y} A ${ARC_R} ${ARC_R} 0 0 ${yawRateDps > 0 ? 0 : 1} ${end.x} ${end.y}`;
}

/**
 * Heading-up compass card. The card turns under a fixed car, so the top of the
 * dial is always where the car points. The north mark is the only accent. The
 * arc beside the car shows the turn rate.
 */
export function Compass({ headingDeg, available = true, yawRateDps = 0, size = 250 }: Props) {
  const theme = useTheme();
  const heading = ((headingDeg % 360) + 360) % 360;
  const arc = available ? turnArc(yawRateDps) : null;
  const ticks = Array.from({ length: 36 }, (_, index) => index * 10);
  const compact = size < 200;

  return (
    <Svg
      accessibilityLabel={
        available ? `Heading ${heading.toFixed(0)} degrees` : 'Heading unavailable'
      }
      accessibilityRole="image"
      height={size}
      preserveAspectRatio="xMidYMid meet"
      viewBox="0 0 200 200"
      width="100%"
    >
      <Circle cx={CENTRE} cy={CENTRE} fill="none" r={RING} stroke={theme.colors.border} strokeWidth="1" />

      <G opacity={available ? 1 : 0.35} transform={`rotate(${available ? -heading : 0} ${CENTRE} ${CENTRE})`}>
        {ticks.map((deg) => {
          const major = deg % 30 === 0;
          const outer = onRing(deg, RING);
          const inner = onRing(deg, RING - (major ? MAJOR_LEN : MINOR_LEN));
          return (
            <Line
              key={deg}
              stroke={major ? theme.colors.text : theme.colors.muted}
              strokeWidth={major ? 1.5 : 1}
              x1={outer.x}
              x2={inner.x}
              y1={outer.y}
              y2={inner.y}
            />
          );
        })}
        {ticks
          .filter((deg) => deg % (compact ? 90 : 30) === 0)
          .map((deg) => {
            const at = onRing(deg, LABEL_R);
            const cardinal = CARDINALS[deg];
            return (
              <G key={`l${deg}`} transform={`rotate(${deg} ${at.x} ${at.y})`}>
                <SvgLabel
                  fill={cardinal ? theme.colors.text : theme.colors.muted}
                  fontSize={compact ? Math.ceil((12 * 200) / size) : cardinal ? 15 : 12}
                  fontWeight={cardinal ? '600' : '400'}
                  textAnchor="middle"
                  x={at.x}
                  y={at.y + (cardinal ? 5 : 3.5)}
                >
                  {cardinal ?? String(deg / 10)}
                </SvgLabel>
              </G>
            );
          })}
        <Polygon fill={theme.colors.accent} points={`${CENTRE},${CENTRE - RING - 8} ${CENTRE - 6},${CENTRE - RING + 2} ${CENTRE + 6},${CENTRE - RING + 2}`} />
      </G>

      {/* Fixed parts: the lubber mark at the top and the car. */}
      <Polygon fill={theme.colors.text} points={`${CENTRE},${CENTRE - LUBBER_TIP_R} ${CENTRE - 5},${CENTRE - LUBBER_BASE_R} ${CENTRE + 5},${CENTRE - LUBBER_BASE_R}`} />
      {arc !== null ? <Path d={arc} fill="none" stroke={theme.colors.accent} strokeLinecap="round" strokeWidth="4" /> : null}
      <G transform={`translate(${CENTRE} ${CENTRE}) scale(1.32) translate(${-CENTRE} ${-CENTRE})`}>
        <Rect x={CENTRE - 18.5} y={CENTRE - 1.5} width="11" height="1.6" rx="0.7" fill={theme.colors.warning} opacity={0.85} />
        <Rect x={CENTRE - 19.5} y={CENTRE - 2.4} width="3.6" height="3.2" rx="0.8" fill={theme.colors.warning} />
        <Rect x={CENTRE - 4} y={CENTRE - 18} width="8" height="3.4" rx="1" fill={theme.colors.warning} opacity={0.9} />
        <Path
          d={`M ${CENTRE - 8} ${CENTRE - 14.5} L ${CENTRE + 8} ${CENTRE - 14.5} L ${CENTRE + 9} ${CENTRE + 1} L ${CENTRE + 7} ${CENTRE + 2} L ${CENTRE + 7} ${CENTRE + 13} L ${CENTRE + 8.5} ${CENTRE + 14} L ${CENTRE + 7} ${CENTRE + 16.5} L ${CENTRE - 7} ${CENTRE + 16.5} L ${CENTRE - 8.5} ${CENTRE + 14} L ${CENTRE - 7} ${CENTRE + 13} L ${CENTRE - 7} ${CENTRE + 2} L ${CENTRE - 9} ${CENTRE + 1} Z`}
          fill={theme.colors.elevated}
          stroke={theme.colors.border}
          strokeLinejoin="round"
          strokeWidth="0.8"
        />
        <Rect x={CENTRE - 5.5} y={CENTRE - 8.5} width="11" height="3.2" rx="1" fill={theme.colors.info} opacity={0.9} />
        <Rect x={CENTRE - 4.8} y={CENTRE + 1} width="9.6" height="12" rx="1.5" fill={theme.colors.success} opacity={0.78} />
        {/* Rear 65x19 mm drive wheels */}
        <Rect x={CENTRE - 12} y={CENTRE + 2} width="3.8" height="11.5" rx="1.2" fill={theme.colors.text} opacity={0.9} />
        <Rect x={CENTRE + 8.2} y={CENTRE + 2} width="3.8" height="11.5" rx="1.2" fill={theme.colors.text} opacity={0.9} />
      </G>
    </Svg>
  );
}
