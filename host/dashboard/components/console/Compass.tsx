import Svg, { Circle, G, Line, Path, Polygon } from 'react-native-svg';

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
          .filter((deg) => deg % 30 === 0)
          .map((deg) => {
            const at = onRing(deg, LABEL_R);
            const cardinal = CARDINALS[deg];
            return (
              <G key={`l${deg}`} transform={`rotate(${deg} ${at.x} ${at.y})`}>
                <SvgLabel
                  fill={cardinal ? theme.colors.text : theme.colors.muted}
                  fontSize={cardinal ? 15 : 11.5}
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
      <Polygon fill={theme.colors.text} points={`${CENTRE},${CENTRE - RING + 14} ${CENTRE - 5},${CENTRE - RING + 24} ${CENTRE + 5},${CENTRE - RING + 24}`} />
      {arc !== null ? <Path d={arc} fill="none" stroke={theme.colors.accent} strokeLinecap="round" strokeWidth="4" /> : null}
      <G>
        <Path
          d={`M ${CENTRE - 11} ${CENTRE + 18} L ${CENTRE - 11} ${CENTRE - 6} L ${CENTRE - 5} ${CENTRE - 18} L ${CENTRE + 5} ${CENTRE - 18} L ${CENTRE + 11} ${CENTRE - 6} L ${CENTRE + 11} ${CENTRE + 18} Z`}
          fill={theme.colors.card}
          stroke={theme.colors.text}
          strokeLinejoin="round"
          strokeWidth="1.5"
        />
        <Line stroke={theme.colors.text} strokeWidth="1.5" x1={CENTRE - 15} x2={CENTRE - 15} y1={CENTRE - 10} y2={CENTRE - 2} />
        <Line stroke={theme.colors.text} strokeWidth="1.5" x1={CENTRE + 15} x2={CENTRE + 15} y1={CENTRE - 10} y2={CENTRE - 2} />
        <Line stroke={theme.colors.text} strokeWidth="1.5" x1={CENTRE - 15} x2={CENTRE - 15} y1={CENTRE + 6} y2={CENTRE + 14} />
        <Line stroke={theme.colors.text} strokeWidth="1.5" x1={CENTRE + 15} x2={CENTRE + 15} y1={CENTRE + 6} y2={CENTRE + 14} />
      </G>
    </Svg>
  );
}
