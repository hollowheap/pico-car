import { Text, View } from 'react-native';
import Svg, { Circle, G, Line, Polygon, Polyline, Rect } from 'react-native-svg';

import type { Course } from '../../lib/course';
import type { EventKind } from '../../lib/events';
import { useTheme } from '../../lib/theme-provider';

type Props = { course: Course };

const WIDTH = 360;
const HEIGHT = 300;
const PAD = 28;
/** The scale bar is the largest of these that fits in a quarter of the width (mm). */
const SCALE_STEPS_MM = [100, 200, 500, 1000, 2000, 5000, 10000];
/** The smallest span the plot fits, so a short run is not blown up (mm). */
const MIN_SPAN_MM = 600;

/** Legend text of each mark shape. */
export const MARK_LEGEND: { kind: EventKind; label: string }[] = [
  { kind: 'barcode', label: 'barcode' },
  { kind: 'turn', label: 'turn' },
  { kind: 'obstacle', label: 'obstacle' },
  { kind: 'hump', label: 'hump' },
  { kind: 'line', label: 'line lost or found' },
  { kind: 'attitude', label: 'lift or overturn' },
  { kind: 'fault', label: 'fault' },
];

function Mark({ kind, x, y, color }: { kind: EventKind; x: number; y: number; color: string }) {
  switch (kind) {
    case 'barcode':
      return <Rect fill={color} height="7" width="7" x={x - 3.5} y={y - 3.5} />;
    case 'turn':
      return <Polygon fill="none" points={`${x},${y - 5} ${x + 5},${y} ${x},${y + 5} ${x - 5},${y}`} stroke={color} strokeWidth="1.5" />;
    case 'obstacle':
      return <Polygon fill={color} points={`${x},${y - 5} ${x + 5},${y + 4} ${x - 5},${y + 4}`} />;
    case 'hump':
      return <Polygon fill={color} points={`${x},${y - 5} ${x + 5},${y} ${x},${y + 5} ${x - 5},${y}`} />;
    case 'line':
      return <Circle cx={x} cy={y} fill="none" r="4" stroke={color} strokeWidth="1.5" />;
    case 'attitude':
      return <Rect fill={color} height="3" width="11" x={x - 5.5} y={y - 1.5} />;
    default:
      return (
        <G>
          <Line stroke={color} strokeWidth="2" x1={x - 4} x2={x + 4} y1={y - 4} y2={y + 4} />
          <Line stroke={color} strokeWidth="2" x1={x - 4} x2={x + 4} y1={y + 4} y2={y - 4} />
        </G>
      );
  }
}

/**
 * The course the car drove, dead-reckoned from the odometer and the compass
 * heading. North is up. It is a trace, not a map: it drifts with the heading
 * error. Marks sit where the event's odometer puts them.
 */
export function CourseTrace({ course }: Props) {
  const theme = useTheme();
  const { bounds, points, markers } = course;
  const spanX = Math.max(MIN_SPAN_MM, bounds.maxX - bounds.minX);
  const spanY = Math.max(MIN_SPAN_MM, bounds.maxY - bounds.minY);
  const scale = Math.min((WIDTH - 2 * PAD) / spanX, (HEIGHT - 2 * PAD) / spanY);
  const originX = PAD + ((WIDTH - 2 * PAD) - (bounds.maxX - bounds.minX) * scale) / 2;
  const originY = PAD + ((HEIGHT - 2 * PAD) - (bounds.maxY - bounds.minY) * scale) / 2;
  const sx = (x: number) => originX + (x - bounds.minX) * scale;
  const sy = (y: number) => HEIGHT - originY - (y - bounds.minY) * scale;
  const last = points[points.length - 1];
  const bar = [...SCALE_STEPS_MM].reverse().find((mm) => mm * scale <= WIDTH / 4) ?? SCALE_STEPS_MM[0] ?? 100;

  return (
    <View style={{ gap: 4 }}>
    <Svg
      accessibilityLabel={`Course trace, ${points.length} points, ${markers.length} marks`}
      accessibilityRole="image"
      height={HEIGHT * 1.2}
      preserveAspectRatio="xMidYMid meet"
      viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
      width="100%"
    >
      <Rect fill="none" height={HEIGHT - 2} stroke={theme.colors.border} strokeWidth="1" width={WIDTH - 2} x="1" y="1" />
      {points.length >= 2 ? (
        <Polyline
          fill="none"
          points={points.map((p) => `${sx(p.x).toFixed(1)},${sy(p.y).toFixed(1)}`).join(' ')}
          stroke={theme.colors.muted}
          strokeLinejoin="round"
          strokeWidth="2"
        />
      ) : null}
      {points[0] !== undefined ? (
        <Circle cx={sx(points[0].x)} cy={sy(points[0].y)} fill="none" r="5" stroke={theme.colors.text} strokeWidth="1.5" />
      ) : null}
      {markers.map((marker) => (
        <Mark
          color={marker.kind === 'fault' ? theme.colors.danger : theme.colors.text}
          key={`${marker.eventId}-${marker.odometerMm}`}
          kind={marker.kind}
          x={sx(marker.x)}
          y={sy(marker.y)}
        />
      ))}
      {last !== undefined ? <Circle cx={sx(last.x)} cy={sy(last.y)} fill={theme.colors.accent} r="5" /> : null}
      <G>
        <Line stroke={theme.colors.text} strokeWidth="1.5" x1={PAD} x2={PAD + bar * scale} y1={HEIGHT - 12} y2={HEIGHT - 12} />
      </G>
      <G>
        <Polygon fill={theme.colors.text} points={`${WIDTH - 20},12 ${WIDTH - 25},24 ${WIDTH - 15},24`} />
      </G>
    </Svg>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        Bar at the bottom left: {bar >= 1000 ? `${bar / 1000} m` : `${bar} mm`}. Arrow at the top right: north.
      </Text>
    </View>
  );
}
