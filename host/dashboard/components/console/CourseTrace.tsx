import { useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';
import Svg, { Circle, G, Line, Polygon, Polyline, Rect } from 'react-native-svg';

import type { Course } from '../../lib/course';
import type { EventKind } from '../../lib/events';
import { useTheme } from '../../lib/theme-provider';

type Props = { course: Course; height?: number; darkCanvas?: boolean };

const DEFAULT_WIDTH = 560;
const PAD = 32;
const GRID_SPACING = 28;

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
export function CourseTrace({ course, height: plotHeight, darkCanvas = false }: Props) {
  const theme = useTheme();
  const [width, setWidth] = useState(DEFAULT_WIDTH);
  const height = plotHeight ?? (width < 440 ? 228 : 280);
  const colors = darkCanvas ? {
    ...theme.colors,
    elevated: '#0e1929',
    card: '#18283e',
    border: '#27394e',
    text: '#e8f0ff',
    muted: '#a8bbd1',
    accent: '#78b7ff',
    warning: '#f8bc64',
    danger: '#ffb4ab',
  } : theme.colors;
  const { bounds, points, markers } = course;
  const spanX = Math.max(MIN_SPAN_MM, bounds.maxX - bounds.minX);
  const spanY = Math.max(MIN_SPAN_MM, bounds.maxY - bounds.minY);
  const scale = Math.min((width - 2 * PAD) / spanX, (height - 2 * PAD) / spanY);
  const originX = PAD + ((width - 2 * PAD) - (bounds.maxX - bounds.minX) * scale) / 2;
  const originY = PAD + ((height - 2 * PAD) - (bounds.maxY - bounds.minY) * scale) / 2;
  const sx = (x: number) => originX + (x - bounds.minX) * scale;
  const sy = (y: number) => height - originY - (y - bounds.minY) * scale;
  const last = points[points.length - 1];
  const travelled = bounds.maxX > bounds.minX || bounds.maxY > bounds.minY;
  const bar = [...SCALE_STEPS_MM].reverse().find((mm) => mm * scale <= width / 4) ?? SCALE_STEPS_MM[0] ?? 100;

  return (
    <View
      onLayout={({ nativeEvent }) => setWidth(Math.max(1, Math.round(nativeEvent.layout.width)))}
      style={[styles.plot, { backgroundColor: colors.elevated, borderColor: colors.border }]}
      testID="course-plot"
    >
      <Svg
        accessibilityLabel={`Course trace, ${points.length} points, ${markers.length} marks`}
        accessibilityRole="image"
        height={height}
        preserveAspectRatio="xMidYMid meet"
        viewBox={`0 0 ${width} ${height}`}
        width="100%"
      >
        <G stroke={colors.border} strokeWidth="1">
          {Array.from({ length: Math.floor(width / GRID_SPACING) }, (_, index) => {
            const x = (index + 1) * GRID_SPACING;
            return <Line key={x} x1={x} x2={x} y1={0} y2={height} />;
          })}
          {Array.from({ length: Math.floor(height / GRID_SPACING) }, (_, index) => {
            const y = (index + 1) * GRID_SPACING;
            return <Line key={y} x1={0} x2={width} y1={y} y2={y} />;
          })}
        </G>
        {points.length >= 2 ? (
          <Polyline
            fill="none"
            points={points.map((p) => `${sx(p.x).toFixed(1)},${sy(p.y).toFixed(1)}`).join(' ')}
            stroke={colors.accent}
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth="3"
          />
        ) : null}
        {points[0] !== undefined ? (
          <Circle cx={sx(points[0].x)} cy={sy(points[0].y)} fill="none" r="5" stroke={colors.text} strokeWidth="1.5" />
        ) : null}
        {markers.map((marker) => (
          <Mark
            color={marker.kind === 'fault' ? colors.danger : marker.kind === 'obstacle' ? colors.warning : colors.text}
            key={`${marker.eventId}-${marker.odometerMm}`}
            kind={marker.kind}
            x={sx(marker.x)}
            y={sy(marker.y)}
          />
        ))}
        {last !== undefined ? <Circle cx={sx(last.x)} cy={sy(last.y)} fill={colors.accent} r="5" /> : null}
        <Line
          stroke={colors.muted}
          strokeWidth="1.5"
          x1={14}
          x2={14 + bar * scale}
          y1={height - 12}
          y2={height - 12}
        />
      </Svg>
      <Text style={[theme.typography.caption, styles.north, { color: colors.muted }]}>North ↑</Text>
      <Text style={[theme.typography.caption, styles.scale, { color: colors.muted }]}>
        {bar >= 1000 ? (bar / 1000) + ' m' : bar + ' mm'}
      </Text>
      {!travelled && markers.length === 0 ? (
        <View pointerEvents="none" style={styles.empty}>
          <Text style={[theme.typography.caption, styles.emptyLabel, { color: colors.muted, backgroundColor: colors.card }]}>
            No course movement recorded
          </Text>
        </View>
      ) : null}
    </View>
  );
}

const styles = StyleSheet.create({
  plot: { borderRadius: 16, borderWidth: 1, overflow: 'hidden', position: 'relative' },
  north: { position: 'absolute', right: 13, top: 12 },
  scale: { bottom: 18, left: 14, position: 'absolute' },
  empty: { alignItems: 'center', bottom: 0, justifyContent: 'center', left: 0, position: 'absolute', right: 0, top: 0 },
  emptyLabel: { borderRadius: 18, paddingHorizontal: 12, paddingVertical: 8 },
});
