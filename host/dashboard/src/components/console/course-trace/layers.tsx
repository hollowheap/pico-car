import { useMemo } from 'react';
import Svg, { Circle, Defs, Ellipse, G, Line, LinearGradient, Pattern, Polygon, Polyline, RadialGradient, Rect, Stop, Text as SvgText } from 'react-native-svg';

import { eventKey, type Course, type CourseMarker, type CoursePoint } from '../../../lib/course';
import type { CourseTraceSegment } from '../../../../../bridge/src/console/course.js';

import type { EventKind } from '../../../../../bridge/src/console/events.js';
import { useTheme } from '../../../lib/theme-provider';
import { SurfaceStroke } from '../SurfaceStroke';

export const SPEED_MAX_MMS = 180;
export const SPEED_STOPS: readonly { mms: number; rgb: [number, number, number] }[] = [
  { mms: 0, rgb: [59, 130, 246] },    // blue (stopped / creep)
  { mms: 45, rgb: [6, 182, 212] },    // cyan (slow / crawl)
  { mms: 90, rgb: [34, 197, 94] },    // green (cruise)
  { mms: 135, rgb: [250, 204, 21] },  // yellow (fast)
  { mms: 180, rgb: [239, 68, 68] },   // red-orange (sprint)
];
export const RAMP_CLIMB_COLOR = '#F59E0B';
export const UNKNOWN_DASH = '5 4';
const LABEL_SIZE = 13;

function speedGradientColor(speedMms: number | null, fallback: string): string {
  if (speedMms === null || !Number.isFinite(speedMms)) return fallback;
  const clamped = Math.max(0, Math.min(SPEED_MAX_MMS, speedMms));
  for (let i = 1; i < SPEED_STOPS.length; i += 1) {
    const a = SPEED_STOPS[i - 1]!;
    const b = SPEED_STOPS[i]!;
    if (clamped <= b.mms) {
      const t = (clamped - a.mms) / Math.max(1, b.mms - a.mms);
      const r = Math.round(a.rgb[0] + (b.rgb[0] - a.rgb[0]) * t);
      const g = Math.round(a.rgb[1] + (b.rgb[1] - a.rgb[1]) * t);
      const bl = Math.round(a.rgb[2] + (b.rgb[2] - a.rgb[2]) * t);
      return `rgb(${r},${g},${bl})`;
    }
  }
  const last = SPEED_STOPS.at(-1)!.rgb;
  return `rgb(${last[0]},${last[1]},${last[2]})`;
}

export function Mark({ kind, x, y, color }: { kind: EventKind; x: number; y: number; color: string }) {
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

/** Largest metric step (1, 2, or 5 times a power of ten) within the target width. */
export function scaleBarMm(limitMm: number): number {
  const unit = 10 ** Math.floor(Math.log10(limitMm));
  return unit * (limitMm >= 5 * unit ? 5 : limitMm >= 2 * unit ? 2 : 1);
}

function headingArrow(x: number, y: number, headingDeg: number): string {
  const angle = (headingDeg * Math.PI) / 180;
  const east = Math.sin(angle);
  const north = Math.cos(angle);
  return [
    [x + east * 11, y - north * 11],
    [x - east * 6 - north * 6, y + north * 6 - east * 6],
    [x - east * 6 + north * 6, y + north * 6 + east * 6],
  ].map(([px, py]) => `${px!.toFixed(1)},${py!.toFixed(1)}`).join(' ');
}

/** The mark of an event kind at icon size, so the timeline and the trace match. */
export function EventGlyph({ kind }: { kind: EventKind }) {
  const theme = useTheme();
  return (
    <Svg aria-hidden height={14} viewBox="0 0 14 14" width={14}>
      <Mark color={kind === 'fault' ? theme.colors.danger : theme.colors.muted} kind={kind} x={7} y={7} />
    </Svg>
  );
}

export type TraceFrame = { scale: number; midX: number; midY: number; carX: number; carY: number; heading: number | null };

type CourseLayersProps = {
  accessibilityLabel: string;
  cellPx: number;
  course: Course;
  frame: TraceFrame;
  gridId: string;
  interactiveMarkers: boolean;
  northDeg: number;
  origin: CoursePoint;
  plot: { width: number; height: number };
  scaleText: string;
  selectedMarker: CourseMarker | undefined;
  strokes: CourseTraceSegment[];
  tapeRuns: CoursePoint[][];
};

export function CourseLayers({
  accessibilityLabel, cellPx, course, frame, gridId, interactiveMarkers, northDeg, origin, plot,
  scaleText, selectedMarker, strokes, tapeRuns,
}: CourseLayersProps) {
  const theme = useTheme();
  const { scale, midX, midY, carX, carY, heading: drawnHeading } = frame;
  const { points, markers, obstacle: contact } = course;
  const first = points[0];
  const last = points.at(-1);
  const hasPositions = last !== undefined;
  const showLastFix = !course.positionKnown && course.headingReference !== 'unavailable';
  const compass = course.headingReference === 'compass';
  const targetCarX = last?.x ?? 0;
  const targetCarY = last?.y ?? 0;
  const sx = (x: number) => plot.width / 2 + (x - midX) * scale;
  const sy = (y: number) => plot.height / 2 - (y - midY) * scale;
  const drawing = useMemo(() => strokes.map(({ points: path, band, speedMms: segSpeed, slip, recent }) => ({
    points: path.map(({ x, y }) => `${(plot.width / 2 + (x - midX) * scale).toFixed(1)},${(plot.height / 2 - (y - midY) * scale).toFixed(1)}`).join(' '),
    color: slip ? theme.colors.danger : band === 'unknown' ? theme.colors.muted : speedGradientColor(segSpeed, theme.colors.muted),
    dash: slip ? '4 3' : band === 'unknown' ? UNKNOWN_DASH : undefined,
    opacity: slip ? 0.95 : recent ? 0.92 : 0.68,
    width: slip ? 3.2 : recent ? 2.8 : 2,
  })), [strokes, plot, midX, midY, scale, theme.colors.muted, theme.colors.danger]);
  const tapePolylines = useMemo(() => tapeRuns.map((run) =>
    run.map(({ x, y }) => `${(plot.width / 2 + (x - midX) * scale).toFixed(1)},${(plot.height / 2 - (y - midY) * scale).toFixed(1)}`).join(' '),
  ), [tapeRuns, plot, midX, midY, scale]);

  return (
    <Svg
      accessibilityLabel={accessibilityLabel}
      accessibilityRole="image"
      height="100%"
      viewBox={`0 0 ${plot.width} ${plot.height}`}
      width="100%"
    >
      <Defs>
        <Pattern id={gridId} patternUnits="userSpaceOnUse" width={cellPx} height={cellPx}
          patternTransform={`translate(${sx(origin.x)} ${sy(origin.y)}) rotate(${northDeg})`}>
          <Rect x={0} y={0} width={cellPx} height={cellPx} fill="none" stroke={theme.colors.border} strokeWidth={1} />
        </Pattern>
        <LinearGradient id={`${gridId}-ramp-grad`} x1="0%" y1="0%" x2="0%" y2="100%">
          <Stop offset="0%" stopColor={RAMP_CLIMB_COLOR} stopOpacity="0" />
          <Stop offset="30%" stopColor={RAMP_CLIMB_COLOR} stopOpacity="0.28" />
          <Stop offset="50%" stopColor="#EF4444" stopOpacity="0.46" />
          <Stop offset="70%" stopColor={RAMP_CLIMB_COLOR} stopOpacity="0.28" />
          <Stop offset="100%" stopColor={RAMP_CLIMB_COLOR} stopOpacity="0" />
        </LinearGradient>
        <RadialGradient id={`${gridId}-ramp-core`} cx="50%" cy="50%" r="50%">
          <Stop offset="0%" stopColor="#EF4444" stopOpacity="0.52" />
          <Stop offset="55%" stopColor={RAMP_CLIMB_COLOR} stopOpacity="0.24" />
          <Stop offset="100%" stopColor={RAMP_CLIMB_COLOR} stopOpacity="0" />
        </RadialGradient>
      </Defs>
      <Rect testID="course-grid" width="100%" height="100%" fill={`url(#${gridId})`} />
      {course.humps.map((h, index) => {
        const cx = sx(h.x);
        const cy = sy(h.y);
        const w = Math.max(16, h.widthMm * scale * 1.15);
        const l = Math.max(20, h.lengthMm * scale * 1.15);
        const intensity = h.peakMm === null ? 0.85 : Math.max(0.55, Math.min(1, h.peakMm / 28));
        return (
          <G key={`hump-2d-${index}-${h.timestamp}`} testID={`course-hump-${index}`} transform={`translate(${cx.toFixed(1)} ${cy.toFixed(1)}) rotate(${h.headingDeg.toFixed(1)})`}>
            <Rect x={-w / 2} y={-l / 2} width={w} height={l} rx={w * 0.35} fill={`url(#${gridId}-ramp-grad)`} opacity={intensity} />
            <Ellipse cx={0} cy={0} rx={w * 0.48} ry={l * 0.34} fill={`url(#${gridId}-ramp-core)`} opacity={intensity} />
            {h.peakMm !== null ? (
              <SvgText fill={RAMP_CLIMB_COLOR} fontSize={12} textAnchor="middle" x={0} y={-l / 2 - 3}>{h.peakMm} mm</SvgText>
            ) : null}
          </G>
        );
      })}
      {tapePolylines.map((pts, index) => (
        <Polyline key={`tape-${index}`} fill="none" points={pts} stroke={theme.colors.text}
          strokeOpacity={0.92} strokeLinejoin="round" strokeLinecap="round" strokeWidth={5.5} />
      ))}
      {course.junctions.map((j) => {
        const cx = sx(j.x);
        const cy = sy(j.y);
        const rad = (j.headingDeg * Math.PI) / 180;
        const fx = Math.sin(rad);
        const fy = -Math.cos(rad);
        const rx = Math.cos(rad);
        const ry = Math.sin(rad);
        const len = 13;
        const branches: [number, number][] = [];
        if (j.direction === 'left' || j.direction === 'straight' || j.direction === 'cross') branches.push([-rx * len, -ry * len]);
        if (j.direction === 'right' || j.direction === 'straight' || j.direction === 'cross') branches.push([rx * len, ry * len]);
        if (j.direction === 'left' || j.direction === 'right' || j.direction === 'cross') branches.push([fx * (len * 0.7), fy * (len * 0.7)]);
        return (
          <G key={`junc-${j.eventId}-${j.timestamp}`} testID={`course-junction-${j.eventId}`}>
            {branches.map(([dx, dy], bi) => (
              <Line key={bi} x1={cx} y1={cy} x2={cx + dx} y2={cy + dy}
                stroke={theme.colors.text} strokeOpacity={0.82} strokeWidth={5.5} strokeLinecap="round" />
            ))}
            <Circle cx={cx} cy={cy} r={3.5} fill={theme.colors.card} stroke={theme.colors.text} strokeWidth={1.6} />
          </G>
        );
      })}
      {drawing.map((path, index) => (
        <Polyline key={index} fill="none" points={path.points} stroke={path.color} strokeOpacity={path.opacity}
          strokeDasharray={path.dash} strokeLinejoin="round" strokeLinecap="round" strokeWidth={path.width} />
      ))}
      {first !== undefined && drawing.length > 0 ? (
        <Circle cx={sx(first.x)} cy={sy(first.y)} fill={theme.colors.card} r="5" stroke={theme.colors.text} strokeWidth="1.5"
          testID="course-origin" />
      ) : null}
      {contact ? <Circle testID="course-front-contact" cx={sx(contact.x)} cy={sy(contact.y)}
        r={3} fill={theme.colors.danger} /> : null}
      {course.surfaces.map((face, index) => (
        <SurfaceStroke key={index} surface={face} project={(p) => ({ x: sx(p.x), y: sy(p.y) })}
          color={theme.colors.text} pathId={`course-surface-${index}`}
          startId={`course-edge-${index}-start`} endId={`course-edge-${index}-end`} />
      ))}
      {!interactiveMarkers ? markers.map((marker) => (
        <Mark
          color={marker.kind === 'fault' ? theme.colors.danger : theme.colors.text}
          key={eventKey(marker.event)}
          kind={marker.kind}
          x={sx(marker.x)}
          y={sy(marker.y)}
        />
      )) : null}
      {selectedMarker === undefined ? null : (
        <Circle cx={sx(selectedMarker.x)} cy={sy(selectedMarker.y)} fill="none" r="11" stroke={theme.colors.accent} strokeWidth="2" />
      )}
      {course.positionKnown ? <G testID="course-current-pose" transform={`translate(${sx(carX)} ${sy(carY)})`}>
        {drawnHeading === null ? (
          <Circle cx={0} cy={0} fill="none" r="7" stroke={theme.colors.accent} strokeWidth="2.5" />
        ) : (
          <Polygon fill={theme.colors.accent} points={headingArrow(0, 0, drawnHeading)}
            stroke={theme.colors.card} strokeWidth="1.5" />
        )}
      </G> : showLastFix ? (
        <Circle cx={sx(targetCarX)} cy={sy(targetCarY)} fill="none" r="7"
          stroke={theme.colors.accent} strokeWidth="2.5" />
      ) : null}
      {hasPositions ? <G testID="course-scale">
        <Line stroke={theme.colors.text} strokeWidth="1.5" x1={10} x2={10 + cellPx} y1={plot.height - 34} y2={plot.height - 34} />
        <SvgText fill={theme.colors.text} fontSize={LABEL_SIZE} x={10 + cellPx + 7} y={plot.height - 30}>{scaleText}</SvgText>
      </G> : null}
      {compass ? <G>
        <Polygon fill={theme.colors.text} points={headingArrow(plot.width - 17, 16, northDeg)} />
        <SvgText fill={theme.colors.text} fontSize={LABEL_SIZE} textAnchor="middle" x={plot.width - 17} y={38}>N</SvgText>
      </G> : null}
    </Svg>
  );
}
