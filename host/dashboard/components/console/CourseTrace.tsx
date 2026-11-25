import { useEffect, useId, useMemo, useRef, useState, type ComponentRef } from 'react';
import { Platform, Pressable, StyleSheet, Text, View, type LayoutChangeEvent } from 'react-native';
import Svg, { Circle, Defs, Ellipse, G, Line, LinearGradient, Pattern, Polygon, Polyline, RadialGradient, Rect, Stop, Text as SvgText } from 'react-native-svg';

import { courseTapeRuns, courseTraceSegments, eventKey, orientCourse, type Course, type CourseMarker, type CourseOrientation, type CoursePoint } from '../../lib/course';
import type { EventKind } from '../../lib/events';
import type { LinkStatus } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import type { RunSummary } from '../../lib/run-summary';
import type { EventRow } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import { useReducedMotion } from '../../lib/useReducedMotion';
import { Button } from '../ui/button';
import { metres } from './format';
import { Panel } from './Panel';
import { SurfaceStroke } from './SurfaceStroke';

type Props = {
  course: Course;
  summary: RunSummary;
  speedMms: number | null;
  speedLive: boolean;
  orientation?: CourseOrientation;
  onOrientationChange?: (orientation: CourseOrientation) => void;
  /** Allocated drawing height in pixels, independent of the route and legend. */
  height: number;
  linkStatus?: LinkStatus;
  status?: PanelStatus;
  selectedEvent?: EventRow | null;
  onSelectEvent?: (event: EventRow, restoreFocus: () => boolean) => void;
  onSelectCluster?: (events: EventRow[], restoreFocus: () => boolean) => void;
};

const INSET = 32;
const LABEL_SIZE = 13;
const UNKNOWN_DASH = '5 4';
const MARKER_TARGET = 44;
/** The smallest span the plot fits, so a short run is not blown up (mm). */
const MIN_SPAN_MM = 600;
const TRACE_TRANSITION_MS = 140;
const MIN_ZOOM = 0.5;
const MAX_ZOOM = 8;
const ZOOM_STEP = 1.35;
const clampZoom = (z: number) => Math.round(Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, z)) * 100) / 100;
const SPEED_MAX_MMS = 180;
const SPEED_STOPS: readonly { mms: number; rgb: [number, number, number] }[] = [
  { mms: 0, rgb: [59, 130, 246] },    // blue (stopped / creep)
  { mms: 45, rgb: [6, 182, 212] },    // cyan (slow / crawl)
  { mms: 90, rgb: [34, 197, 94] },    // green (cruise)
  { mms: 135, rgb: [250, 204, 21] },  // yellow (fast)
  { mms: 180, rgb: [239, 68, 68] },   // red-orange (sprint)
];
const RAMP_CLIMB_COLOR = '#F59E0B';
const RAMP_DESCEND_COLOR = '#A855F7';

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

type TraceFrame = { scale: number; midX: number; midY: number; carX: number; carY: number; heading: number | null };

function useTraceFrame(target: TraceFrame, scope: string, animate: boolean): TraceFrame {
  const current = useRef({ scope, frame: target });
  const [display, setDisplay] = useState(current.current);

  useEffect(() => {
    const from = current.current;
    if (!animate || from.scope !== scope) {
      current.current = { scope, frame: target };
      setDisplay(current.current);
      return;
    }
    const start = from.frame;
    if (start.scale === target.scale && start.midX === target.midX && start.midY === target.midY &&
        start.carX === target.carX && start.carY === target.carY && start.heading === target.heading) return;

    let animation = 0;
    let startedAt: number | undefined;
    const draw = (now: number) => {
      startedAt ??= now;
      const t = Math.min((now - startedAt) / TRACE_TRANSITION_MS, 1);
      const eased = t === 1 ? 1 : 1 - 2 ** (-10 * t);
      const lerp = (a: number, b: number) => a + (b - a) * eased;
      const delta = start.heading === null || target.heading === null ? null
        : ((target.heading - start.heading + 540) % 360) - 180;
      current.current = { scope, frame: t === 1 ? target : {
        scale: lerp(start.scale, target.scale), midX: lerp(start.midX, target.midX), midY: lerp(start.midY, target.midY),
        carX: lerp(start.carX, target.carX), carY: lerp(start.carY, target.carY),
        heading: delta === null || start.heading === null ? target.heading : (start.heading + delta * eased + 360) % 360,
      } };
      setDisplay(current.current);
      if (t < 1) animation = requestAnimationFrame(draw);
    };
    animation = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(animation);
  }, [animate, scope, target]);

  return animate && display.scope === scope ? display.frame : target;
}

function orientationState(pressed: boolean) {
  return Platform.OS === 'web'
    ? { accessibilityRole: 'button', 'aria-pressed': pressed } as const
    : { accessibilityRole: 'radio', accessibilityState: { checked: pressed } } as const;
}

type ScreenMarker = CoursePoint & { marker: CourseMarker };

/** Merge overlapping hit targets, including overlaps connected through other markers. */
function groupMarkers(markers: ScreenMarker[], canGroup: boolean): ScreenMarker[][] {
  if (!canGroup) return markers.map((marker) => [marker]);
  const groups: ScreenMarker[][] = [];
  for (const marker of markers) {
    const merged = [marker];
    for (let i = groups.length - 1; i >= 0; i -= 1) {
      const group = groups[i];
      if (group?.some(({ x, y }) => Math.abs(x - marker.x) < MARKER_TARGET && Math.abs(y - marker.y) < MARKER_TARGET)) {
        merged.push(...group);
        groups.splice(i, 1);
      }
    }
    merged.sort((a, b) => a.marker.event.timestamp - b.marker.event.timestamp);
    groups.push(merged);
  }
  return groups;
}

function MarkerControl({ group, selectedKey, onSelectEvent, onSelectCluster }: {
  group: ScreenMarker[];
  selectedKey: string | null;
  onSelectEvent: NonNullable<Props['onSelectEvent']>;
  onSelectCluster: Props['onSelectCluster'];
}) {
  const theme = useTheme();
  const target = useRef<ComponentRef<typeof Pressable>>(null);
  const [focused, setFocused] = useState(false);
  const anchor = group[0];
  if (anchor === undefined) return null;

  const event = anchor.marker.event;
  const cluster = group.length > 1;
  const selected = group.some(({ marker }) => eventKey(marker.event) === selectedKey);
  const pressedState = Platform.OS === 'web' ? { 'aria-pressed': selected } : { accessibilityState: { selected } };
  const label = cluster
    ? `${group.length} overlapping events`
    : `${event.text.title}, ${new Date(event.receivedAt ?? event.timestamp).toLocaleTimeString([], { hour12: false })}`;
  return (
    <Pressable
      {...pressedState}
      accessibilityLabel={label}
      accessibilityRole="button"
      onBlur={() => setFocused(false)}
      onFocus={() => setFocused(true)}
      onPress={() => {
        const restoreFocus = () => {
          const marker = target.current;
          if (marker === null) return false;
          marker.focus();
          return true;
        };
        if (cluster) onSelectCluster?.(group.slice().reverse().map(({ marker }) => marker.event), restoreFocus);
        else onSelectEvent(event, restoreFocus);
      }}
      ref={target}
      style={[
        styles.markerTarget,
        { left: anchor.x - MARKER_TARGET / 2, top: anchor.y - MARKER_TARGET / 2 },
        focused ? { borderColor: theme.colors.accent } : null,
      ]}
      testID={`course-${cluster ? 'event-group' : 'event'}-${eventKey(event)}`}
    >
      <View style={[
        styles.markerIcon,
        { backgroundColor: selected ? theme.colors.accent : theme.colors.card, borderColor: theme.colors.accent },
        cluster ? styles.cluster : null,
      ]}>
        {cluster ? (
          <Text style={[theme.typography.label, { color: selected ? theme.colors.onAccent : theme.colors.text }]}>
            {group.length}
          </Text>
        ) : (
          <Svg aria-hidden height={16} viewBox="0 0 16 16" width={16}>
            <Mark color={selected ? theme.colors.onAccent : anchor.marker.kind === 'fault' ? theme.colors.danger : theme.colors.text}
              kind={anchor.marker.kind} x={8} y={8} />
          </Svg>
        )}
      </View>
    </Pressable>
  );
}

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

/** Largest metric step (1, 2, or 5 times a power of ten) within the target width. */
function scaleBarMm(limitMm: number): number {
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

export function CourseTrace({
  course: worldCourse,
  summary, speedMms, speedLive,
  orientation = 'north-up',
  onOrientationChange,
  height,
  linkStatus = 'open',
  status,
  selectedEvent,
  onSelectEvent,
  onSelectCluster,
}: Props) {
  const theme = useTheme();
  const [plot, setPlot] = useState({ width: 0, height: 0 });
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);
  const plotRef = useRef<ComponentRef<typeof View>>(null);
  const panRef = useRef(pan);
  panRef.current = pan;
  const zoomRef = useRef(zoom);
  zoomRef.current = zoom;
  const reduced = useReducedMotion();
  const gridId = 'course-grid-' + useId().replace(/[^a-zA-Z0-9_-]/g, '');
  const gradId = `${gridId}-speed`;
  const { course, headingDeg: heading, northDeg, origin } = useMemo(() => orientCourse(worldCourse, orientation), [worldCourse, orientation]);

  useEffect(() => {
    setZoom(1);
    setPan({ x: 0, y: 0 });
  }, [orientation, course.bootTimestamp]);

  useEffect(() => {
    if (Platform.OS !== 'web') return;
    const node = plotRef.current as unknown as HTMLElement | null;
    if (!node || typeof node.addEventListener !== 'function') return;

    let active = false;
    let startX = 0;
    let startY = 0;
    let origX = 0;
    let origY = 0;

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const prevZoom = zoomRef.current;
      const factor = e.deltaY < 0 ? 1.18 : 1 / 1.18;
      const nextZoom = clampZoom(prevZoom * factor);
      if (nextZoom === prevZoom) return;
      const rect = node.getBoundingClientRect();
      const cx = e.clientX - rect.left - rect.width / 2;
      const cy = e.clientY - rect.top - rect.height / 2;
      const ratio = nextZoom / prevZoom;
      const cur = panRef.current;
      const nextPan = {
        x: cx - (cx - cur.x) * ratio,
        y: cy - (cy - cur.y) * ratio,
      };
      zoomRef.current = nextZoom;
      panRef.current = nextPan;
      setZoom(nextZoom);
      setPan(nextPan);
    };
    const onPointerDown = (e: PointerEvent) => {
      if (e.button !== 0 && e.pointerType === 'mouse') return;
      const target = e.target as HTMLElement | null;
      if (target?.closest?.('[role="button"], button')) return;
      e.preventDefault();
      active = true;
      setDragging(true);
      startX = e.clientX;
      startY = e.clientY;
      origX = panRef.current.x;
      origY = panRef.current.y;
      try { node.setPointerCapture?.(e.pointerId); } catch {}
    };
    const onPointerMove = (e: PointerEvent) => {
      if (!active) return;
      const nextPan = { x: origX + (e.clientX - startX), y: origY + (e.clientY - startY) };
      panRef.current = nextPan;
      setPan(nextPan);
    };
    const onPointerUp = (e: PointerEvent) => {
      if (!active) return;
      active = false;
      setDragging(false);
      try { node.releasePointerCapture?.(e.pointerId); } catch {}
    };
    const onDoubleClick = (e: MouseEvent) => {
      const target = e.target as HTMLElement | null;
      if (target?.closest?.('[role="button"], button')) return;
      zoomRef.current = 1;
      panRef.current = { x: 0, y: 0 };
      setZoom(1);
      setPan({ x: 0, y: 0 });
    };

    node.addEventListener('wheel', onWheel, { passive: false });
    node.addEventListener('pointerdown', onPointerDown);
    node.addEventListener('dblclick', onDoubleClick);
    window.addEventListener('pointermove', onPointerMove);
    window.addEventListener('pointerup', onPointerUp);
    window.addEventListener('pointercancel', onPointerUp);
    return () => {
      node.removeEventListener('wheel', onWheel);
      node.removeEventListener('pointerdown', onPointerDown);
      node.removeEventListener('dblclick', onDoubleClick);
      window.removeEventListener('pointermove', onPointerMove);
      window.removeEventListener('pointerup', onPointerUp);
      window.removeEventListener('pointercancel', onPointerUp);
    };
  }, []);
  const carUp = orientation === 'car-up' && heading !== null;
  const compass = course.headingReference === 'compass';
  const upLabel = compass ? 'North up' : 'Trace up';
  const strokes = useMemo(() => courseTraceSegments(course), [course]);
  const tapeRuns = useMemo(() => courseTapeRuns(course), [course]);
  const { bounds, points, markers, obstacle, forwardReturn } = course;
  const plotBounds = useMemo(() => {
    let { minX, maxX, minY, maxY } = bounds;
    for (const run of tapeRuns) {
      for (const pt of run) {
        if (pt.x < minX) minX = pt.x;
        if (pt.x > maxX) maxX = pt.x;
        if (pt.y < minY) minY = pt.y;
        if (pt.y > maxY) maxY = pt.y;
      }
    }
    return { minX, maxX, minY, maxY };
  }, [bounds, tapeRuns]);
  const contact = obstacle;
  const beam = forwardReturn?.beam;
  const returnLabel = forwardReturn === null ? null : `Last near sonar return ${forwardReturn.rangeMm} mm`;
  const first = points[0];
  const last = points.at(-1);
  const hasPositions = last !== undefined;
  const showLastFix = !course.positionKnown && course.headingReference !== 'unavailable';
  const showUnknown = strokes.some((stroke) => stroke.band === 'unknown');
  const showSlip = strokes.some((stroke) => stroke.slip);
  const caption = [theme.typography.caption, { color: theme.colors.muted }];
  const seconds = Math.round(summary.durationS);
  const elapsed = `${Math.floor(seconds / 60)}:${String(seconds % 60).padStart(2, '0')}`;
  const hasPlot = plot.width > 0 && plot.height > 0;
  const baseScale = Math.min(
    Math.max(1, plot.width - 2 * INSET) / Math.max(MIN_SPAN_MM, plotBounds.maxX - plotBounds.minX),
    Math.max(1, plot.height - 2 * INSET - 24) / Math.max(MIN_SPAN_MM, plotBounds.maxY - plotBounds.minY),
  );
  const targetScale = baseScale * zoom;
  const targetMidX = (plotBounds.minX + plotBounds.maxX) / 2 - pan.x / Math.max(targetScale, 1e-6);
  const targetMidY = (plotBounds.minY + plotBounds.maxY) / 2 + pan.y / Math.max(targetScale, 1e-6);
  const targetCarX = last?.x ?? 0;
  const targetCarY = last?.y ?? 0;
  const isDefaultView = zoom === 1 && pan.x === 0 && pan.y === 0;
  const target = useMemo(() => ({ scale: targetScale, midX: targetMidX, midY: targetMidY, carX: targetCarX,
    carY: targetCarY, heading }), [targetScale, targetMidX, targetMidY, targetCarX, targetCarY, heading]);
  const scope = `${course.bootTimestamp ?? ''}/${course.headingReference}/${course.positionKnown}/${orientation}/${plot.width}/${plot.height}/${heading === null}/${zoom}/${pan.x}/${pan.y}`;
  const { scale, midX, midY, carX, carY, heading: drawnHeading } = useTraceFrame(
    target, scope, !reduced && isDefaultView && course.positionKnown && hasPlot && linkStatus === 'open' && (status === undefined || status.kind === 'ready'),
  );
  const sx = (x: number) => plot.width / 2 + (x - midX) * scale;
  const sy = (y: number) => plot.height / 2 - (y - midY) * scale;
  const bar = hasPlot ? scaleBarMm(Math.min(plot.width / 4, 90) / scale) : 0;
  const cellPx = bar * scale;
  const scaleText = bar >= 1000 ? `${bar / 1000} m` : `${bar} mm`;
  const drawing = useMemo(() => strokes.map(({ points: path, band, speedMms: segSpeed, pitchDeg, hump, slip, recent }) => ({
    points: path.map(({ x, y }) => `${(plot.width / 2 + (x - midX) * scale).toFixed(1)},${(plot.height / 2 - (y - midY) * scale).toFixed(1)}`).join(' '),
    color: slip ? theme.colors.danger : band === 'unknown' ? theme.colors.muted : speedGradientColor(segSpeed, theme.colors.muted),
    rampColor: hump ? ((pitchDeg ?? 0) < -2 ? RAMP_DESCEND_COLOR : RAMP_CLIMB_COLOR) : null,
    slip,
    dash: slip ? '4 3' : band === 'unknown' ? UNKNOWN_DASH : undefined,
    opacity: slip ? 0.95 : recent ? 0.92 : 0.68,
    width: slip ? 3.2 : recent ? 2.8 : 2,
  })), [strokes, plot, midX, midY, scale, theme.colors.muted, theme.colors.danger]);
  const tapePolylines = useMemo(() => tapeRuns.map((run) =>
    run.map(({ x, y }) => `${(plot.width / 2 + (x - midX) * scale).toFixed(1)},${(plot.height / 2 - (y - midY) * scale).toFixed(1)}`).join(' '),
  ), [tapeRuns, plot, midX, midY, scale]);
  const groupedMarkers = useMemo(() => groupMarkers(markers.map((marker) => ({
    marker,
    x: plot.width / 2 + (marker.x - midX) * scale,
    y: plot.height / 2 - (marker.y - midY) * scale,
  })), onSelectCluster !== undefined), [markers, plot, onSelectCluster, midX, midY, scale]);
  const selectedKey = selectedEvent == null ? null : eventKey(selectedEvent);
  const selectedMarker = selectedKey === null ? undefined : markers.find((marker) => eventKey(marker.event) === selectedKey);
  const onPlotLayout = ({ nativeEvent }: LayoutChangeEvent) => {
    const { width, height: plotHeight } = nativeEvent.layout;
    setPlot((previous) => previous.width === width && previous.height === plotHeight
      ? previous
      : { width, height: plotHeight });
  };

  return (
    <Panel fill icon="trace" testID="course-trace" title="Course trace"
      right={onOrientationChange !== undefined ? (
          <Button.Group accessibilityLabel="Course orientation" accessibilityRole={Platform.OS === 'web' ? undefined : 'radiogroup'} style={styles.controls}>
            <Button {...orientationState(!carUp)} compact disabled={!hasPositions}
              onPress={() => onOrientationChange('north-up')} variant={carUp ? 'secondary' : 'primary'}>
              <Button.Text style={theme.typography.label}>{upLabel}</Button.Text>
            </Button>
            <Button {...orientationState(carUp)} compact disabled={heading === null}
              onPress={() => onOrientationChange('car-up')} variant={carUp ? 'primary' : 'secondary'}>
              <Button.Text style={theme.typography.label}>Car up</Button.Text>
            </Button>
          </Button.Group>
        ) : undefined}>
      <View style={styles.legend}>
        <Text testID="course-speed" style={[theme.typography.label, { color: theme.colors.text }]}>
          {speedMms === null ? 'Speed --' : `${speedLive ? 'Speed' : 'Last speed'} ${speedMms} mm/s`}
        </Text>
        {summary.hasTravel ? <Text testID="course-travel" style={caption}>
          {`${summary.scope} ${metres(summary.distanceMm)} in ${elapsed}, avg ${Math.round(summary.averageMms)} mm/s`}
        </Text> : null}
      </View>
      <View
        onLayout={onPlotLayout}
        ref={plotRef}
        style={[
          styles.plotBox,
          { minHeight: height },
          Platform.OS === 'web'
            ? ({ cursor: dragging ? 'grabbing' : 'grab', touchAction: 'none', userSelect: 'none' } as object)
            : null,
        ]}
      >
        {hasPlot ? (
          <Svg
            accessibilityLabel={[
              `Course trace, ${points.length} point${points.length === 1 ? '' : 's'}, ${markers.length} mark${markers.length === 1 ? '' : 's'}`,
              carUp ? 'car heading up' : upLabel.toLowerCase(),
              hasPositions ? `${scaleText} per grid square` : null,
              course.positionKnown ? null : 'current position unavailable',
              obstacle === null ? null : 'Front contact',
              course.surfaces.length ? `${course.surfaces.length} observed obstacle surfaces` : null,
              beam == null ? null : returnLabel,
            ].filter(Boolean).join(', ')}
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
            {onSelectEvent === undefined ? markers.map((marker) => (
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
              <Line stroke={theme.colors.text} strokeWidth="1.5" x1={10} x2={10 + bar * scale} y1={plot.height - 34} y2={plot.height - 34} />
              <SvgText fill={theme.colors.text} fontSize={LABEL_SIZE} x={10 + bar * scale + 7} y={plot.height - 30}>{scaleText}</SvgText>
            </G> : null}
            {compass ? <G>
              <Polygon fill={theme.colors.text} points={headingArrow(plot.width - 17, 16, northDeg)} />
              <SvgText fill={theme.colors.text} fontSize={LABEL_SIZE} textAnchor="middle" x={plot.width - 17} y={38}>N</SvgText>
            </G> : null}
          </Svg>
        ) : null}
        {!hasPositions ? <View pointerEvents="none" style={styles.empty}>
          <Text accessibilityLiveRegion="polite" style={[caption, { backgroundColor: theme.colors.card }]}>
            {linkStatus === 'open' ? 'Waiting for course samples' : 'Course offline'}
          </Text>
        </View> : null}
        {hasPositions && hasPlot && onSelectEvent !== undefined ? groupedMarkers.map((group) => {
          const anchor = group[0];
          return anchor === undefined ? null : (
            <MarkerControl group={group} key={eventKey(anchor.marker.event)} onSelectCluster={onSelectCluster}
              onSelectEvent={onSelectEvent} selectedKey={selectedKey} />
          );
        }) : null}
        {hasPositions ? (
          <View style={styles.hudTopLeft}>
            <View style={[styles.zoomBar, { backgroundColor: theme.colors.card, borderColor: theme.colors.border, borderRadius: theme.radii.control }]}>
              <Pressable
                accessibilityLabel="Zoom out"
                accessibilityRole="button"
                hitSlop={6}
                onPress={() => setZoom((z) => clampZoom(z / ZOOM_STEP))}
                style={styles.zoomBtn}
                testID="course-zoom-out"
              >
                <Text style={[theme.typography.label, { color: theme.colors.text }]}>−</Text>
              </Pressable>
              <Pressable
                accessibilityLabel="Reset zoom"
                accessibilityRole="button"
                hitSlop={6}
                onPress={() => { setZoom(1); setPan({ x: 0, y: 0 }); }}
                style={[styles.zoomReset, { borderColor: theme.colors.border }]}
                testID="course-zoom-reset"
              >
                <Text style={[theme.typography.caption, styles.zoomText, { color: isDefaultView ? theme.colors.muted : theme.colors.accent }]}>
                  {isDefaultView ? '1.0x' : zoom === 1 ? 'Fit' : `${zoom.toFixed(1)}x`}
                </Text>
              </Pressable>
              <Pressable
                accessibilityLabel="Zoom in"
                accessibilityRole="button"
                hitSlop={6}
                onPress={() => setZoom((z) => clampZoom(z * ZOOM_STEP))}
                style={styles.zoomBtn}
                testID="course-zoom-in"
              >
                <Text style={[theme.typography.label, { color: theme.colors.text }]}>+</Text>
              </Pressable>
            </View>
          </View>
        ) : null}
        {returnLabel === null ? null : (
          <View pointerEvents="none" style={[styles.returnPill, { backgroundColor: theme.colors.dangerSurface, borderColor: theme.colors.danger, borderRadius: theme.radii.control }]}>
            <Text numberOfLines={1} style={[theme.typography.caption, { color: theme.colors.danger }]}>
              {beam == null ? `${returnLabel} (no loc)` : returnLabel}
            </Text>
          </View>
        )}
        <View
          accessibilityLabel="Course legend"
          pointerEvents="none"
          style={[styles.embeddedLegend, { backgroundColor: theme.colors.card, borderColor: theme.colors.border, borderRadius: theme.radii.control }]}
          testID="course-legend"
        >
          <View style={styles.band}>
            <Text style={caption}>0</Text>
            <Svg aria-hidden height={6} width={44}>
              <Defs>
                <LinearGradient id={gradId} x1="0%" x2="100%" y1="0%" y2="0%">
                  {SPEED_STOPS.map(({ mms, rgb }) => (
                    <Stop key={mms} offset={`${Math.round((mms / SPEED_MAX_MMS) * 100)}%`} stopColor={`rgb(${rgb[0]},${rgb[1]},${rgb[2]})`} />
                  ))}
                </LinearGradient>
              </Defs>
              <Rect fill={`url(#${gradId})`} height={4} rx={2} width={44} x={0} y={1} />
            </Svg>
            <Text style={caption}>{SPEED_MAX_MMS}+</Text>
          </View>
          <View style={styles.band}>
            <Svg aria-hidden height={8} width={12}>
              <Line stroke={theme.colors.text} strokeOpacity={0.92} strokeWidth={3.5} strokeLinecap="round" x1={2} x2={10} y1={4} y2={4} />
            </Svg>
            <Text style={caption}>Tape</Text>
          </View>
          <View style={styles.band}>
            <Svg aria-hidden height={8} width={12}>
              <Line stroke={RAMP_CLIMB_COLOR} strokeWidth={4} strokeLinecap="round" x1={2} x2={10} y1={4} y2={4} />
            </Svg>
            <Text style={caption}>Ramp</Text>
          </View>
          <Text style={caption}>○ Junc</Text>
          <Text style={caption}>{course.positionKnown ? '▲ Car' : '○ Last'}</Text>
          <View style={styles.band}>
            <Svg aria-hidden height={8} width={8}>
              <Circle cx={4} cy={4} r={3} fill={theme.colors.danger} />
            </Svg>
            <Text style={caption}>Contact</Text>
          </View>
          {showSlip ? (
            <View style={styles.band}>
              <Svg aria-hidden height={8} width={12}>
                <Line stroke={theme.colors.danger} strokeDasharray="4 3" strokeWidth={2.5} x1={0} x2={12} y1={4} y2={4} />
              </Svg>
              <Text style={caption}>Slip</Text>
            </View>
          ) : null}
          {showUnknown ? (
            <View style={styles.band}>
              <Svg aria-hidden height={8} width={12}>
                <Line stroke={theme.colors.muted} strokeDasharray={UNKNOWN_DASH} strokeWidth={2} x1={0} x2={12} y1={4} y2={4} />
              </Svg>
              <Text style={caption}>Est.</Text>
            </View>
          ) : null}
        </View>
      </View>
    </Panel>
  );
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

const styles = StyleSheet.create({
  plotBox: { flex: 1, overflow: 'hidden', position: 'relative' },
  controls: { flexDirection: 'row', gap: 4 },
  empty: { ...StyleSheet.absoluteFill, alignItems: 'center', justifyContent: 'center' },
  legend: { flexDirection: 'row', flexWrap: 'wrap', columnGap: 12, rowGap: 4 },
  band: { alignItems: 'center', flexDirection: 'row', gap: 4 },
  hudTopLeft: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 6,
    left: 6,
    position: 'absolute',
    top: 6,
    zIndex: 2,
  },
  zoomBar: {
    alignItems: 'center',
    borderWidth: 1,
    flexDirection: 'row',
    opacity: 0.92,
    overflow: 'hidden',
  },
  zoomBtn: { alignItems: 'center', height: 32, justifyContent: 'center', width: 32 },
  zoomReset: { alignItems: 'center', borderLeftWidth: 1, borderRightWidth: 1, height: 32, justifyContent: 'center', minWidth: 40, paddingHorizontal: 8 },
  zoomText: { fontVariant: ['tabular-nums'] },
  returnPill: {
    borderWidth: 1,
    bottom: 30,
    opacity: 0.94,
    paddingHorizontal: 6,
    paddingVertical: 2,
    position: 'absolute',
    right: 6,
  },
  embeddedLegend: {
    alignItems: 'center',
    borderWidth: 1,
    bottom: 4,
    columnGap: 8,
    flexDirection: 'row',
    flexWrap: 'wrap',
    left: 6,
    opacity: 0.9,
    paddingHorizontal: 8,
    paddingVertical: 3,
    position: 'absolute',
    right: 6,
    rowGap: 2,
  },
  markerTarget: {
    alignItems: 'center',
    borderColor: 'transparent',
    borderRadius: MARKER_TARGET / 2,
    borderWidth: 2,
    height: MARKER_TARGET,
    justifyContent: 'center',
    position: 'absolute',
    width: MARKER_TARGET,
  },
  markerIcon: { alignItems: 'center', borderRadius: 10, height: 18, justifyContent: 'center', minWidth: 18 },
  cluster: { borderWidth: 1, height: 22, minWidth: 22, paddingHorizontal: 3 },
});
