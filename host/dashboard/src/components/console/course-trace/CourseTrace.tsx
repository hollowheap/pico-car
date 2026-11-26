import { useEffect, useId, useMemo, useRef, useState, type ComponentRef } from 'react';
import { Platform, Pressable, StyleSheet, Text, View, type LayoutChangeEvent } from 'react-native';
import Svg, { Defs, Line, LinearGradient, Rect, Stop } from 'react-native-svg';

import type { CourseDrawing } from '../../../lib/console-view';
import { eventKey, orientCourse, type Course, type CourseMarker, type CourseOrientation, type CoursePoint } from '../../../lib/course';
import type { LinkStatus } from '../../../lib/link';
import type { PanelStatus } from '../../../lib/panel-state';
import type { RunSummary } from '../../../../../bridge/src/console/run-summary.js';
import type { EventRow } from '../../../../../bridge/src/console/telemetry-store.js';
import { useTheme } from '../../../lib/theme-provider';
import { useReducedMotion } from '../../../lib/useReducedMotion';
import { Button } from '../../ui/button';
import { metres } from '../format';
import { Panel } from '../Panel';
import { CourseLayers, courseSpeedScaleMms, Mark, scaleBarMm, SPEED_STOPS, UNKNOWN_DASH, type TraceFrame } from './layers';

type Props = {
  course: Course;
  drawing: CourseDrawing[CourseOrientation];
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
const MARKER_TARGET = 44;
/** The smallest span the plot fits, so a short run is not blown up (mm). */
const MIN_SPAN_MM = 600;
const TRACE_TRANSITION_MS = 140;
const MIN_ZOOM = 0.5;
const MAX_ZOOM = 8;
const ZOOM_STEP = 1.35;
const clampZoom = (z: number) => Math.round(Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, z)) * 100) / 100;

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

export function CourseTrace({
  course: worldCourse,
  drawing: { strokes, tapeRuns },
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
  const { bounds, points, markers } = course;
  const last = points.at(-1);
  const hasPositions = last !== undefined;
  const speedScaleMms = useMemo(() => courseSpeedScaleMms(strokes), [strokes]);
  const showUnknown = strokes.some((stroke) => stroke.band === 'unknown');
  const showSlip = strokes.some((stroke) => stroke.slip);
  const caption = [theme.typography.caption, { color: theme.colors.muted }];
  const seconds = Math.round(summary.durationS);
  const elapsed = `${Math.floor(seconds / 60)}:${String(seconds % 60).padStart(2, '0')}`;
  const hasPlot = plot.width > 0 && plot.height > 0;
  const baseScale = Math.min(
    Math.max(1, plot.width - 2 * INSET) / Math.max(MIN_SPAN_MM, bounds.maxX - bounds.minX),
    Math.max(1, plot.height - 2 * INSET - 24) / Math.max(MIN_SPAN_MM, bounds.maxY - bounds.minY),
  );
  const targetScale = baseScale * zoom;
  const targetMidX = (bounds.minX + bounds.maxX) / 2 - pan.x / Math.max(targetScale, 1e-6);
  const targetMidY = (bounds.minY + bounds.maxY) / 2 + pan.y / Math.max(targetScale, 1e-6);
  const targetCarX = last?.x ?? 0;
  const targetCarY = last?.y ?? 0;
  const isDefaultView = zoom === 1 && pan.x === 0 && pan.y === 0;
  const target = useMemo(() => ({ scale: targetScale, midX: targetMidX, midY: targetMidY, carX: targetCarX,
    carY: targetCarY, heading }), [targetScale, targetMidX, targetMidY, targetCarX, targetCarY, heading]);
  const scope = `${course.bootTimestamp ?? ''}/${course.headingReference}/${course.positionKnown}/${orientation}/${plot.width}/${plot.height}/${heading === null}/${zoom}/${pan.x}/${pan.y}`;
  const frame = useTraceFrame(
    target, scope, !reduced && isDefaultView && course.positionKnown && hasPlot && linkStatus === 'open' && (status === undefined || status.kind === 'ready'),
  );
  const { scale, midX, midY } = frame;
  const bar = hasPlot ? scaleBarMm(Math.min(plot.width / 4, 90) / scale) : 0;
  const cellPx = bar * scale;
  const scaleText = bar >= 1000 ? `${bar / 1000} m` : `${bar} mm`;
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
          <CourseLayers
            accessibilityLabel={[
              `Course trace, ${points.length} point${points.length === 1 ? '' : 's'}, ${markers.length} mark${markers.length === 1 ? '' : 's'}`,
              carUp ? 'car heading up' : upLabel.toLowerCase(),
              hasPositions ? `${scaleText} per grid square` : null,
              course.positionKnown ? null : 'current position unavailable',
            ].filter(Boolean).join(', ')}
            cellPx={cellPx}
            course={course}
            frame={frame}
            gridId={gridId}
            interactiveMarkers={onSelectEvent !== undefined}
            northDeg={northDeg}
            origin={origin}
            plot={plot}
            scaleText={scaleText}
            selectedMarker={selectedMarker}
            speedScaleMms={speedScaleMms}
            strokes={strokes}
            tapeRuns={tapeRuns}
          />
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
                  {SPEED_STOPS.map(({ fraction, rgb }) => (
                    <Stop key={fraction} offset={`${Math.round(fraction * 100)}%`} stopColor={`rgb(${rgb[0]},${rgb[1]},${rgb[2]})`} />
                  ))}
                </LinearGradient>
              </Defs>
              <Rect fill={`url(#${gradId})`} height={4} rx={2} width={44} x={0} y={1} />
            </Svg>
            <Text style={caption}>{speedScaleMms} mm/s</Text>
          </View>
          <Text style={caption}>{course.positionKnown ? '▲ Car' : '○ Last'}</Text>
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
