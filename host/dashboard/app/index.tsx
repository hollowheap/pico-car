import { useLocalSearchParams } from 'expo-router';
import { useCallback, useEffect, useMemo, useRef, useState, type ComponentRef } from 'react';
import { Platform, ScrollView, StyleSheet, View, useWindowDimensions } from 'react-native';

import { AttitudePanel, DynamicsPanel } from '../components/console/AttitudePanel';
import { CourseTrace } from '../components/console/CourseTrace';
import { EventTimeline } from '../components/console/EventTimeline';
import { FaultGrid } from '../components/console/FaultGrid';
import { FrameLogPanel, type FrameReceipt } from '../components/console/FrameLogPanel';
import { LineBarcodePanel } from '../components/console/LineBarcodePanel';
import { MotionPanel } from '../components/console/MotionPanel';
import { ObstaclePanel } from '../components/console/ObstaclePanel';
import { Panel, PanelDense } from '../components/console/Panel';
import { TabbedCard } from '../components/console/TabbedCard';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { consoleFrame } from '../lib/console-frame';
import { buildCourse, latestCourseSpeed, type Course, type CourseOrientation } from '../lib/course';
import { focusElement } from '../lib/focus';
import { FRAME_LOG_LIMIT } from '../lib/frame-log';
import { snapshotClockOffset } from '../lib/link';
import { panelStatus } from '../lib/panel-state';
import { summarizeRun } from '../lib/run-summary';
import { statusLossPercent } from '../lib/telemetry-store';
import { useTelemetry } from '../lib/telemetry-context';
import { useEventInspection } from '../lib/useEventInspection';
import { useNow } from '../lib/useNow';

const ACTIVITY_TABS = { events: 'Events', frames: 'Frames' } as const;
const DESTINATIONS = {
  course: 'course', summary: 'course', drive: 'motion', attitude: 'physics', physics: 'physics', motion: 'motion',
  line: 'line', obstacle: 'obstacle', faults: 'faults', events: 'events', frames: 'frames',
} as const;
type Destination = typeof DESTINATIONS[keyof typeof DESTINATIONS];
type ActivityTab = keyof typeof ACTIVITY_TABS;
const POSE_PAIR_GRACE_MS = 32;
const COLUMN_GAP = 10;

function useSettledCourse(candidate: Course, runStart: number | null, awaitingPair: string | null): Course {
  const [last, setLast] = useState({ course: candidate, runStart });
  const [expired, setExpired] = useState<string | null>(null);
  const wait = awaitingPair !== null && awaitingPair !== expired && last.runStart === runStart && last.course.positionKnown;

  useEffect(() => {
    if (!wait) {
      setLast((previous) => previous.course === candidate && previous.runStart === runStart ? previous : { course: candidate, runStart });
    }
  }, [candidate, runStart, wait]);
  useEffect(() => {
    if (!wait || awaitingPair === null) return;
    const timeout = setTimeout(() => setExpired(awaitingPair), POSE_PAIR_GRACE_MS);
    return () => clearTimeout(timeout);
  }, [awaitingPair, wait]);

  return wait ? last.course : candidate;
}

export default function ConsoleScreen() {
  const { store } = useTelemetry();
  return <ConsoleDashboard key={store.resetReceiptId ?? store.resetAt ?? 'boot'} />;
}

function ConsoleDashboard() {
  const { section, event } = useLocalSearchParams<{ section?: string; event?: string }>();
  const { snapshot, store, frameLog } = useTelemetry();
  const { width, height: windowHeight } = useWindowDimensions();
  const clockOffsetMs = snapshotClockOffset(snapshot, store.latest.status);
  const nowMs = useNow() + clockOffsetMs;
  const [orientation, setOrientation] = useState<CourseOrientation>('north-up');
  const scroll = useRef<ComponentRef<typeof ScrollView>>(null);
  const sections = useRef<Partial<Record<Destination, ComponentRef<typeof View> | null>>>({});
  const latest = store.latest;
  const columns = width >= 760;
  const threeColumns = width >= 1120;
  const { largeDesktop, inset: pageInset, maxWidth } = consoleFrame(width, windowHeight);
  const gridGap = largeDesktop ? 14 : COLUMN_GAP;
  const gridHeight = windowHeight <= 810 ? 720 : Math.min(1160, windowHeight - (largeDesktop ? 84 : 76));
  const state = (kind: string) => panelStatus(snapshot.status, latest[kind], nowMs, kind);
  const requested = section !== undefined && Object.hasOwn(DESTINATIONS, section)
    ? DESTINATIONS[section as keyof typeof DESTINATIONS] : undefined;
  const linkedPanel = event && (section === undefined || requested === 'course') ? 'events' : requested;
  const requestKey = `${section ?? ''}/${event ?? ''}`;
  const requestedTab = linkedPanel === 'frames' || linkedPanel === 'events' ? linkedPanel : undefined;
  const [selection, setSelection] = useState<{ requestKey: string; tab: ActivityTab }>({
    requestKey, tab: requestedTab ?? 'events',
  });
  const pageRequest = section === undefined || requested !== undefined;
  const tab = pageRequest && selection.requestKey !== requestKey ? requestedTab ?? selection.tab : selection.tab;
  if (pageRequest && selection.requestKey !== requestKey) setSelection({ requestKey, tab });
  const [selectedReceipt, setSelectedReceipt] = useState<(FrameReceipt & { revision: number }) | null>(null);
  const receipt = selectedReceipt !== null && tab === 'frames' && selectedReceipt.revision === frameLog.revision &&
    selectedReceipt.id >= frameLog.nextId - FRAME_LOG_LIMIT ? selectedReceipt : null;
  if (selectedReceipt !== null && receipt === null) setSelectedReceipt(null);
  const focusPanel = useCallback((target: Destination) => {
    const panel = sections.current[target];
    if (panel === undefined || panel === null) return;
    if (Platform.OS === 'web') {
      focusElement(panel, { scrollBlock: target === 'course' ? 'start' : 'nearest' });
      return;
    }
    const focus = () => focusElement(panel);
    const content = scroll.current?.getInnerViewRef();
    if (content == null) focus();
    else panel.measureLayout(content, (_x, y) => {
      scroll.current?.scrollTo({ y, animated: false });
      focus();
    }, focus);
  }, []);
  useEffect(() => {
    if (linkedPanel === undefined) return;
    const frame = requestAnimationFrame(() => focusPanel(linkedPanel));
    return () => cancelAnimationFrame(frame);
  }, [focusPanel, linkedPanel, requestKey]);
  useEffect(() => {
    if (receipt?.packet.packet !== 'physics') return;
    const frame = requestAnimationFrame(() => focusPanel('physics'));
    return () => cancelAnimationFrame(frame);
  }, [focusPanel, receipt]);
  const show = (panel: Destination, focus = false) => {
    if (panel === 'events' || panel === 'frames') setSelection({ requestKey, tab: panel });
    if (focus) requestAnimationFrame(() => focusPanel(panel));
  };
  const panelRef = (target: Destination) => (element: ComponentRef<typeof View> | null) => { sections.current[target] = element; };
  const restoreMarker = (restore: () => boolean) => () => {
    requestAnimationFrame(() => { if (!restore()) focusPanel('course'); });
  };

  const candidate = useMemo(
    () => buildCourse(store.trail, store.events, store.obstacle, store.trailStart, store.forwardReturn),
    [store.trail, store.events, store.obstacle, store.trailStart, store.forwardReturn],
  );
  const statusState = state('status');
  const summary = useMemo(() => summarizeRun(store.trail, store.events, store.trailStart),
    [store.trail, store.events, store.trailStart]);
  const lastPose = store.trail.at(-1);
  // A status frame can reach the browser just before its same-sequence motion
  // frame. Keep the preceding plot for at most two frames while that pair lands.
  const awaitingPair = snapshot.status === 'open' && statusState.kind === 'ready' &&
    !candidate.positionKnown && lastPose?.headingDeg === null && lastPose.seq !== undefined && lastPose.wheels === undefined
    ? `${store.trailStart ?? ''}/${lastPose.seq}/${lastPose.timestamp}` : null;
  const course = useSettledCourse(candidate, store.trailStart, awaitingPair);
  const inspection = useEventInspection(course, store.events, store.trailStart, event);
  const controlsHeight = largeDesktop ? 148 : 128;
  const courseHeight = threeColumns
    ? Math.round((gridHeight - gridGap) * 0.5)
    : (columns ? 180 : 168) + (width < 380 ? 184 : 158);
  const plotHeight = threeColumns ? Math.max(196, courseHeight - 102) : columns ? 180 : 168;
  const activityHeight = gridHeight - gridGap - courseHeight;
  // Column 2 (4 cards): Line (172), Obstacle (214), Motion (176), Controls (128) — exact 720 px fit on 1280x800.
  const col2Avail = gridHeight - 3 * gridGap - controlsHeight;
  const lineHeight = Math.round(col2Avail * 0.33);
  const motionHeight = Math.round(col2Avail * 0.31);
  const obstacleHeight = col2Avail - lineHeight - motionHeight;
  // Column 3 (3 cards): Physics, Dynamics, Faults — exact 720 px fit on 1280x800.
  const col3Avail = gridHeight - 2 * gridGap;
  const faultsHeight = Math.min(Math.round(col3Avail * 0.34), 250);
  const physicsHeight = Math.round((col3Avail - faultsHeight) * 0.54);
  const dynamicsHeight = col3Avail - faultsHeight - physicsHeight;
  const eventList = <EventTimeline events={store.events} inspection={inspection} onShowCourse={() => show('course', true)}
    runStart={store.trailStart} />;

  return (
    <ScrollView contentContainerStyle={[styles.page, { padding: pageInset, maxWidth }]} ref={scroll} testID="console-page">
      <PanelDense.Provider value>
        <View style={[styles.group, { gap: gridGap }, columns && styles.row]}>
          <View style={[styles.group, { gap: gridGap }, columns && styles.column]}>
            <View ref={panelRef('course')} tabIndex={-1} testID="console-course"
              style={{ height: courseHeight }}>
              <CourseTrace
                course={course}
                summary={summary}
                speedMms={latestCourseSpeed(latest.status, lastPose)}
                speedLive={statusState.kind === 'ready' && snapshot.status === 'open' && snapshot.bridge?.mqtt !== 'disconnected'}
                height={plotHeight}
                linkStatus={snapshot.status}
                onOrientationChange={setOrientation}
                onSelectCluster={(events, restore) => {
                  inspection.selectCluster(events, restoreMarker(restore));
                  show('events', true);
                }}
                onSelectEvent={(event, restore) => {
                  inspection.selectEvent(event, restoreMarker(restore));
                  show('events', true);
                }}
                orientation={orientation}
                selectedEvent={inspection.selected}
                status={statusState}
              />
            </View>
            <View style={threeColumns && [styles.cardActivity, { height: activityHeight }]}>
              <TabbedCard id="activity" title="Activity" icon="events" fill={threeColumns}
                style={threeColumns && styles.fillCard}
                tabs={ACTIVITY_TABS} active={tab} onChange={show} panelRef={panelRef} bodies={{
                  events: <View style={threeColumns ? styles.desktopFrameBody : styles.frameBody}>{eventList}</View>,
                  frames: <View style={threeColumns ? styles.desktopFrameBody : styles.frameBody}>
                    <FrameLogPanel log={frameLog} selectedId={receipt?.id ?? null}
                    onSelect={(row) => setSelectedReceipt(row === null ? null : { ...row, revision: frameLog.revision })} />
                  </View>,
                }} />
            </View>
          </View>
          <View style={[styles.group, { gap: gridGap }, columns && styles.column, threeColumns && [styles.instrumentGrid, { flexBasis: gridGap }]]}>
            <View style={[styles.group, { gap: gridGap }, threeColumns && styles.column]}>
              <View ref={panelRef('line')} tabIndex={-1} testID="console-line" style={threeColumns && { height: lineHeight }}>
                <LineBarcodePanel diag={latest.diag} events={store.events} rates={store.diagRates} summary={summary}
                  lineSignals={store.lineSignals} barcodeEdges={store.barcodeEdges} nowMs={nowMs}
                  clockOffsetMs={clockOffsetMs} runStartMs={store.trailStart}
                  status={state('status')} statusPacket={latest.status} style={threeColumns && styles.fillCard} />
              </View>
              <View ref={panelRef('obstacle')} tabIndex={-1} testID="console-obstacle" style={threeColumns && { height: obstacleHeight }}>
                <ObstaclePanel course={course} nowMs={nowMs} obstacle={latest.obstacle} ping={store.obstaclePing} summary={summary}
                  profile={latest.profile} status={state('obstacle')} style={threeColumns && styles.fillCard} />
              </View>
              <View ref={panelRef('motion')} tabIndex={-1} testID="console-motion" style={threeColumns && { height: motionHeight }}>
                <MotionPanel motion={latest.motion} history={store.wheelHistory} nowMs={nowMs}
                  clockOffsetMs={clockOffsetMs} status={state('motion')}
                  summary={summary} style={threeColumns && styles.fillCard} />
              </View>
              <View style={threeColumns && { height: controlsHeight }}>
                <Panel icon="controls" testID="console-controls" title="Controls" style={threeColumns && styles.fillCard}>
                  <ControlsPanel card />
                </Panel>
              </View>
            </View>
            <View style={[styles.group, { gap: gridGap }, threeColumns && styles.column, threeColumns && { height: gridHeight }]}>
              <View accessibilityLabel="Physics" nativeID="physics-card" role="region" ref={panelRef('physics')} tabIndex={-1}
                testID="console-physics" style={threeColumns && { height: physicsHeight }}>
                <AttitudePanel physics={latest.physics} pitchHistory={store.pitchHistory} nowMs={nowMs}
                  status={state('physics')} statusPacket={latest.status} summary={summary}
                  style={threeColumns && styles.fillCard}
                  recorded={receipt?.packet.packet === 'physics' ? {
                    packet: receipt.packet,
                    returnToLive: () => {
                      setSelectedReceipt(null);
                      requestAnimationFrame(() => { if (!receipt.restoreFocus()) focusPanel('frames'); });
                    },
                  } : undefined} />
              </View>
              <View testID="console-dynamics" style={threeColumns && { height: dynamicsHeight }}>
                <DynamicsPanel physics={latest.physics} status={state('physics')} summary={summary}
                  style={threeColumns && styles.fillCard} />
              </View>
              <View ref={panelRef('faults')} tabIndex={-1} testID="console-faults" style={threeColumns && { height: faultsHeight }}>
                <FaultGrid level={latest.status?.values.faults} obstacle={latest.obstacle} diag={latest.diag}
                  rates={store.diagRates} lossPercent={statusLossPercent(store)} status={state('status')}
                  sticky={latest.diag?.values.sticky_faults} run={summary} style={threeColumns && styles.fillCard} />
              </View>
            </View>
          </View>
        </View>
      </PanelDense.Provider>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  cardActivity: { overflow: 'hidden' },
  cardPhysics: { flex: 1, minHeight: 236 },
  fillCard: { flex: 1 },
  desktopFrameBody: { flex: 1, minHeight: 0 },
  frameBody: { height: 274 },
  page: { alignSelf: 'center', width: '100%' },
  group: { gap: COLUMN_GAP },
  row: { alignItems: 'flex-start', flexDirection: 'row' },
  column: { flex: 1, minWidth: 0 },
  // Account for the gutter inside the two-column instrument group before distributing the remaining width.
  instrumentGrid: { alignItems: 'flex-start', flexDirection: 'row', flexBasis: COLUMN_GAP, flexGrow: 2 },
});
