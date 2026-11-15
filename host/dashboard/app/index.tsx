import { useMemo, type ReactNode } from 'react';
import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { AttitudePanel } from '../components/console/AttitudePanel';
import { Brand, ConsoleNav, ThemeToggle } from '../components/console/ConsoleNav';
import { CourseTrace, MARK_LEGEND } from '../components/console/CourseTrace';
import { EventTimeline } from '../components/console/EventTimeline';
import { FaultGrid } from '../components/console/FaultGrid';
import { HealthPanel } from '../components/console/HealthPanel';
import { LineBarcodePanel } from '../components/console/LineBarcodePanel';
import { LinkPanel } from '../components/console/LinkPanel';
import { ModeBanner } from '../components/console/ModeBanner';
import { MotionPanel } from '../components/console/MotionPanel';
import { ObstaclePanel } from '../components/console/ObstaclePanel';
import { Panel, PanelDense, PanelFill } from '../components/console/Panel';
import { RunSummaryPanel } from '../components/console/RunSummaryPanel';
import { StatusRail } from '../components/console/StatusRail';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { buildOverview } from '../components/overview/overview';
import { LINK_CONFIG_ERROR, SERVICE_CONFIG_ERROR, getLinkUrl, getServiceUrl } from '../lib/config';
import { buildCourse } from '../lib/course';
import { FAULT_ESTOP_BIT } from '../lib/faults';
import { panelStatus, type PanelStatus } from '../lib/panel-state';
import { summarizeRun } from '../lib/run-summary';
import { useTelemetry } from '../lib/telemetry-context';
import { useTheme } from '../lib/theme-provider';
import { useCommand } from '../lib/useCommand';
import { useNow } from '../lib/useNow';

/**
 * Window width from which the console uses the three-column grid (checked at
 * 1440x900 and 1920x1080). A narrower window gets the one-column page.
 */
const WIDE_PX = 1360;
/** Drawn height of the course trace in the grid. It keeps its shape. */
const TRACE_PX = 300;
const EVENT_LIST_PX = 520;

/** The whole console on one page: live instruments, health, then the run so far. */
export default function ConsoleScreen() {
  const linkUrl = getLinkUrl();
  const serviceUrl = getServiceUrl();
  const { snapshot, packets, store } = useTelemetry();
  const { feedback, feedbackError, send } = useCommand(serviceUrl, packets);
  const nowMs = useNow(250);
  const overview = buildOverview(snapshot, nowMs);
  const latest = store.latest;
  const faults = latest.status?.values.faults;
  const emergencyLatched = faults !== undefined && (faults & (1 << FAULT_ESTOP_BIT)) !== 0;
  const commandsDisabled = snapshot.status !== 'open' || serviceUrl === null;
  const disabledReason = serviceUrl === null ? SERVICE_CONFIG_ERROR : 'Link offline. Commands are disabled.';
  const theme = useTheme();
  const insets = useSafeAreaInsets();
  const { width, height } = useWindowDimensions();
  const wide = width >= WIDE_PX;
  // In the grid a panel takes the height of its row, so the cards of a row end on one line.
  const grow = wide ? styles.grow : undefined;
  const state = (kind: string) => panelStatus(snapshot.status, latest[kind], nowMs, kind);

  // The store is seeded from the bridge history by useLink, so it also holds the run so far.
  const course = useMemo(() => buildCourse(store.trail, store.events), [store.trail, store.events]);
  const summary = useMemo(() => summarizeRun(store.trail, store.events), [store.trail, store.events]);
  const runStatus: PanelStatus | undefined =
    store.trail.length < 2 && store.events.length === 0
      ? { kind: 'empty', message: 'No frames recorded yet. Drive the car.' }
      : undefined;

  const motion = <MotionPanel key="motion" motion={latest.motion} speeds={store.speeds} status={state('motion')} style={grow} />;
  const line = (
    <LineBarcodePanel key="line" diag={latest.diag} events={store.events} statusPacket={latest.status} status={state('status')} style={grow} />
  );
  const attitude = (
    <AttitudePanel key="attitude" physics={latest.physics} statusPacket={latest.status} status={state('physics')} style={grow} />
  );
  const obstacle = (
    <ObstaclePanel key="obstacle" obstacle={latest.obstacle} profile={latest.profile} status={state('obstacle')} style={grow} />
  );
  const faultGrid = <FaultGrid key="faults" level={faults} status={state('status')} sticky={latest.diag?.values.sticky_faults} />;
  const health = <HealthPanel key="health" diag={latest.diag} motion={latest.motion} status={state('diag')} />;
  const link = <LinkPanel key="link" nowMs={nowMs} snapshot={snapshot} status={state('heartbeat')} store={store} />;
  const trace = (
    <Panel key="trace" status={runStatus} testID="course-trace" icon="trace" title="Course trace">
      <CourseTrace course={course} height={wide ? TRACE_PX : undefined} />
      {wide ? null : (
        <>
          <Text style={[theme.typography.caption, styles.prose, { color: theme.colors.muted }]}>
            Dead-reckoned from the odometer and the compass heading, north up. It drifts with the heading error and is not a map.
            Open ring: start. Blue dot: the car now.
          </Text>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Marks: {MARK_LEGEND.map((item) => item.label).join(', ')}.
          </Text>
        </>
      )}
    </Panel>
  );
  const events = (
    <EventTimeline key="events" events={store.events} limit={200} maxHeight={wide ? undefined : EVENT_LIST_PX} status={runStatus} />
  );

  const configErrors = (
    <>
      {linkUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{LINK_CONFIG_ERROR}</Text>
      ) : null}
      {serviceUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{SERVICE_CONFIG_ERROR}</Text>
      ) : null}
    </>
  );
  const banner = <ModeBanner faults={faults} overview={overview} statusAgeMs={overview.statusAgeMs} />;
  const compactBanner = <ModeBanner compact faults={faults} overview={overview} statusAgeMs={overview.statusAgeMs} />;
  const controls = (
    <ControlsPanel
      disabled={commandsDisabled}
      disabledReason={disabledReason}
      emergencyLatched={emergencyLatched}
      feedback={feedback}
      feedbackError={feedbackError}
      onAction={send}
      title="Mission commands"
    />
  );
  const runSummary = <RunSummaryPanel key="summary" status={runStatus} summary={summary} />;
  // A panel in a row of the grid. `weight` is its share of the row width.
  const part = (node: ReactNode, key: string, weight = 1) => (
    <View key={key} style={[styles.part, { flex: weight }]}>
      {node}
    </View>
  );

  if (wide) {
    // Three columns of unequal width. Each panel takes the height of its
    // content; only the event list takes the height that is left, and it
    // scrolls inside. The page scrolls when the window is too short.
    return (
      <ScrollView
        contentContainerStyle={[styles.widePage, { minHeight: height, paddingTop: insets.top + 16 }]}
        showsVerticalScrollIndicator={false}
        style={{ backgroundColor: theme.colors.canvas }}
      >
        <View style={styles.header}>
          <Brand />
          <View style={styles.rail}>
            <StatusRail aside={compactBanner} compact latest={latest} nowMs={nowMs} overview={overview} snapshot={snapshot} />
          </View>
          <ThemeToggle />
        </View>
        {configErrors}
        {controls}
        <PanelDense.Provider value>
          <View style={styles.grid}>
            <View style={[styles.column, styles.columnInstruments]}>
              <View style={styles.pair}>{[part(motion, 'motion'), part(attitude, 'attitude')]}</View>
              <View style={styles.pair}>
                <View style={[styles.column, styles.columnSide]}>{[line, link]}</View>
                {part(obstacle, 'obstacle', 3)}
              </View>
            </View>
            <View style={[styles.column, styles.columnRun]}>{[trace, runSummary, health]}</View>
            <View style={[styles.column, styles.columnLog]}>
              {faultGrid}
              <View style={styles.fillCell}>
                <PanelFill.Provider value>{events}</PanelFill.Provider>
              </View>
            </View>
          </View>
        </PanelDense.Provider>
      </ScrollView>
    );
  }

  return (
    <ScrollView
      contentContainerStyle={[styles.page, { paddingTop: insets.top + 16 }]}
      showsVerticalScrollIndicator={false}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav />
      {configErrors}
      <StatusRail aside={banner} latest={latest} nowMs={nowMs} overview={overview} snapshot={snapshot} />
      {controls}
      <View style={styles.stack}>
        {[motion, attitude, line, obstacle, faultGrid, runSummary, trace, events, health, link]}
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 16, padding: 16, paddingBottom: 32 },
  widePage: { gap: 16, padding: 16 },
  header: { alignItems: 'center', flexDirection: 'row', gap: 16 },
  rail: { flex: 1, minWidth: 0 },
  grid: { alignItems: 'stretch', flexDirection: 'row', flexGrow: 1, gap: 16 },
  column: { gap: 16, minWidth: 0 },
  columnInstruments: { flex: 5 },
  columnRun: { flex: 4 },
  columnLog: { flex: 3 },
  columnSide: { flex: 2 },
  pair: { alignItems: 'stretch', flexDirection: 'row', gap: 16 },
  grow: { flexGrow: 1 },
  part: { minWidth: 0 },
  // The event list: its own content must not set the height of the grid.
  fillCell: { flexBasis: 0, flexGrow: 1, minHeight: 280 },
  // About 75 characters of caption text.
  prose: { maxWidth: 440 },
  stack: { gap: 16 },
});
