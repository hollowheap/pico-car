import { useMemo } from 'react';
import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { AttitudePanel } from '../components/console/AttitudePanel';
import { ConsoleNav } from '../components/console/ConsoleNav';
import { CourseTrace, MARK_LEGEND } from '../components/console/CourseTrace';
import { EventTimeline } from '../components/console/EventTimeline';
import { FaultGrid } from '../components/console/FaultGrid';
import { HealthPanel } from '../components/console/HealthPanel';
import { LineBarcodePanel } from '../components/console/LineBarcodePanel';
import { LinkPanel } from '../components/console/LinkPanel';
import { ModeBanner } from '../components/console/ModeBanner';
import { MotionPanel } from '../components/console/MotionPanel';
import { ObstaclePanel } from '../components/console/ObstaclePanel';
import { Panel } from '../components/console/Panel';
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

/** Width from which the instruments sit in columns. */
const WIDE_PX = 1100;
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
  const { width } = useWindowDimensions();
  const wide = width >= WIDE_PX;
  const state = (kind: string) => panelStatus(snapshot.status, latest[kind], nowMs, kind);

  // The store is seeded from the bridge history by useLink, so it also holds the run so far.
  const course = useMemo(() => buildCourse(store.trail, store.events), [store.trail, store.events]);
  const summary = useMemo(() => summarizeRun(store.trail, store.events), [store.trail, store.events]);
  const runStatus: PanelStatus | undefined =
    store.trail.length < 2 && store.events.length === 0
      ? { kind: 'empty', message: 'No frames recorded yet. Drive the car.' }
      : undefined;
  const cell = wide ? styles.cell : undefined;

  const motion = <MotionPanel key="motion" motion={latest.motion} speeds={store.speeds} status={state('motion')} />;
  const line = (
    <LineBarcodePanel key="line" diag={latest.diag} events={store.events} statusPacket={latest.status} status={state('status')} />
  );
  const attitude = (
    <AttitudePanel key="attitude" physics={latest.physics} statusPacket={latest.status} status={state('physics')} />
  );
  const obstacle = (
    <ObstaclePanel key="obstacle" obstacle={latest.obstacle} profile={latest.profile} status={state('obstacle')} />
  );
  const faultGrid = <FaultGrid key="faults" level={faults} status={state('status')} sticky={latest.diag?.values.sticky_faults} />;
  const health = <HealthPanel key="health" diag={latest.diag} motion={latest.motion} status={state('diag')} />;
  const link = <LinkPanel key="link" nowMs={nowMs} snapshot={snapshot} status={state('heartbeat')} store={store} />;
  const trace = (
    <Panel key="trace" status={runStatus} style={cell} testID="course-trace" icon="trace" title="Course trace">
      <CourseTrace course={course} />
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        Dead-reckoned from the odometer and the compass heading, north up. It drifts with the heading error and is not a map.
        Open ring: start. Blue dot: the car now.
      </Text>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        Marks: {MARK_LEGEND.map((item) => item.label).join(', ')}.
      </Text>
    </Panel>
  );
  const events = (
    <EventTimeline key="events" events={store.events} limit={200} maxHeight={EVENT_LIST_PX} status={runStatus} style={cell} />
  );

  return (
    <ScrollView
      contentContainerStyle={[styles.page, { paddingTop: insets.top + 16 }]}
      showsVerticalScrollIndicator={false}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav />
      {linkUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{LINK_CONFIG_ERROR}</Text>
      ) : null}
      {serviceUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{SERVICE_CONFIG_ERROR}</Text>
      ) : null}
      <StatusRail
        aside={<ModeBanner faults={faults} overview={overview} statusAgeMs={overview.statusAgeMs} />}
        latest={latest}
        nowMs={nowMs}
        overview={overview}
        snapshot={snapshot}
      />
      <ControlsPanel
        disabled={commandsDisabled}
        disabledReason={disabledReason}
        emergencyLatched={emergencyLatched}
        feedback={feedback}
        feedbackError={feedbackError}
        onAction={send}
        title="Mission commands"
      />
      {wide ? (
        <>
          <View style={styles.columns}>
            <View style={styles.column}>{[motion, line, link]}</View>
            <View style={styles.column}>{[attitude, obstacle]}</View>
            <View style={styles.column}>{[faultGrid, health]}</View>
          </View>
          <RunSummaryPanel status={runStatus} summary={summary} />
          <View style={styles.columns}>{[trace, events]}</View>
        </>
      ) : (
        <View style={styles.stack}>
          {[motion, attitude, line, obstacle, faultGrid]}
          <RunSummaryPanel status={runStatus} summary={summary} />
          {[trace, events, health, link]}
        </View>
      )}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 16, padding: 16, paddingBottom: 32 },
  columns: { alignItems: 'flex-start', flexDirection: 'row', gap: 16 },
  column: { flex: 1, gap: 16, minWidth: 0 },
  cell: { flex: 1, minWidth: 0 },
  stack: { gap: 16 },
});
