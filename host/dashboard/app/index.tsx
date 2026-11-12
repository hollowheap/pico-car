import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { AttitudePanel } from '../components/console/AttitudePanel';
import { ConsoleNav } from '../components/console/ConsoleNav';
import { EventTimeline } from '../components/console/EventTimeline';
import { FaultGrid } from '../components/console/FaultGrid';
import { LineBarcodePanel } from '../components/console/LineBarcodePanel';
import { ModeBanner } from '../components/console/ModeBanner';
import { MotionPanel } from '../components/console/MotionPanel';
import { ObstaclePanel } from '../components/console/ObstaclePanel';
import { PacketLogPanel } from '../components/console/PacketLogPanel';
import { StatusRail } from '../components/console/StatusRail';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { buildOverview } from '../components/overview/overview';
import { LINK_CONFIG_ERROR, SERVICE_CONFIG_ERROR, getLinkUrl, getServiceUrl } from '../lib/config';
import { FAULT_ESTOP_BIT } from '../lib/faults';
import { panelStatus } from '../lib/panel-state';
import { useTelemetry } from '../lib/telemetry-context';
import { useTheme } from '../lib/theme-provider';
import { useCommand } from '../lib/useCommand';
import { useNow } from '../lib/useNow';

/** Width from which the instruments sit in three columns. */
const WIDE_PX = 1100;
const EVENT_LIST_PX = 380;

export default function LiveScreen() {
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
  const events = <EventTimeline key="events" events={store.events} maxHeight={EVENT_LIST_PX} />;

  return (
    <ScrollView
      contentContainerStyle={[styles.page, { paddingTop: insets.top + 16 }]}
      showsVerticalScrollIndicator={false}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav active="live" />
      {linkUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{LINK_CONFIG_ERROR}</Text>
      ) : null}
      {serviceUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{SERVICE_CONFIG_ERROR}</Text>
      ) : null}
      <StatusRail nowMs={nowMs} overview={overview} latest={store.latest} snapshot={snapshot} />
      <ModeBanner faults={faults} overview={overview} statusAgeMs={overview.statusAgeMs} />
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
        <View style={styles.columns}>
          <View style={styles.column}>{[motion, line]}</View>
          <View style={styles.column}>{[attitude, obstacle]}</View>
          <View style={styles.column}>{[faultGrid, events]}</View>
        </View>
      ) : (
        <View style={styles.stack}>{[motion, attitude, line, obstacle, faultGrid, events]}</View>
      )}
      <PacketLogPanel connected={snapshot.status === 'open'} packets={packets} />
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 16, padding: 16, paddingBottom: 32 },
  columns: { alignItems: 'flex-start', flexDirection: 'row', gap: 16 },
  column: { flex: 1, gap: 16 },
  stack: { gap: 16 },
});
