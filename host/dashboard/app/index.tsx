import { useLocalSearchParams, useRouter } from 'expo-router';
import { useCallback, useEffect, useState } from 'react';
import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { AttitudePanel } from '../components/console/AttitudePanel';
import { ConsoleNav } from '../components/console/ConsoleNav';
import { DiagnosticsDrawer } from '../components/console/DiagnosticsDrawer';
import { LiveCoursePanel } from '../components/console/LiveCoursePanel';
import { LiveStatus } from '../components/console/ModeBanner';
import { MotionPanel } from '../components/console/MotionPanel';
import { ObstaclePanel } from '../components/console/ObstaclePanel';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { buildOverview } from '../components/overview/overview';
import { deviceSentMs } from '../lib/command';
import { LINK_CONFIG_ERROR, SERVICE_CONFIG_ERROR, getLinkUrl, getServiceUrl } from '../lib/config';
import { FAULT_ESTOP_BIT } from '../lib/faults';
import { panelStatus } from '../lib/panel-state';
import { useTelemetry } from '../lib/telemetry-context';
import { useTheme } from '../lib/theme-provider';
import { useCommand } from '../lib/useCommand';
import { useNow } from '../lib/useNow';

export default function LiveScreen() {
  const router = useRouter();
  const { diagnostics } = useLocalSearchParams<{ diagnostics?: string }>();
  const [drawerOpen, setDrawerOpen] = useState(diagnostics === 'open');
  useEffect(() => {
    if (diagnostics === 'open') setDrawerOpen(true);
  }, [diagnostics]);

  const linkUrl = getLinkUrl();
  const serviceUrl = getServiceUrl();
  const { snapshot, packets, store } = useTelemetry();
  const { feedback, feedbackError, send } = useCommand(serviceUrl, packets);
  const nowMs = useNow(250);
  const overview = buildOverview(snapshot, nowMs);
  const latest = store.latest;
  const faults = latest.status?.values.faults;
  const emergencyLatched = faults !== undefined && (faults & (1 << FAULT_ESTOP_BIT)) !== 0;
  const linkOffline = snapshot.status !== 'open' || snapshot.bridge?.mqtt === 'disconnected';
  const noHeartbeat = deviceSentMs(packets, nowMs) === null;
  const commandsDisabled = linkOffline || serviceUrl === null || noHeartbeat;
  const disabledReason = serviceUrl === null ? SERVICE_CONFIG_ERROR
    : linkOffline ? 'Link offline. Commands are paused.'
    : 'Commands paused until the next heartbeat.';
  const theme = useTheme();
  const insets = useSafeAreaInsets();
  const { width, height } = useWindowDimensions();
  const wide = width >= 900;
  const state = (kind: string) => panelStatus(snapshot.status, latest[kind], nowMs, kind);
  const closeDiagnostics = useCallback(() => {
    setDrawerOpen(false);
    if (diagnostics === 'open') router.setParams({ diagnostics: '' });
  }, [diagnostics, router]);

  return (
    <ScrollView
      contentContainerStyle={[styles.page, {
        paddingTop: insets.top + 8,
        paddingHorizontal: wide ? 24 : 14,
      }]}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav
        active="live"
        onDiagnostics={() => setDrawerOpen(true)}
        status={<LiveStatus faults={faults} overview={overview} />}
      />
      {linkUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{LINK_CONFIG_ERROR}</Text>
      ) : null}
      <ControlsPanel
        disabled={commandsDisabled}
        disabledReason={disabledReason}
        emergencyLatched={emergencyLatched}
        feedback={feedback}
        feedbackError={feedbackError}
        onAction={send}
      />
      <View
        style={[styles.workspace, wide && styles.split, {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
        }]}
        testID="live-workspace"
      >
        <View style={wide ? styles.courseWide : undefined}>
          <LiveCoursePanel
            events={store.events}
            plotHeight={wide ? Math.max(515, height - 330) : width >= 600 ? 410 : 300}
            serviceUrl={serviceUrl}
            status={latest.status}
            trail={store.trail}
          />
        </View>
        <View style={[styles.instruments, wide && styles.instrumentColumn, { borderColor: theme.colors.border }]} testID="live-instruments">
          <ObstaclePanel obstacle={latest.obstacle} profile={latest.profile} status={state('obstacle')} />
          <View style={[styles.separator, { backgroundColor: theme.colors.border }]} />
          <AttitudePanel physics={latest.physics} statusPacket={latest.status} status={state('physics')} />
          <View style={[styles.separator, { backgroundColor: theme.colors.border }]} />
          <MotionPanel
            graphHeight={wide ? Math.max(40, Math.min(118, height - 815)) : 40}
            motion={latest.motion}
            speeds={store.speeds}
            status={state('motion')}
            style={wide ? styles.fill : undefined}
          />
        </View>
      </View>
      {drawerOpen ? (
        <DiagnosticsDrawer
          nowMs={nowMs}
          onClose={closeDiagnostics}
          overview={overview}
          packets={packets}
          snapshot={snapshot}
          store={store}
        />
      ) : null}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 12, paddingBottom: 28, width: '100%' },
  workspace: { borderRadius: 22, borderWidth: 1, overflow: 'hidden' },
  split: { flexDirection: 'row' },
  courseWide: { flex: 1.65, minWidth: 0 },
  instruments: { minWidth: 0 },
  instrumentColumn: { borderLeftWidth: 1, flex: 1 },
  separator: { height: 1, marginHorizontal: 20 },
  fill: { flexGrow: 1 },
});
