import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { AttitudePanel } from '../components/console/AttitudePanel';
import { ConsoleNav } from '../components/console/ConsoleNav';
import { DiagnosticsPanel } from '../components/console/DiagnosticsPanel';
import { KinematicsPanel } from '../components/console/KinematicsPanel';
import { PacketLogPanel } from '../components/console/PacketLogPanel';
import { StatusRail } from '../components/console/StatusRail';
import { SystemPanel } from '../components/console/SystemPanel';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { LINK_CONFIG_ERROR, SERVICE_CONFIG_ERROR, getLinkUrl, getServiceUrl } from '../lib/config';
import { useCommand } from '../lib/useCommand';
import { buildOverview } from '../components/overview/overview';
import { useNow } from '../lib/useNow';
import { useLink } from '../lib/useLink';
import { useTheme } from '../lib/theme-provider';

export default function LiveScreen() {
  const linkUrl = getLinkUrl();
  const serviceUrl = getServiceUrl();
  const { snapshot, packets } = useLink(linkUrl ?? undefined);
  const { feedback, feedbackError, send } = useCommand(serviceUrl, packets);
  const nowMs = useNow(250);
  const overview = buildOverview(snapshot, nowMs);
  const lastStatus = packets.find((packet) => packet.packet === 'status');
  const lastSystem = packets.find((packet) => packet.packet === 'system');
  const emergencyLatched =
    ((lastStatus?.values.faults ?? 0) & (1 << 9)) !== 0;
  const commandsDisabled = snapshot.status !== 'open' || serviceUrl === null;
  const disabledReason =
    serviceUrl === null
      ? SERVICE_CONFIG_ERROR
      : 'Link offline. Commands are disabled.';
  const theme = useTheme();
  const insets = useSafeAreaInsets();
  const { width } = useWindowDimensions();
  const wide = width >= 1024;

  return (
    <ScrollView
      showsVerticalScrollIndicator={false}
      contentContainerStyle={[
        styles.page,
        { paddingTop: insets.top + 16 },
      ]}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav active="live" />
      {linkUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>
          {LINK_CONFIG_ERROR}
        </Text>
      ) : null}
      {serviceUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>
          {SERVICE_CONFIG_ERROR}
        </Text>
      ) : null}
      <StatusRail
        snapshot={snapshot}
        overview={overview}
        packets={packets}
        nowMs={nowMs}
      />
      <ControlsPanel
        title="Mission commands"
        disabled={commandsDisabled}
        disabledReason={disabledReason}
        emergencyLatched={emergencyLatched}
        feedback={feedback}
        feedbackError={feedbackError}
        onAction={send}
      />
      <View style={styles.grid}>
        <KinematicsPanel
          status={lastStatus}
          packets={packets}
          style={wide ? styles.widePanel : styles.fullPanel}
        />
        <AttitudePanel
          status={lastStatus}
          style={wide ? styles.narrowPanel : styles.fullPanel}
        />
        <DiagnosticsPanel
          status={lastStatus}
          overview={overview}
          style={wide ? styles.narrowPanel : styles.fullPanel}
        />
        <SystemPanel
          system={lastSystem}
          style={wide ? styles.narrowPanel : styles.fullPanel}
        />
        <PacketLogPanel
          connected={snapshot.status === 'open'}
          packets={packets}
          style={wide ? styles.widePanel : styles.fullPanel}
        />
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 16, padding: 16, paddingBottom: 32 },
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 16 },
  widePanel: { flexBasis: '64%' },
  narrowPanel: { flexBasis: '32%' },
  fullPanel: { flexBasis: '100%' },
});
