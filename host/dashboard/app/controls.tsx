import { StyleSheet, Text, View } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { ConsoleNav } from '../components/console/ConsoleNav';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { LINK_CONFIG_ERROR, SERVICE_CONFIG_ERROR, getLinkUrl, getServiceUrl } from '../lib/config';
import { useCommand } from '../lib/useCommand';
import { useLink } from '../lib/useLink';
import { useTheme } from '../lib/theme-provider';

export default function ControlsScreen() {
  const theme = useTheme();
  const insets = useSafeAreaInsets();
  const linkUrl = getLinkUrl();
  const serviceUrl = getServiceUrl();
  const { snapshot, packets } = useLink(linkUrl ?? undefined);
  const { feedback, feedbackError, send } = useCommand(serviceUrl, packets);
  const lastStatus = packets.find((packet) => packet.packet === 'status');
  const emergencyLatched =
    ((lastStatus?.values.faults ?? 0) & (1 << 9)) !== 0;

  return (
    <View style={[styles.page, { backgroundColor: theme.colors.canvas, paddingTop: insets.top + 16 }]}>
      <ConsoleNav active="controls" />
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
      <Text style={[theme.typography.heading, { color: theme.colors.text }]}>
        Run controls
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
        Start and stop need confirmation. Reset is rejected while the emergency
        latch is set.
      </Text>
      <ControlsPanel
        emergencyLatched={emergencyLatched}
        disabled={snapshot.status !== 'open'}
        feedback={feedback}
        feedbackError={feedbackError}
        onAction={send}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  page: { flex: 1, gap: 16, padding: 24 },
});
