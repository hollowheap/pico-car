import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket, LinkSnapshot } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { enumLabel, heartbeatAgeMs } from '../../lib/schema';
import { statusLossPercent, type TelemetryStore } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import { formatAge } from './format';
import { Panel } from './Panel';

type Props = {
  snapshot: LinkSnapshot;
  store: TelemetryStore;
  nowMs: number;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** The RSSI bar runs from -90 dBm (empty) to -30 dBm (full). */
const RSSI_EMPTY = -90;
const RSSI_FULL = -30;

function Row({ label, value }: { label: string; value: string }) {
  const theme = useTheme();
  return (
    <View style={styles.row}>
      <Text style={[theme.typography.body, { color: theme.colors.muted }]}>{label}</Text>
      <Text style={[theme.typography.body, styles.value, { color: theme.colors.text }]}>{value}</Text>
    </View>
  );
}

/** Wi-Fi signal, frame loss from gaps in seq, queue drops, uptime and the last reset. */
export function LinkPanel({ snapshot, store, nowMs, status, style }: Props) {
  const theme = useTheme();
  const hb = store.latest.heartbeat?.values;
  const diag = store.latest.diag?.values;
  const loss = statusLossPercent(store);
  const rssi = hb?.wifi_rssi;
  const bar = rssi === undefined ? 0 : Math.max(0, Math.min(1, (rssi - RSSI_EMPTY) / (RSSI_FULL - RSSI_EMPTY)));
  const hbAge = store.latest.heartbeat === undefined ? null : heartbeatAgeMs(nowMs, store.latest.heartbeat.timestamp);
  const reset = hb === undefined ? '--' : enumLabel('resetReason', hb.reset_reason).toLowerCase();

  return (
    <Panel status={status} style={style} testID="link-panel" title="Link">
      <View
        accessibilityLabel={`Signal ${rssi ?? 'unknown'} dBm`}
        accessibilityRole="progressbar"
        style={[styles.bar, { backgroundColor: theme.colors.elevated, borderRadius: theme.radii.nav }]}
      >
        <View style={[styles.fill, { backgroundColor: theme.colors.text, width: `${bar * 100}%` }]} />
      </View>
      <Row label="Signal" value={rssi === undefined ? '--' : `${rssi} dBm`} />
      <Row label="Link state" value={hb === undefined ? '--' : enumLabel('linkState', hb.link_state).toLowerCase()} />
      <Row label="Dashboard link" value={snapshot.status} />
      <Row label="Bridge to MQTT" value={snapshot.bridge?.mqtt ?? 'unknown'} />
      <Row label="Heartbeat age" value={formatAge(hbAge)} />
      <Row label="Frames lost (status)" value={loss === null ? '--' : `${loss.toFixed(1)} %`} />
      <Row label="Car uptime" value={hb === undefined ? '--' : `${hb.uptime_s} s`} />
      <Row label="Last reset" value={reset} />
      <Row label="Queue drops" value={`${hb?.drops ?? '--'} (since boot, car) · ${diag?.telem_queue_drops ?? '--'} telemetry · ${diag?.net_queue_drops ?? '--'} network`} />
    </Panel>
  );
}

const styles = StyleSheet.create({
  bar: { height: 10 },
  fill: { bottom: 0, left: 0, position: 'absolute', top: 0 },
  row: { flexDirection: 'row', gap: 12, justifyContent: 'space-between' },
  value: { flexShrink: 1, textAlign: 'right' },
});
