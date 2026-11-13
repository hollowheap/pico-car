import { StyleSheet, Text, View } from 'react-native';

import type { LinkPacket, LinkSnapshot } from '../../lib/link';
import { heartbeatAgeMs } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import type { Overview } from '../overview/overview';
import { bitLabels, enumLabel } from '../../lib/schema';
import { faultHex, formatAge } from './format';

type Props = {
  snapshot: LinkSnapshot;
  overview: Overview;
  /** The newest packet of each kind, by packet name. */
  latest: Record<string, LinkPacket>;
  nowMs: number;
};

function Metric({
  label,
  value,
  color,
}: {
  label: string;
  value: string;
  color: string;
}) {
  const theme = useTheme();

  return (
    <View accessibilityLabel={`${label}: ${value}`} style={styles.metric}>
      <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
        {label}
      </Text>
      <Text numberOfLines={1} style={[theme.typography.body, styles.metricValue, { color }]}>{value}</Text>
    </View>
  );
}

export function StatusRail({ snapshot, overview, latest, nowMs }: Props) {
  const theme = useTheme();
  const lastStatus = latest.status;
  const lastHeartbeat = latest.heartbeat;
  const heartbeatAge =
    lastHeartbeat === undefined
      ? null
      : heartbeatAgeMs(nowMs, lastHeartbeat.timestamp);
  const packetAge =
    snapshot.lastPacketAt === null ? null : nowMs - snapshot.lastPacketAt;
  const healthColor =
    overview.health === 'fault'
      ? theme.colors.danger
      : overview.health === 'healthy'
        ? theme.colors.text
        : theme.colors.muted;
  const holds = bitLabels('holdMask', lastStatus?.values.hold_mask);
  const holdText = lastStatus === undefined ? '--' : holds.length === 0 ? 'none' : holds.join(', ').toLowerCase();
  const linkColor =
    snapshot.status === 'open' ? theme.colors.text : theme.colors.muted;
  const bridge = snapshot.bridge;
  const bridgeValue =
    bridge?.mqtt === 'connected'
      ? 'UP'
      : bridge?.mqtt === 'disconnected'
        ? 'DOWN'
        : '--';
  const bridgeColor =
    bridge?.mqtt === 'connected'
      ? theme.colors.text
      : bridge?.mqtt === 'disconnected'
        ? theme.colors.danger
        : theme.colors.muted;

  return (
    <View
      style={[
        styles.rail,
        {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
        },
      ]}
    >
      <View style={styles.metrics}>
        <Metric label="LINK" value={snapshot.status.toUpperCase()} color={linkColor} />
        <Metric label="BRIDGE" value={bridgeValue} color={bridgeColor} />
        <Metric label="STATE" value={enumLabel('runState', lastStatus?.values.state)} color={theme.colors.text} />
        <Metric label="MODE" value={enumLabel('vehicleMode', lastStatus?.values.mode)} color={theme.colors.text} />
        <Metric
          label="SPEED"
          value={lastStatus === undefined ? '--' : `${lastStatus.values.speed_mms} mm/s`}
          color={theme.colors.text}
        />
        <Metric label="HOLD" value={holdText} color={holds.length > 0 ? theme.colors.text : theme.colors.muted} />
        <Metric label="HEALTH" value={overview.health.toUpperCase()} color={healthColor} />
        <Metric
          label="FAULTS"
          value={faultHex(lastStatus?.values.faults)}
          color={(lastStatus?.values.faults ?? 0) === 0 ? theme.colors.text : theme.colors.danger}
        />
        <Metric label="HEARTBEAT" value={formatAge(heartbeatAge)} color={theme.colors.text} />
        <Metric label="PACKET" value={formatAge(packetAge)} color={theme.colors.text} />
        <Metric
          label="RSSI"
          value={lastHeartbeat === undefined ? '--' : `${lastHeartbeat.values.wifi_rssi ?? '--'} dBm`}
          color={theme.colors.text}
        />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  rail: {
    borderWidth: 1,
    borderRadius: 16,
    gap: 12,
    paddingHorizontal: 16,
    paddingVertical: 14,
  },
  metrics: { flexDirection: 'row', flexWrap: 'wrap', gap: 12 },
  metric: { flexBasis: 80, flexGrow: 1, gap: 4, minWidth: 76 },
  metricValue: { fontVariant: ['tabular-nums'], fontWeight: '600' },
});
