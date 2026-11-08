import { StyleSheet, Text, View } from 'react-native';

import type { LinkPacket, LinkSnapshot } from '../../lib/link';
import { heartbeatAgeMs } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import type { Overview } from '../overview/overview';
import { faultHex, formatAge, motionName } from './format';

type Props = {
  snapshot: LinkSnapshot;
  overview: Overview;
  packets: LinkPacket[];
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
    <View style={styles.metric}>
      <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
        {label}
      </Text>
      <Text style={[theme.typography.body, { color }]}>{value}</Text>
    </View>
  );
}

export function StatusRail({ snapshot, overview, packets, nowMs }: Props) {
  const theme = useTheme();
  const lastStatus = packets.find((packet) => packet.packet === 'status');
  const lastHeartbeat = packets.find((packet) => packet.packet === 'heartbeat');
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
        <Metric
          label="HEALTH"
          value={overview.health.toUpperCase()}
          color={healthColor}
        />
        <Metric
          label="HEARTBEAT"
          value={formatAge(heartbeatAge)}
          color={theme.colors.text}
        />
        <Metric
          label="PACKET"
          value={formatAge(packetAge)}
          color={theme.colors.text}
        />
        <Metric
          label="STATE"
          value={motionName(lastStatus?.values.state)}
          color={theme.colors.text}
        />
        <Metric
          label="FAULTS"
          value={faultHex(lastStatus?.values.faults)}
          color={
            (lastStatus?.values.faults ?? 0) === 0
              ? theme.colors.text
              : theme.colors.danger
          }
        />
        <Metric
          label="RSSI"
          value={
            lastHeartbeat === undefined
              ? '--'
              : `${lastHeartbeat.values.wifi_rssi ?? '--'} dBm`
          }
          color={theme.colors.text}
        />
        <Metric
          label="UPTIME"
          value={
            lastHeartbeat === undefined
              ? '--'
              : `${lastHeartbeat.values.uptime_s ?? '--'} s`
          }
          color={theme.colors.text}
        />
        <Metric
          label="DROPS"
          value={
            lastHeartbeat === undefined
              ? '--'
              : String(lastHeartbeat.values.drops ?? '--')
          }
          color={theme.colors.text}
        />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  rail: {
    borderBottomWidth: 1,
    gap: 12,
    paddingHorizontal: 16,
    paddingVertical: 14,
  },
  metrics: { flexDirection: 'row', flexWrap: 'wrap', gap: 18 },
  metric: { gap: 2, minWidth: 64 },
});
