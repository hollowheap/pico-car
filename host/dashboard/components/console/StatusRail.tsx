import type { ReactNode } from 'react';
import { StyleSheet, Text, View } from 'react-native';

import type { LinkPacket, LinkSnapshot } from '../../lib/link';
import { heartbeatAgeMs } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import type { Overview } from '../overview/overview';
import { bitLabels, enumLabel } from '../../lib/schema';
import { faultHex, formatAge } from './format';
import { StatusPill, type PillTone } from './StatusPill';

type Props = {
  snapshot: LinkSnapshot;
  overview: Overview;
  /** The newest packet of each kind, by packet name. */
  latest: Record<string, LinkPacket>;
  nowMs: number;
  /** The banner, shown at the right end of the rail so it never pushes the page down. */
  aside?: ReactNode;
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
      <Text style={[theme.typography.body, styles.number, { color }]}>{value}</Text>
    </View>
  );
}

export function StatusRail({ snapshot, overview, latest, nowMs, aside }: Props) {
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

  const mode = enumLabel('vehicleMode', lastStatus?.values.mode);
  const state = enumLabel('runState', lastStatus?.values.state);
  const speed = lastStatus?.values.speed_mms;
  const healthTone: PillTone =
    overview.health === 'fault' ? 'danger' : overview.health === 'healthy' ? 'active' : 'muted';

  return (
    <View
      accessibilityLabel="Car status"
      accessibilityRole="summary"
      style={[
        styles.rail,
        {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.card,
        },
      ]}
    >
      <View style={styles.hero}>
        <Text style={[theme.typography.label, { color: theme.colors.muted }]}>MODE</Text>
        <Text style={[theme.typography.heading, styles.mode, { color: theme.colors.text }]}>
          {lastStatus === undefined ? '--' : mode}
        </Text>
        <View style={styles.pills}>
          <StatusPill label={lastStatus === undefined ? 'NO DATA' : state} tone={state === 'RUNNING' ? 'active' : 'neutral'} />
          <StatusPill label={overview.health.toUpperCase()} tone={healthTone} />
        </View>
      </View>
      <View style={[styles.speed, { borderColor: theme.colors.border }]}>
        <Text style={[theme.typography.label, { color: theme.colors.muted }]}>SPEED</Text>
        <Text style={[theme.typography.headingLg, styles.number, { color: theme.colors.text }]}>
          {speed ?? '--'}
          <Text style={[theme.typography.body, { color: theme.colors.muted }]}> mm/s</Text>
        </Text>
      </View>
      <View style={styles.metrics}>
        <Metric label="LINK" value={snapshot.status.toUpperCase()} color={linkColor} />
        <Metric label="BRIDGE" value={bridgeValue} color={bridgeColor} />
        <Metric label="HOLD" value={holdText} color={holds.length > 0 ? theme.colors.text : theme.colors.muted} />
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
      {aside}
    </View>
  );
}

const styles = StyleSheet.create({
  hero: { gap: 4, minWidth: 180 },
  mode: { letterSpacing: 0.5 },
  pills: { flexDirection: 'row', flexWrap: 'wrap', gap: 6, marginTop: 4 },
  speed: { borderLeftWidth: 1, gap: 2, minWidth: 170, paddingLeft: 20 },
  number: { fontVariant: ['tabular-nums'] },
  rail: {
    alignItems: 'center',
    borderWidth: 1,
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 20,
    paddingHorizontal: 20,
    paddingVertical: 16,
  },
  metrics: { flex: 1, flexDirection: 'row', flexWrap: 'wrap', gap: 20, minWidth: 260 },
  metric: { gap: 2, minWidth: 84 },
});
