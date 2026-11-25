import { StyleSheet, Text, View } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { STALE_AFTER_MS } from '../../lib/panel-state';
import type { TelemetryStore } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';

// The radio starts at -32768 until sampled. The heartbeat clamps that to int8.
const RSSI_UNAVAILABLE = -128;

function uptime(seconds: number): string {
  const minutes = Math.floor(seconds / 60);
  const hours = Math.floor(minutes / 60);
  if (hours >= 24) return `${Math.floor(hours / 24)}d ${hours % 24}h`;
  if (hours > 0) return `${hours}h ${minutes % 60}m`;
  if (minutes > 0) return `${minutes}m ${seconds % 60}s`;
  return `${seconds}s`;
}

function missedTicks(motion?: number, physics?: number, telemetry?: number): string {
  if (motion === undefined && physics === undefined && telemetry === undefined) return '--';
  if (motion === 0 && physics === 0 && telemetry === 0) return '0';
  return `Motion ${motion ?? '--'}   Physics ${physics ?? '--'}   Telemetry ${telemetry ?? '--'}`;
}

export function NavbarReadings({ latest, connected, nowMs }: {
  latest: TelemetryStore['latest']; connected: boolean; nowMs: number;
}) {
  const theme = useTheme();
  const current = (packet: LinkPacket | undefined) =>
    connected && packet && nowMs - packet.timestamp <= STALE_AFTER_MS[packet.packet]! ? packet.values : undefined;
  const heartbeat = current(latest.heartbeat);
  const signal = heartbeat?.wifi_rssi;
  const motion = current(latest.motion);
  const diag = current(latest.diag);
  const readings = [
    ['nav-signal', 'Signal', signal === undefined || signal === RSSI_UNAVAILABLE ? '--' : `${signal} dBm`],
    ['nav-uptime', 'Car uptime', heartbeat?.uptime_s === undefined ? '--' : uptime(heartbeat.uptime_s)],
    ['nav-ticks', 'Missed ticks', missedTicks(motion?.overruns, diag?.physics_overruns, diag?.telemetry_overruns)],
  ] as const;

  return (
    <View style={styles.readings} testID="navbar-readings">
      {readings.map(([id, label, value]) => (
        <Text key={id} testID={id} style={[theme.typography.caption, { color: theme.colors.muted }]}>
          {label}{'  '}<Text style={[theme.typography.label, styles.value, { color: theme.colors.text }]}>{value}</Text>
        </Text>
      ))}
    </View>
  );
}

const styles = StyleSheet.create({
  readings: { alignItems: 'baseline', flexDirection: 'row', flexWrap: 'wrap', columnGap: 16, rowGap: 2 },
  value: { fontVariant: ['tabular-nums'] },
});
