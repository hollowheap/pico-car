import { StyleSheet, Text, View } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { bitLabels, enumLabel } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import type { OverviewHealth } from '../overview/overview';
import { StatusPill } from './StatusPill';

export function StatusRail({ packet, health, compact, hasNotice }: {
  hasNotice: boolean;
  packet?: LinkPacket;
  health: OverviewHealth;
  compact: boolean;
}) {
  const theme = useTheme();
  const current = health === 'healthy' || health === 'fault';
  const values = current ? packet?.values : undefined;
  const mode = values === undefined ? null : enumLabel('vehicleMode', values.mode);
  const state = values === undefined ? null : enumLabel('runState', values.state);
  const holds = bitLabels('holdMask', values?.hold_mask);
  const caption = [theme.typography.caption, styles.fit, { color: theme.colors.muted }];
  const awaiting = health === 'stale' && packet === undefined;

  return (
    <View accessibilityLabel="Car status" accessibilityRole="summary" style={[styles.rail, compact ? styles.compact : null]}>
      <View style={styles.reading}>
        {mode === null || hasNotice ? null : <Text numberOfLines={1} style={[theme.typography.label, styles.fit, { color: theme.colors.text }]} testID="vehicle-mode">{mode}</Text>}
        {state === null || state === mode || hasNotice || (state === 'READY' && holds.length > 0) ? null : (
          <View style={styles.fit}><StatusPill label={state} tone={state === 'RUNNING' ? 'active' : 'neutral'} /></View>
        )}
        {health === 'stale' ? (
          <View accessibilityLabel={awaiting ? 'Awaiting status. Vehicle commands unavailable.' : 'Telemetry stale. Vehicle commands unavailable.'}
            accessibilityLiveRegion="polite" aria-live="polite" role="status">
            <StatusPill label={awaiting ? 'Awaiting status' : 'STALE'} tone="muted" />
          </View>
        ) : null}
      </View>
      {holds.length > 0 && (!hasNotice || holds.length > 1) ? <Text numberOfLines={1} style={caption}>Hold {holds.join(', ').toLowerCase()}</Text> : null}
    </View>
  );
}

const styles = StyleSheet.create({
  rail: { alignItems: 'center', flex: 1, flexDirection: 'row', gap: 16, minWidth: 0 },
  compact: { alignItems: 'stretch', flexDirection: 'column', gap: 4 },
  reading: { alignItems: 'center', flexDirection: 'row', flexShrink: 1, gap: 8, minWidth: 0 },
  fit: { flexShrink: 1 },
});
