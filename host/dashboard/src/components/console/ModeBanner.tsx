import { StyleSheet, Text, View } from 'react-native';

import type { ConsoleView } from '../../lib/console-view';
import { FAULT_BITS } from '../../lib/schema.generated';
import { useTheme } from '../../lib/theme-provider';
import type { Overview } from '../overview/overview';

const PICKUP_STOP_TITLE = 'STOPPED AFTER PICKUP';

export type BannerMessage = { title: string; action: string };

export function currentHoldMessage(hold: ConsoleView['hold'], nowMs: number): BannerMessage | null {
  if (hold === null || hold.freshness === undefined) return hold;
  const { from, through, fallback } = hold.freshness;
  return nowMs >= from && nowMs <= through ? { title: hold.title, action: hold.action } : fallback;
}

/** The one message that must not scroll away, with what to do. Null for none. */
export function bannerMessage(overview: Overview, faults: number | undefined, holdMessage: BannerMessage | null = null): BannerMessage | null {
  if (overview.health === 'offline') {
    return { title: 'OFFLINE', action: 'Telemetry link unavailable. The displayed readings are from the last update. Commands are off.' };
  }
  const report = modeReport(faults, holdMessage);
  return report !== null && overview.health === 'stale'
    ? { title: `LAST REPORTED ${report.title}`, action: 'Waiting for fresh vehicle status.' }
    : report;
}

function modeReport(faults: number | undefined, holdMessage: BannerMessage | null): BannerMessage | null {
  const set = (bit: number) => faults !== undefined && (faults & (1 << bit)) !== 0;
  if (set(FAULT_BITS.estopLatched)) {
    return { title: 'EMERGENCY STOP', action: 'The stop is latched until the car reboots. Switch the car off and on.' };
  }
  if (set(FAULT_BITS.overturned)) {
    return { title: 'OVERTURNED', action: 'Set the car down upright and still, then press Start.' };
  }
  if (set(FAULT_BITS.lifted)) {
    if (holdMessage?.title === PICKUP_STOP_TITLE) return holdMessage;
    return { title: 'PICKUP DETECTED', action: 'Set down, then press Start.' };
  }
  if (set(FAULT_BITS.taskStalled)) {
    return { title: 'TASK STALLED', action: 'Check wheel motion and the missed ticks in the navbar.' };
  }
  if (set(FAULT_BITS.obstacleStopped)) {
    return { title: 'STOPPED BY OBSTACLE', action: 'Clear the path, then press Start.' };
  }
  return holdMessage;
}

/** Persistent recovery guidance for connection issues and reported safety conditions. */
export function ModeBanner({ message, offline, compact = false }: { message: BannerMessage | null; offline: boolean; compact?: boolean }) {
  const theme = useTheme();
  if (message === null) return null;

  return (
    <View
      accessibilityLiveRegion="assertive"
      accessibilityRole="alert"
      style={[
        styles.banner,
        compact ? styles.bannerCompact : null,
      ]}
      testID="mode-banner"
    >
      <Text numberOfLines={compact ? 1 : undefined}
        style={[theme.typography.label, styles.noticeText, { color: offline ? theme.colors.muted : message.title === 'UPRIGHT' ? theme.colors.text : theme.colors.danger }]}>{message.title}</Text>
      <Text numberOfLines={compact ? 2 : undefined} style={[compact ? theme.typography.caption : theme.typography.body, styles.noticeText, { color: theme.colors.text }]}>
        {message.action}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  noticeText: { textAlign: 'right' },
  // Plain text at the end of the rail: a box here is a card inside a card.
  banner: { flexBasis: 320, flexGrow: 1, gap: 2, maxWidth: 420 },
  // In the wide header the notice must not take width from the metrics.
  bannerCompact: { flexBasis: 'auto', flexGrow: 0, flexShrink: 1, maxWidth: '100%' },
});
