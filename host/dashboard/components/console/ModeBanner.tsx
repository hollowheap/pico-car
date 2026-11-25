import { StyleSheet, Text, View } from 'react-native';

import type { Overview } from '../overview/overview';
import { useTheme } from '../../lib/theme-provider';

type Props = {
  overview: Overview;
  faults: number | undefined;
  /** Seconds since the last status packet, for the stale text. */
  statusAgeMs: number | null;
};

const BIT_ESTOP = 9;
const BIT_LIFTED = 11;
const BIT_OVERTURNED = 12;
const BIT_OBSTACLE_STOPPED = 13;
const BIT_TASK_STALLED = 14;

export type BannerMessage = { title: string; action: string };

/** The one message that must not scroll away, with what to do. Null for none. */
export function bannerMessage(overview: Overview, faults: number | undefined): BannerMessage | null {
  const set = (bit: number) => faults !== undefined && (faults & (1 << bit)) !== 0;
  if (overview.health === 'offline') {
    return { title: 'OFFLINE', action: 'No link to the bridge. The numbers below are the last values. Commands are off.' };
  }
  if (overview.health === 'stale') {
    return { title: 'STALE', action: 'No status packet from the car. Check the car power and the Wi-Fi.' };
  }
  if (set(BIT_ESTOP)) {
    return { title: 'EMERGENCY STOP', action: 'The stop is latched until the car reboots. Switch the car off and on.' };
  }
  if (set(BIT_OVERTURNED)) {
    return { title: 'OVERTURNED', action: 'Set the car upright, then send RESET, then START.' };
  }
  if (set(BIT_LIFTED)) {
    return { title: 'LIFTED', action: 'The wheels are held. Set the car down, then send START.' };
  }
  if (set(BIT_TASK_STALLED)) {
    return { title: 'TASK STALLED', action: 'A task missed its heartbeat or the car does not move. See Systems.' };
  }
  if (set(BIT_OBSTACLE_STOPPED)) {
    return { title: 'STOPPED BY OBSTACLE', action: 'Clear the path, then send RESET, then START.' };
  }
  return null;
}

/** A persistent banner for offline, stale, emergency, lift and overturn. Never a toast. */
export function ModeBanner({ overview, faults }: Props) {
  const theme = useTheme();
  const message = bannerMessage(overview, faults);
  if (message === null) return null;
  const critical = overview.health !== 'offline' && overview.health !== 'stale';

  return (
    <View
      accessibilityLiveRegion="assertive"
      accessibilityRole="alert"
      style={[
        styles.banner,
        {
          backgroundColor: critical ? theme.colors.dangerSurface : theme.colors.elevated,
          borderColor: critical ? theme.colors.danger : theme.colors.border,
          borderRadius: theme.radii.card,
        },
      ]}
      testID="mode-banner"
    >
      <Text style={[theme.typography.label, { color: critical ? theme.colors.danger : theme.colors.text }]}>{message.title}</Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>{message.action}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  banner: { borderWidth: 1, gap: 2, padding: 12 },
});
