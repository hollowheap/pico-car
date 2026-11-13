import { StyleSheet, Text, View } from 'react-native';

import type { Overview } from '../overview/overview';
import { faultTiles } from '../../lib/faults';
import { useTheme } from '../../lib/theme-provider';

type Props = {
  overview: Overview;
  faults: number | undefined;
};

const BIT_ESTOP = 9;
const BIT_LIFTED = 11;
const BIT_OVERTURNED = 12;
const BIT_OBSTACLE_STOPPED = 13;
const BIT_TASK_STALLED = 14;

const CONNECTION_LOSS_NOTICE_MS = 5000;

export type BannerMessage = { title: string; action: string };

/** A short gap keeps the last readings visible and uses a quieter message. */
export function bannerMessage(overview: Overview, faults: number | undefined): BannerMessage | null {
  const set = (bit: number) => faults !== undefined && (faults & (1 << bit)) !== 0;
  if (overview.health === 'offline') {
    return { title: 'OFFLINE', action: 'No link to the bridge. The numbers below are the last values. Commands are off.' };
  }
  if (overview.health === 'stale') {
    if (overview.statusAgeMs === null) {
      return { title: 'WAITING FOR THE CAR', action: 'The dashboard is connected. Waiting for the first status packet.' };
    }
    if (overview.statusAgeMs < CONNECTION_LOSS_NOTICE_MS) {
      return { title: 'TELEMETRY DELAYED', action: 'Showing the last received readings.' };
    }
    return { title: 'NO RECENT TELEMETRY', action: 'Check that the car is powered and connected to Wi-Fi.' };
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
    return { title: 'TASK STALLED', action: 'A task missed its heartbeat or the car does not move. See Diagnostics.' };
  }
  if (set(BIT_OBSTACLE_STOPPED)) {
    return { title: 'STOPPED BY OBSTACLE', action: 'Clear the path, then send RESET, then START.' };
  }
  return null;
}

/** Keep connection and the highest priority active fault visible in the header. */
export function LiveStatus({ overview, faults }: Props) {
  const { colors, mode, typography } = useTheme();
  const detail = bannerMessage(overview, faults);
  const active = overview.health === 'fault' ? faultTiles(faults, undefined).filter((fault) => fault.state === 'set') : [];
  const top = active.find((fault) => fault.bit === BIT_ESTOP)
    ?? active.find((fault) => fault.bit === BIT_OVERTURNED)
    ?? active.find((fault) => fault.severity === 'critical') ?? active[0];
  const online = overview.health === 'healthy' || overview.health === 'fault';
  const label = online ? 'Live telemetry' : overview.health === 'offline' ? 'Link offline'
    : overview.statusAgeMs === null ? 'Waiting for car'
    : overview.statusAgeMs < CONNECTION_LOSS_NOTICE_MS ? 'Telemetry delayed' : 'Telemetry stale';
  const dot = online ? (mode === 'dark' ? '#69d5a2' : '#137a50') : colors.warning;

  return (
    <View accessibilityLiveRegion={top ? 'assertive' : 'polite'} style={styles.row} testID="live-status">
      <View accessibilityLabel={online ? label : detail?.action ?? label} style={styles.link}>
        <View style={[styles.dot, { backgroundColor: dot }]} />
        <Text style={[typography.caption, { color: colors.text, fontWeight: '600' }]}>{label}</Text>
      </View>
      {top ? (
        <View
          accessibilityLabel={top.label + '. ' + (detail?.action ?? top.hint)}
          style={[styles.fault, { backgroundColor: top.severity === 'critical' ? colors.dangerSurface : colors.elevated }]}
          testID="header-fault"
        >
          <Text style={[typography.caption, { color: top.severity === 'critical' ? colors.danger : colors.warning, fontWeight: '600' }]}>
            {top.label}{active.length > 1 ? ' +' + (active.length - 1) : ''}
          </Text>
        </View>
      ) : null}
    </View>
  );
}

const styles = StyleSheet.create({
  row: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 6 },
  link: { alignItems: 'center', flexDirection: 'row', gap: 6, paddingVertical: 6 },
  dot: { borderRadius: 4, height: 8, width: 8 },
  fault: { borderRadius: 12, paddingHorizontal: 8, paddingVertical: 5 },
});
