import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { bitLabels, enumLabel } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import { Panel } from './Panel';

type Props = {
  system: LinkPacket | undefined;
  style?: StyleProp<ViewStyle>;
};

const FLAG_ACTIVE = 1 << 0;
const FLAG_FINISHED = 1 << 1;
const FLAG_HEIGHT_VALID = 1 << 2;
const FLAG_CALIBRATED = 1 << 3;

function valueOr(value: number | undefined, unit: string): string {
  return value === undefined ? '--' : `${value} ${unit}`;
}

function faultSummary(names: string[] | null): string {
  if (names === null) {
    return '--';
  }
  return names.length === 0 ? 'none' : names.join(', ');
}

export function SystemPanel({ system, style }: Props) {
  const theme = useTheme();
  const values = system?.values;
  const flags = values?.physics_flags ?? 0;
  const pitchCdeg = values?.pitch_cdeg;
  const pitch =
    pitchCdeg === undefined ? '--' : `${(pitchCdeg / 100).toFixed(2)} deg`;
  const stateFlags = [
    flags & FLAG_ACTIVE ? 'active' : null,
    flags & FLAG_FINISHED ? 'finished' : null,
    flags & FLAG_HEIGHT_VALID ? 'height valid' : null,
    flags & FLAG_CALIBRATED ? 'calibrated' : 'calibrating',
  ]
    .filter(Boolean)
    .join(', ');
  const obstacleFaults =
    values?.obstacle_faults === undefined
      ? null
      : bitLabels('obstacleFaults', values.obstacle_faults);
  const physicsFaults =
    values?.physics_faults === undefined
      ? null
      : bitLabels('physicsFaults', values.physics_faults);

  return (
    <Panel
      style={style}
      title="Onboard state"
    >
      <View style={styles.rows}>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-obstacle"
        >
          obstacle: {enumLabel('obstacleState', values?.obstacle_state)}, distance{' '}
          {valueOr(values?.obstacle_dist_mm, 'mm')}, cause{' '}
          {enumLabel('obstacleCause', values?.obstacle_cause)}, req{' '}
          {values?.obstacle_req_id ?? '--'}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-echo"
        >
          echo {enumLabel('echoClass', values?.echo_class)}, faults{' '}
          {faultSummary(obstacleFaults)}, seq {values?.obstacle_seq ?? '--'}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-hump"
        >
          hump: {enumLabel('physicsState', values?.physics_state)}, motion{' '}
          {enumLabel('physicsMotion', values?.physics_motion)}, peak{' '}
          {valueOr(values?.hump_peak_mm, 'mm')}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-attitude"
        >
          pitch {pitch}, faults {faultSummary(physicsFaults)}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.muted }]}
          testID="system-flags"
        >
          {stateFlags.length === 0 ? 'no hump flags' : stateFlags}
        </Text>
      </View>
    </Panel>
  );
}

const styles = StyleSheet.create({
  rows: { gap: 4 },
});
