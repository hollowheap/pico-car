import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { Panel } from './Panel';

type Props = {
  system: LinkPacket | undefined;
  style?: StyleProp<ViewStyle>;
};

const FLAG_ACTIVE = 1 << 0;
const FLAG_FINISHED = 1 << 1;
const FLAG_HEIGHT_VALID = 1 << 2;

function valueOr(value: number | undefined, unit: string): string {
  return value === undefined ? '--' : `${value} ${unit}`;
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
  ]
    .filter(Boolean)
    .join(', ');

  return (
    <Panel
      style={style}
      title="Onboard state"
      right={
        <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
          B4 / B5
        </Text>
      }
    >
      <View style={styles.rows}>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-obstacle"
        >
          obstacle: state {values?.obstacle_state ?? '--'}, distance{' '}
          {valueOr(values?.obstacle_dist_mm, 'mm')}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-echo"
        >
          echo class {values?.echo_class ?? '--'}, faults{' '}
          {values?.obstacle_faults ?? 0}, seq {values?.obstacle_seq ?? '--'}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-hump"
        >
          hump: state {values?.physics_state ?? '--'}, motion{' '}
          {values?.physics_motion ?? '--'}, peak{' '}
          {valueOr(values?.hump_peak_mm, 'mm')}
        </Text>
        <Text
          style={[theme.typography.body, { color: theme.colors.text }]}
          testID="system-attitude"
        >
          pitch {pitch}, faults {values?.physics_faults ?? 0}
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
