import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { useTheme } from '../../lib/theme-provider';
import { Panel } from './Panel';

type Props = {
  diag: LinkPacket | undefined;
  motion: LinkPacket | undefined;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** Liveness bits of include/supervisor/health_ids.h. */
const TASKS = ['Motion', 'Line', 'Obstacle', 'Physics', 'Telemetry', 'Connection', 'Barcode'];
const ECHOES: [string, string][] = [
  ['echo_valid', 'valid'],
  ['echo_open', 'open'],
  ['echo_too_close', 'too close'],
  ['echo_no_echo', 'no echo'],
  ['echo_glitch', 'glitch'],
  ['echo_stuck', 'stuck'],
];

/** Which tasks show a pulse, how late the loops run, and the sonar echo mix. */
export function HealthPanel({ diag, motion, status, style }: Props) {
  const theme = useTheme();
  const d = diag?.values;
  const mask = d?.task_alive_mask;

  return (
    <Panel status={status} style={style} testID="health-panel" title="Tasks and timing">
      <View style={styles.tasks}>
        {TASKS.map((name, bit) => {
          const alive = mask === undefined ? undefined : (mask & (1 << bit)) !== 0;
          return (
            <View
              accessibilityLabel={`${name} task ${alive === undefined ? 'no data' : alive ? 'alive' : 'no pulse'}`}
              accessible
              key={name}
              style={[styles.task, { borderRadius: theme.radii.input }]}
            >
              <Text style={[theme.typography.body, { color: theme.colors.text }]}>{name}</Text>
              <Text style={[theme.typography.label, { color: alive === false ? theme.colors.danger : theme.colors.muted }]}>
                {alive === undefined ? 'NO DATA' : alive ? 'ALIVE' : 'NO PULSE'}
              </Text>
            </View>
          );
        })}
      </View>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        A task without a pulse has not started yet or missed its heartbeat for 500 ms.
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        Missed ticks: motion {motion?.values.overruns ?? '--'} · physics {d?.physics_overruns ?? '--'} · telemetry {d?.telemetry_overruns ?? '--'}
        {' '}· supervisor passes {d?.supervisor_passes ?? '--'}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        Sonar echoes per second: {ECHOES.map(([key, label]) => `${label} ${d?.[key] ?? '--'}`).join(' · ')}
      </Text>
    </Panel>
  );
}

const styles = StyleSheet.create({
  tasks: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  task: { flexBasis: '30%', flexGrow: 1, gap: 2, minWidth: 100, padding: 8 },
});
