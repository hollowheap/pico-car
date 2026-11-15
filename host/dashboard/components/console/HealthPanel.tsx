import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { useTheme } from '../../lib/theme-provider';
import { Panel, useDense } from './Panel';

type Props = {
  diag: LinkPacket | undefined;
  motion: LinkPacket | undefined;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** Liveness bits of include/supervisor/health_ids.h. */
const TASKS = ['Motion', 'Line', 'Obstacle', 'Physics', 'Telemetry', 'Connection', 'Barcode'];
/** Stack order of src/main.c (g_stack_base), for the diag stack mark. */
const STACKS = ['startup', 'motion', 'physics', 'obstacle', 'line', 'telemetry', 'connection', 'barcode', 'supervisor'];
/** Every task keeps at least 25 % of its 4096-byte stack free. */
const STACK_MIN_FREE = 1024;
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
  const dense = useDense();
  const text = dense ? theme.typography.caption : theme.typography.body;

  return (
    <Panel icon="tasks" status={status} style={style} testID="health-panel" title="Tasks and timing">
      <View style={styles.tasks}>
        {TASKS.map((name, bit) => {
          const alive = mask === undefined ? undefined : (mask & (1 << bit)) !== 0;
          return (
            <View
              accessibilityLabel={`${name} task ${alive === undefined ? 'no data' : alive ? 'alive' : 'no pulse'}`}
              accessible
              key={name}
              style={[styles.task, dense ? styles.taskDense : null, { borderRadius: theme.radii.input }]}
            >
              <Text style={[text, { color: theme.colors.text }]}>{name}</Text>
              <Text style={[theme.typography.label, { color: alive === false ? theme.colors.danger : theme.colors.muted }]}>
                {alive === undefined ? 'NO DATA' : alive ? 'ALIVE' : 'NO PULSE'}
              </Text>
            </View>
          );
        })}
      </View>
      {dense ? null : (
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
          A task without a pulse has not started yet or missed its heartbeat for 500 ms.
        </Text>
      )}
      <Text style={[text, { color: theme.colors.text }]}>
        Missed ticks: motion {motion?.values.overruns ?? '--'} · physics {d?.physics_overruns ?? '--'} · telemetry {d?.telemetry_overruns ?? '--'}
        {' '}· supervisor passes {d?.supervisor_passes ?? '--'}
      </Text>
      <Text
        style={[
          text,
          { color: d?.stack_min_free !== undefined && d.stack_min_free < STACK_MIN_FREE ? theme.colors.danger : theme.colors.text },
        ]}
      >
        Smallest free stack since boot:{' '}
        {d?.stack_min_free === undefined || d.stack_min_free === 0xffff
          ? '--'
          : `${d.stack_min_free} B (${STACKS[d.stack_min_task ?? 0] ?? `task ${d.stack_min_task}`})`}
      </Text>
      <Text style={[text, { color: theme.colors.text }]}>
        Sonar echoes per second: {ECHOES.map(([key, label]) => `${label} ${d?.[key] ?? '--'}`).join(' · ')}
      </Text>
    </Panel>
  );
}

const styles = StyleSheet.create({
  tasks: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  task: { flexBasis: '30%', flexGrow: 1, gap: 2, minWidth: 100, padding: 8 },
  taskDense: { flexBasis: '22%', gap: 0, minWidth: 64, padding: 2 },
});
