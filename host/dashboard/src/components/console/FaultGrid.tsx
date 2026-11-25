import { useState } from 'react';
import { Pressable, StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { ConsoleView } from '../../lib/console-view';
import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { bitLabels, enumLabel } from '../../lib/schema';
import type { DiagRates } from '../../../../bridge/src/console/telemetry-store.js';
import { useTheme } from '../../lib/theme-provider';
import { MetricCell } from './MetricCell';
import { Panel, useExpanded } from './Panel';
import { SectionPicker } from './SectionPicker';
import { StatusPill } from './StatusPill';

type Props = {
  level: number | undefined;
  tiles: ConsoleView['faults'];
  obstacle?: LinkPacket;
  diag?: LinkPacket;
  rates?: DiagRates;
  lossPercent?: number | null;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

const VIEWS = { matrix: 'Matrix', health: 'Health' } as const;

const SHORT_LABEL: Record<number, string> = {
  0: 'Wi-Fi',
  1: 'MQTT',
  2: 'Stall',
  3: 'Encoder',
  4: 'Line',
  5: 'IMU',
  6: 'Speed',
  7: 'Tilt',
  8: 'Sonar',
  9: 'E-stop',
  10: 'Echo',
  11: 'Lifted',
  12: 'Flip',
  13: 'Obstacle',
  14: 'Task',
  15: 'Reset',
};

const STATE_LABEL: Record<ConsoleView['faults'][number]['state'], string> = {
  set: 'Active',
  'was-set': 'Previously set',
  ok: 'OK',
  unknown: 'Unknown',
};

const TASKS = ['Motion', 'Line', 'Obstacle', 'Physics', 'Telem', 'Conn', 'Barcode'] as const;

export function FaultGrid({ level, tiles, obstacle, diag, rates, lossPercent, status, style }: Props) {
  const theme = useTheme();
  const expanded = useExpanded();
  const [view, setView] = useState<keyof typeof VIEWS>('matrix');
  const [selectedBit, setSelectedBit] = useState<number | null>(null);
  const [hoveredBit, setHoveredBit] = useState<number | null>(null);
  const [focusedBit, setFocusedBit] = useState<number | null>(null);
  const showMatrix = expanded || view === 'matrix';
  const inspected = showMatrix ? tiles.find(tile => tile.bit === (hoveredBit ?? focusedBit ?? selectedBit)) : undefined;
  const o = obstacle?.values;
  const d = diag?.values;
  const obstacleFaults = o === undefined ? [] : bitLabels('obstacleFaults', o.faults);
  const stopped = o !== undefined && o.state === 0 && o.cause !== 0;
  const stopCause = stopped ? enumLabel('obstacleCause', o.cause) : null;
  const activeCount = tiles.filter((t) => t.state === 'set').length;
  const historyCount = tiles.filter((t) => t.state === 'was-set').length;

  const echoValid = rates?.echo_valid ?? d?.echo_valid ?? 0;
  const echoOpen = rates?.echo_open ?? d?.echo_open ?? 0;
  const echoBad =
    (rates?.echo_too_close ?? d?.echo_too_close ?? 0) +
    (rates?.echo_no_echo ?? d?.echo_no_echo ?? 0) +
    (rates?.echo_glitch ?? d?.echo_glitch ?? 0) +
    (rates?.echo_stuck ?? d?.echo_stuck ?? 0);
  const echoTotal = echoValid + echoOpen + echoBad;

  const statusLabel = stopCause ? `${stopCause} STOP` : level === undefined ? 'NO DATA'
    : activeCount > 0 ? `${activeCount} ACTIVE` : historyCount > 0 ? `${historyCount} PAST` : 'CLEAR';
  const statusTone = stopCause || activeCount > 0 ? 'danger' : level === undefined ? 'muted'
    : historyCount > 0 ? 'warning' : 'success';
  const detail = inspected ? `${inspected.label}: ${inspected.hint}` : obstacleFaults.length > 0
    ? `Sonar: ${obstacleFaults.join(', ').toLowerCase()}`
    : showMatrix ? 'Select a fault for details' : o === undefined ? 'Waiting for sonar status' : 'No sonar fault flags';
  return (
    <Panel
      fill
      empty={level === undefined}
      icon="faults"
      status={status}
      style={style}
      testID="fault-grid"
      title="Faults"
      right={
        <View style={styles.headerMeta}>
          <View style={styles.headerStatus} testID={stopCause ? 'fault-stop-cause' : undefined}>
            <StatusPill label={statusLabel} tone={statusTone} />
          </View>
          {expanded ? null : <SectionPicker id="faults-view" label="Faults views" options={VIEWS} value={view} onChange={setView} />}
        </View>
      }
    >
      {showMatrix ? (
        <View style={styles.matrix} testID="fault-matrix">
          {tiles.map((tile) => {
            const active = tile.state === 'set';
            const critical = active && tile.severity === 'critical';
            const warn = active && tile.severity === 'warn';
            const past = tile.state === 'was-set';
            const unknown = tile.state === 'unknown';
            const selected = inspected?.bit === tile.bit;
            const borderColor = selected
              ? theme.colors.text
              : critical
                ? theme.colors.danger
                : warn
                  ? theme.colors.warning
                  : past
                    ? theme.colors.info
                    : theme.colors.border;
            const dotColor = critical
              ? theme.colors.danger
              : warn
                ? theme.colors.warning
                : past
                  ? theme.colors.info
                  : unknown
                    ? theme.colors.border
                    : theme.colors.success;
            const textColor = critical
              ? theme.colors.danger
              : warn
                ? theme.colors.warning
                : past
                  ? theme.colors.info
                  : theme.colors.muted;
            return (
              <Pressable
                key={tile.bit}
                testID={`fault-tile-${tile.bit}`}
                accessibilityLabel={`${tile.label}, ${STATE_LABEL[tile.state]}${active ? ', ' + tile.severity : ''}`}
                accessibilityRole="button"
                onHoverIn={() => setHoveredBit(tile.bit)}
                onHoverOut={() => setHoveredBit(current => current === tile.bit ? null : current)}
                onFocus={() => setFocusedBit(tile.bit)}
                onBlur={() => setFocusedBit(current => current === tile.bit ? null : current)}
                onPress={() => setSelectedBit(current => current === tile.bit ? null : tile.bit)}
                style={[
                  styles.cell,
                  {
                    borderColor,
                    backgroundColor: critical
                      ? theme.colors.dangerSurface
                      : warn
                        ? theme.colors.warningSurface
                        : past
                          ? theme.colors.infoSurface
                          : 'transparent',
                    opacity: unknown ? 0.55 : 1,
                  },
                ]}
              >
                <View style={[styles.dot, { backgroundColor: dotColor, opacity: active || past ? 1 : 0.35 }]} />
                <Text numberOfLines={1} style={[theme.typography.caption, styles.cellLabel, { color: textColor }]}>
                  {SHORT_LABEL[tile.bit] ?? tile.label}
                </Text>
              </Pressable>
            );
          })}
        </View>
      ) : null}
      {expanded || view === 'health' ? (
        <View style={[styles.health, expanded && [styles.foldSection, { borderTopColor: theme.colors.border }]]} testID="fault-health">
          <View style={styles.taskRow}>
            {TASKS.map((name, bit) => {
              const mask = d?.task_alive_mask;
              const alive = mask !== undefined && ((mask >> bit) & 1) !== 0;
              return (
                <View
                  key={name}
                  style={[
                    styles.taskChip,
                    {
                      borderColor: mask === undefined ? theme.colors.border : alive ? theme.colors.success : theme.colors.danger,
                      backgroundColor: mask === undefined ? 'transparent' : alive ? theme.colors.successSurface : theme.colors.dangerSurface,
                      opacity: mask === undefined ? 0.55 : 1,
                    },
                  ]}
                >
                  <Text style={[theme.typography.caption, styles.cellLabel, {
                    color: mask === undefined ? theme.colors.muted : alive ? theme.colors.success : theme.colors.danger,
                  }]}>
                    {name}
                  </Text>
                </View>
              );
            })}
          </View>
          <View style={styles.metricRow}>
            <MetricCell
              label="Stack min"
              value={d?.stack_min_free === undefined ? '--' : `${d.stack_min_free} B`}
              tone={d?.stack_min_free !== undefined && d.stack_min_free < 256 ? 'danger' : 'default'}
            />
            <MetricCell
              label="Q drops"
              value={d?.telem_queue_drops === undefined ? '--' : `${d.telem_queue_drops}/${d.net_queue_drops ?? 0}`}
              tone={(d?.telem_queue_drops ?? 0) > 0 || (d?.net_queue_drops ?? 0) > 0 ? 'danger' : 'default'}
            />
            <MetricCell
              label="Frame loss"
              value={lossPercent === undefined || lossPercent === null ? '--' : `${lossPercent.toFixed(1)}%`}
              tone={lossPercent !== undefined && lossPercent !== null && lossPercent >= 5 ? 'danger' : 'default'}
            />
            <MetricCell
              label="Cmd rej"
              value={d === undefined ? '--' : String((d.cmd_rej_length ?? 0) + (d.cmd_rej_opcode ?? 0) + (d.cmd_rej_run ?? 0) + (d.cmd_rej_old_id ?? 0) + (d.cmd_rej_age ?? 0))}
            />
          </View>
          <View style={styles.echoRow}>
            <View style={styles.echoHead}>
              <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>Sonar echo quality</Text>
              <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
                {echoTotal > 0 ? `${echoValid} ok · ${echoOpen} open · ${echoBad} bad` : '--'}
              </Text>
            </View>
            <View style={[styles.echoTrack, { backgroundColor: theme.colors.border }]}>
              {echoTotal > 0 ? (
                <>
                  <View style={{ flex: echoValid, backgroundColor: theme.colors.success }} />
                  <View style={{ flex: echoOpen, backgroundColor: theme.colors.info }} />
                  <View style={{ flex: echoBad, backgroundColor: theme.colors.danger }} />
                </>
              ) : null}
            </View>
          </View>
        </View>
      ) : null}
      <View style={[styles.detail, { height: theme.typography.caption.lineHeight * 2 }]} testID="fault-detail">
        <Text accessibilityLiveRegion="polite" numberOfLines={2} style={[theme.typography.caption, {
          color: inspected ? theme.colors.text : obstacleFaults.length > 0 ? theme.colors.danger : theme.colors.muted,
        }]}>{detail}</Text>
      </View>
    </Panel>
  );
}

const styles = StyleSheet.create({
  headerMeta: { flexDirection: 'row', alignItems: 'center', gap: 8, flexShrink: 1, minWidth: 0 },
  headerStatus: { flexShrink: 1, minWidth: 0 },
  detail: { flexShrink: 0, justifyContent: 'center', marginTop: 'auto' },
  matrix: { flex: 1, minHeight: 0, flexDirection: 'row', flexWrap: 'wrap', alignContent: 'stretch', gap: 4 },
  cell: {
    flexBasis: '23%',
    flexGrow: 1,
    minHeight: 26,
    minWidth: 0,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 4,
    paddingHorizontal: 6,
    paddingVertical: 5,
    borderRadius: 5,
    borderWidth: 1,
  },
  dot: { width: 5, height: 5, borderRadius: 3, flexShrink: 0 },
  cellLabel: { flexShrink: 1 },
  health: { gap: 8 },
  foldSection: { borderTopWidth: 1, paddingTop: 8 },
  taskRow: { flexDirection: 'row', flexWrap: 'wrap', gap: 4 },
  taskChip: { paddingHorizontal: 6, paddingVertical: 2, borderRadius: 5, borderWidth: 1 },
  metricRow: { flexDirection: 'row', gap: 8 },
  echoRow: { gap: 3 },
  echoHead: { flexDirection: 'row', justifyContent: 'space-between', gap: 8 },
  echoTrack: { flexDirection: 'row', height: 5, borderRadius: 3, overflow: 'hidden' },
});
