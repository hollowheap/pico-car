import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import { FAULT_META } from '../../lib/faults';
import type { PanelStatus } from '../../lib/panel-state';
import type { RunSummary } from '../../lib/run-summary';
import { useTheme } from '../../lib/theme-provider';
import { Panel } from './Panel';

type Props = {
  summary: RunSummary;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

const SECONDS_PER_MINUTE = 60;

function duration(seconds: number): string {
  const whole = Math.round(seconds);
  return `${Math.floor(whole / SECONDS_PER_MINUTE)}:${String(whole % SECONDS_PER_MINUTE).padStart(2, '0')}`;
}

function Item({ label, value }: { label: string; value: string }) {
  const theme = useTheme();
  return (
    <View style={styles.item}>
      <Text style={[theme.typography.label, { color: theme.colors.muted }]}>{label}</Text>
      <Text style={[theme.typography.heading, { color: theme.colors.text }]}>{value}</Text>
    </View>
  );
}

/**
 * One summary of the latest run: from the last run start (or the first sample
 * the dashboard has) to the newest. The wire has no run id, so there is no list
 * of runs and no comparison.
 */
export function RunSummaryPanel({ summary, status, style }: Props) {
  const theme = useTheme();
  const faultNames = summary.faultBits.map((bit) => FAULT_META[bit]?.label ?? `bit ${bit}`);

  return (
    <Panel status={status} style={style} testID="run-summary" title="Latest run">
      <View style={styles.grid}>
        <Item label="DISTANCE" value={`${(summary.distanceMm / 1000).toFixed(2)} m`} />
        <Item label="TIME" value={duration(summary.durationS)} />
        <Item label="AVERAGE" value={`${Math.round(summary.averageMms)} mm/s`} />
        <Item label="TURNS" value={String(summary.turns)} />
        <Item label="BARCODES" value={`${summary.barcodesOk} read, ${summary.barcodesRefused} refused`} />
        <Item label="OBSTACLES" value={`${summary.obstacles} seen, ${summary.bypasses} passed`} />
        <Item label="HUMPS" value={`${summary.humps}${summary.humps > 0 ? `, tallest ${summary.tallestHumpMm} mm` : ''}`} />
        <Item label="LINE LOST" value={`${summary.lineLosses} times`} />
        <Item label="LIFTED" value={`${summary.lifts} times`} />
        <Item label="STOPS" value={String(summary.stops)} />
      </View>
      <Text style={[theme.typography.body, { color: summary.faults > 0 ? theme.colors.text : theme.colors.muted }]}>
        {summary.faults === 0 ? 'No fault was set.' : `${summary.faults} fault events: ${faultNames.join(', ')}.`}
      </Text>
    </Panel>
  );
}

const styles = StyleSheet.create({
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 16 },
  item: { flexBasis: 150, flexGrow: 1, gap: 2 },
});
