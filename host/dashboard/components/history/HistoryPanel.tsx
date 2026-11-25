import { StyleSheet, Text, View } from 'react-native';

import type { HistoryRecord } from '../../lib/history';
import { useTheme } from '../../lib/theme-provider';
import { formatTime } from '../console/format';
import { summarizeHistory } from './history';

type Props = {
  records: HistoryRecord[];
};

export function HistoryPanel({ records }: Props) {
  const summary = summarizeHistory(records);
  const theme = useTheme();

  return (
    <View
      style={[
        styles.card,
        {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.card,
          padding: theme.spacing.lg,
        },
      ]}
    >
      <Text
        style={[theme.typography.headingSm, { color: theme.colors.text }]}
      >
        Runs and frames
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]} testID="history-total">
        total: {summary.total}, showing {summary.rows.length} of {summary.total}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.muted }]} testID="history-counts">
        {Object.entries(summary.counts)
          .map(([packet, count]) => `${packet} ${count}`)
          .join('  ')}
      </Text>
      {summary.rows.map((row, index) => (
        <View key={`${row.timestamp}-${row.packet}-${index}`} style={styles.row}>
          <View style={styles.rowHead}>
            <Text
              style={[
                theme.typography.body,
                { color: theme.colors.text, fontWeight: '600' },
              ]}
            >
              {row.packet}
            </Text>
            <Text
              style={[
                theme.typography.caption,
                { color: theme.colors.muted },
              ]}
            >
              {formatTime(row.timestamp)}
            </Text>
          </View>
          <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
            {row.detail}
          </Text>
        </View>
      ))}
    </View>
  );
}

const styles = StyleSheet.create({
  card: { gap: 8, borderWidth: 1 },
  row: { gap: 2 },
  rowHead: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
});
