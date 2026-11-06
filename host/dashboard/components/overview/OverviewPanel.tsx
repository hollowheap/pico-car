import { StyleSheet, Text, View } from 'react-native';

import type { LinkSnapshot } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { buildOverview } from './overview';

type Props = {
  snapshot: LinkSnapshot;
  nowMs: number;
};

export function OverviewPanel({ snapshot, nowMs }: Props) {
  const overview = buildOverview(snapshot, nowMs);
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
        Live state
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]} testID="overview-health">
        health: {overview.health}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]} testID="overview-link">
        link: {overview.status}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.muted }]} testID="overview-heartbeat">
        heartbeat:{' '}
        {overview.heartbeatAgeMs === null
          ? 'none'
          : `${overview.heartbeatAgeMs} ms`}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]} testID="overview-speed">
        speed:{' '}
        {overview.speedMms === null ? 'n/a' : `${overview.speedMms} mm/s`}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]} testID="overview-faults">
        faults:{' '}
        {overview.faults.length === 0 ? 'none' : overview.faults.join(', ')}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  card: { gap: 6, borderWidth: 1 },
});
