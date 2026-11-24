import { Text } from 'react-native';

import type { RunSummary } from '../../lib/run-summary';
import { useTheme } from '../../lib/theme-provider';

type Count = [number | undefined, string, string?];

export function EventCounts({ scope, counts, detail, testID }: {
  scope: RunSummary['scope'];
  counts: Count[];
  detail?: string;
  testID: string;
}) {
  const theme = useTheme();
  const readings = counts.flatMap(([count, singular, plural]) => count === undefined || count === 0
    ? [] : [`${count} ${count === 1 ? singular : plural ?? singular + 's'}`]);
  if (detail !== undefined) readings.push(detail);
  if (readings.length === 0) return null;
  return <Text testID={testID} style={[theme.typography.caption, { color: theme.colors.muted }]}>
    {`${scope}: ${readings.join(', ')}`}
  </Text>;
}
