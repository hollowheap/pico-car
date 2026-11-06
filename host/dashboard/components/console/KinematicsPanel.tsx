import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { Panel } from './Panel';
import { Sparkline } from './Sparkline';

type Props = {
  status: LinkPacket | undefined;
  packets: LinkPacket[];
  style?: StyleProp<ViewStyle>;
};

export function KinematicsPanel({ status, packets, style }: Props) {
  const theme = useTheme();
  const values = status?.values;
  const speeds = packets
    .filter((packet) => packet.packet === 'status')
    .slice(0, 60)
    .reverse()
    .map((packet) => packet.values.speed_mms)
    .filter((value): value is number => typeof value === 'number');

  return (
    <Panel style={style} title="Vehicle dynamics">
      <View style={styles.row}>
        <View style={styles.metric}>
          <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
            SPEED
          </Text>
          <Text style={[theme.typography.heading, { color: theme.colors.text }]}>
            {values?.speed_mms ?? '--'}{' '}
            <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
              mm/s
            </Text>
          </Text>
        </View>
        <View style={styles.metric}>
          <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
            DISTANCE
          </Text>
          <Text style={[theme.typography.heading, { color: theme.colors.text }]}>
            {values?.dist_mm ?? '--'}{' '}
            <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
              mm
            </Text>
          </Text>
        </View>
        <View style={styles.metric}>
          <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
            HEADING
          </Text>
          <Text style={[theme.typography.heading, { color: theme.colors.text }]}>
            {values?.heading_deg ?? '--'}{' '}
            <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
              deg
            </Text>
          </Text>
        </View>
      </View>

      <View style={styles.chart}>
        <View style={styles.chartHead}>
          <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
            SPEED HISTORY
          </Text>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            last {speeds.length} status frames
          </Text>
        </View>
        <Sparkline values={speeds} color={theme.colors.accent} />
      </View>
    </Panel>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', flexWrap: 'wrap', gap: 18 },
  metric: { gap: 2, minWidth: 90 },
  chart: { gap: 6 },
  chartHead: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
});
