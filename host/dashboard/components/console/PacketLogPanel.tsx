import { useState } from 'react';
import { ScrollView, StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';
import { formatTime, packetSummary } from './format';
import { Panel } from './Panel';

type Filter =
  | 'ALL'
  | 'STATUS'
  | 'MOTION'
  | 'PHYSICS'
  | 'OBSTACLE'
  | 'DIAG'
  | 'EVENT'
  | 'PROFILE'
  | 'HEARTBEAT';

const FILTERS: { key: Filter; label: string }[] = [
  { key: 'ALL', label: 'ALL' },
  { key: 'STATUS', label: 'STAT' },
  { key: 'MOTION', label: 'MOT' },
  { key: 'PHYSICS', label: 'PHY' },
  { key: 'OBSTACLE', label: 'OBS' },
  { key: 'DIAG', label: 'DIAG' },
  { key: 'EVENT', label: 'EVT' },
  { key: 'PROFILE', label: 'SCAN' },
  { key: 'HEARTBEAT', label: 'HB' },
];

/** One log row is about this tall; the scroller needs an explicit height. */
const ROW_PX = 48;
const LOG_MAX_PX = 300;

type Props = {
  packets: LinkPacket[];
  connected: boolean;
  style?: StyleProp<ViewStyle>;
};

export function PacketLogPanel({ packets, connected, style }: Props) {
  const theme = useTheme();
  const [filter, setFilter] = useState<Filter>('ALL');
  const visible = packets
    .filter(
      (packet) =>
        filter === 'ALL' || packet.packet.toUpperCase() === filter,
    )
    .slice(0, 40);

  return (
    <Panel style={style} title="Live packet log">
      <View style={styles.filters}>
        {FILTERS.map((item) => (
          <Button
            key={item.key}
            accessibilityState={{ selected: filter === item.key }}
            compact
            variant={filter === item.key ? 'primary' : 'secondary'}
            onPress={() => setFilter(item.key)}
          >
            <Button.Text style={theme.typography.label}>
              {item.label}
            </Button.Text>
          </Button>
        ))}
      </View>

      {!connected && packets.length > 0 ? (
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
          Link offline. Showing the last received frames.
        </Text>
      ) : null}
      <ScrollView
        showsVerticalScrollIndicator={false}
        style={{ height: Math.min(LOG_MAX_PX, visible.length * ROW_PX) }}
        contentContainerStyle={styles.logContent}
      >
        {visible.length === 0 ? (
          <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
            No packets yet.
          </Text>
        ) : (
          visible.map((packet, index) => (
            <View
              key={`${packet.timestamp}-${index}`}
              style={[styles.row, { borderColor: theme.colors.border }]}
            >
              <View style={styles.rowHead}>
                <Text
                  style={[
                    theme.typography.caption,
                    { color: theme.colors.text, fontWeight: '600' },
                  ]}
                >
                  {packet.packet.toUpperCase()}
                </Text>
                <Text
                  style={[
                    theme.typography.caption,
                    { color: theme.colors.muted },
                  ]}
                >
                  {formatTime(packet.timestamp)}
                </Text>
              </View>
              <Text
                style={[
                  theme.typography.caption,
                  { color: theme.colors.muted },
                ]}
              >
                {packetSummary(packet)}
              </Text>
            </View>
          ))
        )}
      </ScrollView>
    </Panel>
  );
}

const styles = StyleSheet.create({
  filters: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  logContent: { gap: 8, paddingRight: 4 },
  row: { borderTopWidth: 1, gap: 2, paddingTop: 8 },
  rowHead: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
});
