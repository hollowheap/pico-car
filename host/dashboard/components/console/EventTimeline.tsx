import { useState } from 'react';
import { ScrollView, StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { EventKind } from '../../lib/events';
import type { PanelStatus } from '../../lib/panel-state';
import { enumLabel } from '../../lib/schema';
import type { EventRow } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';
import { formatTime, metres } from './format';
import { Panel } from './Panel';

type Props = {
  events: EventRow[];
  /** How many rows to draw at most. */
  limit?: number;
  /** The panel height in px. Without it the list grows. */
  maxHeight?: number;
  title?: string;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** One row is about this tall. A scroller needs an explicit height in this layout: with only a max height it collapses and the rows spill over the next panel. */
const ROW_PX = 60;

type Filter = 'all' | 'attention' | 'drive' | 'commands';

const FILTERS: { key: Filter; label: string; kinds: EventKind[] | null }[] = [
  { key: 'all', label: 'All', kinds: null },
  { key: 'attention', label: 'Attention', kinds: ['fault', 'attitude', 'obstacle', 'link'] },
  { key: 'drive', label: 'Drive', kinds: ['barcode', 'turn', 'line', 'hump'] },
  { key: 'commands', label: 'Commands', kinds: ['command', 'system'] },
];

const KIND_WORD: Record<EventKind, string> = {
  system: 'SYSTEM',
  command: 'COMMAND',
  barcode: 'BARCODE',
  turn: 'TURN',
  line: 'LINE',
  obstacle: 'OBSTACLE',
  hump: 'HUMP',
  attitude: 'ATTITUDE',
  fault: 'FAULT',
  link: 'LINK',
};

/** Events with the schema name, the time, the odometer and the decoded p1 and p2. */
export function EventTimeline({ events, limit = 60, maxHeight, title = 'Events', status, style }: Props) {
  const theme = useTheme();
  const [filter, setFilter] = useState<Filter>('all');
  const kinds = FILTERS.find((item) => item.key === filter)?.kinds ?? null;
  const rows = events
    .filter((event) => kinds === null || kinds.includes(event.text.kind))
    .slice(-limit)
    .reverse();

  return (
    <Panel status={status} style={style} testID="event-timeline" title={title}>
      <View style={styles.filters}>
        {FILTERS.map((item) => (
          <Button
            accessibilityState={{ selected: filter === item.key }}
            compact
            key={item.key}
            onPress={() => setFilter(item.key)}
            variant={filter === item.key ? 'primary' : 'secondary'}
          >
            <Button.Text style={theme.typography.label}>{item.label.toUpperCase()}</Button.Text>
          </Button>
        ))}
      </View>
      {rows.length === 0 ? (
        <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
          {events.length === 0 ? 'No events yet.' : 'No events in this filter.'}
        </Text>
      ) : (
        <ScrollView
          nestedScrollEnabled
          showsVerticalScrollIndicator={false}
          style={maxHeight ? { height: Math.min(maxHeight, rows.length * ROW_PX) } : undefined}
        >
          {rows.map((event) => (
            <View
              key={`${event.eventId}-${event.timestamp}-${event.type}`}
              style={[styles.row, { borderColor: theme.colors.border }]}
            >
              <View style={styles.rowHead}>
                <Text style={[theme.typography.body, { color: event.text.kind === 'fault' ? theme.colors.danger : theme.colors.text, fontWeight: '600' }]}>
                  {event.text.title}
                </Text>
                <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
                  {formatTime(event.timestamp)} · {metres(event.odometerMm)}
                </Text>
              </View>
              <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
                {KIND_WORD[event.text.kind]} · {enumLabel('eventSource', event.source).toLowerCase()}
                {event.text.detail === '' ? '' : ` · ${event.text.detail}`}
              </Text>
            </View>
          ))}
        </ScrollView>
      )}
    </Panel>
  );
}

const styles = StyleSheet.create({
  filters: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  row: { borderTopWidth: 1, gap: 2, paddingVertical: 8 },
  rowHead: { alignItems: 'baseline', flexDirection: 'row', gap: 8, justifyContent: 'space-between' },
});
