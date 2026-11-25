import { useRef, useState, type ComponentRef } from 'react';
import { Platform, Pressable, ScrollView, StyleSheet, Text, View } from 'react-native';

import { focusElement } from '../../lib/focus';
import { readFrameSlice, type FrameLog } from '../../lib/frame-log';
import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { useLogWindow } from '../../lib/useLogWindow';
import { Button } from '../ui/button';

export type FrameReceipt = { id: number; packet: LinkPacket; restoreFocus: () => boolean };

type Props = {
  log: FrameLog;
  selectedId: number | null;
  onSelect: (receipt: FrameReceipt | null) => void;
};
const clock = new Intl.DateTimeFormat(undefined, {
  hour: '2-digit', minute: '2-digit', second: '2-digit', fractionalSecondDigits: 3, hourCycle: 'h23',
});

export function FrameLogPanel(props: Props) {
  return <FramePage key={props.log.revision} {...props} />;
}

function receiptTime(timestamp: number): string {
  const date = new Date(timestamp);
  return Number.isNaN(date.valueOf()) ? 'Time unavailable' : clock.format(date);
}

const FRAME_ROW_EST_PX = 34;

const PACKET_LABELS: Record<string, string> = {
  status: 'Status',
  motion: 'Motion',
  event: 'Event',
  obstacle: 'Obstacle',
  physics: 'Physics',
  profile: 'Profile',
  barcode_sample: 'Barcode',
  heartbeat: 'Heartbeat',
  diag: 'Diagnostics',
};

function packetLabel(packet: string): string {
  const known = PACKET_LABELS[packet];
  if (known !== undefined) return known;
  return packet
    .split('_')
    .filter(Boolean)
    .map((part, index) => (index === 0 ? part[0]!.toUpperCase() + part.slice(1) : part))
    .join(' ');
}

const FRAME_FILTERS: { key: string | null; label: string }[] = [
  { key: null, label: 'All' },
  { key: 'status', label: 'Status' },
  { key: 'motion', label: 'Motion' },
  { key: 'event', label: 'Event' },
  { key: 'obstacle', label: 'Obstacle' },
  { key: 'physics', label: 'Physics' },
  { key: 'profile', label: 'Profile' },
  { key: 'barcode_sample', label: 'Barcode' },
  { key: 'heartbeat', label: 'Heartbeat' },
  { key: 'diag', label: 'Diagnostics' },
];

function FramePage({ log, selectedId, onSelect }: Props) {
  const theme = useTheme();
  const rowRefs = useRef(new Map<number, ComponentRef<typeof View>>());
  const [packetFilter, setPacketFilter] = useState<string | null>(null);
  const totalCount = readFrameSlice(log, 0, 0, packetFilter).count;
  const win = useLogWindow(totalCount, FRAME_ROW_EST_PX, { resetKey: packetFilter ?? 'all' });
  const { rows, count } = readFrameSlice(log, win.window.start, win.window.end, packetFilter);
  const selected = rows.find(({ id }) => id === selectedId);
  const labelStyle = [theme.typography.caption, { color: theme.colors.muted }];
  const focusRow = (id: number) => focusElement(rowRefs.current.get(id), { scrollBlock: 'nearest' });

  return (
    <View style={styles.panel} testID="frame-log">
      <View style={styles.filterBar}>
        <ScrollView
          contentContainerStyle={styles.filters}
          horizontal
          nestedScrollEnabled
          showsHorizontalScrollIndicator={false}
          style={styles.filterScroll}
        >
          {FRAME_FILTERS.map((item) => {
            const active = packetFilter === item.key;
            return (
              <Button
                accessibilityState={Platform.OS === 'web' ? undefined : { selected: active }}
                aria-pressed={active}
                compact
                key={item.label}
                onPress={() => setPacketFilter(item.key)}
                style={styles.filterButton}
                variant={active ? 'primary' : 'secondary'}
              >
                <Button.Text style={theme.typography.label}>{item.label}</Button.Text>
              </Button>
            );
          })}
        </ScrollView>
      </View>
      {count === 0 ? <Text style={labelStyle}>{packetFilter === null ? 'No telemetry frames received yet.' : 'No frames in this filter.'}</Text> : (
        <>
          <Text style={[labelStyle, styles.summary]}>
            {win.window.start + 1}–{win.window.end} of {count} {count === 1 ? 'frame' : 'frames'}{win.window.pinned ? ', Live' : ', Paused'}
          </Text>
          <View style={styles.inspector}>
            <ScrollView
              accessibilityLabel="Received frames"
              contentContainerStyle={styles.rows}
              nestedScrollEnabled
              onScroll={win.onScroll}
              ref={win.scrollRef}
              role="region"
              scrollEventThrottle={64}
              style={styles.list}
              tabIndex={0}
            >
              {win.olderCount > 0 ? (
                <Pressable onPress={() => win.loadOlder(0)} style={styles.edgeHint}>
                  <Text style={labelStyle}>↑ {win.olderCount} older frames</Text>
                </Pressable>
              ) : null}
              {rows.map(({ id, packet }) => {
                const active = selectedId === id;
                const time = receiptTime(packet.receivedAt ?? packet.timestamp);
                const name = packetLabel(packet.packet);
                return (
                  <Pressable
                    accessibilityLabel={`${name}, received ${time}`}
                    accessibilityRole="button"
                    accessibilityState={Platform.OS === 'web' ? undefined : { selected: active }}
                    aria-controls={packet.packet === 'physics' ? 'physics-card' : active ? 'frame-details' : undefined}
                    aria-pressed={active}
                    key={id}
                    onPress={() => {
                      if (!active) win.unpin();
                      onSelect(active ? null : { id, packet, restoreFocus: () => focusRow(id) });
                    }}
                    ref={(node) => { if (node) rowRefs.current.set(id, node); else rowRefs.current.delete(id); }}
                    style={[
                      styles.row,
                      {
                        borderTopColor: theme.colors.border,
                        borderLeftColor: active ? theme.colors.accent : 'transparent',
                        backgroundColor: active ? theme.colors.elevated : 'transparent',
                      },
                    ]}
                    testID={`frame-row-${id}`}
                  >
                    <Text numberOfLines={1} style={[theme.typography.caption, styles.rowName, { color: theme.colors.text }]}>
                      {name}
                    </Text>
                    <Text style={[labelStyle, styles.rowTime]}>{time}</Text>
                  </Pressable>
                );
              })}
              {win.newerCount > 0 ? (
                <Pressable onPress={() => win.loadNewer(0)} style={styles.edgeHint}>
                  <Text style={[theme.typography.caption, { color: theme.colors.accent }]}>↓ {win.newerCount} newer frames</Text>
                </Pressable>
              ) : null}
            </ScrollView>
            {selected === undefined || selected.packet.packet === 'physics' ? null : <DecodedFrameFields packet={selected.packet} />}
          </View>
        </>
      )}
    </View>
  );
}

export function DecodedFrameFields({ packet, recordedPhysics = false }: { packet: LinkPacket; recordedPhysics?: boolean }) {
  const theme = useTheme();
  const fields = Object.entries(packet.values);
  const caption = [theme.typography.caption, { color: theme.colors.muted }];
  const time = receiptTime(packet.receivedAt ?? packet.timestamp);
  const receipt = recordedPhysics ? packet.topic : `${time}${packet.topic ? `, ${packet.topic}` : ''}`;
  return (
    <ScrollView accessibilityLabel={recordedPhysics ? 'Recorded physics fields' : 'Decoded frame fields'}
      nativeID={recordedPhysics ? undefined : 'frame-details'} role="region" nestedScrollEnabled
      style={recordedPhysics ? styles.recording : [styles.details, { borderLeftColor: theme.colors.border }]}
      contentContainerStyle={styles.fields} tabIndex={0}>
      <Text accessibilityRole="header" aria-level={3} style={[theme.typography.subheading, { color: theme.colors.text }]}>
        {recordedPhysics ? `Recorded at ${time}` : `${packetLabel(packet.packet)} fields`}
      </Text>
      {receipt ? <Text style={caption}>{receipt}</Text> : null}
      {fields.map(([field, value]) => (
        <View key={field} style={styles.field}>
          <Text style={[caption, styles.fieldName]}>{field}</Text>
          <Text style={[theme.typography.caption, styles.value, { color: theme.colors.text }]}>{String(value)}</Text>
        </View>
      ))}
      {fields.length === 0 ? <Text style={caption}>No decoded fields.</Text> : null}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  panel: { flex: 1, minHeight: 0, gap: 6 },
  filterBar: { flexDirection: 'row', alignItems: 'center' },
  filterScroll: { flex: 1, minWidth: 0 },
  filters: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  filterButton: { minHeight: 34, minWidth: 40, paddingHorizontal: 9 },
  summary: { fontVariant: ['tabular-nums'] },
  inspector: { flex: 1, minHeight: 0, flexDirection: 'row', gap: 8 },
  list: { flex: 4, minWidth: 0 },
  edgeHint: { alignItems: 'center', paddingVertical: 4 },
  rows: { gap: 0 },
  row: {
    alignItems: 'center',
    borderLeftWidth: 2,
    borderTopWidth: 1,
    flexDirection: 'row',
    gap: 8,
    justifyContent: 'space-between',
    minHeight: 34,
    paddingHorizontal: 6,
    paddingVertical: 5,
  },
  rowName: { flexShrink: 1, fontWeight: '600' },
  rowTime: { fontVariant: ['tabular-nums'] },
  details: { borderLeftWidth: 1, flex: 5, minWidth: 0, paddingLeft: 10 },
  fields: { gap: 5 },
  recording: { height: 132 },
  field: { alignItems: 'baseline', flexDirection: 'row', justifyContent: 'space-between', columnGap: 8 },
  fieldName: { flexShrink: 1 },
  value: { flexShrink: 1, fontVariant: ['tabular-nums'] },
  pager: { flexDirection: 'row', gap: 8 },
});
