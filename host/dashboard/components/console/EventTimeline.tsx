import { useCallback, useEffect, useMemo, useRef, useState, type ComponentRef } from 'react';
import { Platform, Pressable, ScrollView, StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import { groupActivity } from '../../lib/activity';
import { eventKey } from '../../lib/course';
import type { EventKind } from '../../lib/events';
import { useLogWindow } from '../../lib/useLogWindow';
import type { PanelStatus } from '../../lib/panel-state';
import { enumLabel } from '../../lib/schema';
import type { EventRow } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import type { EventInspection } from '../../lib/useEventInspection';
import { Button } from '../ui/button';
import { EventGlyph } from './CourseTrace';
import { formatTime, metres } from './format';
import { Panel, useDense } from './Panel';

type Props = {
  events: EventRow[];
  inspection: EventInspection;
  onShowCourse?: () => void;
  /** Optional space for event rows in compact layouts. */
  maxHeight?: number;
  runStart?: number | null;
  title?: string;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

const PAGE_SIZE = 3;

type Filter = 'all' | 'fault' | 'obstacle' | 'line' | 'turn' | 'barcode' | 'physics' | 'command' | 'system';

const FILTERS: { key: Filter; label: string; kinds: EventKind[] | null }[] = [
  { key: 'all', label: 'All', kinds: null },
  { key: 'fault', label: 'Faults', kinds: ['fault'] },
  { key: 'obstacle', label: 'Obstacles', kinds: ['obstacle'] },
  { key: 'line', label: 'Line', kinds: ['line'] },
  { key: 'turn', label: 'Turns', kinds: ['turn'] },
  { key: 'barcode', label: 'Barcodes', kinds: ['barcode'] },
  { key: 'physics', label: 'Physics', kinds: ['hump', 'attitude'] },
  { key: 'command', label: 'Commands', kinds: ['command'] },
  { key: 'system', label: 'System', kinds: ['system', 'link'] },
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

const eventTime = (event: EventRow) => formatTime(event.receivedAt ?? event.timestamp);
const eventLabel = (event: EventRow) => `${event.text.title}, ${eventTime(event)}`;

function TimelineRow({ compact, event, count, selected, focus, onSelect, onShown }: {
  compact: boolean;
  event: EventRow;
  count: number;
  selected: boolean;
  focus: boolean;
  onSelect: EventInspection['selectEvent'];
  onShown: () => void;
}) {
  const theme = useTheme();
  const trigger = useRef<ComponentRef<typeof View>>(null);

  useEffect(() => {
    if (focus) {
      trigger.current?.focus();
      onShown();
    }
  }, [focus, onShown]);

  const kindColor = {
    fault: theme.colors.danger, obstacle: theme.colors.warning, line: theme.colors.info, turn: theme.colors.accent,
    hump: theme.colors.success, attitude: theme.colors.success, barcode: theme.colors.accent,
    command: theme.colors.text, system: theme.colors.border, link: theme.colors.border,
  }[event.text.kind];

  return (
    <Pressable
      accessibilityLabel={count > 1 ? `Pickup detected, ${count} records, ${eventTime(event)}` : eventLabel(event)}
      accessibilityRole="button"
      accessibilityState={Platform.OS === 'web' ? undefined : { selected }}
      aria-pressed={selected}
      onPress={() => onSelect(event, () => trigger.current?.focus())}
      ref={trigger}
      style={[
        styles.row,
        compact ? styles.rowCompact : null,
        { borderColor: theme.colors.border, borderLeftColor: kindColor, backgroundColor: selected ? theme.colors.elevated : 'transparent' },
      ]}
    >
      <View style={styles.rowHead}>
        <View style={styles.titleRow}>
          <EventGlyph kind={event.text.kind} />
          <Text
            numberOfLines={compact ? 1 : undefined}
            style={[
              compact ? theme.typography.caption : theme.typography.body,
              styles.titleText,
              { color: event.text.kind === 'fault' ? theme.colors.danger : theme.colors.text },
            ]}
          >
            {count > 1 ? `Pickup detected · ${count} records` : event.text.title}
          </Text>
        </View>
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
          {eventTime(event)}, {metres(event.odometerMm)}
        </Text>
      </View>
      {compact ? null : (
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
          {KIND_WORD[event.text.kind]}, {enumLabel('eventSource', event.source).toLowerCase()}
          {event.text.detail === '' ? '' : `, ${event.text.detail}`}
        </Text>
      )}
    </Pressable>
  );
}

function ClusterChooser({ cluster, choose, clear }: Pick<EventInspection, 'cluster' | 'choose' | 'clear'>) {
  const theme = useTheme();
  const dense = useDense();
  const buttonStyle = dense ? styles.denseButton : undefined;
  const actionText = dense ? theme.typography.caption : theme.typography.body;
  const [currentPage, setPage] = useState(0);
  const lastPage = Math.ceil(cluster.length / PAGE_SIZE) - 1;
  const page = Math.min(currentPage, lastPage);
  const members = cluster.slice(page * PAGE_SIZE, (page + 1) * PAGE_SIZE);

  return (
    <View accessibilityLabel="Choose an event" role="region" style={styles.detail}>
      <Text style={[theme.typography.subheading, { color: theme.colors.text }]}>Choose an event</Text>
      {members.map((member) => (
        <Button
          accessibilityLabel={eventLabel(member)}
          compact
          key={eventKey(member)}
          onPress={() => choose(member)}
          style={buttonStyle}
          variant="secondary"
        >
          <Button.Text style={actionText}>{eventLabel(member)}</Button.Text>
        </Button>
      ))}
      <View style={styles.pager}>
        {lastPage > 0 ? (
          <>
            <Button accessibilityLabel="Previous events" compact disabled={page === 0}
              onPress={() => setPage(page - 1)} style={buttonStyle} variant="secondary">
              <Button.Text style={actionText}>Previous</Button.Text>
            </Button>
            <Text accessibilityLiveRegion="polite" style={[theme.typography.caption, { color: theme.colors.muted }]}>
              {page + 1} of {lastPage + 1}
            </Text>
            <Button accessibilityLabel="Next events" compact disabled={page === lastPage}
              onPress={() => setPage(page + 1)} style={buttonStyle} variant="secondary">
              <Button.Text style={actionText}>Next</Button.Text>
            </Button>
          </>
        ) : null}
        <Button compact onPress={clear} style={buttonStyle} variant="secondary"><Button.Text style={actionText}>Clear</Button.Text></Button>
      </View>
    </View>
  );
}

function InspectionDetail({ inspection, onShowCourse, onShow }: {
  inspection: EventInspection;
  onShowCourse?: () => void;
  onShow: () => void;
}) {
  const theme = useTheme();
  const dense = useDense();
  const buttonStyle = dense ? styles.denseButton : undefined;
  const actionText = dense ? theme.typography.caption : theme.typography.body;
  const { cluster, clear, marker, selected } = inspection;
  const text = [theme.typography.caption, { color: theme.colors.text }];
  const clearButton = (
    <Button compact onPress={clear} style={buttonStyle} variant="secondary"><Button.Text style={actionText}>Clear</Button.Text></Button>
  );

  const newest = cluster[0];
  if (newest !== undefined) return <ClusterChooser key={eventKey(newest)}
    cluster={cluster} choose={inspection.choose} clear={clear} />;
  if (selected === null) return null;

  return (
    <View accessibilityLabel="Selected event" role="region" style={styles.detail}>
      <Text style={[theme.typography.subheading, { color: theme.colors.text }]}>{selected.text.title}</Text>
      {selected.text.detail === '' ? null : <Text style={text}>{selected.text.detail}</Text>}
      <Text style={text}>
        {KIND_WORD[selected.text.kind]}, {enumLabel('eventSource', selected.source).toLowerCase()}, {eventTime(selected)}, {metres(selected.odometerMm)}
        {marker?.speedMms == null ? '' : `, ${marker.speedMms} mm/s`}
      </Text>
      {marker === null ? <Text style={text}>Location unavailable</Text> : null}
      <View style={styles.pager}>
        <Button compact onPress={onShow} style={buttonStyle} variant="secondary">
          <Button.Text style={actionText}>Show in events</Button.Text>
        </Button>
        {clearButton}
        {marker === null || onShowCourse === undefined ? null : (
          <Button compact onPress={onShowCourse} style={buttonStyle} variant="secondary">
            <Button.Text style={actionText}>Show on course</Button.Text>
          </Button>
        )}
      </View>
    </View>
  );
}

const EVENT_ROW_EST_PX = 38;

/** Events with the schema name, the time, the odometer and the decoded p1 and p2. */
export function EventTimeline({
  events, inspection, onShowCourse,
  title = 'Events', status, style,
}: Props) {
  const theme = useTheme();
  const dense = useDense();
  const buttonStyle = dense ? styles.denseButton : undefined;
  const [filter, setFilter] = useState<Filter>('all');
  const [reveal, setReveal] = useState<string | null>(null);
  const revealShown = useCallback(() => setReveal(null), []);
  const kinds = FILTERS.find((item) => item.key === filter)?.kinds ?? null;
  const matchesFilter = (event: EventRow) => kinds === null || kinds.includes(event.text.kind);
  // Oldest first so the latest event sits at the bottom of the scroll viewport.
  const groups = useMemo(() => groupActivity(events), [events]);
  const filterCounts = useMemo(() => {
    const counts: Record<Filter, number> = {
      all: groups.length, fault: 0, obstacle: 0, line: 0, turn: 0, barcode: 0, physics: 0, command: 0, system: 0,
    };
    for (const group of groups) {
      for (const item of FILTERS) {
        if (item.kinds !== null && group.members.some((e) => item.kinds!.includes(e.text.kind))) {
          counts[item.key] += 1;
        }
      }
    }
    return counts;
  }, [groups]);
  const rows = useMemo(() => groups.filter((group) => group.members.some(matchesFilter)), [groups, kinds]);
  const log = useLogWindow(rows.length, EVENT_ROW_EST_PX, { resetKey: filter });
  const visibleRows = rows.slice(log.window.start, log.window.end);
  const selectedKey = inspection.selected === null ? null : eventKey(inspection.selected);

  const showSelected = () => {
    if (selectedKey === null) return;
    const index = groups.findIndex((group) => group.members.some((event) => eventKey(event) === selectedKey));
    if (index === -1) return;
    setFilter('all');
    log.jumpToIndex(index, groups.length);
    setReveal(eventKey(groups[index]!.event));
  };

  const detail = <InspectionDetail inspection={inspection} onShowCourse={onShowCourse} onShow={showSelected} />;
  if (dense && inspection.cluster.length > 0) {
    return (
      <Panel icon="events" status={status} style={style} testID="event-timeline" title={title}>
        {detail}
      </Panel>
    );
  }

  return (
    <Panel icon="events" fill status={status} style={style} testID="event-timeline" title={title}>
      <View style={styles.filterBar}>
        <ScrollView
          contentContainerStyle={styles.filters}
          horizontal
          nestedScrollEnabled
          showsHorizontalScrollIndicator={false}
          style={styles.filterScroll}
        >
          {FILTERS.map((item) => {
            const n = filterCounts[item.key];
            const active = filter === item.key;
            return (
              <Button
                accessibilityState={Platform.OS === 'web' ? undefined : { selected: active }}
                aria-pressed={active}
                compact
                key={item.key}
                onPress={() => setFilter(item.key)}
                style={buttonStyle}
                variant={active ? 'primary' : 'secondary'}
              >
                <Button.Text style={theme.typography.label}>
                  {item.key !== 'all' && n > 0 ? `${item.label.toUpperCase()} ${n}` : item.label.toUpperCase()}
                </Button.Text>
              </Button>
            );
          })}
        </ScrollView>
        {rows.length > 0 ? (
          <Text style={[theme.typography.caption, { color: theme.colors.muted, fontVariant: ['tabular-nums'] }]}>{rows.length}</Text>
        ) : null}
      </View>
      {detail}
      {rows.length === 0 ? (
        <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
          {events.length === 0 ? 'No events yet.' : 'No events in this filter.'}
        </Text>
      ) : (
        <ScrollView
          accessibilityLabel="Activity events"
          nestedScrollEnabled
          onScroll={log.onScroll}
          ref={log.scrollRef}
          scrollEventThrottle={64}
          style={styles.list}
          testID="activity-events-scroll"
        >
          {log.olderCount > 0 ? (
            <Pressable onPress={() => log.loadOlder(0)} style={styles.edgeHint}>
              <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>↑ {log.olderCount} older events</Text>
            </Pressable>
          ) : null}
          {visibleRows.map(({ event, members }) => {
            const key = eventKey(event);
            return (
              <TimelineRow
                compact={dense}
                event={event}
                count={members.length}
                focus={reveal === key}
                key={key}
                onSelect={members.length === 1 ? inspection.selectEvent : (_, restore) => inspection.selectCluster(members, restore ?? (() => {}))}
                onShown={revealShown}
                selected={members.some((e) => eventKey(e) === selectedKey)}
              />
            );
          })}
          {log.newerCount > 0 ? (
            <Pressable onPress={() => log.loadNewer(0)} style={styles.edgeHint}>
              <Text style={[theme.typography.caption, { color: theme.colors.accent }]}>↓ {log.newerCount} newer events</Text>
            </Pressable>
          ) : null}
        </ScrollView>
      )}
    </Panel>
  );
}

const styles = StyleSheet.create({
  filterBar: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 6 },
  filterScroll: { flex: 1, minWidth: 0 },
  filters: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  list: { flex: 1, minHeight: 0 },
  edgeHint: { alignItems: 'center', paddingVertical: 6 },
  row: { borderTopWidth: 1, borderLeftWidth: 3, gap: 2, paddingVertical: 8, paddingLeft: 8 },
  rowCompact: { justifyContent: 'center', minHeight: 38, paddingVertical: 4 },
  detail: { gap: 6 },
  rowHead: { alignItems: 'center', flexDirection: 'row', gap: 8, justifyContent: 'space-between' },
  titleRow: { alignItems: 'center', flexDirection: 'row', flexShrink: 1, gap: 8 },
  titleText: { flexShrink: 1, fontWeight: '600' },
  pager: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  denseButton: { minHeight: 34, minWidth: 40, paddingHorizontal: 9 },
});
