import { useEffect, useMemo, useState } from 'react';
import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { ConsoleNav } from '../components/console/ConsoleNav';
import { CourseTrace, MARK_LEGEND } from '../components/console/CourseTrace';
import { EventTimeline } from '../components/console/EventTimeline';
import { Panel } from '../components/console/Panel';
import { RunSummaryPanel } from '../components/console/RunSummaryPanel';
import { Button } from '../components/ui/button';
import { SERVICE_CONFIG_ERROR, getServiceUrl } from '../lib/config';
import { buildCourse } from '../lib/course';
import { fetchHistory, storeFromHistory } from '../lib/history';
import type { PanelStatus } from '../lib/panel-state';
import { summarizeRun } from '../lib/run-summary';
import { useTelemetry } from '../lib/telemetry-context';
import type { TelemetryStore } from '../lib/telemetry-store';
import { useTheme } from '../lib/theme-provider';

/** Records fetched from the bridge. The bridge keeps 10000. */
const HISTORY_LIMIT = 10000;
const WIDE_PX = 1000;
const EVENT_LIST_PX = 420;

type History =
  | { status: 'loading' }
  | { status: 'ready'; store: TelemetryStore }
  | { status: 'error'; message: string };

/** What happened: the summary of the latest run, the course trace and every event. */
export default function RunScreen() {
  const serviceUrl = getServiceUrl();
  const { store: live } = useTelemetry();
  const [history, setHistory] = useState<History>({ status: 'loading' });
  const [request, setRequest] = useState(0);
  const theme = useTheme();
  const insets = useSafeAreaInsets();
  const { width } = useWindowDimensions();
  const wide = width >= WIDE_PX;

  useEffect(() => {
    let active = true;
    setHistory({ status: 'loading' });
    if (serviceUrl === null) {
      setHistory({ status: 'error', message: SERVICE_CONFIG_ERROR });
      return () => {
        active = false;
      };
    }
    fetchHistory(serviceUrl, HISTORY_LIMIT)
      .then((records) => active && setHistory({ status: 'ready', store: storeFromHistory(records) }))
      .catch((reason: Error) => active && setHistory({ status: 'error', message: reason.message }));
    return () => {
      active = false;
    };
  }, [request, serviceUrl]);

  // The live store is newer. Use the history only when this page was opened first.
  const store = live.events.length > 0 || live.trail.length > 1 || history.status !== 'ready' ? live : history.store;
  const course = useMemo(() => buildCourse(store.trail, store.events), [store.trail, store.events]);
  const summary = useMemo(() => summarizeRun(store.trail, store.events), [store.trail, store.events]);
  const empty = store.trail.length < 2 && store.events.length === 0;
  const status: PanelStatus | undefined = empty
    ? history.status === 'loading'
      ? { kind: 'loading', message: 'Loading the recorded frames.' }
      : history.status === 'error'
        ? { kind: 'error', message: history.message }
        : { kind: 'empty', message: 'No frames recorded yet. Drive the car, then open this screen.' }
    : undefined;

  return (
    <ScrollView
      contentContainerStyle={[styles.page, { paddingHorizontal: wide ? 24 : 14, paddingTop: insets.top + 8 }]}
      showsVerticalScrollIndicator={false}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav active="run" />

      <View style={styles.head}>
        <Text accessibilityRole="header" style={[styles.title, { color: theme.colors.text }]}>Run review</Text>
        <Button disabled={history.status === 'loading'} onPress={() => setRequest((value) => value + 1)} variant="secondary">
          <Button.Text>Reload history</Button.Text>
        </Button>
      </View>
      <RunSummaryPanel status={status} summary={summary} />
      <View style={[styles.row, wide ? styles.rowWide : null]}>
        <Panel status={status} style={wide ? styles.course : undefined} testID="course-trace" title="Course trace">
          <CourseTrace course={course} darkCanvas height={wide ? 440 : 300} />
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Estimated from wheel travel and compass heading. The open ring is the start and the blue dot is the last recorded position.
          </Text>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Amber triangles mark where the car detected obstacles. Other markers show {MARK_LEGEND.filter((item) => item.kind !== 'obstacle').map((item) => item.label).join(', ')}.
          </Text>
        </Panel>
        <EventTimeline
          events={store.events}
          limit={200}
          maxHeight={EVENT_LIST_PX}
          status={status}
          style={wide ? styles.wideCell : undefined}
          title="Event timeline"
        />
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 15, paddingBottom: 36, width: '100%' },
  head: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 12, justifyContent: 'space-between', paddingVertical: 7 },
  title: { fontSize: 31, fontWeight: '700', letterSpacing: -0.9, lineHeight: 38 },
  row: { gap: 16 },
  rowWide: { alignItems: 'flex-start', flexDirection: 'row' },
  course: { flex: 1.6, minWidth: 0 },
  wideCell: { flex: 1, minWidth: 0 },
});
