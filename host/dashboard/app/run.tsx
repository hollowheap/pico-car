import { useEffect, useMemo, useState } from 'react';
import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { ConsoleNav } from '../components/console/ConsoleNav';
import { CourseTrace, MARK_LEGEND } from '../components/console/CourseTrace';
import { EventTimeline } from '../components/console/EventTimeline';
import { ModeBanner } from '../components/console/ModeBanner';
import { Panel } from '../components/console/Panel';
import { RunSummaryPanel } from '../components/console/RunSummaryPanel';
import { StatusRail } from '../components/console/StatusRail';
import { Button } from '../components/ui/button';
import { buildOverview } from '../components/overview/overview';
import { SERVICE_CONFIG_ERROR, getServiceUrl } from '../lib/config';
import { buildCourse } from '../lib/course';
import { fetchHistory, storeFromHistory } from '../lib/history';
import type { PanelStatus } from '../lib/panel-state';
import { summarizeRun } from '../lib/run-summary';
import { useTelemetry } from '../lib/telemetry-context';
import type { TelemetryStore } from '../lib/telemetry-store';
import { useTheme } from '../lib/theme-provider';
import { useNow } from '../lib/useNow';

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
  const { snapshot, store: live } = useTelemetry();
  const [history, setHistory] = useState<History>({ status: 'loading' });
  const [request, setRequest] = useState(0);
  const nowMs = useNow(500);
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
  const overview = buildOverview(snapshot, nowMs);
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
      contentContainerStyle={[styles.page, { paddingTop: insets.top + 16 }]}
      showsVerticalScrollIndicator={false}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav active="run" />
      <StatusRail nowMs={nowMs} overview={overview} latest={live.latest} snapshot={snapshot} />
      <ModeBanner faults={live.latest.status?.values.faults} overview={overview} statusAgeMs={overview.statusAgeMs} />
      <View style={styles.head}>
        <Text style={[theme.typography.heading, { color: theme.colors.text }]}>What happened</Text>
        <Button disabled={history.status === 'loading'} onPress={() => setRequest((value) => value + 1)} variant="secondary">
          <Button.Text>Reload history</Button.Text>
        </Button>
      </View>
      <RunSummaryPanel status={status} summary={summary} />
      <View style={[styles.row, wide ? styles.rowWide : null]}>
        <Panel status={status} style={wide ? styles.wideCell : undefined} testID="course-trace" title="Course trace">
          <CourseTrace course={course} />
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Dead-reckoned from the odometer and the compass heading, north up. It drifts with the heading error and is not a map.
            Open ring: start. Blue dot: the car now.
          </Text>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            Marks: {MARK_LEGEND.map((item) => item.label).join(', ')}.
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
  page: { gap: 16, padding: 16, paddingBottom: 32 },
  head: { alignItems: 'center', flexDirection: 'row', gap: 12, justifyContent: 'space-between' },
  row: { gap: 16 },
  rowWide: { alignItems: 'flex-start', flexDirection: 'row' },
  wideCell: { flex: 1 },
});
