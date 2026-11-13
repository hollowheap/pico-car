import { Link } from 'expo-router';
import { memo, useEffect, useMemo, useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';

import { buildCourse } from '../../lib/course';
import { fetchHistory, storeFromHistory } from '../../lib/history';
import type { LinkPacket } from '../../lib/link';
import { enumLabel } from '../../lib/schema';
import type { EventRow, TrailPoint } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import { CourseTrace } from './CourseTrace';

const HISTORY_LIMIT = 3000;

export const LiveCoursePanel = memo(function LiveCoursePanel({
  events,
  plotHeight,
  status,
  trail,
  serviceUrl,
}: {
  events: EventRow[];
  plotHeight: number;
  status: LinkPacket | undefined;
  trail: TrailPoint[];
  serviceUrl: string | null;
}) {
  const { typography } = useTheme();
  const [recorded, setRecorded] = useState<{ events: EventRow[]; trail: TrailPoint[] }>();
  const hasLiveTrace = trail.length > 0;
  const source = hasLiveTrace ? undefined : recorded;
  const trace = source?.trail ?? trail;
  const marks = source?.events ?? events;
  const course = useMemo(() => buildCourse(trace, marks), [trace, marks]);

  useEffect(() => {
    if (serviceUrl === null || hasLiveTrace) return;
    let active = true;
    void fetchHistory(serviceUrl, HISTORY_LIMIT)
      .then((records) => {
        if (!active) return;
        const { events: savedEvents, trail: savedTrail } = storeFromHistory(records);
        setRecorded({ events: savedEvents, trail: savedTrail });
      })
      .catch(() => {
        // Live readings can still draw the course when history is unavailable.
      });
    return () => { active = false; };
  }, [hasLiveTrace, serviceUrl]);

  const last = trace[trace.length - 1];
  const isRecorded = source !== undefined && trace.length > 0;
  const distance = last === undefined ? '—' : (last.odometerMm / 1000).toFixed(2) + ' m';
  const mode = status === undefined ? null : enumLabel('vehicleMode', status.values.mode);

  return (
    <View style={styles.surface} testID="live-course-trace">
      <View style={styles.head}>
        <View style={styles.heading}>
          <Text accessibilityRole="header" style={[typography.heading, styles.title]}>Course trace</Text>
          <Text style={[typography.caption, styles.label]}>{isRecorded ? 'Recorded run' : 'Estimated route'}</Text>
        </View>
        <Link href="/run" style={{ ...typography.caption, color: '#b4d8ff', fontWeight: '600', paddingVertical: 8 }}>
          Run review →
        </Link>
      </View>
      <CourseTrace course={course} darkCanvas height={plotHeight} />
      <View style={styles.footer}>
        <View style={styles.legend}>
          <View style={styles.legendLine} />
          <Text style={[typography.caption, styles.label]}>Driven path</Text>
          <Text style={styles.marker}>▲</Text>
          <Text style={[typography.caption, styles.label]}>Obstacle detected</Text>
        </View>
        <Text style={[typography.caption, styles.label]}>{mode === null ? distance : mode + '  ·  ' + distance}</Text>
      </View>
    </View>
  );
});

const styles = StyleSheet.create({
  surface: { backgroundColor: '#0c1523', flexGrow: 1, gap: 13, padding: 15 },
  head: { alignItems: 'center', flexDirection: 'row', gap: 8, justifyContent: 'space-between', paddingHorizontal: 3 },
  heading: { gap: 2 },
  title: { color: '#edf4ff', fontSize: 24, letterSpacing: -0.65 },
  label: { color: '#b4c7de' },
  footer: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 8, justifyContent: 'space-between', paddingHorizontal: 3 },
  legend: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  legendLine: { backgroundColor: '#78b7ff', borderRadius: 2, height: 3, width: 15 },
  marker: { color: '#f8bc64', fontSize: 13, marginLeft: 5 },
});
