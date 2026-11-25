import { useState, type ReactNode } from 'react';
import { ScrollView, StyleSheet, Text, useWindowDimensions, View } from 'react-native';

import type { ConsoleView } from '../../lib/console-view';
import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';
import { bearingName, resultName, SonarLegend, SonarPlot, type SonarBearing } from './SonarPlot';

const PER_PAGE = 4;
const STACK_BELOW_PX = 560;

/** A recorded profile uses the scan-start frame. A lone live return uses the current servo frame. */
export function ScanReadings({ profile, scan, live, hugging, children }: {
  profile?: LinkPacket;
  scan: ConsoleView['scan'];
  live?: LinkPacket['values'];
  hugging: boolean;
  children?: ReactNode;
}) {
  const theme = useTheme();
  const [page, setPage] = useState(0);
  const servo = live?.servo_deci;
  const aim = servo !== undefined && Number.isFinite(servo) && Math.abs(servo) <= 200 ? servo / 10 : undefined;
  const liveReading: SonarBearing | undefined = aim === undefined || live?.echo_class === undefined ? undefined
    : { slot: 'live', angleDeg: aim, echoClass: live.echo_class, distMm: live.dist_mm ?? 0 };
  const readings = profile ? scan.readings : liveReading === undefined ? [] : [liveReading];
  const pages = Math.ceil(readings.length / PER_PAGE);
  const label = [theme.typography.caption, { color: theme.colors.text }];
  const stacked = useWindowDimensions().width < STACK_BELOW_PX;
  const sweep = { points: readings, status: profile?.values.status, movingBypass: profile ? scan.stationaryContact : hugging };
  return (
    <View style={[styles.split, stacked ? styles.stacked : null]}>
      <View style={[styles.sonar, stacked ? styles.sonarStacked : null]}>
        <SonarPlot {...sweep} recorded={profile !== undefined} aim={aim} />
      </View>
      <ScrollView style={styles.side} contentContainerStyle={styles.readings}>
        <SonarLegend {...sweep} />
        {children}
        {profile ? (
          <View style={styles.recorded}>
            <View style={styles.recordedHead}>
              <Text style={[theme.typography.label, { color: theme.colors.muted }]}>Recorded bearings</Text>
              {pages > 1 ? <View style={styles.pager}>
                <Button compact variant="secondary" accessibilityLabel="Previous bearings" disabled={page === 0} onPress={() => setPage(page - 1)}
                  style={styles.step}><Button.Text style={theme.typography.caption}>Prev</Button.Text></Button>
                <Text accessibilityLiveRegion="polite" style={label}>{page + 1} / {pages}</Text>
                <Button compact variant="secondary" accessibilityLabel="Next bearings" disabled={page + 1 === pages} onPress={() => setPage(page + 1)}
                  style={styles.step}><Button.Text style={theme.typography.caption}>Next</Button.Text></Button>
              </View> : null}
            </View>
            {readings.length === 0 ? <Text style={label}>No measured ranges.</Text> : (
              <View style={styles.bearings}>
                {readings.slice(page * PER_PAGE, (page + 1) * PER_PAGE).map(p =>
                  <Text key={p.slot} style={[label, styles.row]}>{bearingName(p.angleDeg)}: {resultName(p)}</Text>)}
              </View>
            )}
          </View>
        ) : null}
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  split: { flex: 1, flexDirection: 'row', gap: 12, minHeight: 0 },
  stacked: { flexDirection: 'column' },
  sonar: { flex: 1.15, minWidth: 0, minHeight: 0 },
  sonarStacked: { flex: 1, minHeight: 140 },
  side: { flex: 1, minWidth: 0, minHeight: 0 },
  readings: { flexGrow: 1, gap: 8, justifyContent: 'space-between' },
  recorded: { gap: 4 },
  recordedHead: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 8 },
  bearings: { flexDirection: 'row', flexWrap: 'wrap', columnGap: 12, rowGap: 2 },
  row: { fontVariant: ['tabular-nums'] },
  pager: { flexDirection: 'row', alignItems: 'center', gap: 4 },
  step: { paddingHorizontal: 8 },
});
