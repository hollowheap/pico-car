import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { ConsoleView } from '../../lib/console-view';
import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import type { RunSummary } from '../../../../bridge/src/console/run-summary.js';
import { isRecentSonarPing } from '../../lib/scan';
import { enumLabel } from '../../lib/schema';
import { BIT_FIELDS, ENUMS } from '../../lib/schema.generated';
import { useTheme } from '../../lib/theme-provider';
import { formatTime, sizeMm } from './format';
import { EventCounts } from './EventCounts';
import { MetricCell } from './MetricCell';
import { Panel, useDense } from './Panel';
import { ScanReadings } from './ScanReadings';
import { StatusPill } from './StatusPill';

type Props = {
  summary?: RunSummary;
  scan: ConsoleView['scan'];
  obstacle: LinkPacket | undefined;
  ping: LinkPacket | undefined;
  profile: LinkPacket | undefined;
  nowMs: number;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
  summaryOnly?: boolean;
};

const DECI = 10;

const degrees = (deci: number) => `${deci / DECI}°`;
const hugDistance = (mm: number | undefined) =>
  mm !== undefined && Number.isFinite(mm) && mm > 0 ? `${mm} mm` : null;

export function ObstaclePanel({ obstacle, ping, profile, scan, nowMs, status, style, summary, summaryOnly = false }: Props) {
  const theme = useTheme();
  const counts = summary && <EventCounts testID="obstacle-events" scope={summary.scope}
    counts={[[summary.obstacles, 'obstacle'], [summary.bypasses, 'bypass', 'bypasses']]} />;
  const o = obstacle?.values;
  const ready = status?.kind === 'ready';
  const measurement = ready && isRecentSonarPing(ping, nowMs) ? ping.values : undefined;
  const servoDeci = measurement?.servo_deci;
  const range = measurement?.dist_mm;
  const bodyDeci = ready ? o?.body_deci : undefined;
  const stopped = o !== undefined && o.state === 0 && o.cause !== 0;
  const hasEventCounts = summary !== undefined && (summary.obstacles > 0 || summary.bypasses > 0);
  const dense = useDense();
  const profileStatus = profile?.values.status;
  const sparse = profileStatus === ENUMS.profileStatus.pilotSparse;
  const probeSide = sparse && ((profile?.values.flags ?? 0) & (1 << BIT_FIELDS.profileFlags.probeSideValid))
    ? profile?.values.side : undefined;
  const scanName = probeSide === ENUMS.hugSide.left ? 'Left edge probe' : probeSide === ENUMS.hugSide.right
    ? 'Right edge probe' : sparse ? 'Pilot sweep' : 'Scan';
  const scanTime = profile ? `${scanName} at ${formatTime(profile.timestamp)}. ` : '';
  const text = dense ? theme.typography.caption : theme.typography.body;
  const hugPhase = o?.hug_state === undefined ? 'no data' :
    ((o.flags ?? 0) & (1 << BIT_FIELDS.obstacleFlags.wall)) && o.hug_state !== ENUMS.hugState.idle
      ? 'Following wall' : enumLabel('hugState', o.hug_state).toLowerCase();
  const hug = o?.hug_state === undefined || o.hug_state === ENUMS.hugState.idle ? undefined : o;
  const hugSide = hug?.hug_side;
  const hugMetrics = [
    ['Side', hugSide === ENUMS.hugSide.left || hugSide === ENUMS.hugSide.right ? enumLabel('hugSide', hugSide).toLowerCase() : null],
    ['Front', hugDistance(hug?.face_mm)],
    ['Width', hugDistance(hug?.width_mm)],
    ['Depth', hugDistance(hug?.breadth_mm)],
  ] as const;

  const hugInPill = hug !== undefined && o?.state === ENUMS.obstacleState.monitor;

  return (
    <Panel
      fill
      empty={!summaryOnly && obstacle === undefined && ping === undefined && profile === undefined}
      icon="obstacle"
      right={
        hugInPill ? (
          <StatusPill accessibilityLabel={`Hug phase ${hugPhase}`} label={hugPhase.toUpperCase()} tone="warning" />
        ) : (
          <StatusPill label={o === undefined ? 'NO DATA' : enumLabel('obstacleState', o.state)}
            tone={o === undefined ? 'muted' : stopped ? 'danger' : o.state === ENUMS.obstacleState.monitor ? 'success' : 'warning'} />
        )
      }
      status={summaryOnly || status?.kind === 'stale' ? status : undefined}
      style={style}
      testID="obstacle-panel"
      title="Obstacle"
    >
      {summaryOnly ? (
        <View testID="hug-status" style={styles.sonarHug}>
          <Text accessibilityLabel={`Hug phase ${hugPhase}`} style={[text, { color: theme.colors.text }]}>Hug {hugPhase}</Text>
          {hugMetrics.filter(([, value]) => value !== null).map(([label, value]) => (
            <Text key={label} style={[theme.typography.caption, styles.hugValue, { color: theme.colors.muted }]}>
              {label} <Text style={{ color: theme.colors.text }}>{value}</Text>
            </Text>
          ))}
          {hasEventCounts ? counts : null}
        </View>
      ) : (
        <View style={styles.scan}>
          <ScanReadings key={`${profile?.timestamp}/${profile?.values.req_id}`} profile={profile} scan={scan}
            live={measurement} hugging={hug !== undefined}>
            <View style={styles.info}>
              <View accessibilityLabel="Range and bearings" style={styles.grid}>
                {[
                  ['Range', range !== undefined && range > 0 ? `${range} mm` : '--'],
                  ['Echo', measurement === undefined ? '--' : enumLabel('echoClass', measurement.echo_class).toLowerCase()],
                  ['Servo', servoDeci === undefined ? '--' : degrees(servoDeci)],
                  ['Body', bodyDeci === undefined ? '--' : degrees(bodyDeci)],
                ].map(([label, value]) => <MetricCell key={label} label={label!} value={value!} style={styles.cell} />)}
              </View>
              {hug !== undefined ? (
                <View style={[styles.grid, styles.hugGrid, { borderTopColor: theme.colors.border }]} testID="hug-status">
                  {hugInPill ? null : <MetricCell label="Phase" value={hugPhase} tone="warning" style={styles.cell} />}
                  {hugMetrics.filter(([, value]) => value !== null).map(([label, value]) => (
                    <MetricCell key={label} label={label} value={value!} style={styles.cell} />
                  ))}
                </View>
              ) : null}
              {profile ? (
                <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
                  {scanTime + ((profile?.values.closest_mm ?? 0) > 0 ? `Closest return ${sizeMm(profile?.values.closest_mm)}.` : '')}
                </Text>
              ) : null}
              {hasEventCounts ? counts : null}
            </View>
          </ScanReadings>
        </View>
      )}
      {!summaryOnly && status?.kind === 'error' ? <Text accessibilityLiveRegion="polite"
        style={[text, { color: theme.colors.danger }]}>{status.message}</Text> : null}
    </Panel>
  );
}

const styles = StyleSheet.create({
  sonarHug: { flexDirection: 'row', flexWrap: 'wrap', alignItems: 'center', columnGap: 8, rowGap: 2 },
  hugValue: { fontVariant: ['tabular-nums'] },
  scan: { flex: 1, gap: 6, minHeight: 0, position: 'relative' },
  info: { gap: 8 },
  grid: { flexDirection: 'row', flexWrap: 'wrap', columnGap: 10, rowGap: 6 },
  hugGrid: { borderTopWidth: 1, paddingTop: 6 },
  cell: { flexBasis: '42%' },
});
