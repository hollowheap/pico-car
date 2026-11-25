import { ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { ConsoleNav } from '../components/console/ConsoleNav';
import { FaultGrid } from '../components/console/FaultGrid';
import { HealthPanel } from '../components/console/HealthPanel';
import { LinkPanel } from '../components/console/LinkPanel';
import { ModeBanner } from '../components/console/ModeBanner';
import { StatusRail } from '../components/console/StatusRail';
import { buildOverview } from '../components/overview/overview';
import { LINK_CONFIG_ERROR, getLinkUrl } from '../lib/config';
import { panelStatus } from '../lib/panel-state';
import { useTelemetry } from '../lib/telemetry-context';
import { useTheme } from '../lib/theme-provider';
import { useNow } from '../lib/useNow';

const WIDE_PX = 900;

/** Health of the car: faults, tasks, timing and the link. */
export default function SystemsScreen() {
  const linkUrl = getLinkUrl();
  const { snapshot, store } = useTelemetry();
  const nowMs = useNow(250);
  const overview = buildOverview(snapshot, nowMs);
  const latest = store.latest;
  const theme = useTheme();
  const insets = useSafeAreaInsets();
  const { width } = useWindowDimensions();
  const wide = width >= WIDE_PX;
  const state = (kind: string) => panelStatus(snapshot.status, latest[kind], nowMs, kind);

  return (
    <ScrollView
      contentContainerStyle={[styles.page, { paddingTop: insets.top + 16 }]}
      showsVerticalScrollIndicator={false}
      style={{ backgroundColor: theme.colors.canvas }}
    >
      <ConsoleNav active="systems" />
      {linkUrl === null ? (
        <Text style={[theme.typography.body, { color: theme.colors.danger }]}>{LINK_CONFIG_ERROR}</Text>
      ) : null}
      <StatusRail nowMs={nowMs} overview={overview} latest={store.latest} snapshot={snapshot} />
      <ModeBanner faults={latest.status?.values.faults} overview={overview} statusAgeMs={overview.statusAgeMs} />
      <FaultGrid
        level={latest.status?.values.faults}
        status={state('status')}
        sticky={latest.diag?.values.sticky_faults}
      />
      <View style={[styles.row, wide ? styles.rowWide : null]}>
        <HealthPanel diag={latest.diag} motion={latest.motion} status={state('diag')} style={wide ? styles.cell : undefined} />
        <LinkPanel nowMs={nowMs} snapshot={snapshot} status={state('heartbeat')} store={store} style={wide ? styles.cell : undefined} />
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  page: { gap: 16, padding: 16, paddingBottom: 32 },
  row: { gap: 16 },
  rowWide: { alignItems: 'flex-start', flexDirection: 'row' },
  cell: { flex: 1 },
});
