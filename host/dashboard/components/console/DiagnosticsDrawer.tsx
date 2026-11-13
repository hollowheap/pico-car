import { useEffect, useState } from 'react';
import { Modal, Platform, Pressable, ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import type { LinkPacket, LinkSnapshot } from '../../lib/link';
import { panelStatus } from '../../lib/panel-state';
import type { TelemetryStore } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import type { Overview } from '../overview/overview';
import { EventTimeline } from './EventTimeline';
import { FaultGrid } from './FaultGrid';
import { HealthPanel } from './HealthPanel';
import { LineBarcodePanel } from './LineBarcodePanel';
import { LinkPanel } from './LinkPanel';
import { PacketLogPanel } from './PacketLogPanel';
import { StatusRail } from './StatusRail';

const SECTIONS = ['Faults', 'Events', 'Sensors', 'Link'] as const;
type Section = typeof SECTIONS[number];

type Props = {
  nowMs: number;
  onClose: () => void;
  overview: Overview;
  packets: LinkPacket[];
  snapshot: LinkSnapshot;
  store: TelemetryStore;
};

export function DiagnosticsDrawer({ nowMs, onClose, overview, packets, snapshot, store }: Props) {
  const [section, setSection] = useState<Section>('Faults');
  useEffect(() => {
    if (Platform.OS !== 'web') return;
    const closeOnEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', closeOnEscape);
    return () => window.removeEventListener('keydown', closeOnEscape);
  }, [onClose]);
  const { colors, typography } = useTheme();
  const { width } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const latest = store.latest;
  const state = (kind: string) => panelStatus(snapshot.status, latest[kind], nowMs, kind);

  return (
    <Modal animationType="fade" onRequestClose={onClose} transparent visible>
      <View style={styles.overlay}>
        <Pressable accessible={false} onPress={onClose} style={styles.backdrop} />
        <View
          accessibilityLabel="Diagnostics"
          accessibilityViewIsModal
          style={[styles.drawer, {
            backgroundColor: colors.canvas,
            paddingBottom: insets.bottom,
            paddingTop: insets.top,
            width: Math.min(width, 570),
          }]}
          testID="diagnostics-drawer"
        >
          <View style={[styles.header, { backgroundColor: colors.card, borderColor: colors.border }]}>
            <View style={styles.titleRow}>
              <Text accessibilityRole="header" style={[typography.heading, { color: colors.text }]}>Diagnostics</Text>
              <Pressable accessibilityLabel="Close diagnostics" accessibilityRole="button" onPress={onClose}
                style={[styles.close, { backgroundColor: colors.elevated }]} testID="close-diagnostics">
                <Text style={[typography.headingSm, { color: colors.text }]}>×</Text>
              </Pressable>
            </View>
            <View style={[styles.tabs, { backgroundColor: colors.elevated }]}>
              {SECTIONS.map((item) => (
                <Pressable
                  accessibilityRole="tab"
                  accessibilityState={{ selected: section === item }}
                  key={item}
                  onPress={() => setSection(item)}
                  style={[styles.tab, { backgroundColor: section === item ? colors.card : 'transparent' }]}
                >
                  <Text style={[typography.caption, { color: section === item ? colors.text : colors.muted, fontWeight: '600' }]}>{item}</Text>
                </Pressable>
              ))}
            </View>
          </View>
          <ScrollView contentContainerStyle={styles.content} key={section}>
            {section === 'Faults' ? (
              <>
                <FaultGrid level={latest.status?.values.faults} status={state('status')} sticky={latest.diag?.values.sticky_faults} />
                <HealthPanel diag={latest.diag} motion={latest.motion} status={state('diag')} />
              </>
            ) : null}
            {section === 'Events' ? <EventTimeline events={store.events} limit={100} maxHeight={540} /> : null}
            {section === 'Sensors' ? (
              <LineBarcodePanel diag={latest.diag} events={store.events} status={state('status')} statusPacket={latest.status} />
            ) : null}
            {section === 'Link' ? (
              <>
                <StatusRail latest={latest} nowMs={nowMs} overview={overview} snapshot={snapshot} />
                <LinkPanel nowMs={nowMs} snapshot={snapshot} status={state('heartbeat')} store={store} />
                <PacketLogPanel connected={snapshot.status === 'open'} packets={packets} />
              </>
            ) : null}
          </ScrollView>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: { flex: 1, flexDirection: 'row', justifyContent: 'flex-end' },
  backdrop: { backgroundColor: '#08111c70', bottom: 0, left: 0, position: 'absolute', right: 0, top: 0 },
  drawer: { boxShadow: '-12px 0 48px #07132335', flexShrink: 0 },
  header: { borderBottomWidth: 1, gap: 12, paddingHorizontal: 18, paddingVertical: 13 },
  titleRow: { alignItems: 'center', flexDirection: 'row', justifyContent: 'space-between' },
  close: { alignItems: 'center', borderRadius: 22, height: 44, justifyContent: 'center', width: 44 },
  tabs: { borderRadius: 14, flexDirection: 'row', gap: 3, padding: 4 },
  tab: { alignItems: 'center', borderRadius: 11, flex: 1, justifyContent: 'center', minHeight: 36 },
  content: { gap: 12, padding: 14, paddingBottom: 32 },
});
