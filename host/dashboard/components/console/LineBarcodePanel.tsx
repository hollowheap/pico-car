import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { enumLabel } from '../../lib/schema';
import type { EventRow } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';
import { formatTime } from './format';
import { Panel } from './Panel';

type Props = {
  statusPacket: LinkPacket | undefined;
  diag: LinkPacket | undefined;
  events: EventRow[];
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

const BIT_LEFT = 0;
const BIT_RIGHT = 1;
const BIT_BARCODE = 2;

function Lamp({ name, on }: { name: string; on: boolean | undefined }) {
  const theme = useTheme();
  return (
    <View
      accessibilityLabel={`${name} sensor ${on === undefined ? 'no data' : on ? 'on the line' : 'off'}`}
      accessible
      style={styles.lamp}
    >
      <View
        style={[
          styles.dot,
          {
            backgroundColor: on ? theme.colors.text : 'transparent',
            borderRadius: theme.radii.button,
            borderColor: on === undefined ? theme.colors.border : theme.colors.text,
          },
        ]}
      />
      <Text style={[theme.typography.label, { color: theme.colors.muted }]}>{name}</Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>{on === undefined ? '--' : on ? 'ON' : 'OFF'}</Text>
    </View>
  );
}

/**
 * What the line sensors see now, how the follower copes (edges per second,
 * lost count), and the last barcodes with the action each one caused.
 */
export function LineBarcodePanel({ statusPacket, diag, events, status, style }: Props) {
  const theme = useTheme();
  const bits = statusPacket?.values.line_bits;
  const on = (bit: number) => (bits === undefined ? undefined : (bits & (1 << bit)) !== 0);
  const d = diag?.values;
  const code = d?.last_barcode ?? 0;
  const letter = code > 0 ? String.fromCharCode(code) : null;
  const barcodes = events.filter((event) => event.text.kind === 'barcode').slice(-4).reverse();

  return (
    <Panel status={status} style={style} testID="line-panel" title="Line and barcode">
      <View style={styles.lamps}>
        <Lamp name="LEFT" on={on(BIT_LEFT)} />
        <Lamp name="RIGHT" on={on(BIT_RIGHT)} />
        <Lamp name="CODE" on={on(BIT_BARCODE)} />
      </View>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        Sensor changes {d?.line_edges_per_s ?? '--'}/s ({d?.line_raw_edges_per_s ?? '--'}/s raw) · lost {d?.line_lost_count ?? '--'} times
        · longest search {d?.line_search_mm ?? '--'} mm
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        Last barcode: {letter ?? 'none'}
        {letter !== null && d !== undefined ? ` (${enumLabel('barcodeAction', d.last_barcode_action).toLowerCase()})` : ''} · read {d?.barcode_ok ?? '--'} · refused{' '}
        {d?.barcode_bad ?? '--'} · edge drops {d?.barcode_edge_drops ?? '--'}
      </Text>
      {barcodes.length === 0 ? (
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>No barcode events yet.</Text>
      ) : (
        <View style={styles.list}>
          {barcodes.map((event) => (
            <View key={`${event.eventId}-${event.timestamp}`} style={styles.row}>
              <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>{formatTime(event.timestamp)}</Text>
              <Text style={[theme.typography.caption, { color: theme.colors.text }]}>
                {event.text.title}: {event.text.detail}
              </Text>
            </View>
          ))}
        </View>
      )}
    </Panel>
  );
}

const styles = StyleSheet.create({
  lamps: { flexDirection: 'row', gap: 20 },
  lamp: { alignItems: 'center', gap: 4 },
  dot: { borderWidth: 2, height: 24, width: 24 },
  list: { gap: 4 },
  row: { flexDirection: 'row', gap: 8 },
});
