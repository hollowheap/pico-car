import { ScrollView, StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import { faultTiles, STATE_WORD, summarizeFaults, type FaultTile } from '../../lib/faults';
import type { PanelStatus } from '../../lib/panel-state';
import { useTheme } from '../../lib/theme-provider';
import { faultHex } from './format';
import { Panel } from './Panel';

type Props = {
  /** The fault word of the status packet: set now. */
  level: number | undefined;
  /** The sticky word of the diag packet: set since the last RESET. */
  sticky: number | undefined;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

function Tile({ tile }: { tile: FaultTile }) {
  const theme = useTheme();
  const critical = tile.state === 'set' && tile.severity === 'critical';
  const set = tile.state === 'set';
  const color = critical ? theme.colors.danger : set ? theme.colors.text : theme.colors.muted;
  const word = set ? `${STATE_WORD[tile.state]} · ${tile.severity.toUpperCase()}` : STATE_WORD[tile.state];

  return (
    <View
      accessibilityLabel={`Bit ${tile.bit}, ${tile.label}: ${word}`}
      accessible
      style={[
        styles.tile,
        {
          backgroundColor: critical ? theme.colors.dangerSurface : set ? theme.colors.elevated : 'transparent',
          borderRadius: theme.radii.input,
        },
      ]}
      testID={`fault-tile-${tile.bit}`}
    >
      <Text style={[theme.typography.caption, { color: set ? theme.colors.text : theme.colors.muted }]}>bit {tile.bit}</Text>
      <Text numberOfLines={2} style={[theme.typography.body, styles.tileName, { color: set ? color : theme.colors.text, fontWeight: set ? '600' : '400' }]}>
        {tile.label}
      </Text>
      <Text style={[theme.typography.label, { color }]}>{word}</Text>
    </View>
  );
}

/**
 * Sixteen tiles in bit order, four by four. Each says set now, set before
 * (sticky since the last RESET) or ok, and what to do about a set fault.
 */
export function FaultGrid({ level, sticky, status, style }: Props) {
  const theme = useTheme();
  const tiles = faultTiles(level, sticky);
  const summary = summarizeFaults(tiles);
  const active = tiles.filter((tile) => tile.state === 'set');

  return (
    <Panel
      right={
        <Text style={[theme.typography.label, { color: summary.critical > 0 ? theme.colors.danger : theme.colors.muted }]}>
          {level === undefined ? '--' : `${summary.set} of 16 set · ${faultHex(level)}`}
        </Text>
      }
      status={status}
      style={style}
      testID="fault-grid"
      title="Faults"
    >
      <View style={styles.grid}>
        {tiles.map((tile) => (
          <Tile key={tile.bit} tile={tile} />
        ))}
      </View>
      <ScrollView accessibilityLabel="Active fault details" nestedScrollEnabled style={styles.details} contentContainerStyle={styles.todo}>
        {active.length === 0 ? (
          <Text style={[theme.typography.body, { color: theme.colors.muted }]}>No active faults</Text>
        ) : active.map((tile) => (
          <Text key={tile.bit} style={[theme.typography.body, { color: theme.colors.text }]}>
            <Text style={{ fontWeight: '600' }}>{tile.label}.</Text> {tile.hint}
          </Text>
        ))}
        {summary.wasSet > 0 ? (
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            WAS SET means the fault occurred since the last reset and has cleared.
          </Text>
        ) : null}
      </ScrollView>
    </Panel>
  );
}

const styles = StyleSheet.create({
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  tile: { flexBasis: '22%', flexGrow: 1, gap: 2, minWidth: 110, padding: 8 },
  tileName: { height: 40, lineHeight: 20 },
  details: { height: 116, flexGrow: 0, flexShrink: 0 },
  todo: { gap: 4, paddingRight: 8 },
});
