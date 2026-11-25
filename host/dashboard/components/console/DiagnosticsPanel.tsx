import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { FAULT_BITS, type Overview } from '../overview/overview';
import { faultHex } from './format';
import { Panel } from './Panel';

type Props = {
  status: LinkPacket | undefined;
  overview: Overview;
  style?: StyleProp<ViewStyle>;
};

export function DiagnosticsPanel({ status, overview, style }: Props) {
  const theme = useTheme();
  const mask = status?.values.faults ?? 0;
  const active = overview.faults;

  return (
    <Panel
      style={style}
      title="Faults"
      right={
        <Text
          style={[
            theme.typography.label,
            {
              color: mask === 0 ? theme.colors.muted : theme.colors.danger,
            },
          ]}
        >
          {faultHex(status?.values.faults)}
        </Text>
      }
    >
      <View
        style={[
          styles.banner,
          {
            backgroundColor:
              active.length === 0
                ? theme.colors.elevated
                : theme.colors.dangerSurface,
            borderColor: theme.colors.border,
          },
        ]}
      >
        <Text
          style={[
            theme.typography.body,
            {
              color:
                active.length === 0
                  ? theme.colors.text
                  : theme.colors.danger,
            },
          ]}
        >
          {active.length === 0
            ? 'No active faults'
            : active.join(', ')}
        </Text>
      </View>

      <View style={styles.bits}>
        {FAULT_BITS.map((fault) => {
          const set = (mask & (1 << fault.bit)) !== 0;
          return (
            <View key={fault.bit} style={styles.bitRow}>
              <Text
                style={[
                  theme.typography.caption,
                  { color: theme.colors.muted },
                ]}
              >
                bit {fault.bit} {fault.name}
              </Text>
              <Text
                style={[
                  theme.typography.caption,
                  {
                    color: set ? theme.colors.danger : theme.colors.muted,
                    fontWeight: set ? '600' : '400',
                  },
                ]}
              >
                {set ? '1' : '0'}
              </Text>
            </View>
          );
        })}
      </View>
    </Panel>
  );
}

const styles = StyleSheet.create({
  banner: { borderWidth: 1, borderRadius: 8, padding: 10 },
  bits: { gap: 4 },
  bitRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
});
