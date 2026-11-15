import { StyleSheet, Text, View } from 'react-native';

import { useTheme } from '../../lib/theme-provider';

export type PillTone = 'neutral' | 'active' | 'danger' | 'muted';

type Props = { label: string; tone?: PillTone };

/**
 * A short state word with a dot (DESIGN.md: status pill). It has no box of its
 * own, so it is not a card inside a panel. The tone never carries
 * the meaning alone: the word does. `active` is the one place the accent marks
 * a live state.
 */
export function StatusPill({ label, tone = 'neutral' }: Props) {
  const theme = useTheme();
  const colors = {
    neutral: { fg: theme.colors.text, dot: theme.colors.text },
    active: { fg: theme.colors.text, dot: theme.colors.accent },
    danger: { fg: theme.colors.danger, dot: theme.colors.danger },
    muted: { fg: theme.colors.muted, dot: theme.colors.muted },
  }[tone];

  return (
    <View
      accessibilityLabel={label}
      accessible
      style={styles.pill}
    >
      <View style={[styles.dot, { backgroundColor: colors.dot, borderRadius: theme.radii.button }]} />
      <Text numberOfLines={1} style={[theme.typography.label, { color: colors.fg }]}>
        {label}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  pill: { alignItems: 'center', alignSelf: 'flex-start', flexDirection: 'row', gap: 6 },
  dot: { height: 6, width: 6 },
});
