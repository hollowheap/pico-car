import { StyleSheet, Text, View } from 'react-native';

import { useTheme } from '../../lib/theme-provider';

export type PillTone = 'neutral' | 'active' | 'danger' | 'muted';

type Props = { label: string; tone?: PillTone };

/**
 * A short state word in a pill (DESIGN.md: status pill). The tone never carries
 * the meaning alone: the word does. `active` is the one place the accent marks
 * a live state.
 */
export function StatusPill({ label, tone = 'neutral' }: Props) {
  const theme = useTheme();
  const colors = {
    neutral: { bg: theme.colors.elevated, fg: theme.colors.text, dot: theme.colors.text },
    active: { bg: theme.colors.elevated, fg: theme.colors.text, dot: theme.colors.accent },
    danger: { bg: theme.colors.dangerSurface, fg: theme.colors.danger, dot: theme.colors.danger },
    muted: { bg: theme.colors.elevated, fg: theme.colors.muted, dot: theme.colors.muted },
  }[tone];

  return (
    <View
      accessibilityLabel={label}
      accessible
      style={[styles.pill, { backgroundColor: colors.bg, borderRadius: theme.radii.button }]}
    >
      <View style={[styles.dot, { backgroundColor: colors.dot, borderRadius: theme.radii.button }]} />
      <Text numberOfLines={1} style={[theme.typography.label, { color: colors.fg }]}>
        {label}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  pill: { alignItems: 'center', alignSelf: 'flex-start', flexDirection: 'row', gap: 6, paddingHorizontal: 12, paddingVertical: 4 },
  dot: { height: 6, width: 6 },
});
