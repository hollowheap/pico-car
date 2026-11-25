import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import { useTheme } from '../../lib/theme-provider';

type Props = {
  label: string;
  value: string;
  tone?: 'default' | 'accent' | 'success' | 'warning' | 'info' | 'danger' | 'muted';
  testID?: string;
  style?: StyleProp<ViewStyle>;
};

export function MetricCell({ label, value, tone = 'default', testID, style }: Props) {
  const theme = useTheme();
  const valueColor = {
    default: theme.colors.text,
    accent: theme.colors.accent,
    success: theme.colors.success,
    warning: theme.colors.warning,
    info: theme.colors.info,
    danger: theme.colors.danger,
    muted: theme.colors.muted,
  }[tone];
  return (
    <View style={[styles.cell, style]}>
      <Text numberOfLines={1} style={[theme.typography.caption, styles.label, { color: theme.colors.muted }]}>
        {label}
      </Text>
      <Text numberOfLines={1} style={[theme.typography.caption, styles.value, { color: valueColor }]} testID={testID}>
        {value}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  cell: { flex: 1, minWidth: 0, gap: 1 },
  label: { letterSpacing: 0.3, fontSize: 11, lineHeight: 14 },
  value: { fontVariant: ['tabular-nums'] },
});
