import { Text } from 'react-native';

import { useTheme } from '../../lib/theme-provider';

export type PillTone = 'neutral' | 'active' | 'success' | 'warning' | 'info' | 'danger' | 'muted';

type Props = { label: string; tone?: PillTone; accessibilityLabel?: string };

export function StatusPill({ label, tone = 'neutral', accessibilityLabel }: Props) {
  const theme = useTheme();
  const color = {
    neutral: theme.colors.text,
    active: theme.colors.info,
    success: theme.colors.success,
    warning: theme.colors.warning,
    info: theme.colors.info,
    danger: theme.colors.danger,
    muted: theme.colors.muted,
  }[tone];
  return <Text accessibilityLabel={accessibilityLabel} numberOfLines={1} style={[theme.typography.label, { color }]}>{label}</Text>;
}
