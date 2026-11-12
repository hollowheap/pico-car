import type { ReactNode } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { PanelStatus } from '../../lib/panel-state';
import { useTheme } from '../../lib/theme-provider';

type Props = {
  title: string;
  right?: ReactNode;
  children: ReactNode;
  style?: StyleProp<ViewStyle>;
  /** Loading, empty, stale or error. Missing means ready. */
  status?: PanelStatus;
  testID?: string;
};

/**
 * A card with a heading. A panel that has no data shows one line that says why
 * instead of zeros. A stale panel keeps its values, dimmed, under the reason.
 */
export function Panel({ title, right, children, style, status, testID }: Props) {
  const theme = useTheme();
  const kind = status?.kind ?? 'ready';
  const showBody = kind === 'ready' || kind === 'stale';

  return (
    <View
      accessibilityLabel={title}
      accessibilityRole="summary"
      style={[
        styles.panel,
        {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.card,
          padding: theme.spacing.lg,
        },
        style,
      ]}
      testID={testID}
    >
      <View style={styles.head}>
        <Text accessibilityRole="header" style={[theme.typography.headingSm, { color: theme.colors.text }]}>
          {title}
        </Text>
        {right}
      </View>
      {status !== undefined && status.kind !== 'ready' ? (
        <Text
          accessibilityLiveRegion="polite"
          style={[
            theme.typography.body,
            { color: kind === 'error' ? theme.colors.danger : theme.colors.muted },
          ]}
        >
          {kind === 'stale' ? 'STALE. ' : ''}
          {status.message}
        </Text>
      ) : null}
      {showBody ? <View style={[styles.body, kind === 'stale' ? styles.stale : null]}>{children}</View> : null}
    </View>
  );
}

const styles = StyleSheet.create({
  panel: { borderWidth: 1, gap: 12 },
  head: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
  body: { gap: 12 },
  stale: { opacity: 0.5 },
});
