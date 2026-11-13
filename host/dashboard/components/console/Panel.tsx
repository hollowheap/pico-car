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
  showPlaceholder?: boolean;
  embedded?: boolean;
};

/**
 * A card with a heading. A panel that has no data shows one line that says why
 * instead of zeros. A stale panel keeps its values, dimmed, under the reason.
 */
export function Panel({ title, right, children, style, status, testID, showPlaceholder = false, embedded = false }: Props) {
  const theme = useTheme();
  const kind = status?.kind ?? 'ready';
  const hasReadings = kind === 'ready' || kind === 'stale';
  const showBody = hasReadings || (showPlaceholder && kind !== 'error');
  const message = status?.kind === 'stale' ? status.message : '';
  const embeddedMessage = embedded && status !== undefined && status.kind !== 'ready' ? status.message : message;

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
          paddingHorizontal: embedded ? 20 : theme.spacing.xl,
          paddingVertical: embedded ? 12 : theme.spacing.xl,
          boxShadow: theme.mode === 'dark' ? '0 8px 24px #00000025' : '0 8px 24px #2238540a',
        },
        embedded ? styles.embedded : null,
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
      {status !== undefined && (hasReadings || embedded) ? (
        <View style={styles.freshness}>
          {embeddedMessage !== '' ? (
            <Text numberOfLines={1} style={[theme.typography.caption, { color: kind === 'error' ? theme.colors.danger : theme.colors.muted }]}>
              {embeddedMessage}
            </Text>
          ) : null}
        </View>
      ) : null}
      {!embedded && status !== undefined && status.kind !== 'ready' && status.kind !== 'stale' ? (
        <Text
          accessibilityLiveRegion="polite"
          style={[
            theme.typography.body,
            { color: kind === 'error' ? theme.colors.danger : theme.colors.muted },
          ]}
        >
          {status.message}
        </Text>
      ) : null}
      {showBody ? <View style={[styles.body, embedded && styles.embeddedBody, !hasReadings || kind === 'stale' ? styles.stale : null]}>{children}</View> : null}
    </View>
  );
}

const styles = StyleSheet.create({
  panel: { borderWidth: 1, gap: 12, minWidth: 0 },
  embedded: { borderRadius: 0, borderWidth: 0, boxShadow: 'none', gap: 8 },
  head: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
  freshness: { height: 14, marginBottom: -6, marginTop: -6 },
  body: { flexGrow: 1, gap: 12 },
  embeddedBody: { gap: 8 },
  stale: { opacity: 0.72 },
});
