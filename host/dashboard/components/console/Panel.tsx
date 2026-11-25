import type { ReactNode } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { PanelStatus } from '../../lib/panel-state';
import { PanelIcon, type IconName } from './PanelIcon';
import { useTheme } from '../../lib/theme-provider';

type Props = {
  title: string;
  right?: ReactNode;
  children: ReactNode;
  style?: StyleProp<ViewStyle>;
  /** Loading, empty, stale or error. Missing means ready. */
  status?: PanelStatus;
  testID?: string;
  icon?: IconName;
};

/**
 * A card with a heading. A panel that has no data shows one line that says why
 * instead of zeros. A stale panel keeps its values, dimmed, with STALE in the heading.
 */
export function Panel({ title, right, children, style, status, testID, icon }: Props) {
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
      <View style={[styles.head, { borderColor: theme.colors.border }]}>
        <View style={styles.title}>
          {icon ? <PanelIcon name={icon} /> : null}
          <Text accessibilityRole="header" style={[theme.typography.headingSm, { color: theme.colors.text }]}>
            {title}
          </Text>
        </View>
        <View style={styles.right}>
          {kind === 'stale' ? (
            <Text
              accessibilityLabel={status?.kind === 'stale' ? status.message : undefined}
              accessibilityLiveRegion="polite"
              style={[theme.typography.label, { color: theme.colors.muted }]}
            >
              STALE
            </Text>
          ) : null}
          {right}
        </View>
      </View>
      {status !== undefined && status.kind !== 'ready' && status.kind !== 'stale' ? (
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
    flexWrap: 'wrap',
    gap: 8,
    borderBottomWidth: 1,
    paddingBottom: 12,
  },
  title: { alignItems: 'center', flexDirection: 'row', gap: 8 },
  right: { alignItems: 'center', flexDirection: 'row', gap: 8 },
  body: { gap: 12 },
  stale: { opacity: 0.5 },
});
