import { createContext, useContext, type ReactNode } from 'react';
import { ScrollView, StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

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
  /** In a fill cell: no scroller; the body stretches so a drawing in it can scale to the cell. */
  stretch?: boolean;
};

/**
 * True inside a fixed-height grid: a panel then fills its cell and its body
 * scrolls inside the panel, so the page itself never scrolls.
 */
export const PanelFill = createContext(false);

/** True inside the one-screen grid: drop captions, use smaller type. */
export const PanelDense = createContext(false);

export function useDense(): boolean {
  return useContext(PanelDense);
}

/**
 * A card with a heading. A panel that has no data shows one line that says why
 * instead of zeros. A stale panel keeps its values at full contrast, with STALE in
 * the heading (dimming the body put muted text below a readable contrast).
 */
export function Panel({ title, right, children, style, status, testID, icon, stretch = false }: Props) {
  const theme = useTheme();
  const kind = status?.kind ?? 'ready';
  const showBody = kind === 'ready' || kind === 'stale';
  const fill = useContext(PanelFill);
  const dense = useContext(PanelDense);
  const bodyStyle = [styles.body, dense ? styles.bodyFill : null];

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
          padding: dense ? theme.spacing.md : theme.spacing.lg,
        },
        dense ? styles.dense : null,
        fill ? styles.fill : null,
        style,
      ]}
      testID={testID}
    >
      <View style={[styles.head, dense ? styles.headFill : null, { borderColor: theme.colors.border }]}>
        <View style={styles.title}>
          {icon ? <PanelIcon name={icon} /> : null}
          <Text accessibilityRole="header" aria-level={2} style={[dense ? theme.typography.subheading : theme.typography.headingSm, { color: theme.colors.text }]}>
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
      {showBody && fill && stretch ? <View style={[bodyStyle, styles.scroll]}>{children}</View> : null}
      {showBody && fill && !stretch ? (
        <ScrollView contentContainerStyle={bodyStyle} style={styles.scroll}>
          {children}
        </ScrollView>
      ) : null}
      {showBody && !fill ? <View style={bodyStyle}>{children}</View> : null}
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
    paddingBottom: 16,
  },
  title: { alignItems: 'center', flexDirection: 'row', gap: 8 },
  right: { alignItems: 'center', flexDirection: 'row', gap: 8 },
  body: { gap: 12 },
  bodyFill: { gap: 8 },
  dense: { gap: 8 },
  fill: { flex: 1, minHeight: 0 },
  headFill: { paddingBottom: 8 },
  scroll: { flex: 1, minHeight: 0 },
});
