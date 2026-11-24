import { createContext, useContext, type ComponentRef, type ReactNode, type Ref } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { PanelStatus } from '../../lib/panel-state';
import { PanelIcon, type IconName } from './PanelIcon';
import { useTheme } from '../../lib/theme-provider';

type Props = {
  title: string;
  right?: ReactNode;
  /** History and counters can arrive before a card's live packet. */
  footer?: ReactNode;
  children: ReactNode;
  fill?: boolean;
  style?: StyleProp<ViewStyle>;
  /** Extra body layout, such as spreading content over a tall card. */
  bodyStyle?: StyleProp<ViewStyle>;
  /** No packet has arrived yet: dim the body and say what the card waits for. */
  empty?: boolean;
  /** Loading, empty, stale or error. Missing means ready. */
  status?: PanelStatus;
  testID?: string;
  icon?: IconName;
  headingRef?: Ref<ComponentRef<typeof View>>;
};

/** Use the compact instrument-card spacing and type scale. */
export const PanelDense = createContext(false);
/** True on large desktops where cards are tall enough to fold tabs into one view. */
export const PanelExpanded = createContext(false);
/** Keep a panel's content and status when its card already supplies the heading. */
export const PanelEmbedded = createContext(false);

export function useDense(): boolean {
  return useContext(PanelDense);
}

export function useExpanded(): boolean {
  return useContext(PanelExpanded);
}

/**
 * A card with a heading. A panel that has no data shows one line that says why
 * instead of zeros. The navbar reports link freshness for the full console.
 */
export function Panel({ title, right, footer, children, fill, style, bodyStyle: bodyExtra, empty, status, testID, icon, headingRef }: Props) {
  const theme = useTheme();
  const kind = status?.kind ?? 'ready';
  const showBody = kind === 'ready' || kind === 'stale';
  const dense = useContext(PanelDense);
  const embedded = useContext(PanelEmbedded);
  const hasRight = right !== undefined && right !== null;
  const showHead = !embedded || hasRight;
  const bodyStyle = [styles.body, dense ? styles.bodyFill : null, fill && styles.fill, bodyExtra,
    kind === 'stale' ? styles.stale : null, empty ? styles.empty : null];
  const titleStyle = [dense ? theme.typography.subheading : theme.typography.headingSm, { color: theme.colors.text }];

  return (
    <View
      accessibilityLabel={title}
      accessibilityHint={status?.kind === 'stale' ? status.message : undefined}
      accessibilityRole="summary"
      style={[
        embedded ? styles.embedded : [styles.panel, {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.card,
          padding: dense ? theme.spacing.md : theme.spacing.lg,
        }],
        dense ? styles.dense : null,
        fill && styles.fill,
        style,
      ]}
      testID={testID}
    >
      {showHead ? <View style={embedded ? styles.embeddedHead : [styles.head, dense ? styles.headFill : null,
        { borderColor: theme.colors.border }]}
        testID={testID === undefined ? undefined : `${testID}-header`}>
        {embedded ? null : <View style={styles.title}>
          {icon ? <PanelIcon name={icon} /> : null}
          {headingRef === undefined ? (
            <Text accessibilityRole="header" aria-level={2} style={titleStyle}>{title}</Text>
          ) : (
            <View accessible accessibilityLabel={title} accessibilityRole="header" aria-level={2} ref={headingRef} tabIndex={-1}>
              <Text style={titleStyle}>{title}</Text>
            </View>
          )}
        </View>}
        {hasRight ? <View style={styles.right}>{right}</View> : null}
      </View> : null}
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
      {showBody ? <View style={bodyStyle}>
        {children}
        {empty ? <View pointerEvents="none" style={styles.emptyNote}>
          <View style={[StyleSheet.absoluteFill, { backgroundColor: theme.colors.card, opacity: 0.82 }]} />
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>Waiting for vehicle data</Text>
        </View> : null}
      </View> : null}
      {footer}
    </View>
  );
}

const styles = StyleSheet.create({
  fill: { flex: 1, minHeight: 0, overflow: 'hidden' },
  panel: { borderWidth: 1, gap: 12 },
  embedded: { gap: 8 },
  embeddedHead: { alignItems: 'flex-end' },
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
  right: { alignItems: 'center', flexDirection: 'row', flexShrink: 1, justifyContent: 'flex-end', gap: 8, marginStart: 'auto', minWidth: 0, maxWidth: '100%' },
  body: { gap: 12 },
  bodyFill: { gap: 6 },
  stale: { opacity: 0.55 },
  empty: { position: 'relative' },
  emptyNote: { position: 'absolute', top: 0, right: 0, bottom: 0, left: 0, alignItems: 'center', justifyContent: 'center' },
  dense: { gap: 6 },
  headFill: { paddingBottom: 6 },
});
