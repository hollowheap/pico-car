import type { ReactNode } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import { useTheme } from '../../lib/theme-provider';

type Props = {
  title: string;
  right?: ReactNode;
  children: ReactNode;
  style?: StyleProp<ViewStyle>;
};

export function Panel({ title, right, children, style }: Props) {
  const theme = useTheme();

  return (
    <View
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
    >
      <View style={styles.head}>
        <Text
          style={[theme.typography.headingSm, { color: theme.colors.text }]}
        >
          {title}
        </Text>
        {right}
      </View>
      {children}
    </View>
  );
}

const styles = StyleSheet.create({
  panel: { borderWidth: 1, flexGrow: 1, flexBasis: 260, gap: 12 },
  head: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 8,
  },
});
