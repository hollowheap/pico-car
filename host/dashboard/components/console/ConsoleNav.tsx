import { router } from 'expo-router';
import { useState } from 'react';
import {
  Platform,
  Pressable,
  StyleSheet,
  Text,
  View,
  type ViewStyle,
} from 'react-native';

import { useTheme } from '../../lib/theme-provider';

type Page = 'live' | 'history' | 'controls';

const ITEMS: { key: Page; label: string; href: '/' | '/history' | '/controls' }[] = [
  { key: 'live', label: 'Live', href: '/' },
  { key: 'history', label: 'History', href: '/history' },
  { key: 'controls', label: 'Controls', href: '/controls' },
];

type Props = {
  active: Page;
};

function NavItem({
  active,
  href,
  label,
}: {
  active: boolean;
  href: '/' | '/history' | '/controls';
  label: string;
}) {
  const theme = useTheme();
  const [focused, setFocused] = useState(false);
  const focusRing =
    Platform.OS === 'web'
      ? ({
          outlineStyle: 'solid',
          outlineWidth: 2,
          outlineColor: theme.colors.accent,
          outlineOffset: 2,
        } as ViewStyle)
      : {};

  return (
    <Pressable
      accessibilityRole="link"
      accessibilityState={{ selected: active }}
      focusable
      onBlur={() => setFocused(false)}
      onFocus={() => setFocused(true)}
      onPress={() => router.push(href)}
      style={({ pressed }) => [
        styles.item,
        focused && focusRing,
        pressed && styles.pressed,
      ]}
    >
      <Text
        style={[
          theme.typography.body,
          {
            color: active ? theme.colors.accent : theme.colors.text,
            fontWeight: active ? '600' : '400',
          },
        ]}
      >
        {label}
      </Text>
      <View
        style={[
          styles.indicator,
          {
            backgroundColor: active
              ? theme.colors.accent
              : 'transparent',
          },
        ]}
      />
    </Pressable>
  );
}

export function ConsoleNav({ active }: Props) {
  const theme = useTheme();

  return (
    <View style={[styles.row, { borderColor: theme.colors.border }]}>
      {ITEMS.map((item) => (
        <NavItem
          key={item.key}
          active={active === item.key}
          href={item.href}
          label={item.label}
        />
      ))}
    </View>
  );
}

const styles = StyleSheet.create({
  row: { borderBottomWidth: 1, flexDirection: 'row', gap: 20 },
  item: { gap: 4, paddingBottom: 8, paddingTop: 4 },
  pressed: { opacity: 0.7 },
  indicator: { borderRadius: 2, height: 2, width: '100%' },
});
