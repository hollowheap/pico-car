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

import type { Theme } from '../../lib/theme';
import { useTheme } from '../../lib/theme-provider';
import { CarIcon } from './CarIcon';

type Page = 'live' | 'history';

const ITEMS: { key: Page; label: string; href: '/' | '/history' }[] = [
  { key: 'live', label: 'Live', href: '/' },
  { key: 'history', label: 'History', href: '/history' },
];

type Props = {
  active: Page;
};

function webFocusRing(theme: Theme): ViewStyle | undefined {
  if (Platform.OS !== 'web') {
    return undefined;
  }
  return {
    outlineStyle: 'solid',
    outlineWidth: 2,
    outlineColor: theme.colors.accent,
    outlineOffset: 2,
  } as ViewStyle;
}

function Brand() {
  const theme = useTheme();
  const [focused, setFocused] = useState(false);
  const [hovered, setHovered] = useState(false);

  return (
    <Pressable
      accessibilityLabel="PICO-CAR home"
      accessibilityRole="link"
      focusable
      onBlur={() => setFocused(false)}
      onFocus={() => setFocused(true)}
      onHoverIn={() => setHovered(true)}
      onHoverOut={() => setHovered(false)}
      onPress={() => router.push('/')}
      style={({ pressed }) => [
        styles.brand,
        focused ? webFocusRing(theme) : null,
        pressed && styles.pressed,
      ]}
    >
      <CarIcon color={theme.colors.accent} />
      <Text
        style={[
          theme.typography.headingSm,
          styles.brandText,
          { color: hovered ? theme.colors.accent : theme.colors.text },
        ]}
      >
        PICO-CAR
      </Text>
    </Pressable>
  );
}

function NavItem({
  active,
  href,
  label,
}: {
  active: boolean;
  href: '/' | '/history';
  label: string;
}) {
  const theme = useTheme();
  const [focused, setFocused] = useState(false);
  const [hovered, setHovered] = useState(false);
  const color =
    active || hovered ? theme.colors.text : theme.colors.muted;
  const indicatorColor = active
    ? theme.colors.accent
    : hovered
      ? theme.colors.border
      : 'transparent';

  return (
    <Pressable
      accessibilityRole="link"
      accessibilityState={{ selected: active }}
      focusable
      onBlur={() => setFocused(false)}
      onFocus={() => setFocused(true)}
      onHoverIn={() => setHovered(true)}
      onHoverOut={() => setHovered(false)}
      onPress={() => router.push(href)}
      style={({ pressed }) => [
        styles.item,
        focused ? webFocusRing(theme) : null,
        pressed && styles.pressed,
      ]}
    >
      <Text
        style={[
          theme.typography.body,
          { color, fontWeight: active ? '600' : '400' },
        ]}
      >
        {label}
      </Text>
      <View
        style={[styles.indicator, { backgroundColor: indicatorColor }]}
      />
    </Pressable>
  );
}

export function ConsoleNav({ active }: Props) {
  const theme = useTheme();

  return (
    <View style={[styles.header, { borderColor: theme.colors.border }]}>
      <Brand />
      <View style={styles.links}>
        {ITEMS.map((item) => (
          <NavItem
            key={item.key}
            active={active === item.key}
            href={item.href}
            label={item.label}
          />
        ))}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  header: {
    alignItems: 'center',
    borderBottomWidth: 1,
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 12,
    justifyContent: 'space-between',
  },
  brand: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 8,
    minHeight: 44,
  },
  brandText: { fontWeight: '600', letterSpacing: 1.2 },
  links: { flexDirection: 'row', flexWrap: 'wrap', gap: 4 },
  item: {
    alignItems: 'center',
    justifyContent: 'center',
    minHeight: 44,
    minWidth: 44,
    paddingHorizontal: 12,
    position: 'relative',
  },
  pressed: { opacity: 0.7 },
  indicator: {
    borderRadius: 2,
    bottom: 0,
    height: 2,
    left: 12,
    position: 'absolute',
    right: 12,
  },
});
