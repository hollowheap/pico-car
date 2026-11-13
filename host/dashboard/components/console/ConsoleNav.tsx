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
import Svg, { Circle, Path } from 'react-native-svg';

import type { Theme } from '../../lib/theme';
import { useTheme, useThemeToggle } from '../../lib/theme-provider';
import { CarIcon } from './CarIcon';

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

function ThemeToggle() {
  const { mode, colors } = useTheme();
  const toggle = useThemeToggle();
  const dark = mode === 'dark';
  return (
    <Pressable
      accessibilityLabel={dark ? 'Switch to light mode' : 'Switch to dark mode'}
      accessibilityRole="button"
      onPress={toggle}
      style={({ pressed }) => [
        styles.themeButton,
        {
          backgroundColor: pressed ? colors.elevated : colors.card,
          borderColor: colors.border,
          opacity: pressed ? 0.8 : 1,
        },
      ]}
      testID="theme-toggle"
    >
      <Svg height={19} width={19} viewBox="0 0 24 24" fill="none" stroke={colors.text} strokeWidth={1.75} strokeLinecap="round" strokeLinejoin="round">
        {dark ? (
          <>
            <Circle cx={12} cy={12} r={4} />
            <Path d="M12 2.5v2m0 15v2M2.5 12h2m15 0h2M5.3 5.3l1.4 1.4m10.6 10.6 1.4 1.4m0-13.4-1.4 1.4M6.7 17.3l-1.4 1.4" />
          </>
        ) : <Path d="M20.5 15.1A8.7 8.7 0 0 1 8.9 3.5a8.8 8.8 0 1 0 11.6 11.6Z" />}
      </Svg>
    </Pressable>
  );
}

export function ConsoleNav() {
  const theme = useTheme();

  return (
    <View style={[styles.header, { borderColor: theme.colors.border }]}>
      <Brand />
      <ThemeToggle />
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
  themeButton: { alignItems: 'center', borderRadius: 59, borderWidth: 1, height: 44, justifyContent: 'center', width: 44 },
  pressed: { opacity: 0.7 },
});
