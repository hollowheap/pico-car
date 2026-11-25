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

export function Brand({ iconOnly = false }: { iconOnly?: boolean }) {
  const theme = useTheme();

  return (
    <View accessibilityLabel={iconOnly ? 'Pico Car' : undefined} style={styles.brand} testID="console-brand">
      <CarIcon color={theme.colors.accent} />
      {iconOnly ? null : <Text style={[theme.typography.headingSm, styles.brandText, { color: theme.colors.text }]}>
        PICO-CAR
      </Text>}
    </View>
  );
}

export function ThemeToggle() {
  const theme = useTheme();
  const { mode, colors } = theme;
  const toggle = useThemeToggle();
  const [focused, setFocused] = useState(false);
  const dark = mode === 'dark';
  return (
    <Pressable
      accessibilityLabel={dark ? 'Switch to light mode' : 'Switch to dark mode'}
      accessibilityRole="button"
      onBlur={() => setFocused(false)}
      onFocus={() => setFocused(true)}
      onPress={toggle}
      style={({ pressed }) => [
        focused ? webFocusRing(theme) : null,
        styles.themeButton,
        {
          backgroundColor: pressed ? colors.elevated : colors.card,
          borderColor: colors.border,
          borderRadius: theme.radii.control,
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

const styles = StyleSheet.create({
  brand: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 8,
    minHeight: 44,
  },
  brandText: { fontWeight: '600', letterSpacing: 1.2 },
  themeButton: { alignItems: 'center', borderWidth: 1, height: 44, justifyContent: 'center', width: 44 },
});
