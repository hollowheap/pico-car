import { Link, router } from 'expo-router';
import type { ReactNode } from 'react';
import { Pressable, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import Svg, { Circle, Path } from 'react-native-svg';

import { useTheme, useThemeToggle } from '../../lib/theme-provider';
import { CarIcon } from './CarIcon';

type Page = 'live' | 'run';
const ITEMS = [
  { key: 'live', label: 'Live', href: '/' },
  { key: 'run', label: 'Run review', href: '/run' },
] as const;

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

export function ConsoleNav({ active, status, onDiagnostics }: {
  active: Page;
  status?: ReactNode;
  onDiagnostics?: () => void;
}) {
  const { colors, typography } = useTheme();
  const { width } = useWindowDimensions();
  const narrow = width < 760;
  const compact = width <= 360;
  const links = (
    <View style={styles.links}>
      {ITEMS.map(({ key, label, href }) => {
        const selected = key === active;
        return (
          <Pressable
            accessibilityRole="link"
            accessibilityState={{ selected }}
            key={key}
            onPress={() => router.navigate(href)}
            style={({ pressed }) => [
              styles.item,
              { borderBottomColor: selected ? colors.accent : 'transparent', backgroundColor: pressed ? colors.elevated : 'transparent' },
            ]}
          >
            <Text style={[typography.body, styles.navLabel, { color: selected ? colors.text : colors.muted, fontWeight: selected ? '700' : '500' }]}>
              {label}
            </Text>
          </Pressable>
        );
      })}
    </View>
  );

  return (
    <View style={[styles.header, { borderColor: colors.border }]}>
      <View style={[styles.headerRow, compact && styles.compactHeader]}>
        <Link href="/" asChild>
          <Pressable accessibilityLabel="Pico Car home" accessibilityRole="link" style={styles.brand}>
            <View style={[styles.brandIcon, { backgroundColor: colors.accent }]}>
              <CarIcon color={colors.onAccent} size={22} />
            </View>
            <Text style={[typography.headingSm, styles.brandText, { color: colors.text }]}>Pico Car</Text>
          </Pressable>
        </Link>
        {!narrow ? links : null}
        <View style={styles.actions}>
          {!narrow ? status : null}
          {onDiagnostics === undefined ? null : (
            <Pressable
              accessibilityLabel="Diagnostics"
              accessibilityRole="button"
              onPress={onDiagnostics}
              testID="open-diagnostics"
              style={({ pressed }) => [
                styles.diagnostics,
                { backgroundColor: pressed ? colors.elevated : colors.card, borderColor: colors.border },
              ]}
            >
              <Svg height={16} width={16} viewBox="0 0 24 24" fill="none" stroke={colors.accent} strokeWidth={1.8} strokeLinecap="round">
                <Path d="M3 12h4l3-7 4 14 3-7h4" />
              </Svg>
              <Text style={[typography.caption, { color: colors.text, fontWeight: '600' }]}>Diagnostics</Text>
            </Pressable>
          )}
          <ThemeToggle />
        </View>
      </View>
      {narrow ? (
        <View style={[styles.mobileNav, narrow && status !== undefined && styles.compactNav]}>
          {links}
          {status}
        </View>
      ) : null}
    </View>
  );
}

const styles = StyleSheet.create({
  header: { borderBottomWidth: 1, gap: 2 },
  headerRow: { alignItems: 'center', flexDirection: 'row', gap: 20, minHeight: 56 },
  compactHeader: { gap: 8 },
  compactNav: { alignItems: 'flex-start', flexDirection: 'column', gap: 1 },
  mobileNav: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 8, justifyContent: 'space-between' },
  brand: { alignItems: 'center', flexDirection: 'row', gap: 8, minHeight: 44 },
  brandIcon: { alignItems: 'center', borderRadius: 11, height: 35, justifyContent: 'center', width: 35 },
  brandText: { fontSize: 18, letterSpacing: -0.5 },
  actions: { alignItems: 'center', flexDirection: 'row', gap: 8, marginLeft: 'auto' },
  links: { alignItems: 'center', flexDirection: 'row', gap: 8 },
  item: { alignItems: 'center', borderBottomWidth: 2, justifyContent: 'center', minHeight: 44, paddingHorizontal: 7 },
  navLabel: { fontSize: 14, lineHeight: 20 },
  diagnostics: { alignItems: 'center', borderRadius: 20, borderWidth: 1, flexDirection: 'row', gap: 6, minHeight: 40, paddingHorizontal: 10 },
  themeButton: { alignItems: 'center', borderRadius: 22, borderWidth: 1, height: 44, justifyContent: 'center', width: 44 },
});
