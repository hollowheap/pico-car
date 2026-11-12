import { DarkTheme, DefaultTheme, Stack, ThemeProvider as NavigationTheme } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import { useEffect } from 'react';
import { Platform } from 'react-native';

import { GLOBAL_CSS } from '../lib/global-css';

import { TelemetryProvider } from '../lib/telemetry-context';
import { ThemeProvider, useTheme } from '../lib/theme-provider';

function RootNavigator() {
  const theme = useTheme();

  const base = theme.mode === 'dark' ? DarkTheme : DefaultTheme;
  const navigation = {
    ...base,
    colors: { ...base.colors, background: theme.colors.canvas, card: theme.colors.card, border: theme.colors.border },
  };

  return (
    <NavigationTheme value={navigation}>
      <StatusBar style={theme.mode === 'dark' ? 'light' : 'dark'} />
      <Stack
        screenOptions={{
          headerShown: false,
          contentStyle: { backgroundColor: theme.colors.canvas },
        }}
      />
    </NavigationTheme>
  );
}

/** The single-page web build has no +html template, so the global rules go in at run time. */
function useGlobalCss() {
  useEffect(() => {
    if (Platform.OS !== 'web') return undefined;
    const style = document.createElement('style');
    style.setAttribute('data-global-css', '');
    style.textContent = GLOBAL_CSS;
    document.head.appendChild(style);
    return () => style.remove();
  }, []);
}

export default function RootLayout() {
  useGlobalCss();

  return (
    <ThemeProvider>
      <TelemetryProvider>
        <RootNavigator />
      </TelemetryProvider>
    </ThemeProvider>
  );
}
