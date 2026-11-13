import { createContext, useCallback, useContext, useEffect, useMemo, useState, type ReactNode } from 'react';
import { Platform, useColorScheme } from 'react-native';

import { themeFor, type ColorScheme, type Theme } from './theme';

const STORAGE_KEY = 'pico-car.color-scheme';
const ThemeContext = createContext<Theme>(themeFor('light'));
const ThemeToggleContext = createContext<() => void>(() => {});

export function ThemeProvider({ children, mode }: { children: ReactNode; mode?: ColorScheme }) {
  const system = useColorScheme();
  const [preference, setPreference] = useState<ColorScheme | null>(null);
  const resolved = mode ?? preference ?? (system === 'dark' ? 'dark' : 'light');
  const theme = useMemo(() => themeFor(resolved), [resolved]);

  useEffect(() => {
    if (Platform.OS !== 'web') return;
    try {
      const saved = window.localStorage.getItem(STORAGE_KEY);
      if (saved === 'light' || saved === 'dark') setPreference(saved);
    } catch {
      // The theme can still be toggled when browser storage is unavailable.
    }
  }, []);

  useEffect(() => {
    if (Platform.OS === 'web') document.documentElement.style.colorScheme = resolved;
  }, [resolved]);

  const toggle = useCallback(() => {
    const next = resolved === 'dark' ? 'light' : 'dark';
    setPreference(next);
    if (Platform.OS === 'web') {
      try {
        window.localStorage.setItem(STORAGE_KEY, next);
      } catch {
        // Keep the in-memory preference for this page.
      }
    }
  }, [resolved]);

  return (
    <ThemeContext.Provider value={theme}>
      <ThemeToggleContext.Provider value={toggle}>{children}</ThemeToggleContext.Provider>
    </ThemeContext.Provider>
  );
}

export function useTheme(): Theme {
  return useContext(ThemeContext);
}

export function useThemeToggle(): () => void {
  return useContext(ThemeToggleContext);
}
