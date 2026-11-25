import { createContext, useContext, useMemo, type ReactNode } from 'react';
import { useColorScheme } from 'react-native';

import { themeFor, type ColorScheme, type Theme } from './theme';

const ThemeContext = createContext<Theme>(themeFor('dark'));

type Props = {
  children: ReactNode;
  mode?: ColorScheme;
};

export function ThemeProvider({ children, mode }: Props) {
  const system = useColorScheme();
  const resolved: ColorScheme =
    mode ?? (system === 'light' ? 'light' : 'dark');
  const theme = useMemo(() => themeFor(resolved), [resolved]);

  return (
    <ThemeContext.Provider value={theme}>{children}</ThemeContext.Provider>
  );
}

export function useTheme(): Theme {
  return useContext(ThemeContext);
}
