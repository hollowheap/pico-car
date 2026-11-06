export type ColorScheme = 'light' | 'dark';

export type ThemeColors = {
  canvas: string;
  card: string;
  elevated: string;
  text: string;
  muted: string;
  border: string;
  accent: string;
  onAccent: string;
  danger: string;
  dangerSurface: string;
};

/* Dark is the documented REKKI source. The light set derives a near-white
 * surface stack while keeping Signal Blue as the single accent. */
const darkColors: ThemeColors = {
  canvas: '#000000',
  card: '#0d0d0d',
  elevated: '#1f1f1f',
  text: '#ffffff',
  muted: '#858585',
  border: 'rgba(255, 255, 255, 0.12)',
  accent: '#0063e1',
  onAccent: '#ffffff',
  danger: '#ffb4ab',
  dangerSurface: '#2b2c2e',
};

const lightColors: ThemeColors = {
  canvas: '#f7f7f5',
  card: '#ffffff',
  elevated: '#eeeeee',
  text: '#0d0d0d',
  muted: '#52525b',
  border: 'rgba(0, 0, 0, 0.12)',
  accent: '#0063e1',
  onAccent: '#ffffff',
  danger: '#b91c1c',
  dangerSurface: '#fef2f2',
};

export const spacing = {
  xs: 4,
  sm: 8,
  md: 12,
  lg: 16,
  xl: 20,
  xxl: 24,
  section: 80,
} as const;

export const radii = {
  nav: 2,
  input: 8,
  card: 16,
  image: 24,
  button: 59,
} as const;

export const typography = {
  caption: {
    fontSize: 12,
    lineHeight: 14,
    letterSpacing: -0.06,
    fontWeight: '400' as const,
  },
  body: {
    fontSize: 16,
    lineHeight: 24,
    letterSpacing: -0.112,
    fontWeight: '400' as const,
  },
  subheading: {
    fontSize: 18,
    lineHeight: 25,
    letterSpacing: -0.27,
    fontWeight: '400' as const,
  },
  headingSm: {
    fontSize: 20,
    lineHeight: 27,
    letterSpacing: -0.54,
    fontWeight: '400' as const,
  },
  heading: {
    fontSize: 24,
    lineHeight: 32,
    letterSpacing: -0.648,
    fontWeight: '400' as const,
  },
  headingLg: {
    fontSize: 56,
    lineHeight: 70,
    letterSpacing: -2.24,
    fontWeight: '400' as const,
  },
  display: {
    fontSize: 86,
    lineHeight: 86,
    letterSpacing: -5.5,
    fontWeight: '400' as const,
  },
  label: {
    fontSize: 12,
    lineHeight: 14,
    letterSpacing: 0.4,
    fontWeight: '600' as const,
  },
} as const;

export type Theme = {
  mode: ColorScheme;
  colors: ThemeColors;
  spacing: typeof spacing;
  radii: typeof radii;
  typography: typeof typography;
};

export function themeFor(mode: ColorScheme): Theme {
  return {
    mode,
    colors: mode === 'dark' ? darkColors : lightColors,
    spacing,
    radii,
    typography,
  };
}
