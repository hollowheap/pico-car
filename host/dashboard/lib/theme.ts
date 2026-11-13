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
  warning: string;
  dangerSurface: string;
};

const darkColors: ThemeColors = {
  canvas: '#101114',
  card: '#191b20',
  elevated: '#252830',
  text: '#f5f6f8',
  muted: '#aab0be',
  border: 'rgba(224, 232, 255, 0.14)',
  accent: '#71aaff',
  onAccent: '#101a2a',
  danger: '#ffb4ab',
  warning: '#f8bc64',
  dangerSurface: '#352427',
};

const lightColors: ThemeColors = {
  canvas: '#f3f5f8',
  card: '#ffffff',
  elevated: '#eef2f7',
  text: '#182230',
  muted: '#576579',
  border: 'rgba(26, 41, 66, 0.11)',
  accent: '#075dd4',
  onAccent: '#ffffff',
  danger: '#b91c1c',
  warning: '#9a5107',
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
  card: 22,
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
    letterSpacing: -0.55,
    fontWeight: '600' as const,
  },
  heading: {
    fontSize: 24,
    lineHeight: 32,
    letterSpacing: -0.72,
    fontWeight: '600' as const,
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
