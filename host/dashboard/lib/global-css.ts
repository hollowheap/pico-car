import { themeFor } from './theme';

const { accent, onAccent } = themeFor('dark').colors;

/**
 * Rules a component cannot set: the focus ring, the disabled cursor, text
 * selection and reduced motion. Colours come from lib/theme.ts. The web page
 * gets them from app/_layout.tsx (and from app/+html.tsx when the build is
 * static).
 */
export const GLOBAL_CSS = `
::selection { background: ${accent}; color: ${onAccent}; }
:focus-visible { outline: 2px solid ${accent}; outline-offset: 2px; }
[aria-disabled="true"], :disabled { cursor: not-allowed; }
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after { animation: none !important; transition: none !important; scroll-behavior: auto !important; }
}
`;
