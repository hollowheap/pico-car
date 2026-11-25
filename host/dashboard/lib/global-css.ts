import { themeFor } from './theme';

const { accent, onAccent } = themeFor('dark').colors;

/**
 * Rules a component cannot set: the focus ring, the disabled cursor, the
 * hover state, text selection and reduced motion. Colours come from lib/theme.ts. The web page
 * gets them from app/_layout.tsx (and from app/+html.tsx when the build is
 * static).
 */
export const GLOBAL_CSS = `
::selection { background: ${accent}; color: ${onAccent}; }
:focus-visible { outline: 2px solid ${accent}; outline-offset: 2px; }
[aria-disabled="true"], :disabled { cursor: not-allowed; }
button:not(:disabled):hover, [role="button"]:not([aria-disabled="true"]):hover { filter: brightness(1.12); }
@keyframes console-enter { from { opacity: 0.65; } to { opacity: 1; } }
@media (prefers-reduced-motion: no-preference) {
  [data-testid="console-page"] { animation: console-enter 160ms ease-out; }
}
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after { animation: none !important; transition: none !important; scroll-behavior: auto !important; }
}
`;
