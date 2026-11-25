export const PLOT_WINDOW_MS = 10_000;

export function timeToX(at: number, nowMs: number, width: number, windowMs = PLOT_WINDOW_MS): number {
  return width * (1 - (nowMs - at) / windowMs);
}

export function clampX(x: number, width: number): number {
  return Math.max(0, Math.min(width, x));
}
