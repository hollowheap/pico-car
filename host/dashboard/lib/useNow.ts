import { useEffect, useReducer, useState } from 'react';

import { useReducedMotion } from './useReducedMotion';

export const PLOT_PLAYOUT_MS = 220;

export function useNow(intervalMs = 250): number {
  const [, refresh] = useReducer((tick: number) => tick + 1, 0);

  useEffect(() => {
    const handle = setInterval(refresh, intervalMs);
    return () => clearInterval(handle);
  }, [intervalMs]);

  // Packet arrivals render between timer ticks. Age them at that render time.
  return Date.now();
}

/**
 * Display-rate clock delayed by a short playout buffer so incoming 5 Hz packets
 * land before reaching the right edge of scrolling plots.
 */
export function usePlotNow(fallbackNowMs: number, active: boolean, offsetMs = 0): number {
  const reducedMotion = useReducedMotion();
  const smooth = active && !reducedMotion && typeof requestAnimationFrame === 'function';
  const [wallNow, setWallNow] = useState(fallbackNowMs - offsetMs);

  useEffect(() => {
    if (!smooth) return;
    let frame = 0;
    const step = () => {
      setWallNow(Date.now() - PLOT_PLAYOUT_MS);
      frame = requestAnimationFrame(step);
    };
    frame = requestAnimationFrame(step);
    return () => cancelAnimationFrame(frame);
  }, [smooth]);

  return smooth ? wallNow + offsetMs : fallbackNowMs;
}
