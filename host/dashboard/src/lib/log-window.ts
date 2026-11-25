export type LogWindow = {
  /** Inclusive index into the full oldest-first log list. */
  start: number;
  /** Exclusive index into the full oldest-first log list. */
  end: number;
  /** True when the viewport follows the latest entries at the bottom. */
  pinned: boolean;
};

export const DEFAULT_LOG_WINDOW = 40;
export const DEFAULT_LOG_STEP = 20;

export function initialLogWindow(total: number, size = DEFAULT_LOG_WINDOW): LogWindow {
  const end = Math.max(0, total);
  return { start: Math.max(0, end - size), end, pinned: true };
}

/** Reconcile the mounted slice when the backing log grows or resets. */
export function syncLogWindow(window: LogWindow, total: number, size = DEFAULT_LOG_WINDOW): LogWindow {
  const safeTotal = Math.max(0, total);
  if (window.pinned || window.end > safeTotal) {
    return { start: Math.max(0, safeTotal - size), end: safeTotal, pinned: true };
  }
  const end = Math.min(safeTotal, Math.max(window.end, Math.min(safeTotal, window.start + size)));
  const start = Math.max(0, Math.min(window.start, end));
  return { start, end, pinned: end === safeTotal && window.pinned };
}

/** Shift the mounted slice toward older entries (scroll reached top), unloading newer ones. */
export function shiftWindowOlder(
  window: LogWindow,
  total: number,
  size = DEFAULT_LOG_WINDOW,
  step = DEFAULT_LOG_STEP,
): { next: LogWindow; prepended: number } {
  if (window.start <= 0) return { next: window, prepended: 0 };
  const start = Math.max(0, window.start - step);
  const prepended = window.start - start;
  const end = Math.min(Math.max(0, total), start + size);
  return { next: { start, end, pinned: false }, prepended };
}

/** Shift the mounted slice toward newer entries (scroll reached bottom), unloading older ones. */
export function shiftWindowNewer(
  window: LogWindow,
  total: number,
  size = DEFAULT_LOG_WINDOW,
  step = DEFAULT_LOG_STEP,
): { next: LogWindow; appended: number } {
  const safeTotal = Math.max(0, total);
  if (window.end >= safeTotal) {
    return { next: window.pinned ? window : { ...window, pinned: true }, appended: 0 };
  }
  const end = Math.min(safeTotal, window.end + step);
  const appended = end - window.end;
  const start = Math.max(0, end - size);
  return { next: { start, end, pinned: end === safeTotal }, appended };
}
