import { expect, it } from 'vitest';

import { initialLogWindow, shiftWindowNewer, shiftWindowOlder, syncLogWindow } from '../../src/lib/log-window';

it('starts pinned to the latest entries and lazily shifts and unloads windows in both directions', () => {
  let w = initialLogWindow(100, 40);
  expect(w).toEqual({ start: 60, end: 100, pinned: true });

  // Reaching top loads 20 older entries and unloads 20 newest entries.
  const older = shiftWindowOlder(w, 100, 40, 20);
  expect(older.prepended).toBe(20);
  expect(older.next).toEqual({ start: 40, end: 80, pinned: false });

  // While unpinned, new logs arriving do not jump the viewport.
  w = syncLogWindow(older.next, 110, 40);
  expect(w).toEqual({ start: 40, end: 80, pinned: false });

  // Reaching bottom loads 20 newer entries and unloads 20 older entries.
  const newer1 = shiftWindowNewer(w, 110, 40, 20);
  expect(newer1.appended).toBe(20);
  expect(newer1.next).toEqual({ start: 60, end: 100, pinned: false });

  // Reaching bottom again catches up to latest and re-pins.
  const newer2 = shiftWindowNewer(newer1.next, 110, 40, 20);
  expect(newer2.appended).toBe(10);
  expect(newer2.next).toEqual({ start: 70, end: 110, pinned: true });

  // Once pinned, new logs advance the window automatically.
  expect(syncLogWindow(newer2.next, 115, 40)).toEqual({ start: 75, end: 115, pinned: true });
});
