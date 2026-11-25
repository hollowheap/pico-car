import { useEffect, useRef, useState, type ComponentRef } from 'react';
import type { NativeScrollEvent, NativeSyntheticEvent, ScrollView } from 'react-native';

import {
  DEFAULT_LOG_STEP,
  DEFAULT_LOG_WINDOW,
  initialLogWindow,
  shiftWindowNewer,
  shiftWindowOlder,
  syncLogWindow,
  type LogWindow,
} from './log-window';

const SCROLL_EDGE_PX = 32;

export function useLogWindow(
  total: number,
  rowHeightPx: number,
  options?: { windowSize?: number; step?: number; resetKey?: string | number },
) {
  const windowSize = options?.windowSize ?? DEFAULT_LOG_WINDOW;
  const step = options?.step ?? DEFAULT_LOG_STEP;
  const resetKey = options?.resetKey;
  const scrollRef = useRef<ComponentRef<typeof ScrollView>>(null);
  const shiftingRef = useRef(false);
  const [state, setState] = useState<{ key: string | number | undefined; win: LogWindow }>(() => ({
    key: resetKey,
    win: initialLogWindow(total, windowSize),
  }));
  const setWindow = (next: LogWindow) => setState({ key: resetKey, win: next });
  const baseWindow = state.key === resetKey ? state.win : initialLogWindow(total, windowSize);
  const win = syncLogWindow(baseWindow, total, windowSize);
  const olderCount = win.start;
  const newerCount = Math.max(0, total - win.end);

  useEffect(() => {
    if (state.key !== resetKey) {
      setState({ key: resetKey, win: initialLogWindow(total, windowSize) });
    }
  }, [resetKey, state.key, total, windowSize]);

  useEffect(() => {
    if (!win.pinned || win.end === win.start) return;
    const frame = requestAnimationFrame(() => scrollRef.current?.scrollToEnd({ animated: false }));
    return () => cancelAnimationFrame(frame);
  }, [win.pinned, win.end, win.start, resetKey]);

  const loadOlder = (currentY = 0) => {
    const { next, prepended } = shiftWindowOlder(win, total, windowSize, step);
    if (prepended === 0) return;
    shiftingRef.current = true;
    setWindow(next);
    requestAnimationFrame(() => {
      scrollRef.current?.scrollTo({ y: currentY + prepended * rowHeightPx, animated: false });
      shiftingRef.current = false;
    });
  };

  const loadNewer = (currentY = 0) => {
    const { next, appended } = shiftWindowNewer(win, total, windowSize, step);
    if (appended === 0) {
      if (next.pinned !== win.pinned) setWindow(next);
      return;
    }
    const unloadedTop = next.start - win.start;
    shiftingRef.current = true;
    setWindow(next);
    requestAnimationFrame(() => {
      if (next.pinned) {
        scrollRef.current?.scrollToEnd({ animated: false });
      } else {
        scrollRef.current?.scrollTo({ y: Math.max(0, currentY - unloadedTop * rowHeightPx), animated: false });
      }
      shiftingRef.current = false;
    });
  };

  const onScroll = (event: NativeSyntheticEvent<NativeScrollEvent>) => {
    if (shiftingRef.current) return;
    const { contentOffset, contentSize, layoutMeasurement } = event.nativeEvent;
    const y = contentOffset.y;
    const distFromBottom = contentSize.height - (y + layoutMeasurement.height);
    if (y <= SCROLL_EDGE_PX && olderCount > 0) {
      loadOlder(y);
    } else if (distFromBottom <= SCROLL_EDGE_PX) {
      if (newerCount > 0) loadNewer(y);
      else if (!win.pinned) setWindow({ ...win, pinned: true });
    } else if (win.pinned && distFromBottom > SCROLL_EDGE_PX * 2) {
      setWindow({ ...win, pinned: false });
    }
  };

  const jumpToIndex = (index: number, listLength = total) => {
    const start = Math.max(0, index - Math.floor(windowSize / 2));
    const end = Math.min(listLength, start + windowSize);
    setWindow({ start, end, pinned: end === listLength });
  };

  const resetToLatest = (listLength = total) => {
    setWindow(initialLogWindow(listLength, windowSize));
  };

  const unpin = () => {
    if (win.pinned) setWindow({ ...win, pinned: false });
  };

  return {
    scrollRef,
    window: win,
    olderCount,
    newerCount,
    loadOlder,
    loadNewer,
    onScroll,
    jumpToIndex,
    resetToLatest,
    unpin,
  };
}
