import { useFocusEffect } from 'expo-router';
import { useCallback, useState } from 'react';
import { Platform } from 'react-native';

import { eventKey, type Course } from './course';
import type { EventRow } from './telemetry-store';

type Selection =
  | { kind: 'event'; key: string; restoreFocus?: () => void }
  | { kind: 'cluster'; keys: Set<string>; restoreFocus: () => void };

export function useEventInspection(
  course: Course,
  events: EventRow[],
  runStart: number | null,
  requestedEvent?: string,
) {
  // Undefined allows a drill-in request to resolve after the history seed.
  const [selection, setSelection] = useState<Selection | null>();
  const [scope, setScope] = useState({ runStart, requestedEvent });
  const runChanged = scope.runStart !== runStart;
  const routeChanged = scope.requestedEvent !== requestedEvent;
  const active = routeChanged ? undefined : runChanged && selection !== undefined ? null : selection;
  const selectedKey = active === undefined ? requestedEvent : active?.kind === 'event' ? active.key : undefined;
  const selected = selectedKey === undefined ? null
    : events.find((event) => eventKey(event) === selectedKey) ?? null;
  const cluster = active?.kind !== 'cluster' ? []
    : events.filter((event) => active.keys.has(eventKey(event))).sort((a, b) => b.timestamp - a.timestamp);

  if (runChanged || routeChanged) {
    setScope({ runStart, requestedEvent });
    setSelection(active);
  } else if (active?.kind === 'event' && selected === null) {
    setSelection(null);
  } else if (active?.kind === 'cluster' && cluster.length === 0) {
    setSelection(null);
  } else if (active === undefined && selected !== null) {
    setSelection({ kind: 'event', key: eventKey(selected) });
  }

  const restoreFocus = active?.restoreFocus;
  const clear = useCallback(() => {
    restoreFocus?.();
    setSelection(null);
  }, [restoreFocus]);
  const detailOpen = selected !== null || cluster.length > 0;
  useFocusEffect(useCallback(() => {
    if (Platform.OS !== 'web' || !detailOpen) return;
    const close = (event: KeyboardEvent) => {
      if (event.key === 'Escape' && document.querySelector('[role="dialog"][aria-modal="true"]') === null) clear();
    };
    window.addEventListener('keydown', close);
    return () => window.removeEventListener('keydown', close);
  }, [clear, detailOpen]));

  const selectEvent = (event: EventRow, focus?: () => void) =>
    setSelection({ kind: 'event', key: eventKey(event), restoreFocus: focus });
  const selectCluster = (members: EventRow[], focus: () => void) =>
    setSelection({ kind: 'cluster', keys: new Set(members.map(eventKey)), restoreFocus: focus });

  return {
    selected,
    marker: selected === null ? null : course.markers.find((mark) => eventKey(mark.event) === selectedKey) ?? null,
    cluster,
    choose: (member: EventRow) => selectEvent(member, restoreFocus),
    selectEvent,
    selectCluster,
    clear,
  };
}

export type EventInspection = ReturnType<typeof useEventInspection>;
