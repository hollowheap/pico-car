import { useEffect, useRef, useState } from 'react';
import { courseSession } from './course-session';
import { appendFrame, createFrameLog, type FrameLog } from './frame-log';
import { fetchHistory, HISTORY_CATCHUP_LIMIT, syncHistory } from './history';
import {
  LinkClient,
  type LinkPacket,
  type LinkSnapshot,
  type SocketLike,
} from './link';
import { emptyStore, precedesReset, reduceStore, type TelemetryStore } from './telemetry-store';

const initial: LinkSnapshot = {
  status: 'disconnected',
  lastPacketAt: null,
  lastPacket: null,
  bridge: { mqtt: 'unknown', packets: 0, lastPacketAt: null },
};

export type LinkState = {
  snapshot: LinkSnapshot;
  /** The latest packet of each kind, the trail, the events and the loss count. */
  store: TelemetryStore;
  frameLog: FrameLog;
};

const emptyReadings = (revision = 0) => ({ store: emptyStore(), frameLog: createFrameLog([], revision) });

/**
 * Connect to the bridge. When `serviceUrl` is set, the recorded frames fill the
 * trail and the events on startup and after each reconnect.
 */
export function useLink(url: string | undefined, serviceUrl?: string): LinkState {
  const [snapshot, setSnapshot] = useState<LinkSnapshot>(initial);
  const [readings, setReadings] = useState(emptyReadings);
  const revision = useRef(0);

  useEffect(() => {
    let current = emptyReadings(++revision.current);
    let preview: typeof current | null = null;
    setReadings(current);
    if (!url) {
      setSnapshot(initial);
      return;
    }

    const client = new LinkClient({
      url,
      createSocket: (target) => new WebSocket(target) as unknown as SocketLike,
      idleTimeoutMs: 8000,
    });
    const unsubscribeState = client.onState(setSnapshot);
    const session = courseSession(JSON.stringify([url, serviceUrl]));
    const save = () => session.save(current.store);
    if (typeof window !== 'undefined') window.addEventListener?.('pagehide', save);
    const apply = (frames: LinkPacket[], provisional = false) => {
      let { store, frameLog } = provisional && preview !== null ? preview : current;
      if (!provisional && preview !== null) frameLog = { ...frameLog, revision: ++revision.current };
      for (const packet of frames) {
        const next = provisional ? reduceStore(store, packet) : session.reduce(store, packet);
        if (next.resetAt !== store.resetAt || next.resetOrder !== store.resetOrder ||
            next.resetReceiptId !== store.resetReceiptId) {
          frameLog = createFrameLog([], ++revision.current);
        }
        if (!precedesReset(packet, next.resetAt, next.resetOrder)) frameLog = appendFrame(frameLog, packet);
        store = next;
      }
      if (provisional) {
        preview = { store, frameLog };
        setReadings(preview);
      } else {
        preview = null;
        current = { store: session.restore(store), frameLog };
        setReadings(current);
      }
    };
    const unsubscribePacket = serviceUrl
      ? syncHistory(client, (signal) => fetchHistory(serviceUrl, HISTORY_CATCHUP_LIMIT, fetch, signal), apply)
      : client.onPacket((packet) => apply([packet]));
    client.connect();

    return () => {
      if (typeof window !== 'undefined') window.removeEventListener?.('pagehide', save);
      unsubscribeState();
      unsubscribePacket();
      client.close();
    };
  }, [url, serviceUrl]);

  return { snapshot, ...readings };
}
