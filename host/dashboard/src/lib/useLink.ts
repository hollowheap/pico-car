import { useEffect, useState } from 'react';
import { applyConsoleUpdate, EMPTY_CONSOLE, type ConsoleSnapshot } from './console-view';
import { isConsoleSnapshot, LinkClient, type LinkSnapshot, type SocketLike } from './link';

const HTTP_FALLBACK_DELAY_MS = 250;

const initial: LinkSnapshot = {
  status: 'disconnected',
  lastPacketAt: null,
  lastPacket: null,
  bridge: { mqtt: 'unknown', packets: 0, lastPacketAt: null },
};

export type LinkState = Omit<ConsoleSnapshot, 'lastPacket'> & { snapshot: LinkSnapshot };

/** Subscribe to the bridge projection. A new socket receives a complete snapshot. */
export function useLink(url: string | undefined, serviceUrl: string | undefined): LinkState {
  const [snapshot, setSnapshot] = useState<LinkSnapshot>(initial);
  const [readings, setReadings] = useState(EMPTY_CONSOLE);

  useEffect(() => {
    setReadings(EMPTY_CONSOLE);
    setSnapshot(initial);
    let current: ConsoleSnapshot | null = null;
    let frameRevision = 0;
    let receivedLive = false;
    const request = new AbortController();
    const show = (next: ConsoleSnapshot) => {
      current = next;
      setReadings({ ...next, frameLog: { ...next.frameLog, revision: frameRevision } });
    };
    const timer = serviceUrl && setTimeout(() => {
      // The socket usually delivers its own snapshot first. HTTP also works
      // through proxies that disallow WebSocket upgrades.
      void fetch(`${serviceUrl.replace(/\/+$/, '')}/view`, { signal: request.signal })
        .then((response): Promise<unknown> | null => response.ok ? response.json() : null)
        .then((next) => {
          if (!request.signal.aborted && !receivedLive && isConsoleSnapshot(next)) show(next);
        })
        .catch(() => { /* The WebSocket supplies its own snapshot when it connects. */ });
    }, url ? HTTP_FALLBACK_DELAY_MS : 0);
    const cancelFallback = () => {
      if (timer) clearTimeout(timer);
      request.abort();
    };
    if (!url) return cancelFallback;
    const target = new URL(url);
    target.searchParams.set('console', '1');
    const client = new LinkClient({
      url: target.href,
      createSocket: (address) => new WebSocket(address) as unknown as SocketLike,
      idleTimeoutMs: 8000,
    });
    const unsubscribeState = client.onState((state) => {
      if (state.status !== 'open') receivedLive = false;
      setSnapshot(state.status === 'open' && !receivedLive ? { ...state, status: 'connecting' } : state);
    });
    const unsubscribeConsole = client.onConsole((update) => {
      const next = applyConsoleUpdate(receivedLive ? current : null, update);
      if (next === null) return;
      receivedLive = true;
      cancelFallback();
      if (update.type === 'console' || current?.frameLog.revision !== next.frameLog.revision) frameRevision++;
      // A replacement snapshot can reuse bridge row ids after a reconnect.
      show(next);
      setSnapshot(client.snapshot());
    });
    client.connect();
    return () => {
      cancelFallback();
      unsubscribeState();
      unsubscribeConsole();
      client.close();
    };
  }, [url, serviceUrl]);

  return { snapshot, store: readings.store, view: readings.view, frameLog: readings.frameLog };
}
