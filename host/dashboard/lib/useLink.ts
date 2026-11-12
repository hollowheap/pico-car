import { useEffect, useState } from 'react';
import { fetchHistory, storeFromHistory } from './history';
import {
  LinkClient,
  type LinkPacket,
  type LinkSnapshot,
  type SocketLike,
} from './link';
import { emptyStore, reduceStore, type TelemetryStore } from './telemetry-store';

const initial: LinkSnapshot = {
  status: 'disconnected',
  lastPacketAt: null,
  lastPacket: null,
  bridge: { mqtt: 'unknown', packets: 0, lastPacketAt: null },
};

const MAX_PACKETS = 120;

export type LinkState = {
  snapshot: LinkSnapshot;
  /** The newest packets of every kind, newest first (the packet log). */
  packets: LinkPacket[];
  /** The latest packet of each kind, the trail, the events and the loss count. */
  store: TelemetryStore;
};

/** Records fetched from the bridge to fill the store when the page opens. */
const SEED_RECORDS = 10000;

/**
 * Connect to the bridge. When `serviceUrl` is set, the recorded frames fill the
 * trail and the events first, so a page opened mid-run is not empty.
 */
export function useLink(url: string | undefined, serviceUrl?: string): LinkState {
  const [snapshot, setSnapshot] = useState<LinkSnapshot>(initial);
  const [packets, setPackets] = useState<LinkPacket[]>([]);
  const [store, setStore] = useState<TelemetryStore>(emptyStore);

  useEffect(() => {
    setPackets([]);
    setStore(emptyStore());
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
    // Packets that arrive while the history loads are replayed after it.
    let buffer: LinkPacket[] | null = [];
    let active = true;
    const unsubscribePacket = client.onPacket((packet) => {
      setPackets((current) => [packet, ...current].slice(0, MAX_PACKETS));
      setStore((current) => reduceStore(current, packet));
      buffer?.push(packet);
    });
    client.connect();
    if (serviceUrl) {
      fetchHistory(serviceUrl, SEED_RECORDS)
        .then((records) => {
          if (!active) return;
          const seeded = (buffer ?? []).reduce(reduceStore, storeFromHistory(records));
          buffer = null;
          setStore(seeded);
        })
        .catch((reason: unknown) => {
          buffer = null;
          console.warn('The recorded frames could not fill the dashboard:', reason);
        });
    } else {
      buffer = null;
    }

    return () => {
      active = false;
      unsubscribeState();
      unsubscribePacket();
      client.close();
    };
  }, [url, serviceUrl]);

  return { snapshot, packets, store };
}
