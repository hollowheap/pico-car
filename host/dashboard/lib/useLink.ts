import { useEffect, useState } from 'react';
import {
  LinkClient,
  type LinkPacket,
  type LinkSnapshot,
  type SocketLike,
} from './link';

const initial: LinkSnapshot = {
  status: 'disconnected',
  lastPacketAt: null,
  lastPacket: null,
  bridge: { mqtt: 'unknown', packets: 0, lastPacketAt: null },
};

const MAX_PACKETS = 120;

export type LinkState = {
  snapshot: LinkSnapshot;
  packets: LinkPacket[];
};

export function useLink(url: string | undefined): LinkState {
  const [snapshot, setSnapshot] = useState<LinkSnapshot>(initial);
  const [packets, setPackets] = useState<LinkPacket[]>([]);

  useEffect(() => {
    setPackets([]);
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
    const unsubscribePacket = client.onPacket((packet) => {
      setPackets((current) => [packet, ...current].slice(0, MAX_PACKETS));
    });
    client.connect();

    return () => {
      unsubscribeState();
      unsubscribePacket();
      client.close();
    };
  }, [url]);

  return { snapshot, packets };
}
