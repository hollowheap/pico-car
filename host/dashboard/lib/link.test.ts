import { describe, expect, it } from 'vitest';
import { LinkClient, type SocketLike } from './link';

class FakeSocket implements SocketLike {
  onopen: (() => void) | null = null;
  onmessage: ((event: { data: unknown }) => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: (() => void) | null = null;
  sent: string[] = [];

  send(data: string): void {
    this.sent.push(data);
  }

  close(): void {
    this.onclose?.();
  }
}

function linkClient(idleTimeoutMs = 0) {
  const sockets: FakeSocket[] = [];
  const scheduled: Array<() => void> = [];
  const cancelled: unknown[] = [];
  const instance = new LinkClient({
    url: 'ws://link',
    createSocket: () => {
      const socket = new FakeSocket();
      sockets.push(socket);
      return socket;
    },
    schedule: (callback) => {
      scheduled.push(callback);
      return scheduled.length;
    },
    cancel: (handle) => {
      cancelled.push(handle);
    },
    now: () => 5000,
    idleTimeoutMs,
  });
  return {
    instance,
    sockets,
    runScheduled: (index = 0) => scheduled[index]?.(),
    scheduledCount: () => scheduled.length,
    cancelled: () => cancelled,
  };
}

describe('link client', () => {
  it('reports connection state and decoded packets', () => {
    const harness = linkClient();
    harness.instance.connect();
    expect(harness.instance.snapshot().status).toBe('connecting');

    harness.sockets[0]?.onopen?.();
    expect(harness.instance.snapshot().status).toBe('open');

    const packets: string[] = [];
    harness.instance.onPacket((packet) => packets.push(packet.packet));
    harness.sockets[0]?.onmessage?.({
      data: JSON.stringify({
        type: 'packet',
        timestamp: 10,
        packet: 'status',
        values: { speed_mms: 12, dist_mm: 34 },
      }),
    });

    expect(packets).toEqual(['status']);
    expect(harness.instance.snapshot().lastPacketAt).toBe(5000);
    expect(harness.instance.snapshot().lastPacket?.values.speed_mms).toBe(12);
  });

  it('decodes the bridge status frame', () => {
    const harness = linkClient();
    harness.instance.connect();
    harness.sockets[0]?.onopen?.();
    harness.sockets[0]?.onmessage?.({
      data: JSON.stringify({
        type: 'status',
        mqtt: 'connected',
        packets: 4,
        lastPacketAt: 10,
      }),
    });

    expect(harness.instance.snapshot().bridge).toEqual({
      mqtt: 'connected',
      packets: 4,
      lastPacketAt: 10,
    });
    expect(harness.instance.snapshot().lastPacket).toBeNull();
  });

  it('reconnects after an unexpected close', () => {
    const harness = linkClient();
    harness.instance.connect();
    harness.sockets[0]?.onclose?.();

    expect(harness.instance.snapshot().status).toBe('reconnecting');
    harness.runScheduled(0);
    expect(harness.sockets).toHaveLength(2);
    expect(harness.instance.snapshot().status).toBe('connecting');
  });

  it('closes an idle socket and reconnects', () => {
    const harness = linkClient(5000);
    harness.instance.connect();
    harness.sockets[0]?.onopen?.();
    expect(harness.scheduledCount()).toBe(1);

    harness.runScheduled(0);
    expect(harness.instance.snapshot().status).toBe('reconnecting');
    expect(harness.sockets).toHaveLength(1);

    harness.runScheduled(1);
    expect(harness.sockets).toHaveLength(2);
    expect(harness.instance.snapshot().status).toBe('connecting');
  });

  it('does not reconnect after close()', () => {
    const harness = linkClient();
    harness.instance.connect();
    harness.instance.close();

    expect(harness.instance.snapshot().status).toBe('disconnected');
    expect(harness.sockets).toHaveLength(1);
  });

  it('ignores malformed messages', () => {
    const harness = linkClient();
    harness.instance.connect();
    harness.sockets[0]?.onmessage?.({ data: '{not json' });
    expect(harness.instance.snapshot().lastPacket).toBeNull();
    expect(harness.instance.snapshot().bridge?.mqtt).toBe('unknown');
  });
});
