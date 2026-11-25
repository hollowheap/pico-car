import { describe, expect, it, vi } from 'vitest';
import { syncHistory, type HistoryRecord } from './history';
import { LinkClient, type SocketLike } from './link';
import { emptyStore, reduceStore } from './telemetry-store';

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
  it('fills a course gap after reconnect and applies the buffered live overlap once', async () => {
    const harness = linkClient();
    let store = emptyStore();
    const requests: Array<{ signal: AbortSignal; finish: (records: HistoryRecord[]) => void }> = [];
    const applied: HistoryRecord[] = [];
    const frame = (seq: number, timestamp = seq * 200): HistoryRecord => ({
      timestamp, topic: 'car/s', packet: 'status',
      values: { seq, odometer_mm: seq * 20, heading_ddeg: 0, speed_mms: 100 },
    });
    const deliver = (socket: FakeSocket, record: HistoryRecord) =>
      socket.onmessage?.({ data: JSON.stringify({ type: 'packet', ...record }) });
    const close = syncHistory(harness.instance,
      signal => new Promise(finish => requests.push({ signal, finish })),
      packets => {
        for (const packet of packets) {
          applied.push({ ...packet, topic: packet.topic! });
          store = reduceStore(store, packet);
        }
      });
    try {
      harness.instance.connect();
      const first = harness.sockets[0]!;
      first.onopen?.();
      expect(requests[0]?.signal.aborted).toBe(true);
      requests[1]!.finish([frame(1)]);
      await Promise.resolve();
      deliver(first, frame(1));
      deliver(first, frame(2));

      first.close();
      harness.runScheduled();
      const second = harness.sockets[1]!;
      second.onopen?.();
      const sameMillisecond = frame(3, 400);
      const missedMotion = { timestamp: 820, topic: 'car/m', packet: 'motion',
        values: { seq: 4, left_mms: 90, right_mms: 110 } };
      const missedPitch = { timestamp: 830, topic: 'car/p', packet: 'physics',
        values: { seq: 4, pitch_cdeg: 250, flags: 8 } };
      deliver(second, frame(5));
      requests[2]!.finish([frame(6), frame(5), missedPitch, missedMotion, frame(4), sameMillisecond, frame(2), frame(1)]);
      await Promise.resolve();
      deliver(second, frame(6));
      requests[0]!.finish([frame(9)]);
      await Promise.resolve();

      expect(applied).toEqual([frame(1), frame(2), sameMillisecond, frame(4), missedMotion, missedPitch, frame(5), frame(6)]);
      expect(store.trail.map(point => point.odometerMm)).toEqual([20, 40, 60, 80, 100, 120]);
      expect(store.trail[3]?.wheels).toEqual({ leftMms: 90, rightMms: 110 });
      expect(store.wheelHistory.map(sample => sample.at)).toEqual([820]);
      expect(store.pitchHistory.map(sample => [sample.at, sample.pitchDeg])).toEqual([[830, 2.5]]);
      expect(store.statusLost).toBe(0);
    } finally {
      close();
      harness.instance.close();
    }
  });

  it('resumes with live data if the history read fails', async () => {
    const harness = linkClient();
    const warning = vi.spyOn(console, 'warn').mockImplementation(() => {});
    const applied: number[] = [];
    const close = syncHistory(harness.instance,
      async () => { throw new Error('history unavailable'); },
      packets => applied.push(...packets.map(packet => packet.timestamp)));
    try {
      harness.instance.connect();
      const socket = harness.sockets[0]!;
      socket.onopen?.();
      socket.onmessage?.({ data: '{"type":"packet","timestamp":100,"packet":"status","values":{}}' });
      await Promise.resolve();
      socket.onmessage?.({ data: '{"type":"packet","timestamp":200,"packet":"motion","values":{}}' });
      expect(applied).toEqual([100, 200]);
      expect(warning).toHaveBeenCalledTimes(1);
    } finally {
      close();
      harness.instance.close();
      warning.mockRestore();
    }
  });

  it('fills the skipped samples on the next reconnect after a failed history read', async () => {
    const harness = linkClient();
    const reads: Array<{ resolve: (frames: HistoryRecord[]) => void; reject: (error: Error) => void }> = [];
    const warning = vi.spyOn(console, 'warn').mockImplementation(() => {});
    let committed = emptyStore();
    let preview = committed;
    const frame = (seq: number): HistoryRecord => ({
      timestamp: seq * 200, packet: 'status', topic: 'car/s', receiptId: String(seq),
      values: { seq, odometer_mm: seq * 20, heading_ddeg: 0, speed_mms: 100 },
    });
    const close = syncHistory(harness.instance, () => new Promise((resolve, reject) => reads.push({ resolve, reject })),
      (packets, provisional) => {
        preview = packets.reduce(reduceStore, provisional ? preview : committed);
        if (!provisional) committed = preview;
      });
    const deliver = (socket: FakeSocket, seq: number) =>
      socket.onmessage?.({ data: JSON.stringify({ type: 'packet', ...frame(seq) }) });
    try {
      harness.instance.connect();
      const first = harness.sockets[0]!;
      first.onopen?.();
      reads[0]!.resolve([]);
      reads[1]!.resolve([frame(1)]);
      await Promise.resolve();
      deliver(first, 2);
      first.close();
      harness.runScheduled(0);
      const second = harness.sockets[1]!;
      second.onopen?.();
      deliver(second, 4);
      reads[2]!.reject(new Error('temporary failure'));
      await Promise.resolve();
      deliver(second, 5);
      expect(preview.trail.map(point => point.seq)).toEqual([1, 2, 4, 5]);
      expect(committed.trail.map(point => point.seq)).toEqual([1, 2]);
      second.close();
      harness.runScheduled(1);
      const third = harness.sockets[2]!;
      third.onopen?.();
      deliver(third, 6);
      reads[3]!.resolve([frame(7), frame(6), frame(4), frame(3), frame(2), frame(1)]);
      await Promise.resolve();
      deliver(third, 7);
      expect(committed.trail.map(point => point.seq)).toEqual([1, 2, 3, 4, 5, 6, 7]);
      expect(committed.statusLost).toBe(0);
    } finally {
      close();
      harness.instance.close();
      warning.mockRestore();
    }
  });

  it('retries history while the socket stays open and replaces the live preview', async () => {
    vi.useFakeTimers();
    const harness = linkClient();
    const reads: Array<{ resolve: (frames: HistoryRecord[]) => void; reject: (error: Error) => void }> = [];
    const warning = vi.spyOn(console, 'warn').mockImplementation(() => {});
    const applied: Array<{ timestamps: number[]; provisional?: boolean }> = [];
    const close = syncHistory(harness.instance, () => new Promise((resolve, reject) => reads.push({ resolve, reject })),
      (frames, provisional) => applied.push({ timestamps: frames.map(frame => frame.timestamp), provisional }));
    try {
      harness.instance.connect();
      const socket = harness.sockets[0]!;
      socket.onopen?.();
      reads[0]!.resolve([]);
      reads[1]!.reject(new Error('temporary failure'));
      await Promise.resolve();
      const frame = { timestamp: 200, packet: 'status', topic: 'car/s', receiptId: 'second', values: {} };
      socket.onmessage?.({ data: JSON.stringify({ type: 'packet', ...frame }) });
      expect(applied).toEqual([{ timestamps: [200], provisional: true }]);

      await vi.advanceTimersByTimeAsync(2000);
      expect(reads).toHaveLength(3);
      reads[2]!.resolve([frame, { ...frame, timestamp: 100, receiptId: 'first' }]);
      await Promise.resolve();
      expect(applied[1]).toEqual({ timestamps: [100, 200], provisional: undefined });
      expect(harness.sockets).toHaveLength(1);
    } finally {
      close();
      harness.instance.close();
      warning.mockRestore();
      vi.useRealTimers();
    }
  });

  it('fills a reconnect gap across a backward receipt clock step', async () => {
    const harness = linkClient();
    const reads: Array<(frames: HistoryRecord[]) => void> = [];
    let store = emptyStore();
    const applied: number[] = [];
    const frame = (seq: number, timestamp: number, timelineMs: number): HistoryRecord => ({
      timestamp, timelineMs, receiptOrder: seq, receiptId: String(seq), packet: 'status', topic: 'car/s',
      values: { seq, odometer_mm: seq * 20, heading_ddeg: 0, speed_mms: 100 },
    });
    const first = frame(1, 20000, 20000);
    const second = frame(2, 19900, 20200);
    const third = frame(3, 20100, 20400);
    const fourth = frame(4, 20300, 20600);
    const close = syncHistory(harness.instance, () => new Promise(resolve => reads.push(resolve)), frames => {
      applied.push(...frames.map(packet => packet.values.seq!));
      store = frames.reduce(reduceStore, store);
    });
    try {
      harness.instance.connect();
      const original = harness.sockets[0]!;
      original.onopen?.();
      reads[0]!([]);
      reads[1]!([first]);
      await Promise.resolve();
      original.close();
      harness.runScheduled();
      const reconnected = harness.sockets[1]!;
      reconnected.onopen?.();
      reconnected.onmessage?.({ data: JSON.stringify({ type: 'packet', ...third }) });
      reads[2]!([third, first, second]);
      await Promise.resolve();
      reconnected.onmessage?.({ data: JSON.stringify({ type: 'packet', ...fourth }) });

      expect(applied).toEqual([1, 2, 3, 4]);
      expect(store.trail.map(point => point.timestamp)).toEqual([20000, 20200, 20400, 20600]);
      expect(store.latest.status?.receivedAt).toBe(20300);
      expect(store.statusLost).toBe(0);
    } finally {
      close();
      harness.instance.close();
    }
  });

  it('keeps distinct live receipts with matching timestamps and payloads', async () => {
    const harness = linkClient();
    const reads: Array<(frames: HistoryRecord[]) => void> = [];
    const applied: Array<string | undefined> = [];
    const first = { timestamp: 1000, packet: 'diag', topic: 'car/d', values: { cmd_last_id: 7 }, receiptId: 'first' };
    const second = { ...first, receiptId: 'second' };
    const close = syncHistory(harness.instance, () => new Promise(resolve => reads.push(resolve)),
      frames => applied.push(...frames.map(frame => frame.receiptId)));
    try {
      harness.instance.connect();
      harness.sockets[0]!.onopen?.();
      reads[0]!([]);
      reads[1]!([first]);
      await Promise.resolve();
      harness.sockets[0]!.close();
      harness.runScheduled();
      const socket = harness.sockets[1]!;
      socket.onopen?.();
      reads[2]!([first]);
      await Promise.resolve();
      socket.onmessage?.({ data: JSON.stringify({ type: 'packet', ...second }) });
      expect(applied).toEqual(['first', 'second']);
    } finally {
      close();
      harness.instance.close();
    }
  });

  it('ignores a valid JSON null frame and accepts the following packet', () => {
    const harness = linkClient();
    harness.instance.connect();
    const socket = harness.sockets[0]!;
    socket.onopen?.();
    socket.onmessage?.({ data: 'null' });
    socket.onmessage?.({ data: '{"type":"packet","timestamp":10,"packet":"status","values":{"speed_mms":-80}}' });
    expect(harness.instance.snapshot().lastStatus?.values.speed_mms).toBe(-80);
  });

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
        topic: 'car/s',
        values: { speed_mms: 12, dist_mm: 34 },
      }),
    });

    expect(packets).toEqual(['status']);
    expect(harness.instance.snapshot().lastPacketAt).toBe(5000);
    expect(harness.instance.snapshot().lastPacket?.values.speed_mms).toBe(12);
    expect(harness.instance.snapshot().lastPacket?.topic).toBe('car/s');
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
