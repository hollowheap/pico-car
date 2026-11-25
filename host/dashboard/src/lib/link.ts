export type LinkPacket = {
  timestamp: number;
  packet: string;
  topic?: string;
  receiptId?: string;
  receiptOrder?: number;
  timelineMs?: number;
  receivedAt?: number;
  values: Record<string, number>;
};

export function packetOnTimeline(packet: LinkPacket): LinkPacket {
  const { timelineMs, timestamp } = packet;
  return timelineMs === undefined || !Number.isFinite(timelineMs) || timelineMs === timestamp ? packet :
    { ...packet, timestamp: timelineMs, receivedAt: timestamp };
}

export function timelineOffset(packet: LinkPacket | null | undefined): number {
  return packet?.receivedAt === undefined ? 0 : packet.timestamp - packet.receivedAt;
}

/**
 * Offset from the browser clock to the bridge timeline clock. When a live packet
 * has arrived on the WebSocket, `lastPacket.timestamp - lastPacketAt` absorbs
 * both browser-to-bridge clock skew and bridge timeline rebasing.
 */
export function snapshotClockOffset(
  snapshot: Pick<LinkSnapshot, 'lastPacket' | 'lastPacketAt'>,
  fallback?: LinkPacket | null,
): number {
  if (snapshot.lastPacket != null && snapshot.lastPacketAt != null) {
    return snapshot.lastPacket.timestamp - snapshot.lastPacketAt;
  }
  return timelineOffset(fallback);
}

export type BridgeMqtt = 'connected' | 'disconnected' | 'unknown';

export type BridgeStatus = {
  mqtt: BridgeMqtt;
  packets: number;
  lastPacketAt: number | null;
  historyStorage?: 'ready' | 'degraded';
};

export type LinkStatus = 'disconnected' | 'connecting' | 'open' | 'reconnecting';

export type LinkSnapshot = {
  status: LinkStatus;
  lastPacketAt: number | null;
  lastPacket: LinkPacket | null;
  lastStatus?: LinkPacket | null;
  bridge?: BridgeStatus;
};

export type SocketLike = {
  send: (data: string) => void;
  close: () => void;
  onopen: (() => void) | null;
  onmessage: ((event: { data: unknown }) => void) | null;
  onclose: (() => void) | null;
  onerror: (() => void) | null;
};

export type LinkClientOptions = {
  url: string;
  createSocket: (url: string) => SocketLike;
  now?: () => number;
  reconnectDelayMs?: number;
  idleTimeoutMs?: number;
  schedule?: (callback: () => void, delayMs: number) => unknown;
  cancel?: (handle: unknown) => void;
};

type ParsedMessage =
  | { kind: 'packet'; packet: LinkPacket }
  | { kind: 'status'; bridge: BridgeStatus }
  | null;

function parseMessage(data: unknown): ParsedMessage {
  if (typeof data !== 'string') {
    return null;
  }
  let parsed: unknown;
  try {
    parsed = JSON.parse(data);
  } catch {
    return null;
  }
  if (typeof parsed !== 'object' || parsed === null || Array.isArray(parsed)) return null;
  const message = parsed as Record<string, unknown>;

  if (message.type === 'status') {
    const mqtt = message.mqtt;
    if (mqtt !== 'connected' && mqtt !== 'disconnected') {
      return null;
    }
    if (typeof message.packets !== 'number') {
      return null;
    }
    if (
      message.lastPacketAt !== null &&
      typeof message.lastPacketAt !== 'number'
    ) {
      return null;
    }
    return {
      kind: 'status',
      bridge: {
        mqtt,
        packets: message.packets,
        lastPacketAt: message.lastPacketAt,
        ...(message.historyStorage === 'ready' || message.historyStorage === 'degraded'
          ? { historyStorage: message.historyStorage } : {}),
      },
    };
  }

  if (
    message.type !== 'packet' ||
    typeof message.timestamp !== 'number' ||
    typeof message.packet !== 'string' ||
    typeof message.values !== 'object' ||
    message.values === null
  ) {
    return null;
  }
  const values: Record<string, number> = {};
  for (const [key, value] of Object.entries(
    message.values as Record<string, unknown>,
  )) {
    if (typeof value === 'number') {
      values[key] = value;
    }
  }
  return {
    kind: 'packet',
    packet: packetOnTimeline({
      timestamp: message.timestamp,
      packet: message.packet,
      ...(typeof message.topic === 'string' ? { topic: message.topic } : {}),
      ...(typeof message.receiptId === 'string' && message.receiptId !== '' ? { receiptId: message.receiptId } : {}),
      ...(typeof message.receiptOrder === 'number' && Number.isSafeInteger(message.receiptOrder)
        ? { receiptOrder: message.receiptOrder } : {}),
      ...(typeof message.timelineMs === 'number' && Number.isFinite(message.timelineMs)
        ? { timelineMs: message.timelineMs } : {}),
      values,
    }),
  };
}

export class LinkClient {
  private readonly options: LinkClientOptions;
  private readonly now: () => number;
  private readonly schedule: (callback: () => void, delayMs: number) => unknown;
  private readonly cancel: (handle: unknown) => void;
  private socket: SocketLike | null = null;
  private retry: unknown = null;
  private idle: unknown = null;
  private closed = false;
  private state: LinkSnapshot = {
    status: 'disconnected',
    lastPacketAt: null,
    lastPacket: null,
    lastStatus: null,
    bridge: { mqtt: 'unknown', packets: 0, lastPacketAt: null },
  };
  private readonly stateListeners = new Set<(snapshot: LinkSnapshot) => void>();
  private readonly packetListeners = new Set<(packet: LinkPacket) => void>();

  constructor(options: LinkClientOptions) {
    this.options = options;
    this.now = options.now ?? (() => Date.now());
    this.schedule =
      options.schedule ?? ((callback, delay) => setTimeout(callback, delay));
    this.cancel = options.cancel ?? ((handle) => clearTimeout(handle as ReturnType<typeof setTimeout>));
  }

  connect(): void {
    if (this.socket !== null || this.retry !== null) {
      return;
    }
    this.closed = false;
    this.cancelIdle();
    this.publish({ ...this.state, status: 'connecting' });

    const socket = this.options.createSocket(this.options.url);
    this.socket = socket;
    socket.onopen = () => {
      this.armIdle();
      this.publish({ ...this.state, status: 'open' });
    };
    socket.onmessage = (event) => {
      const message = parseMessage(event.data);
      if (message === null) {
        return;
      }
      this.armIdle();
      if (message.kind === 'status') {
        this.publish({ ...this.state, bridge: message.bridge });
        return;
      }
      this.publish({
        ...this.state,
        lastPacketAt: this.now(),
        lastPacket: message.packet,
        lastStatus:
          message.packet.packet === 'status'
            ? message.packet
            : this.state.lastStatus ?? null,
      });
      for (const listener of this.packetListeners) {
        listener(message.packet);
      }
    };
    socket.onerror = () => {
      socket.close();
    };
    socket.onclose = () => {
      this.cancelIdle();
      this.socket = null;
      if (this.closed) {
        this.publish({ ...this.state, status: 'disconnected' });
        return;
      }
      this.publish({ ...this.state, status: 'reconnecting' });
      this.retry = this.schedule(() => {
        this.retry = null;
        this.connect();
      }, this.options.reconnectDelayMs ?? 1000);
    };
  }

  close(): void {
    this.closed = true;
    this.cancelIdle();
    if (this.retry !== null) {
      this.cancel(this.retry);
      this.retry = null;
    }
    if (this.socket !== null) {
      this.socket.close();
      this.socket = null;
    }
    this.publish({ ...this.state, status: 'disconnected' });
  }

  snapshot(): LinkSnapshot {
    return { ...this.state };
  }

  onState(listener: (snapshot: LinkSnapshot) => void): () => void {
    this.stateListeners.add(listener);
    return () => this.stateListeners.delete(listener);
  }

  onPacket(listener: (packet: LinkPacket) => void): () => void {
    this.packetListeners.add(listener);
    return () => this.packetListeners.delete(listener);
  }

  private armIdle(): void {
    this.cancelIdle();
    const timeout = this.options.idleTimeoutMs ?? 0;
    if (timeout <= 0) {
      return;
    }
    this.idle = this.schedule(() => {
      this.idle = null;
      const socket = this.socket;
      if (socket !== null) {
        socket.close();
      }
    }, timeout);
  }

  private cancelIdle(): void {
    if (this.idle !== null) {
      this.cancel(this.idle);
      this.idle = null;
    }
  }

  private publish(next: LinkSnapshot): void {
    this.state = next;
    for (const listener of this.stateListeners) {
      listener({ ...next });
    }
  }
}
