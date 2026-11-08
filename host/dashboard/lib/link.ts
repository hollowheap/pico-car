export type LinkPacket = {
  timestamp: number;
  packet: string;
  values: Record<string, number>;
};

export type BridgeMqtt = 'connected' | 'disconnected' | 'unknown';

export type BridgeStatus = {
  mqtt: BridgeMqtt;
  packets: number;
  lastPacketAt: number | null;
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
  let message: Record<string, unknown>;
  try {
    message = JSON.parse(data) as Record<string, unknown>;
  } catch {
    return null;
  }

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
    packet: {
      timestamp: message.timestamp,
      packet: message.packet,
      values,
    },
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
