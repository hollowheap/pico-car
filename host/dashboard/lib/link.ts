export type LinkPacket = {
  timestamp: number;
  packet: string;
  values: Record<string, number>;
};

export type LinkStatus = 'disconnected' | 'connecting' | 'open' | 'reconnecting';

export type LinkSnapshot = {
  status: LinkStatus;
  lastPacketAt: number | null;
  lastPacket: LinkPacket | null;
  lastStatus?: LinkPacket | null;
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
  schedule?: (callback: () => void, delayMs: number) => unknown;
  cancel?: (handle: unknown) => void;
};

function parsePacket(data: unknown): LinkPacket | null {
  if (typeof data !== 'string') {
    return null;
  }
  try {
    const message = JSON.parse(data) as {
      type?: unknown;
      timestamp?: unknown;
      packet?: unknown;
      values?: unknown;
    };
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
    for (const [key, value] of Object.entries(message.values)) {
      if (typeof value === 'number') {
        values[key] = value;
      }
    }
    return { timestamp: message.timestamp, packet: message.packet, values };
  } catch {
    return null;
  }
}

export class LinkClient {
  private readonly options: LinkClientOptions;
  private readonly now: () => number;
  private readonly schedule: (callback: () => void, delayMs: number) => unknown;
  private readonly cancel: (handle: unknown) => void;
  private socket: SocketLike | null = null;
  private retry: unknown = null;
  private closed = false;
  private state: LinkSnapshot = {
    status: 'disconnected',
    lastPacketAt: null,
    lastPacket: null,
    lastStatus: null,
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
    this.publish({ ...this.state, status: 'connecting' });

    const socket = this.options.createSocket(this.options.url);
    this.socket = socket;
    socket.onopen = () => {
      this.publish({ ...this.state, status: 'open' });
    };
    socket.onmessage = (event) => {
      const packet = parsePacket(event.data);
      if (packet === null) {
        return;
      }
      this.publish({
        ...this.state,
        lastPacketAt: this.now(),
        lastPacket: packet,
        lastStatus:
          packet.packet === 'status' ? packet : this.state.lastStatus ?? null,
      });
      for (const listener of this.packetListeners) {
        listener(packet);
      }
    };
    socket.onerror = () => {
      socket.close();
    };
    socket.onclose = () => {
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

  private publish(next: LinkSnapshot): void {
    this.state = next;
    for (const listener of this.stateListeners) {
      listener({ ...next });
    }
  }
}
