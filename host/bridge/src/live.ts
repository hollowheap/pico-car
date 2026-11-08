import type { Server } from "node:http";
import { WebSocket, WebSocketServer } from "ws";

export type LivePacket = {
  timestamp: number;
  packet: string;
  values: Record<string, number>;
};

export type LiveStatus = {
  mqtt: "connected" | "disconnected";
  packets: number;
  lastPacketAt: number | null;
};

const PING_INTERVAL_MS = 30_000;
const STATUS_INTERVAL_MS = 2_000;

type LiveClient = WebSocket & { isAlive: boolean };

export function encodeLivePacket(packet: LivePacket): string {
  return JSON.stringify({ type: "packet", ...packet });
}

export function encodeLiveStatus(status: LiveStatus): string {
  return JSON.stringify({ type: "status", ...status });
}

export function createLiveServer(
  server: Server,
  getStatus: () => LiveStatus,
): {
  broadcast: (packet: LivePacket) => void;
  broadcastStatus: () => void;
  close: () => void;
} {
  const wss = new WebSocketServer({ server });

  const sendStatus = (client: WebSocket): void => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(encodeLiveStatus(getStatus()));
    }
  };

  const broadcastFrame = (frame: string): void => {
    for (const client of wss.clients) {
      if (client.readyState === WebSocket.OPEN) {
        client.send(frame);
      }
    }
  };

  wss.on("connection", (client: WebSocket) => {
    const live = client as LiveClient;
    live.isAlive = true;
    live.on("pong", () => {
      live.isAlive = true;
    });
    sendStatus(live);
  });

  const statusTimer = setInterval(() => {
    broadcastFrame(encodeLiveStatus(getStatus()));
  }, STATUS_INTERVAL_MS);

  const pingTimer = setInterval(() => {
    for (const client of wss.clients) {
      const live = client as LiveClient;
      if (!live.isAlive) {
        live.terminate();
        continue;
      }
      live.isAlive = false;
      live.ping();
    }
  }, PING_INTERVAL_MS);

  return {
    broadcast(packet: LivePacket): void {
      broadcastFrame(encodeLivePacket(packet));
    },
    broadcastStatus(): void {
      broadcastFrame(encodeLiveStatus(getStatus()));
    },
    close(): void {
      clearInterval(statusTimer);
      clearInterval(pingTimer);
      wss.close();
    },
  };
}
