import type { Server } from "node:http";
import { WebSocket, WebSocketServer } from "ws";
import { consoleUpdate } from "./console/update.js";
import type { ConsoleSnapshot } from "./console/view.js";
import type { HistoryRecord } from "./history.js";

export type LiveStatus = {
  mqtt: "connected" | "disconnected";
  packets: number;
  lastPacketAt: number | null;
  historyStorage: "ready" | "degraded";
};

const PING_INTERVAL_MS = 30_000;
const STATUS_INTERVAL_MS = 2_000;
const CONSOLE_INTERVAL_MS = 100;
const MAX_QUEUED_BYTES = 256 * 1024;

type LiveClient = WebSocket & { isAlive: boolean; console?: true; lastView?: ConsoleSnapshot };

export function encodeLivePacket(packet: HistoryRecord): string {
  return JSON.stringify({ type: "packet", ...packet });
}

export function encodeLiveStatus(status: LiveStatus): string {
  return JSON.stringify({ type: "status", ...status });
}

export function createLiveServer(
  server: Server,
  getStatus: () => LiveStatus,
  getView?: () => ConsoleSnapshot,
): {
  broadcast: (packet: HistoryRecord) => void;
  broadcastStatus: () => void;
  close: () => void;
} {
  const wss = new WebSocketServer({ server, perMessageDeflate: { threshold: 1024 } });

  const sendStatus = (client: WebSocket): void => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(encodeLiveStatus(getStatus()));
    }
  };

  const broadcastFrame = (frame: string, raw = false): void => {
    for (const client of wss.clients) {
      if (client.readyState === WebSocket.OPEN && (!raw || !(client as LiveClient).console)) {
        client.send(frame);
      }
    }
  };

  const sendView = (client: LiveClient, snapshot: ConsoleSnapshot): void => {
    if (snapshot === client.lastView || client.readyState !== WebSocket.OPEN || client.bufferedAmount > MAX_QUEUED_BYTES) return;
    client.send(JSON.stringify(consoleUpdate(snapshot, client.lastView)));
    client.lastView = snapshot;
  };

  wss.on("connection", (client: WebSocket, request) => {
    const live = client as LiveClient;
    live.isAlive = true;
    live.on("pong", () => {
      live.isAlive = true;
    });
    sendStatus(live);
    if (getView && new URL(request.url ?? "/", "http://localhost").searchParams.get("console") === "1") {
      live.console = true;
      sendView(live, getView());
    }
  });

  const viewTimer = setInterval(() => {
    let snapshot: ConsoleSnapshot | undefined;
    for (const client of wss.clients) {
      if ((client as LiveClient).console && getView) sendView(client as LiveClient, snapshot ??= getView());
    }
  }, CONSOLE_INTERVAL_MS);

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
    broadcast(packet: HistoryRecord): void {
      broadcastFrame(encodeLivePacket(packet), true);
    },
    broadcastStatus(): void {
      broadcastFrame(encodeLiveStatus(getStatus()));
    },
    close(): void {
      clearInterval(statusTimer);
      clearInterval(viewTimer);
      clearInterval(pingTimer);
      for (const client of wss.clients) client.terminate();
      wss.close();
    },
  };
}
