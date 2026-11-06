import type { Server } from "node:http";
import { WebSocket, WebSocketServer } from "ws";

export type LivePacket = {
  timestamp: number;
  packet: string;
  values: Record<string, number>;
};

export function encodeLivePacket(packet: LivePacket): string {
  return JSON.stringify({ type: "packet", ...packet });
}

export function createLiveServer(server: Server): {
  broadcast: (packet: LivePacket) => void;
  close: () => void;
} {
  const wss = new WebSocketServer({ server });

  return {
    broadcast(packet: LivePacket): void {
      const frame = encodeLivePacket(packet);
      for (const client of wss.clients) {
        if (client.readyState === WebSocket.OPEN) {
          client.send(frame);
        }
      }
    },
    close(): void {
      wss.close();
    },
  };
}
