import { createServer, type IncomingMessage, type ServerResponse } from "node:http";
import { loadConfig } from "./config.js";
import { encodeCommand, parseCommandRequest, type CommandRequest } from "./command.js";
import { createHistoryStore } from "./history.js";
import { createLiveServer } from "./live.js";
import { createMqttIngest } from "./mqtt.js";
import { loadProtocolSchema } from "./schema.js";

const config = loadConfig();
const schema = loadProtocolSchema(config.schemaPath);
const history = createHistoryStore(config.historyPath, config.historyMaxRecords);
let live: ReturnType<typeof createLiveServer> | null = null;
const ingest = createMqttIngest(config, schema, (packet, topic) => {
  const timestamp = Date.now();
  history.append({
    timestamp,
    topic,
    packet: packet.name,
    values: packet.values,
  });
  live?.broadcast({ timestamp, packet: packet.name, values: packet.values });
  console.log(`${packet.name}: ${JSON.stringify(packet.values)}`);
});

ingest.client.on("connect", () => live?.broadcastStatus());
ingest.client.on("close", () => live?.broadcastStatus());

const corsHeaders = {
  "access-control-allow-origin": "*",
  "access-control-allow-methods": "GET, POST, OPTIONS",
  "access-control-allow-headers": "content-type",
};

function sendJson(
  response: ServerResponse,
  status: number,
  body: unknown,
): void {
  response.writeHead(status, {
    ...corsHeaders,
    "content-type": "application/json",
  });
  response.end(JSON.stringify(body));
}

function readBody(request: IncomingMessage): Promise<string> {
  return new Promise((resolve, reject) => {
    let body = "";
    let size = 0;
    request.setEncoding("utf8");
    request.on("data", (chunk: string) => {
      size += chunk.length;
      if (size > 1024) {
        request.destroy();
        reject(new Error("body too large"));
        return;
      }
      body += chunk;
    });
    request.on("end", () => resolve(body));
    request.on("error", reject);
  });
}

async function handleCommand(
  request: IncomingMessage,
  response: ServerResponse,
): Promise<void> {
  let command: CommandRequest;
  try {
    const raw = await readBody(request);
    command = parseCommandRequest(JSON.parse(raw) as unknown, schema);
  } catch (error) {
    sendJson(response, 400, { error: (error as Error).message });
    return;
  }
  if (!ingest.client.connected) {
    sendJson(response, 503, { error: "mqtt disconnected" });
    return;
  }
  const frame = encodeCommand(schema, command);
  const topic = `${config.topicPrefix}/${schema.topics.command}`;

  try {
    await new Promise<void>((resolve, reject) => {
      ingest.client.publish(topic, frame, { qos: 1 }, (error) => {
        if (error) {
          reject(error);
          return;
        }
        resolve();
      });
    });
  } catch (error) {
    console.error(`command publish failed: ${(error as Error).message}`);
    sendJson(response, 502, { error: "mqtt publish failed" });
    return;
  }
  sendJson(response, 202, { ok: true });
}

const server = createServer((request, response) => {
  const url = new URL(request.url ?? "/", "http://localhost");

  if (request.method === "OPTIONS") {
    response.writeHead(204, corsHeaders);
    response.end();
    return;
  }

  if (request.method === "POST" && url.pathname === "/command") {
    void handleCommand(request, response);
    return;
  }

  if (url.pathname === "/health") {
    sendJson(response, 200, {
      mqtt: ingest.client.connected ? "connected" : "disconnected",
      packets: ingest.stats.packets,
      lastPacketAt: ingest.stats.lastPacketAt,
      lastPacket: ingest.stats.lastPacket,
    });
    return;
  }

  if (url.pathname === "/history") {
    const limit = Number(url.searchParams.get("limit") ?? "100");
    sendJson(response, 200, history.list(limit));
    return;
  }

  sendJson(response, 404, {});
});

live = createLiveServer(server, () => ({
  mqtt: ingest.client.connected ? "connected" : "disconnected",
  packets: ingest.stats.packets,
  lastPacketAt: ingest.stats.lastPacketAt,
}));

server.listen(config.httpPort, () => {
  console.log(`bridge on http://127.0.0.1:${config.httpPort}`);
});

function shutdown(): void {
  live?.close();
  server.close();
  ingest.client.end();
}

process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);
