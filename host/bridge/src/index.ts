import { createServer, type IncomingMessage, type ServerResponse } from "node:http";
import { existsSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import sirv from "sirv";
import { loadConfig } from "./config.js";
import { randomInt } from "node:crypto";
import { CommandTimeoutError, createCommandSession, parseCommandRequest, PUBACK_TIMEOUT_MS, publishWithTimeout, type CommandRequest } from "./command.js";
import { createBarcodeArchive } from "./barcode-archive.js";
import { barcodeMapBytes, DEFAULT_BARCODE_MAP, loadBarcodeMap, parseBarcodeMap, saveBarcodeMap, type BarcodeMap } from "./barcode-map.js";
import { ENUMS, EVENTS } from "./console/schema.generated.js";
import { createHistoryStore } from "./history.js";
import { courseStorage } from "./console/retention.js";
import { createConsoleView } from "./console/view.js";
import { createLiveServer } from "./live.js";
import { startLocalBroker } from "./local-broker.js";
import { createMqttIngest } from "./mqtt.js";
import { loadProtocolSchema } from "./schema.js";

const config = loadConfig();
const station = process.argv.includes("--station");
const dashboard = existsSync(join(config.dashboardPath, "index.html"))
  ? sirv(config.dashboardPath, { single: true, etag: true, ignores: [/^\/(?:assets|_expo)(?:\/|$)/] })
  : undefined;
if (station && !dashboard) throw new Error("Dashboard export missing. Run npm run build from the repository root.");
const schema = loadProtocolSchema(config.schemaPath);
mkdirSync(dirname(config.historyPath), { recursive: true });
const history = createHistoryStore(config.historyPath, config.historyMaxRecords, config.historyMaxEvents);
const barcode = createBarcodeArchive(`${config.historyPath}.barcode.sqlite`);
const barcodeKey = (values: Record<string, number>) => `${values.car_id}/${values.boot_id}/${values.record_id}`;
const retained = new Set(history.list().filter((record) => record.packet === "barcode_sample").map((record) => barcodeKey(record.values)));
try {
  for (const record of barcode.recent(`${config.topicPrefix}/${schema.topics.barcode_sample}`, config.historyMaxRecords)) {
    if (!retained.has(barcodeKey(record.values))) history.append(record);
  }
} catch (error) {
  console.error(`barcode archive: ${(error as Error).message}`);
}
const view = createConsoleView(history.list(config.historyMaxRecords).reverse(), courseStorage(config.historyPath));
const COURSE_SAVE_INTERVAL_MS = 10_000;
const saveCourse = setInterval(view.save, COURSE_SAVE_INTERVAL_MS);
const broker = station && !process.env.MQTT_URL ? await startLocalBroker() : undefined;
process.once("exit", () => broker?.kill());
let live: ReturnType<typeof createLiveServer> | null = null;
const commands = createCommandSession(randomInt(0x10000));
const barcodeMapPath = `${config.historyPath}.barcode-map.json`;
let savedBarcodeMap: BarcodeMap | null = null;
try {
  savedBarcodeMap = loadBarcodeMap(barcodeMapPath);
} catch (error) {
  console.error(`barcode mapping: ${(error as Error).message}`);
}
let barcodeMapRun: number | undefined;
let barcodeMapCmdId: number | null = null;
let barcodeMapApplied = false;
let barcodeMapLastSent = 0;
let barcodeMapAttempts = 0;
const BARCODE_MAP_RETRY_MS = 2000;
const BARCODE_MAP_MAX_ATTEMPTS = 3;

function syncBarcodeMap(): void {
  const now = Date.now();
  if (!savedBarcodeMap || barcodeMapApplied || barcodeMapAttempts >= BARCODE_MAP_MAX_ATTEMPTS ||
      now - barcodeMapLastSent < BARCODE_MAP_RETRY_MS || !ingest.client.connected) return;
  const frame = commands.frame(schema, barcodeMapBytes(savedBarcodeMap), now);
  if (frame === null) return;
  barcodeMapLastSent = now;
  barcodeMapAttempts++;
  barcodeMapCmdId = frame.readUInt16BE(0);
  ingest.client.publish(`${config.topicPrefix}/${schema.topics.command}`, frame, { qos: 1 });
}

const ingest = createMqttIngest(config, schema, (packet, topic) => {
  const timestamp = Date.now();
  if (packet.name === "heartbeat") {
    commands.noteHeartbeat(packet.values, timestamp);
    if (packet.values.run_id !== barcodeMapRun) {
      barcodeMapRun = packet.values.run_id;
      barcodeMapApplied = false;
      barcodeMapCmdId = null;
      barcodeMapAttempts = 0;
    }
    syncBarcodeMap();
  } else if (packet.name === "diag") {
    commands.noteDiag(packet.values);
    syncBarcodeMap();
  } else if (packet.name === "event" && packet.values.event_type === EVENTS.cmdAccept &&
             packet.values.source === ENUMS.eventSource.connection &&
             ((packet.values.p1 ?? 0) & 255) === ENUMS.commandConfig.barcodeMap &&
             packet.values.p2 === barcodeMapCmdId) {
    barcodeMapApplied = true;
  }
  const receipt = history.append({
    timestamp,
    topic,
    packet: packet.name,
    values: packet.values,
  });
  view.append(receipt);
  live?.broadcast(receipt);
  console.log(`${packet.name}: ${JSON.stringify(packet.values)}`);
}, barcode);

ingest.client.on("connect", () => {
  if (!barcodeMapApplied) {
    barcodeMapAttempts = 0;
    barcodeMapLastSent = 0;
  }
  live?.broadcastStatus();
});
ingest.client.on("close", () => live?.broadcastStatus());

const corsHeaders = {
  "access-control-allow-origin": "*",
  "access-control-allow-methods": "GET, POST, PUT, OPTIONS",
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
  const frame = commands.frame(schema, command.action, Date.now());
  if (frame === null) {
    sendJson(response, 503, { error: "no fresh heartbeat or no diag frame from the car" });
    return;
  }
  const topic = `${config.topicPrefix}/${schema.topics.command}`;
  const cmdId = frame.readUInt16BE(0);

  try {
    await publishWithTimeout(ingest.client, topic, frame, PUBACK_TIMEOUT_MS);
  } catch (error) {
    console.error(`command publish failed: ${(error as Error).message}`);
    if (error instanceof CommandTimeoutError) {
      sendJson(response, 504, { error: "mqtt publish timed out" });
    } else {
      sendJson(response, 502, { error: "mqtt publish failed" });
    }
    return;
  }
  sendJson(response, 202, { ok: true, cmdId });
}

function barcodeMapView() {
  const exhausted = barcodeMapAttempts >= BARCODE_MAP_MAX_ATTEMPTS &&
    Date.now() - barcodeMapLastSent >= BARCODE_MAP_RETRY_MS;
  return {
    bindings: savedBarcodeMap ?? DEFAULT_BARCODE_MAP,
    status: !savedBarcodeMap ? "default" : barcodeMapApplied ? "applied" : exhausted ? "unconfirmed" : "pending",
  };
}

async function handleBarcodeMap(request: IncomingMessage, response: ServerResponse): Promise<void> {
  let map: BarcodeMap;
  try {
    map = parseBarcodeMap(JSON.parse(await readBody(request)));
  } catch (error) {
    sendJson(response, 400, { error: (error as Error).message });
    return;
  }
  try {
    saveBarcodeMap(barcodeMapPath, map);
  } catch (error) {
    console.error(`barcode mapping: ${(error as Error).message}`);
    sendJson(response, 503, { error: "Could not save barcode mapping" });
    return;
  }
  savedBarcodeMap = map;
  barcodeMapApplied = false;
  barcodeMapCmdId = null;
  barcodeMapLastSent = 0;
  barcodeMapAttempts = 0;
  syncBarcodeMap();
  sendJson(response, 202, barcodeMapView());
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
  if (url.pathname === "/barcode-map") {
    if (request.method === "GET") {
      response.setHeader("cache-control", "no-store");
      sendJson(response, 200, barcodeMapView());
    } else if (request.method === "PUT") {
      void handleBarcodeMap(request, response);
    } else {
      response.setHeader("allow", "GET, PUT, OPTIONS");
      sendJson(response, 405, { error: "Use GET or PUT for the barcode mapping" });
    }
    return;
  }

  if (url.pathname === "/health") {
    sendJson(response, 200, {
      mqtt: ingest.client.connected ? "connected" : "disconnected",
      packets: ingest.stats.packets,
      lastPacketAt: ingest.stats.lastPacketAt,
      lastPacket: ingest.stats.lastPacket,
      historyStorage: history.storageStatus(),
      barcode: barcode.health(),
    });
    return;
  }

  if (url.pathname === "/history") {
    const rawLimit = url.searchParams.get("limit");
    const parsed = rawLimit === null ? 100 : Number(rawLimit);
    const limit = Number.isFinite(parsed) ? parsed : 100;
    sendJson(response, 200, history.list(limit));
    return;
  }

  if (url.pathname === "/view") {
    if (request.method !== "GET") {
      response.setHeader("allow", "GET, OPTIONS");
      sendJson(response, 405, { error: "use GET for the console view" });
    } else {
      response.setHeader("cache-control", "no-store");
      sendJson(response, 200, view.read());
    }
    return;
  }

  if (url.pathname === "/command") {
    response.setHeader("allow", "POST, OPTIONS");
    sendJson(response, 405, { error: "use POST for commands" });
    return;
  }
  if (dashboard && (request.method === "GET" || request.method === "HEAD")) {
    dashboard(request, response, () => sendJson(response, 404, {}));
    return;
  }
  sendJson(response, 404, {});
});

live = createLiveServer(server, () => ({
  mqtt: ingest.client.connected ? "connected" : "disconnected",
  packets: ingest.stats.packets,
  lastPacketAt: ingest.stats.lastPacketAt,
  historyStorage: history.storageStatus(),
}), view.read);

server.listen(config.httpPort, config.httpHost, () => {
  const address = server.address();
  const host = config.httpHost.includes(":") ? `[${config.httpHost}]` : config.httpHost;
  console.log(`Ground station on http://${host}:${typeof address === "object" && address ? address.port : config.httpPort}`);
});

let closing = false;
function shutdown(): void {
  if (closing) return;
  closing = true;
  clearInterval(saveCourse);
  view.save();
  live?.close();
  server.close();
  ingest.client.end(true);
  barcode.close();
  broker?.kill();
}

broker?.once("exit", (code, signal) => {
  if (!closing) {
    console.error(`Mosquitto exited (${signal ?? code}).`);
    process.exitCode = 1;
    shutdown();
  }
});
process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);
