import { readFileSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";

export type BridgeConfig = {
  mqttUrl: string;
  username?: string;
  password?: string;
  topicPrefix: string;
  httpPort: number;
  httpHost: string;
  dashboardPath: string;
  schemaPath: string;
  carIdPath: string;
  historyPath: string;
  historyMaxRecords: number;
  historyMaxEvents: number;
};

/** The directory host/credentials.sh fills with the broker and bridge files. */
export function credentialsDir(env: NodeJS.ProcessEnv = process.env): string {
  return env.PICO_CAR_CREDENTIALS_DIR ?? fileURLToPath(new URL("../../credentials", import.meta.url));
}

function readEnvFile(path: string): Record<string, string> {
  let text: string;
  try {
    text = readFileSync(path, "utf8");
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code === "ENOENT") return {};
    throw error;
  }
  const values: Record<string, string> = {};
  for (const line of text.split("\n")) {
    const at = line.indexOf("=");
    if (at > 0) values[line.slice(0, at)] = line.slice(at + 1);
  }
  return values;
}

export function loadConfig(env: NodeJS.ProcessEnv = process.env): BridgeConfig {
  const login = env.MQTT_USERNAME === undefined ? readEnvFile(join(credentialsDir(env), "bridge.env")) : {};
  return {
    mqttUrl: env.MQTT_URL || "mqtt://localhost:1883",
    username: env.MQTT_USERNAME ?? login.MQTT_USERNAME,
    password: env.MQTT_PASSWORD ?? login.MQTT_PASSWORD,
    topicPrefix: env.MQTT_TOPIC_PREFIX ?? "car/PICO-CAR",
    httpPort: Number(env.BRIDGE_HTTP_PORT ?? "5281"),
    httpHost: env.BRIDGE_HTTP_HOST ?? "127.0.0.1",
    dashboardPath: fileURLToPath(new URL("../../dashboard/dist/", import.meta.url)),
    schemaPath: env.PROTOCOL_SCHEMA ?? fileURLToPath(new URL("../../../protocol/telemetry.yaml", import.meta.url)),
    carIdPath: env.CAR_ID_PATH ?? join(credentialsDir(env), "car-id"),
    historyPath: env.BRIDGE_HISTORY_PATH ?? fileURLToPath(new URL("../history.jsonl", import.meta.url)),
    historyMaxRecords: Number(env.BRIDGE_HISTORY_MAX_RECORDS ?? "10000"),
    historyMaxEvents: Number(env.BRIDGE_HISTORY_MAX_EVENTS ?? "2000"),
  };
}
