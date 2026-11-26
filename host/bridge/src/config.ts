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
  historyPath: string;
  historyMaxRecords: number;
  historyMaxEvents: number;
};

export function loadConfig(env: NodeJS.ProcessEnv = process.env): BridgeConfig {
  return {
    mqttUrl: env.MQTT_URL || "mqtt://localhost:1883",
    username: env.MQTT_USERNAME,
    password: env.MQTT_PASSWORD,
    topicPrefix: env.MQTT_TOPIC_PREFIX ?? "car/PICO-CAR",
    httpPort: Number(env.BRIDGE_HTTP_PORT ?? "5281"),
    httpHost: env.BRIDGE_HTTP_HOST ?? "127.0.0.1",
    dashboardPath: env.BRIDGE_DASHBOARD_PATH ?? fileURLToPath(new URL("../../dashboard/dist/", import.meta.url)),
    schemaPath: env.PROTOCOL_SCHEMA ?? fileURLToPath(new URL("../../../protocol/telemetry.yaml", import.meta.url)),
    historyPath: env.BRIDGE_HISTORY_PATH ?? fileURLToPath(new URL("../../../testing/logs/bridge/history.jsonl", import.meta.url)),
    historyMaxRecords: Number(env.BRIDGE_HISTORY_MAX_RECORDS ?? "10000"),
    historyMaxEvents: Number(env.BRIDGE_HISTORY_MAX_EVENTS ?? "2000"),
  };
}
