export type BridgeConfig = {
  mqttUrl: string;
  username?: string;
  password?: string;
  topicPrefix: string;
  httpPort: number;
  schemaPath: string;
  historyPath: string;
  historyMaxRecords: number;
};

export function loadConfig(env: NodeJS.ProcessEnv = process.env): BridgeConfig {
  return {
    mqttUrl: env.MQTT_URL || "mqtt://localhost:1883",
    username: env.MQTT_USERNAME,
    password: env.MQTT_PASSWORD,
    topicPrefix: env.MQTT_TOPIC_PREFIX ?? "car/PICO-CAR",
    httpPort: Number(env.BRIDGE_HTTP_PORT ?? "8080"),
    schemaPath: env.PROTOCOL_SCHEMA ?? "../../protocol/telemetry.yaml",
    historyPath: env.BRIDGE_HISTORY_PATH ?? "history.jsonl",
    historyMaxRecords: Number(env.BRIDGE_HISTORY_MAX_RECORDS ?? "10000"),
  };
}
