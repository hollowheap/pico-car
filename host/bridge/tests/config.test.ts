import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { loadConfig } from "../src/config.js";

describe("bridge config", () => {
  it("defaults to the local broker and topic prefix", () => {
    const config = loadConfig({});
    expect(config.mqttUrl).toBe("mqtt://localhost:1883");
    expect(config.topicPrefix).toBe("car/PICO-CAR");
  });

  it("honors MQTT_URL and MQTT_TOPIC_PREFIX", () => {
    const config = loadConfig({
      MQTT_URL: "mqtt://10.0.0.2:1884",
      MQTT_TOPIC_PREFIX: "car/TEST",
    });
    expect(config.mqttUrl).toBe("mqtt://10.0.0.2:1884");
    expect(config.topicPrefix).toBe("car/TEST");
  });

  it("records to testing/logs/bridge unless BRIDGE_HISTORY_PATH is set", () => {
    const repoRoot = fileURLToPath(new URL("../../../", import.meta.url));
    expect(loadConfig({}).historyPath).toBe(`${repoRoot}testing/logs/bridge/history.jsonl`);
    expect(loadConfig({ BRIDGE_HISTORY_PATH: "/tmp/run.jsonl" }).historyPath).toBe("/tmp/run.jsonl");
  });
});
