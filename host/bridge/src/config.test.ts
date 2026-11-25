import { describe, expect, it } from "vitest";
import { loadConfig } from "./config.js";

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
});
