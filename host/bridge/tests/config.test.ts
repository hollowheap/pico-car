import { mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import { credentialsDir, loadConfig } from "../src/config.js";

const dirs: string[] = [];

function credentialsWith(files: Record<string, string>): string {
  const dir = mkdtempSync(join(tmpdir(), "bridge-credentials-"));
  dirs.push(dir);
  for (const [name, text] of Object.entries(files)) writeFileSync(join(dir, name), text);
  return dir;
}

afterEach(() => {
  for (const dir of dirs.splice(0)) rmSync(dir, { recursive: true, force: true });
});

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

  it("reads the bridge login from the generated credentials file", () => {
    const dir = credentialsWith({ "bridge.env": "MQTT_USERNAME=bridge\nMQTT_PASSWORD=from-file\n" });
    const config = loadConfig({ PICO_CAR_CREDENTIALS_DIR: dir });
    expect(config.username).toBe("bridge");
    expect(config.password).toBe("from-file");
  });

  it("prefers the environment over the credentials file", () => {
    const dir = credentialsWith({ "bridge.env": "MQTT_USERNAME=bridge\nMQTT_PASSWORD=from-file\n" });
    const config = loadConfig({
      PICO_CAR_CREDENTIALS_DIR: dir,
      MQTT_USERNAME: "other",
      MQTT_PASSWORD: "from-env",
    });
    expect(config.username).toBe("other");
    expect(config.password).toBe("from-env");
  });

  it("connects without a login when no credentials exist", () => {
    const dir = credentialsWith({});
    const config = loadConfig({ PICO_CAR_CREDENTIALS_DIR: dir });
    expect(config.username).toBeUndefined();
    expect(config.password).toBeUndefined();
  });

  it("locates the credentials directory", () => {
    expect(credentialsDir({ PICO_CAR_CREDENTIALS_DIR: "/tmp/x" })).toBe("/tmp/x");
    expect(credentialsDir({})).toMatch(/host\/credentials$/);
  });
});
