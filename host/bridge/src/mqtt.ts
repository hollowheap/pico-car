import mqtt, { type MqttClient } from "mqtt";
import type { BridgeConfig } from "./config.js";
import {
  decodePacket,
  packetNameForTopic,
  type DecodedPacket,
  type ProtocolSchema,
} from "./schema.js";

export type IngestStats = {
  packets: number;
  lastPacketAt: number | null;
  lastPacket: DecodedPacket | null;
};

export function createMqttIngest(
  config: BridgeConfig,
  schema: ProtocolSchema,
  onPacket: (packet: DecodedPacket, topic: string) => void,
): { client: MqttClient; stats: IngestStats } {
  const client = mqtt.connect(config.mqttUrl, {
    username: config.username,
    password: config.password,
    clean: true,
    reconnectPeriod: 1000,
  });

  const stats: IngestStats = {
    packets: 0,
    lastPacketAt: null,
    lastPacket: null,
  };

  client.on("connect", () => {
    client.subscribe(`${config.topicPrefix}/#`, { qos: 0 });
  });

  client.on("message", (topic, payload) => {
    const name = packetNameForTopic(schema, topic);
    if (!name) {
      return;
    }
    try {
      const packet = decodePacket(schema, name, payload);
      stats.packets += 1;
      stats.lastPacketAt = Date.now();
      stats.lastPacket = packet;
      onPacket(packet, topic);
    } catch (error) {
      console.error(`drop ${topic}: ${(error as Error).message}`);
    }
  });

  return { client, stats };
}
