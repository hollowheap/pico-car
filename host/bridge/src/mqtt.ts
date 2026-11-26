import mqtt, { type MqttClient } from "mqtt";
import { archivedBarcodeValues, barcodeReceipt, decodeBarcodeBatch, type BarcodeArchive } from "./barcode-archive.js";
import type { BridgeConfig } from "./config.js";
import { barcodeSpanKind } from "../../dashboard/src/lib/barcode-span.js";
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
  barcode?: BarcodeArchive,
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
  const barcodeTopic = `${config.topicPrefix}/${schema.topics.barcode_sample}`;
  const receiptTopic = schema.topics.barcode_receipt;
  const batchSize = schema.packets.barcode_batch?.wire_size;
  let heartbeat: Record<string, number> | undefined;
  let lastEvent: Record<string, number> | undefined;

  function accept(packet: DecodedPacket, topic: string, receivedAt: number) {
    const { name, values } = packet;
    if (name === "heartbeat") {
      if (heartbeat && (values.run_id !== heartbeat.run_id || values.car_id !== heartbeat.car_id ||
          values.uptime_s < heartbeat.uptime_s)) lastEvent = undefined;
      heartbeat = values;
    }
    if (name === "event") {
      if (lastEvent && Object.entries(values).every(([field, value]) => lastEvent?.[field] === value)) return;
      lastEvent = values;
    }
    if (name === "barcode_sample") values.element_kind = barcodeSpanKind(values);
    stats.packets += 1;
    stats.lastPacketAt = receivedAt;
    stats.lastPacket = packet;
    onPacket(packet, topic);
  }

  client.on("connect", () => {
    const subscriptions: Record<string, { qos: 0 | 1 }> = {};
    for (const [name, suffix] of Object.entries(schema.topics)) {
      if (name === "command" || name === "barcode_receipt") continue;
      subscriptions[`${config.topicPrefix}/${suffix}`] = { qos: name === "event" || name === "barcode_sample" ? 1 : 0 };
    }
    client.subscribe(subscriptions);
  });

  client.on("error", (error) => {
    console.error(`mqtt: ${error.message}`);
  });

  client.on("message", (topic, payload) => {
    const name = packetNameForTopic(schema, topic);
    if (!name) {
      return;
    }
    try {
      const receivedAt = Date.now();
      if (topic === barcodeTopic && payload.byteLength === batchSize) {
        if (!barcode || !receiptTopic) throw new Error("retained barcode archive unavailable");
        const batch = decodeBarcodeBatch(schema, payload);
        for (const sample of barcode.append(batch, receivedAt)) {
          accept({ name, values: archivedBarcodeValues(batch.carId, batch.bootId, sample) }, topic, receivedAt);
        }
        client.publish(`${config.topicPrefix}/${receiptTopic}`, barcodeReceipt(schema, batch), { qos: 0 });
        return;
      }
      accept(decodePacket(schema, name, payload), topic, receivedAt);
    } catch (error) {
      console.error(`drop ${topic}: ${(error as Error).message}`);
    }
  });

  return { client, stats };
}
