import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { decodePacket, loadProtocolSchema, packetNameForTopic } from "./schema.js";

const schema = loadProtocolSchema(new URL("../../../protocol/telemetry.yaml", import.meta.url).pathname);
const vectors = JSON.parse(
  readFileSync(new URL("../../../protocol/vectors.json", import.meta.url).pathname, "utf8"),
) as { vectors: Record<string, { values: Record<string, number | number[]>; hex: string }> };

function flatten(values: Record<string, number | number[]>): Record<string, number> {
  const flat: Record<string, number> = {};
  for (const [name, value] of Object.entries(values)) {
    if (Array.isArray(value)) {
      value.forEach((item, index) => {
        flat[`${name}_${index}`] = item;
      });
    } else {
      flat[name] = value;
    }
  }
  return flat;
}

describe("protocol schema", () => {
  it("maps topic suffixes to packet names", () => {
    expect(packetNameForTopic(schema, "car/PICO-CAR/s")).toBe("status");
    expect(packetNameForTopic(schema, "car/PICO-CAR/m")).toBe("motion");
    expect(packetNameForTopic(schema, "car/PICO-CAR/p")).toBe("physics");
    expect(packetNameForTopic(schema, "car/PICO-CAR/o")).toBe("obstacle");
    expect(packetNameForTopic(schema, "car/PICO-CAR/d")).toBe("diag");
    expect(packetNameForTopic(schema, "car/PICO-CAR/e")).toBe("event");
    expect(packetNameForTopic(schema, "car/PICO-CAR/h")).toBe("heartbeat");
    expect(packetNameForTopic(schema, "car/PICO-CAR/f")).toBe("profile");
    expect(packetNameForTopic(schema, "car/PICO-CAR/c")).toBeUndefined();
  });

  it("decodes a status frame", () => {
    const payload = new Uint8Array([
      0xff, 0x6a, 0x00, 0x00, 0x03, 0xe8, 0x0e, 0x10, 0x02, 0x01, 0x00, 0x04, 0x00, 0x2a, 0x03, 0x00,
    ]);
    expect(decodePacket(schema, "status", payload).values).toEqual({
      speed_mms: -150,
      odometer_mm: 1000,
      heading_ddeg: 3600,
      state: 2,
      mode: 1,
      faults: 4,
      seq: 42,
      line_bits: 3,
      hold_mask: 0,
    });
  });

  it("decodes every shared vector", () => {
    for (const [name, vector] of Object.entries(vectors.vectors)) {
      const bytes = Uint8Array.from(Buffer.from(vector.hex, "hex"));
      expect(decodePacket(schema, name, bytes).values).toEqual(flatten(vector.values));
    }
  });

  it("rejects a wrong-sized frame", () => {
    expect(() => decodePacket(schema, "heartbeat", new Uint8Array(3))).toThrow(RangeError);
  });
});
