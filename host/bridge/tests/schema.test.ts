import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { decodePacket, loadProtocolSchema, packetNameForTopic } from "../src/schema.js";

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

  it("accepts the extended vehicle status on the same topic as legacy status", () => {
    const legacy = new Uint8Array(16);
    const extended = Uint8Array.of(...legacy, 67);
    expect(decodePacket(schema, "status", legacy).values).not.toHaveProperty("pivot_scale_pct");
    expect(decodePacket(schema, "status", extended)).toEqual({
      name: "status", values: { ...decodePacket(schema, "status", legacy).values, pivot_scale_pct: 67 },
    });
  });

  it("preserves a sonar command's signed angle in the existing event frame", () => {
    const type = packetNameForTopic(schema, "car/PICO-CAR/e");
    expect(type).toBe("event");
    const wire = Uint8Array.of(0, 23, 0x25, 3, 0xff, 0xbf, 0, 0, 0, 0, 0, 65);
    expect(decodePacket(schema, type!, wire).values).toEqual({
      event_id: 23, event_type: 0x25, source: 3, p1: -65, p2: 0, odometer_mm: 65,
    });
  });

  it("rejects a wrong-sized frame", () => {
    expect(() => decodePacket(schema, "heartbeat", new Uint8Array(3))).toThrow(RangeError);
  });

  it("keeps new cumulative diagnostics on the command-correlation stream", () => {
    const bytes = new Uint8Array(73);
    const view = new DataView(bytes.buffer);
    view.setUint16(38, 1234);
    view.setUint32(24, 0x1234ffff);
    const diag = decodePacket(schema, "diag", bytes);
    expect(diag.name).toBe("diag");
    expect(diag.values).toMatchObject({ cmd_last_id: 1234, line_edges_total: 0x1234ffff });
  });

  it("keeps withdrawn wire opcode 10 reserved out of active commands", () => {
    expect(schema.commands.start).toBe(1);
    expect(schema.commands.start_continuous_floor).toBe(9);
    expect(schema.commands).not.toHaveProperty("start_stationary");
    expect(Object.values(schema.commands)).not.toContain(10);
    expect(schema.reserved_commands).toMatchObject({ start_stationary: 10 });
  });
});
