import { describe, expect, it } from "vitest";
import { decodePacket, loadProtocolSchema, packetNameForTopic } from "./schema.js";

const schema = loadProtocolSchema(new URL("../../../protocol/telemetry.yaml", import.meta.url).pathname);

describe("protocol schema", () => {
  it("maps topic suffixes to packet names", () => {
    expect(packetNameForTopic(schema, "car/SE01/s")).toBe("status");
    expect(packetNameForTopic(schema, "car/SE01/e")).toBe("event");
    expect(packetNameForTopic(schema, "car/SE01/h")).toBe("heartbeat");
    expect(packetNameForTopic(schema, "car/SE01/c")).toBeUndefined();
  });

  it("decodes a status frame", () => {
    const payload = new Uint8Array([0x00, 0x96, 0x03, 0xe8, 0x5a, 0x02, 0x00, 0x01]);
    expect(decodePacket(schema, "status", payload).values).toEqual({
      speed_mms: 150,
      dist_mm: 1000,
      heading_deg: 90,
      state: 2,
      faults: 1,
    });
  });

  it("rejects a wrong-sized frame", () => {
    expect(() => decodePacket(schema, "heartbeat", new Uint8Array(3))).toThrow(RangeError);
  });
});
