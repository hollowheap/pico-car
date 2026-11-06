import { describe, expect, it } from "vitest";
import { encodeCommand, parseCommandRequest } from "./command.js";
import type { ProtocolSchema } from "./schema.js";

const schema: ProtocolSchema = {
  topics: { command: "c" },
  packets: {},
  commands: { stop: 0, start: 1, reset: 2 },
};

describe("commands", () => {
  it("encodes the 8-byte big-endian frame", () => {
    const request = parseCommandRequest(
      { action: "reset", cmdId: 7, runId: 3, sentMs: 0x1234 },
      schema,
    );
    expect([...encodeCommand(schema, request)]).toEqual([
      0x00, 0x07, 0x00, 0x03, 0x12, 0x34, 0x02, 0x00,
    ]);
  });

  it("rejects unknown actions and out-of-range fields", () => {
    expect(() =>
      parseCommandRequest({ action: "turn", cmdId: 1, runId: 1, sentMs: 1 }, schema),
    ).toThrow("unknown command");
    expect(() =>
      parseCommandRequest({ action: "start", cmdId: -1, runId: 1, sentMs: 1 }, schema),
    ).toThrow("uint16");
    expect(() =>
      parseCommandRequest({ action: "start", cmdId: 1, runId: 1, sentMs: 0x10000 }, schema),
    ).toThrow("uint16");
  });
});
