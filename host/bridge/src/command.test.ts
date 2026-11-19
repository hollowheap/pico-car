import { describe, expect, it } from "vitest";
import { createCommandSession, encodeCommand, parseCommandRequest } from "./command.js";
import type { ProtocolSchema } from "./schema.js";

const schema: ProtocolSchema = {
  topics: { command: "c" },
  packets: {},
  commands: { stop: 0, start: 1, reset: 2 },
};

/** A session that has seen one heartbeat and one diag frame. */
function readySession(firstCmdId: number, lastCmdId: number) {
  const session = createCommandSession(firstCmdId);
  session.noteHeartbeat({ run_id: 0x1234, uptime_s: 42 }, 0);
  session.noteDiag({ cmd_last_id: lastCmdId });
  return session;
}

const cmdIdOf = (frame: Buffer | null) => frame!.readUInt16BE(0);

describe("commands", () => {
  it("encodes the 9-byte big-endian frame", () => {
    expect([...encodeCommand(schema, "reset", 7, 0xbeef, 0x01020304)]).toEqual([
      0x00, 0x07, 0xbe, 0xef, 0x02, 0x01, 0x02, 0x03, 0x04,
    ]);
    expect([...encodeCommand(schema, "stop", 0x1ffff, 0x10001, 0xffffffff)]).toEqual([
      0xff, 0xff, 0x00, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff,
    ]);
  });

  it("takes only a known action", () => {
    expect(parseCommandRequest({ action: "start" }, schema)).toEqual({ action: "start" });
    expect(() => parseCommandRequest({ action: "turn" }, schema)).toThrow("unknown command");
    expect(() => parseCommandRequest(null, schema)).toThrow("object");
  });
});

describe("command session", () => {
  it("refuses a command until a fresh heartbeat and a diag frame arrived", () => {
    const session = createCommandSession(10);
    expect(session.frame(schema, "start", 1000)).toBeNull();
    session.noteHeartbeat({ run_id: 0x1234, uptime_s: 5 }, 1000);
    // The diag frame gives the last id the car accepted.
    expect(session.frame(schema, "start", 1000)).toBeNull();
    session.noteDiag({ cmd_last_id: 9 });
    expect(session.frame(schema, "start", 1000 + 3000)).not.toBeNull();
    expect(session.frame(schema, "start", 1000 + 3001)).toBeNull();
  });

  it("echoes the run ID and the uptime of the newest heartbeat", () => {
    const session = readySession(0xffff, 0xfffe);
    expect([...session.frame(schema, "start", 0)!]).toEqual([
      0xff, 0xff, 0x12, 0x34, 0x01, 0x00, 0x00, 0x00, 0x2a,
    ]);
    // The counter wraps; a car reboot gives a new run ID in the next heartbeat.
    session.noteHeartbeat({ run_id: 0x0042, uptime_s: 0x01020304 }, 10);
    expect([...session.frame(schema, "stop", 10)!]).toEqual([
      0x00, 0x00, 0x00, 0x42, 0x00, 0x01, 0x02, 0x03, 0x04,
    ]);
  });

  it("ignores a heartbeat without a run ID or an uptime", () => {
    const session = createCommandSession(1);
    session.noteDiag({ cmd_last_id: 0 });
    session.noteHeartbeat({ uptime_s: 5 }, 0);
    expect(session.frame(schema, "start", 0)).toBeNull();
    session.noteHeartbeat({ run_id: 0x1234 }, 0);
    expect(session.frame(schema, "start", 0)).toBeNull();
  });

  it("moves past the last id the car accepted when the car would refuse the next one", () => {
    // A restarted bridge whose counter is behind the car.
    expect(cmdIdOf(readySession(100, 500).frame(schema, "start", 0))).toBe(501);
    // A repeat of the car's last id.
    expect(cmdIdOf(readySession(500, 500).frame(schema, "start", 0))).toBe(501);
    // Half the sequence or more ahead reads as behind on the car.
    expect(cmdIdOf(readySession(0x8000 + 500, 500).frame(schema, "start", 0))).toBe(501);
    // The sync wraps after 0xffff.
    expect(cmdIdOf(readySession(3, 0xffff).frame(schema, "start", 0))).toBe(3);
    expect(cmdIdOf(readySession(0xfff0, 0xffff).frame(schema, "start", 0))).toBe(0);
  });

  it("keeps a counter the car would accept", () => {
    expect(cmdIdOf(readySession(501, 500).frame(schema, "start", 0))).toBe(501);
    expect(cmdIdOf(readySession(500 + 0x7fff, 500).frame(schema, "start", 0))).toBe(500 + 0x7fff);
  });

  it("never moves back for a diag frame older than its own commands", () => {
    const session = readySession(10, 9);
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe(10);
    expect(cmdIdOf(session.frame(schema, "stop", 0))).toBe(11);
    // The diag frame was built before the car took ids 10 and 11.
    session.noteDiag({ cmd_last_id: 9 });
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe(12);
    // Another sender took 12 to 20 meanwhile.
    session.noteDiag({ cmd_last_id: 20 });
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe(21);
  });
});
