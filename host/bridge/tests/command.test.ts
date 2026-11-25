import { describe, expect, it } from "vitest";
import { createCommandSession, encodeCommand, parseCommandRequest, publishWithTimeout } from "../src/command.js";
import { loadProtocolSchema, type ProtocolSchema } from "../src/schema.js";

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

  it("refuses the withdrawn stationary command in the shipped schema", () => {
    const shipped = loadProtocolSchema(new URL("../../../protocol/telemetry.yaml", import.meta.url).pathname);
    expect(parseCommandRequest({ action: "start" }, shipped)).toEqual({ action: "start" });
    expect(() => parseCommandRequest({ action: "start_stationary" }, shipped)).toThrow("unknown command");
  });

  it("refuses reserved commands during parsing, encoding, and session framing while keeping start aliases", () => {
    const shipped = loadProtocolSchema(new URL("../../../protocol/telemetry.yaml", import.meta.url).pathname);
    expect(encodeCommand(shipped, "start", 1, 0x1234, 5)[4]).toBe(1);
    expect(encodeCommand(shipped, "start_continuous_floor", 2, 0x1234, 5)[4]).toBe(9);
    expect(() => encodeCommand(shipped, "start_stationary", 3, 0x1234, 5)).toThrow("unknown command");

    const session = readySession(10, 9);
    expect(() => session.frame(shipped, "start_stationary", 0)).toThrow("unknown command");
    expect(cmdIdOf(session.frame(shipped, "start", 0))).toBe(10);

    const overlapping: ProtocolSchema = {
      ...schema,
      commands: { ...schema.commands, start_stationary: 10, reused_opcode: 10 },
      reserved_commands: { start_stationary: 10 },
    };
    expect(() => parseCommandRequest({ action: "start_stationary" }, overlapping)).toThrow("unknown command");
    expect(() => parseCommandRequest({ action: "reused_opcode" }, overlapping)).toThrow("unknown command");
    expect(() => encodeCommand(overlapping, "reused_opcode", 1, 1, 1)).toThrow("unknown command");
  });

  it("rejects inherited Object.prototype property names instead of encoding STOP", () => {
    for (const action of ["constructor", "toString", "valueOf", "hasOwnProperty", "__proto__"]) {
      expect(() => parseCommandRequest({ action }, schema)).toThrow("unknown command");
      expect(() => encodeCommand(schema, action, 1, 1, 1)).toThrow("unknown command");
    }
  });

  it("bounds MQTT PUBACK waiting with a timeout", async () => {
    const hanging = { publish: () => {} };
    await expect(publishWithTimeout(hanging, "car/c", Buffer.alloc(9), 20)).rejects.toThrow(/timed out/i);

    const immediate = {
      publish: (_topic: string, _payload: Buffer, _opts: { qos: 1 }, cb: (err?: Error) => void) => cb(),
    };
    await expect(publishWithTimeout(immediate, "car/c", Buffer.alloc(9), 50)).resolves.toBeUndefined();
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
    session.noteHeartbeat({ run_id: 0x1234, uptime_s: 0x01020304 }, 10);
    expect([...session.frame(schema, "stop", 10)!]).toEqual([
      0x00, 0x00, 0x12, 0x34, 0x00, 0x01, 0x02, 0x03, 0x04,
    ]);
  });

  it("waits for current diagnostics before sending commands to a new run", () => {
    const session = readySession(100, 99);
    session.noteHeartbeat({ run_id: 0x0042, uptime_s: 1 }, 10);
    expect(session.frame(schema, "stop", 10)).toBeNull();
    session.noteDiag({});
    expect(session.frame(schema, "stop", 10)).toBeNull();

    // A direct sender already reached id 101 on this run.
    session.noteDiag({ cmd_last_id: 101 });
    expect([...session.frame(schema, "stop", 10)!]).toEqual([
      0x00, 0x66, 0x00, 0x42, 0x00, 0x00, 0x00, 0x00, 0x01,
    ]);
  });

  it("can receive diagnostics before the first heartbeat", () => {
    const session = createCommandSession(100);
    session.noteDiag({ cmd_last_id: 101 });
    session.noteHeartbeat({ run_id: 0x0042, uptime_s: 5 }, 10);
    expect(cmdIdOf(session.frame(schema, "start", 10))).toBe(102);
  });

  it("keeps STOP newer after a repeated pre-START diagnostic", () => {
    const session = readySession(0x7fff + 500, 500);
    const started = cmdIdOf(session.frame(schema, "start", 0));

    session.noteDiag({ cmd_last_id: 500 });

    const stopped = cmdIdOf(session.frame(schema, "stop", 0));
    const ahead = (stopped - started) & 0xffff;
    expect(ahead).toBeGreaterThan(0);
    expect(ahead).toBeLessThan(0x8000);
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

  it("resets the command floor when the car starts a new run", () => {
    const session = readySession(0x8000, 0x7fff);
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe(0x8000);
    session.noteHeartbeat({ run_id: 0x5678, uptime_s: 1 }, 10);
    session.noteDiag({ cmd_last_id: 0 });
    expect(cmdIdOf(session.frame(schema, "start", 10))).toBe(1);
  });

  it("keeps ordering and external sync across 16-bit rollover", () => {
    const session = readySession(0x7ffe, 0xffff);
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe(0x7ffe);
    session.noteDiag({ cmd_last_id: 0xffff });
    expect(cmdIdOf(session.frame(schema, "stop", 0))).toBe(0x7fff);

    const wrapping = readySession(0xfffe, 0xfffd);
    expect(cmdIdOf(wrapping.frame(schema, "start", 0))).toBe(0xfffe);
    wrapping.noteDiag({ cmd_last_id: 0xfffd });
    expect(cmdIdOf(wrapping.frame(schema, "stop", 0))).toBe(0xffff);
    wrapping.noteDiag({ cmd_last_id: 0x0002 });
    expect(cmdIdOf(wrapping.frame(schema, "start", 0))).toBe(0x0003);
  });

  it("tracks genuinely advancing diagnostics even when the next id stays ahead", () => {
    const session = readySession(510, 500);
    session.noteDiag({ cmd_last_id: 505 });
    session.noteDiag({ cmd_last_id: (505 + 0x7fff) & 0xffff });
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe((505 + 0x8000) & 0xffff);
  });

  it("leaves command id and floor unchanged when encoding throws", () => {
    const keepsNext = readySession(10, 9);
    expect(() => keepsNext.frame(schema, "turn", 0)).toThrow("unknown command");
    expect(cmdIdOf(keepsNext.frame(schema, "start", 0))).toBe(10);

    const keepsFloor = readySession(10, 9);
    expect(() => keepsFloor.frame(schema, "turn", 0)).toThrow("unknown command");
    keepsFloor.noteDiag({ cmd_last_id: 10 });
    expect(cmdIdOf(keepsFloor.frame(schema, "start", 0))).toBe(11);
  });

  it("advances the command floor on issued frames so stale advancing diagnostics do not snap next back", () => {
    const session = readySession(500 + 0x7fff, 500);
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe((500 + 0x7fff) & 0xffff);
    expect(cmdIdOf(session.frame(schema, "stop", 0))).toBe((500 + 0x8000) & 0xffff);
    expect(cmdIdOf(session.frame(schema, "start", 0))).toBe((500 + 0x8001) & 0xffff);
    session.noteDiag({ cmd_last_id: 501 });
    expect(cmdIdOf(session.frame(schema, "stop", 0))).toBe((500 + 0x8002) & 0xffff);
  });
});
