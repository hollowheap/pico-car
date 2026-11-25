import type { ProtocolSchema } from './schema.js';

export type CommandRequest = {
  action: string;
};

/** Three missed 1 Hz heartbeats: the car is gone, or rebooted with a new run ID. */
export const HEARTBEAT_FRESH_MS = 3000;
/** Maximum time to wait for an MQTT QoS 1 PUBACK before returning 504. */
export const PUBACK_TIMEOUT_MS = 5000;

export class CommandTimeoutError extends Error {
  constructor(message = 'mqtt publish timed out') {
    super(message);
    this.name = 'CommandTimeoutError';
  }
}

export type PublishClient = {
  publish: (
    topic: string,
    payload: Buffer,
    options: { qos: 1 },
    callback: (error?: Error) => void,
  ) => void;
};

export function publishWithTimeout(
  client: PublishClient,
  topic: string,
  frame: Buffer,
  timeoutMs = PUBACK_TIMEOUT_MS,
): Promise<void> {
  return new Promise<void>((resolve, reject) => {
    let settled = false;
    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      reject(new CommandTimeoutError());
    }, timeoutMs);
    client.publish(topic, frame, { qos: 1 }, (error) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      if (error) reject(error);
      else resolve();
    });
  });
}

export function parseCommandRequest(
  input: unknown,
  schema: ProtocolSchema,
): CommandRequest {
  if (typeof input !== 'object' || input === null) {
    throw new Error('command body must be an object');
  }
  const action = (input as Record<string, unknown>).action;
  if (typeof action !== 'string' || !Object.hasOwn(schema.commands, action)) {
    throw new Error('unknown command action');
  }
  return { action };
}

/** The 9-byte frame, big-endian: cmd_id u16, run_id u16, opcode u8, uptime_s u32. */
export function encodeCommand(
  schema: ProtocolSchema,
  action: string,
  cmdId: number,
  runId: number,
  uptimeS: number,
): Buffer {
  if (!Object.hasOwn(schema.commands, action)) {
    throw new Error('unknown command action');
  }
  const opcode = schema.commands[action];
  if (typeof opcode !== 'number') {
    throw new Error('unknown command action');
  }
  const frame = Buffer.alloc(9);
  frame.writeUInt16BE(cmdId & 0xffff, 0);
  frame.writeUInt16BE(runId & 0xffff, 2);
  frame.writeUInt8(opcode & 0xff, 4);
  frame.writeUInt32BE(uptimeS >>> 0, 5);
  return frame;
}

/** True when the car takes cmdId after lastId: at most 32767 ahead, mod 2^16. */
function isNewer(cmdId: number, lastId: number): boolean {
  const ahead = (cmdId - lastId) & 0xffff;
  return ahead !== 0 && ahead < 0x8000;
}

/**
 * The bridge is the one command sender, so it owns the numbering. The car
 * takes only an id newer than the last one it accepted, which the diag frame
 * reports; a counter the car would refuse jumps to the id after it. The run
 * ID and the uptime of the newest heartbeat go in every frame, so the car
 * refuses a frame that arrives late.
 */
export function createCommandSession(firstCmdId: number) {
  let runId: number | null = null;
  let uptimeS = 0;
  let seenMs = 0;
  let haveDiag = false;
  let next = firstCmdId & 0xffff;

  return {
    noteHeartbeat(values: Record<string, number>, atMs: number): void {
      if (values.run_id !== undefined && values.uptime_s !== undefined) {
        if (runId !== null && values.run_id !== runId) {
          haveDiag = false;
        }
        runId = values.run_id;
        uptimeS = values.uptime_s;
        seenMs = atMs;
      }
    },
    noteDiag(values: Record<string, number>): void {
      const last = values.cmd_last_id;
      if (last === undefined) {
        return;
      }
      haveDiag = true;
      if (!isNewer(next, last)) {
        next = (last + 1) & 0xffff;
      }
    },
    /** The frame for an action, or null without a fresh heartbeat and a diag frame. */
    frame(schema: ProtocolSchema, action: string, atMs: number): Buffer | null {
      if (runId === null || !haveDiag || atMs - seenMs > HEARTBEAT_FRESH_MS) {
        return null;
      }
      const frame = encodeCommand(schema, action, next, runId, uptimeS);
      next = (next + 1) & 0xffff;
      return frame;
    },
  };
}
