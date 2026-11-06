import type { ProtocolSchema } from './schema.js';

export type CommandRequest = {
  action: string;
  cmdId: number;
  runId: number;
  sentMs: number;
};

function isU16(value: unknown): value is number {
  return Number.isInteger(value) && (value as number) >= 0 && (value as number) <= 0xffff;
}

export function parseCommandRequest(
  input: unknown,
  schema: ProtocolSchema,
): CommandRequest {
  if (typeof input !== 'object' || input === null) {
    throw new Error('command body must be an object');
  }
  const body = input as Record<string, unknown>;
  const action = body.action;
  if (typeof action !== 'string' || !(action in schema.commands)) {
    throw new Error('unknown command action');
  }
  if (!isU16(body.cmdId) || !isU16(body.runId) || !isU16(body.sentMs)) {
    throw new Error('cmdId, runId, and sentMs must be uint16');
  }
  return {
    action,
    cmdId: body.cmdId,
    runId: body.runId,
    sentMs: body.sentMs,
  };
}

export function encodeCommand(
  schema: ProtocolSchema,
  request: CommandRequest,
): Buffer {
  const opcode = schema.commands[request.action];
  if (opcode === undefined) {
    throw new Error('unknown command action');
  }
  const frame = Buffer.alloc(8);
  frame.writeUInt16BE(request.cmdId, 0);
  frame.writeUInt16BE(request.runId, 2);
  frame.writeUInt16BE(request.sentMs, 4);
  frame.writeUInt8(opcode & 0xff, 6);
  frame.writeUInt8(0, 7);
  return frame;
}
