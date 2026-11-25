import type { LinkPacket } from './link.js';

export const FRAME_LOG_LIMIT = 5000;
export const FRAME_LOG_BLOCK_SIZE = 100;

export type FrameLog = {
  readonly blocks: readonly (readonly LinkPacket[])[];
  readonly nextId: number;
  readonly revision: number;
};

const firstId = (log: FrameLog) => Math.max(0, log.nextId - FRAME_LOG_LIMIT);

export function createFrameLog(packets: readonly LinkPacket[] = [], revision = 0): FrameLog {
  const retained = packets.slice(-FRAME_LOG_LIMIT);
  const blocks: LinkPacket[][] = [];
  for (let start = 0; start < retained.length; start += FRAME_LOG_BLOCK_SIZE) {
    blocks.push(retained.slice(start, start + FRAME_LOG_BLOCK_SIZE));
  }
  return { blocks, nextId: retained.length, revision };
}

export function appendFrame(log: FrameLog, packet: LinkPacket): FrameLog {
  // Keep old pages stable without copying the full frame history on every packet.
  const blocks = [...log.blocks];
  const tail = blocks.at(-1);
  if (tail === undefined || tail.length === FRAME_LOG_BLOCK_SIZE) blocks.push([packet]);
  else blocks[blocks.length - 1] = [...tail, packet];
  const next = { ...log, blocks, nextId: log.nextId + 1 };
  if (Math.floor(firstId(next) / FRAME_LOG_BLOCK_SIZE) > Math.floor(firstId(log) / FRAME_LOG_BLOCK_SIZE)) blocks.shift();
  return next;
}
