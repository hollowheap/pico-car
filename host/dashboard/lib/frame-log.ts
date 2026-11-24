import type { LinkPacket } from './link';

export const FRAME_LOG_LIMIT = 5000;
const BLOCK_SIZE = 100;
const PAGE_SIZE = 8;

export type FrameLog = {
  readonly blocks: readonly (readonly LinkPacket[])[];
  readonly nextId: number;
  readonly revision: number;
};

const firstId = (log: FrameLog) => Math.max(0, log.nextId - FRAME_LOG_LIMIT);

export function createFrameLog(packets: readonly LinkPacket[] = [], revision = 0): FrameLog {
  const retained = packets.slice(-FRAME_LOG_LIMIT);
  const blocks: LinkPacket[][] = [];
  for (let start = 0; start < retained.length; start += BLOCK_SIZE) {
    blocks.push(retained.slice(start, start + BLOCK_SIZE));
  }
  return { blocks, nextId: retained.length, revision };
}

export function appendFrame(log: FrameLog, packet: LinkPacket): FrameLog {
  // Keep old pages stable without copying the full frame history on every packet.
  const blocks = [...log.blocks];
  const tail = blocks.at(-1);
  if (tail === undefined || tail.length === BLOCK_SIZE) blocks.push([packet]);
  else blocks[blocks.length - 1] = [...tail, packet];
  const next = { ...log, blocks, nextId: log.nextId + 1 };
  if (Math.floor(firstId(next) / BLOCK_SIZE) > Math.floor(firstId(log) / BLOCK_SIZE)) blocks.shift();
  return next;
}

export function readFramePage(log: FrameLog, cursor: number | null) {
  const first = firstId(log);
  const count = log.nextId - first;
  const requested = cursor !== null && cursor <= first ? first + PAGE_SIZE : cursor ?? log.nextId;
  const end = Math.min(log.nextId, requested);
  const start = Math.max(first, end - PAGE_SIZE);
  const firstBlock = Math.floor(first / BLOCK_SIZE);
  const rows: { id: number; packet: LinkPacket }[] = [];

  for (let id = end - 1; id >= start; id -= 1) {
    const packet = log.blocks[Math.floor(id / BLOCK_SIZE) - firstBlock]?.[id % BLOCK_SIZE];
    if (packet !== undefined) rows.push({ id, packet });
  }
  return { rows, start, end, count, older: start > first ? start : null,
    newer: end + PAGE_SIZE < log.nextId ? end + PAGE_SIZE : null };
}

/** Read up to `limit` newest frames, newest first, for scrollable inspection. */
export function readRecentFrames(log: FrameLog, limit = 200) {
  const first = firstId(log);
  const count = log.nextId - first;
  const start = Math.max(first, log.nextId - limit);
  const firstBlock = Math.floor(first / BLOCK_SIZE);
  const rows: { id: number; packet: LinkPacket }[] = [];
  for (let id = log.nextId - 1; id >= start; id -= 1) {
    const packet = log.blocks[Math.floor(id / BLOCK_SIZE) - firstBlock]?.[id % BLOCK_SIZE];
    if (packet !== undefined) rows.push({ id, packet });
  }
  return { rows, count };
}

/** Lazily read an oldest-first [offsetStart, offsetEnd) window out of the retained frame blocks. */
export function readFrameSlice(log: FrameLog, offsetStart: number, offsetEnd: number, packetFilter?: string | null) {
  const first = firstId(log);
  const total = log.nextId - first;
  const firstBlock = Math.floor(first / BLOCK_SIZE);

  if (!packetFilter) {
    const clampedStart = Math.max(0, Math.min(total, offsetStart));
    const clampedEnd = Math.max(clampedStart, Math.min(total, offsetEnd));
    const rows: { id: number; packet: LinkPacket }[] = [];
    for (let offset = clampedStart; offset < clampedEnd; offset += 1) {
      const id = first + offset;
      const packet = log.blocks[Math.floor(id / BLOCK_SIZE) - firstBlock]?.[id % BLOCK_SIZE];
      if (packet !== undefined) rows.push({ id, packet });
    }
    return { rows, count: total };
  }

  const matched: { id: number; packet: LinkPacket }[] = [];
  for (let offset = 0; offset < total; offset += 1) {
    const id = first + offset;
    const packet = log.blocks[Math.floor(id / BLOCK_SIZE) - firstBlock]?.[id % BLOCK_SIZE];
    if (packet !== undefined && packet.packet === packetFilter) matched.push({ id, packet });
  }
  const count = matched.length;
  const clampedStart = Math.max(0, Math.min(count, offsetStart));
  const clampedEnd = Math.max(clampedStart, Math.min(count, offsetEnd));
  return { rows: matched.slice(clampedStart, clampedEnd), count };
}
