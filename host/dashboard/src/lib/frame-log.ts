import type { LinkPacket } from './link';
import { PACKETS } from './schema.generated';

export const FRAME_LOG_LIMIT = 5000;
export const FRAME_LOG_BLOCK_SIZE = 100;
const PAGE_SIZE = 8;

export type FrameLog = {
  readonly blocks: readonly (readonly LinkPacket[])[];
  readonly nextId: number;
  readonly revision: number;
};

const firstId = (log: FrameLog) => Math.max(0, log.nextId - FRAME_LOG_LIMIT);

function bundleFields(packet: LinkPacket, section: 'motion' | 'physics'): LinkPacket {
  const values: LinkPacket['values'] = {};
  for (const { name } of PACKETS[section].fields) {
    const key = name === 'heading_ddeg' ? 'physics_heading_ddeg' : name;
    const value = packet.values[key];
    if (value !== undefined) values[key] = value;
  }
  return { ...packet, packet: section, values };
}

export function readFramePage(log: FrameLog, cursor: number | null) {
  const first = firstId(log);
  const count = log.nextId - first;
  const requested = cursor !== null && cursor <= first ? first + PAGE_SIZE : cursor ?? log.nextId;
  const end = Math.min(log.nextId, requested);
  const start = Math.max(first, end - PAGE_SIZE);
  const firstBlock = Math.floor(first / FRAME_LOG_BLOCK_SIZE);
  const rows: { id: number; packet: LinkPacket }[] = [];

  for (let id = end - 1; id >= start; id -= 1) {
    const packet = log.blocks[Math.floor(id / FRAME_LOG_BLOCK_SIZE) - firstBlock]?.[id % FRAME_LOG_BLOCK_SIZE];
    if (packet !== undefined) rows.push({ id, packet });
  }
  return { rows, start, end, count, older: start > first ? start : null,
    newer: end + PAGE_SIZE < log.nextId ? end + PAGE_SIZE : null };
}

/** Lazily read an oldest-first [offsetStart, offsetEnd) window out of the retained frame blocks. */
export function readFrameSlice(log: FrameLog, offsetStart: number, offsetEnd: number, packetFilter?: string | null) {
  const first = firstId(log);
  const total = log.nextId - first;
  const firstBlock = Math.floor(first / FRAME_LOG_BLOCK_SIZE);

  if (!packetFilter) {
    const clampedStart = Math.max(0, Math.min(total, offsetStart));
    const clampedEnd = Math.max(clampedStart, Math.min(total, offsetEnd));
    const rows: { id: number; packet: LinkPacket }[] = [];
    for (let offset = clampedStart; offset < clampedEnd; offset += 1) {
      const id = first + offset;
      const packet = log.blocks[Math.floor(id / FRAME_LOG_BLOCK_SIZE) - firstBlock]?.[id % FRAME_LOG_BLOCK_SIZE];
      if (packet !== undefined) rows.push({ id, packet });
    }
    return { rows, count: total };
  }

  // A bundled receipt also appears under Motion and Physics. All and Status
  // keep the original frame with all fields and the same receipt ID.
  const section = packetFilter === 'motion' || packetFilter === 'physics' ? packetFilter : undefined;
  const matched: { id: number; packet: LinkPacket }[] = [];
  for (let offset = 0; offset < total; offset += 1) {
    const id = first + offset;
    const packet = log.blocks[Math.floor(id / FRAME_LOG_BLOCK_SIZE) - firstBlock]?.[id % FRAME_LOG_BLOCK_SIZE];
    if (packet !== undefined && (packet.packet === packetFilter ||
        (section !== undefined && packet.packet === 'status' && packet.values.device_sample_us !== undefined))) {
      matched.push({ id, packet });
    }
  }
  const count = matched.length;
  const clampedStart = Math.max(0, Math.min(count, offsetStart));
  const clampedEnd = Math.max(clampedStart, Math.min(count, offsetEnd));
  const rows = matched.slice(clampedStart, clampedEnd);
  return { rows: section === undefined ? rows : rows.map((row) => row.packet.packet === section
    ? row : { ...row, packet: bundleFields(row.packet, section) }), count };
}
