import { FRAME_LOG_LIMIT, FRAME_LOG_BLOCK_SIZE, type FrameLog } from './frame-log.js';
import type { ConsoleSnapshot, ConsoleStore, ConsoleView } from './view.js';

export type FrameLogUpdate = Omit<FrameLog, 'blocks'> & { from: number; blocks: FrameLog['blocks'] };
export type ConsoleUpdate =
  | { type: 'console'; data: ConsoleSnapshot; clockMs: number }
  | { type: 'consoleUpdate'; store: Partial<ConsoleStore>; removed: (keyof ConsoleStore)[]; view: Partial<ConsoleView>;
      lastPacket: ConsoleSnapshot['lastPacket']; frames?: FrameLogUpdate; clockMs: number };

const firstBlock = (log: FrameLog) => Math.floor(Math.max(0, log.nextId - FRAME_LOG_LIMIT) / FRAME_LOG_BLOCK_SIZE);

export function consoleUpdate(current: ConsoleSnapshot, previous?: ConsoleSnapshot, clockMs = Date.now()): ConsoleUpdate {
  if (!previous) return { type: 'console', data: current, clockMs };
  const view = Object.fromEntries(Object.entries(current.view).filter(([key, value]) =>
    value !== previous.view[key as keyof ConsoleView])) as Partial<ConsoleView>;
  const changed = (Object.keys(current.store) as (keyof ConsoleStore)[]).filter((key) => current.store[key] !== previous.store[key]);
  const store = Object.fromEntries(changed.map(key => [key, current.store[key]])) as Partial<ConsoleStore>;
  const removed = changed.filter(key => current.store[key] === undefined);
  const { frameLog: next } = current;
  let frames: FrameLogUpdate | undefined;
  if (next !== previous.frameLog) {
    const first = firstBlock(next);
    const before = firstBlock(previous.frameLog);
    let unchanged = 0;
    if (next.revision === previous.frameLog.revision) {
      while (unchanged < next.blocks.length && next.blocks[unchanged] === previous.frameLog.blocks[first + unchanged - before]) unchanged++;
    }
    frames = { revision: next.revision, nextId: next.nextId, from: first + unchanged, blocks: next.blocks.slice(unchanged) };
  }
  return { type: 'consoleUpdate', store, removed, view, lastPacket: current.lastPacket, frames, clockMs };
}
