import { randomUUID } from "node:crypto";
import { appendFileSync, existsSync, readFileSync, renameSync, writeFileSync } from "node:fs";

export type TelemetryReceipt = {
  timestamp: number;
  topic: string;
  packet: string;
  values: Record<string, number>;
};

export type HistoryRecord = TelemetryReceipt & {
  receiptId: string;
  receiptOrder: number;
  timelineMs: number;
};

export type HistoryStore = {
  append: (record: TelemetryReceipt) => HistoryRecord;
  /** The newest `limit` frames and every kept event, newest first. */
  list: (limit?: number) => HistoryRecord[];
  storageStatus: () => "ready" | "degraded";
};

/** Events are rare but needed for the whole run; frames come at about 19 per second. */
const EVENT_PACKET = "event";

/** Events kept when no limit is given: far more than one run sends. */
export const DEFAULT_MAX_EVENTS = 2000;
const STORAGE_RETRY_MS = 1000;
// Leave room for receipts in the same millisecond without allowing the cursor
// to restart from zero if a bridge's history file is replaced.
const ORDERS_PER_MS = 1000;

function load(path: string): { records: HistoryRecord[]; needsRewrite: boolean } {
  if (!existsSync(path)) {
    return { records: [], needsRewrite: false };
  }
  const contents = readFileSync(path, "utf8");
  let needsRewrite = contents.length > 0 && !contents.endsWith("\n");
  // A power loss can tear the last line; skip it so the bridge still starts.
  const records: HistoryRecord[] = [];
  for (const line of contents.split("\n")) {
    if (line.length === 0) continue;
    try {
      const input = JSON.parse(line) as Partial<HistoryRecord> & TelemetryReceipt;
      if (!Number.isFinite(input?.timestamp) || typeof input.packet !== "string" ||
          typeof input.topic !== "string" || !input.values || typeof input.values !== "object") {
        needsRewrite = true;
        continue;
      }
      const previous = records.at(-1);
      const order = previous?.receiptOrder ?? 0;
      const time = Math.max(previous?.timelineMs ?? -Infinity, input.timestamp);
      const receiptId = typeof input.receiptId === "string" && input.receiptId ? input.receiptId : randomUUID();
      const receiptOrder = input.receiptOrder !== undefined && Number.isSafeInteger(input.receiptOrder) &&
        input.receiptOrder > order ? input.receiptOrder : Math.max(order + 1, input.timestamp * ORDERS_PER_MS);
      const timelineMs = input.timelineMs !== undefined && Number.isFinite(input.timelineMs)
        ? Math.max(time, input.timelineMs) : time;
      needsRewrite ||= receiptId !== input.receiptId || receiptOrder !== input.receiptOrder ||
        timelineMs !== input.timelineMs;
      records.push({ ...input, receiptId, receiptOrder, timelineMs });
    } catch {
      needsRewrite = true;
    }
  }
  return { records, needsRewrite };
}

function save(path: string, records: HistoryRecord[]): void {
  // Write a copy, then rename it, so a crash never leaves a cut-down file.
  const next = `${path}.tmp`;
  writeFileSync(
    next,
    records.map((record) => JSON.stringify(record)).join("\n") + (records.length > 0 ? "\n" : ""),
    "utf8",
  );
  renameSync(next, path);
}

function byTime(a: HistoryRecord, b: HistoryRecord): number {
  return a.timelineMs - b.timelineMs || a.receiptOrder - b.receiptOrder;
}

/**
 * Frames and events are kept in two rings: a flood of frames never pushes
 * the events of a run out. Each record is appended to the file, which is
 * rewritten for compaction or to repair an unterminated line at startup.
 */
export function createHistoryStore(
  path: string,
  maxRecords: number,
  maxEvents: number = DEFAULT_MAX_EVENTS,
): HistoryStore {
  if (maxRecords <= 0 || maxEvents <= 0) {
    throw new RangeError("maxRecords and maxEvents must be positive");
  }

  const { records, needsRewrite } = load(path);
  let lastOrder = records.at(-1)?.receiptOrder ?? 0;
  let lastTimeline = records.at(-1)?.timelineMs ?? -Infinity;
  let lastMonotonic = performance.now();
  let rollback: { timeline: number; monotonic: number } | undefined;
  let frames = records.filter(record => record.packet !== EVENT_PACKET).slice(-maxRecords);
  let events = records.filter(record => record.packet === EVENT_PACKET).slice(-maxEvents);
  let lines = records.length;
  let retryAt: number | undefined;

  const compact = (): void => {
    const kept = [...frames, ...events].sort((a, b) => a.receiptOrder - b.receiptOrder);
    save(path, kept);
    lines = kept.length;
  };

  const persist = (write: () => void): void => {
    try {
      write();
      retryAt = undefined;
    } catch (error) {
      if (retryAt === undefined) console.error("History recording failed:", error);
      retryAt = performance.now() + STORAGE_RETRY_MS;
    }
  };

  if (needsRewrite || lines > frames.length + events.length) persist(compact);

  return {
    append(receipt): HistoryRecord {
      const now = performance.now();
      if (receipt.timestamp < lastTimeline && rollback === undefined) {
        rollback = { timeline: lastTimeline, monotonic: lastMonotonic };
      }
      const elapsedTime = rollback === undefined ? receipt.timestamp :
        rollback.timeline + Math.floor(now - rollback.monotonic);
      if (receipt.timestamp >= elapsedTime) rollback = undefined;
      lastTimeline = Math.max(lastTimeline, receipt.timestamp, elapsedTime);
      lastMonotonic = now;
      lastOrder = Math.max(lastOrder + 1, receipt.timestamp * ORDERS_PER_MS);
      const record = { ...receipt, receiptId: randomUUID(), receiptOrder: lastOrder, timelineMs: lastTimeline };
      if (record.packet === EVENT_PACKET) {
        events.push(record);
        if (events.length > maxEvents) {
          events = events.slice(-maxEvents);
        }
      } else {
        frames.push(record);
        if (frames.length > maxRecords) {
          frames = frames.slice(-maxRecords);
        }
      }
      if (retryAt !== undefined) {
        if (now >= retryAt) persist(compact);
      } else if (lines >= 2 * (maxRecords + maxEvents)) {
        persist(compact);
      } else {
        persist(() => {
          appendFileSync(path, JSON.stringify(record) + "\n", "utf8");
          lines += 1;
        });
      }
      return record;
    },
    list(limit = 100): HistoryRecord[] {
      const count = Number.isFinite(limit) ? limit : 100;
      if (count <= 0) {
        return [];
      }
      return [...frames.slice(-count), ...events].sort((a, b) => byTime(b, a));
    },
    storageStatus: () => retryAt === undefined ? "ready" : "degraded",
  };
}
