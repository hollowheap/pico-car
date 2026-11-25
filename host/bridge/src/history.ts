import { appendFileSync, existsSync, readFileSync, renameSync, writeFileSync } from "node:fs";

export type HistoryRecord = {
  timestamp: number;
  topic: string;
  packet: string;
  values: Record<string, number>;
};

export type HistoryStore = {
  append: (record: HistoryRecord) => void;
  /** The newest `limit` frames and every kept event, newest first. */
  list: (limit?: number) => HistoryRecord[];
};

/** Events are rare but needed for the whole run; frames come at about 19 per second. */
const EVENT_PACKET = "event";

/** Events kept when no limit is given: far more than one run sends. */
export const DEFAULT_MAX_EVENTS = 2000;

function load(path: string): HistoryRecord[] {
  if (!existsSync(path)) {
    return [];
  }
  // A power loss can tear the last line; skip it so the bridge still starts.
  return readFileSync(path, "utf8")
    .split("\n")
    .flatMap((line) => {
      try {
        return line.length > 0 ? [JSON.parse(line) as HistoryRecord] : [];
      } catch {
        return [];
      }
    });
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
  return a.timestamp - b.timestamp;
}

/**
 * Frames and events are kept in two rings: a flood of frames never pushes
 * the events of a run out. Each record is appended to the file; the file is
 * rewritten only when it holds twice the kept records.
 */
export function createHistoryStore(
  path: string,
  maxRecords: number,
  maxEvents: number = DEFAULT_MAX_EVENTS,
): HistoryStore {
  if (maxRecords <= 0 || maxEvents <= 0) {
    throw new RangeError("maxRecords and maxEvents must be positive");
  }

  const loaded = load(path);
  let frames = loaded.filter((record) => record.packet !== EVENT_PACKET).slice(-maxRecords);
  let events = loaded.filter((record) => record.packet === EVENT_PACKET).slice(-maxEvents);
  let lines = loaded.length;

  const compact = (): void => {
    const kept = [...frames, ...events].sort(byTime);
    save(path, kept);
    lines = kept.length;
  };

  if (lines > frames.length + events.length) {
    compact();
  }

  return {
    append(record: HistoryRecord): void {
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
      appendFileSync(path, JSON.stringify(record) + "\n", "utf8");
      lines += 1;
      if (lines > 2 * (maxRecords + maxEvents)) {
        compact();
      }
    },
    list(limit = 100): HistoryRecord[] {
      if (limit <= 0) {
        return [];
      }
      return [...frames.slice(-limit), ...events].sort(byTime).reverse();
    },
  };
}
