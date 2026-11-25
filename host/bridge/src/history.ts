import { existsSync, readFileSync, writeFileSync } from "node:fs";

export type HistoryRecord = {
  timestamp: number;
  topic: string;
  packet: string;
  values: Record<string, number>;
};

export type HistoryStore = {
  append: (record: HistoryRecord) => void;
  list: (limit?: number) => HistoryRecord[];
};

function load(path: string): HistoryRecord[] {
  if (!existsSync(path)) {
    return [];
  }
  return readFileSync(path, "utf8")
    .split("\n")
    .filter((line) => line.length > 0)
    .map((line) => JSON.parse(line) as HistoryRecord);
}

function save(path: string, records: HistoryRecord[]): void {
  writeFileSync(
    path,
    records.map((record) => JSON.stringify(record)).join("\n") + "\n",
    "utf8",
  );
}

export function createHistoryStore(path: string, maxRecords: number): HistoryStore {
  if (maxRecords <= 0) {
    throw new RangeError("maxRecords must be positive");
  }

  let records = load(path).slice(-maxRecords);

  return {
    append(record: HistoryRecord): void {
      records.push(record);
      if (records.length > maxRecords) {
        records = records.slice(-maxRecords);
      }
      save(path, records);
    },
    list(limit = 100): HistoryRecord[] {
      if (limit <= 0) {
        return [];
      }
      return records.slice(-limit).reverse();
    },
  };
}
