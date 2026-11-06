import { mkdtempSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import { createHistoryStore, type HistoryRecord } from "./history.js";

const dirs: string[] = [];

function store(maxRecords = 2) {
  const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
  dirs.push(dir);
  return createHistoryStore(join(dir, "history.jsonl"), maxRecords);
}

function record(packet: string, value: number): HistoryRecord {
  return {
    timestamp: value,
    topic: `car/SE01/${packet}`,
    packet,
    values: { value },
  };
}

afterEach(() => {
  for (const dir of dirs.splice(0)) {
    rmSync(dir, { recursive: true, force: true });
  }
});

describe("history store", () => {
  it("returns newest first and honors the limit", () => {
    const history = store();
    history.append(record("s", 1));
    history.append(record("e", 2));
    history.append(record("h", 3));

    expect(history.list()).toHaveLength(2);
    expect(history.list().map((item) => item.timestamp)).toEqual([3, 2]);
    expect(history.list(1).map((item) => item.timestamp)).toEqual([3]);
  });

  it("reloads persisted records", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    createHistoryStore(path, 10).append(record("s", 7));

    expect(createHistoryStore(path, 10).list(1)[0].timestamp).toBe(7);
  });
});
