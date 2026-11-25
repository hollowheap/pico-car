import { appendFileSync, mkdtempSync, rmSync } from "node:fs";
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
    topic: `car/PICO-CAR/${packet}`,
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

  it("keeps events in their own ring, so frames never flood them out", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const history = createHistoryStore(join(dir, "history.jsonl"), 3, 2);
    history.append(record("event", 1));
    for (let t = 2; t < 50; t += 1) {
      history.append(record("status", t));
    }
    history.append(record("event", 50));

    const all = history.list(3);
    expect(all.filter((item) => item.packet === "event").map((item) => item.timestamp)).toEqual([50, 1]);
    expect(all.filter((item) => item.packet === "status").map((item) => item.timestamp)).toEqual([49, 48, 47]);
    expect(all.map((item) => item.timestamp)).toEqual([50, 49, 48, 47, 1]);
  });

  it("drops the oldest event once the event ring is full", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const history = createHistoryStore(join(dir, "history.jsonl"), 3, 2);
    history.append(record("event", 1));
    history.append(record("event", 2));
    history.append(record("event", 3));

    expect(history.list(10).map((item) => item.timestamp)).toEqual([3, 2]);
  });

  it("appends to the file and reloads both rings after many writes", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const history = createHistoryStore(path, 3, 2);
    for (let t = 1; t <= 40; t += 1) {
      history.append(record(t % 10 === 0 ? "event" : "status", t));
    }

    const again = createHistoryStore(path, 3, 2).list(3).map((item) => item.timestamp);
    expect(again).toEqual([40, 39, 38, 37, 30]);
  });

  it("reloads persisted records", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    createHistoryStore(path, 10).append(record("s", 7));

    expect(createHistoryStore(path, 10).list(1)[0].timestamp).toBe(7);
  });

  it("skips a torn last line from a power loss instead of failing to start", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    createHistoryStore(path, 10).append(record("s", 7));
    appendFileSync(path, '{"timestamp":8,"topic":"car/PI', "utf8");

    expect(createHistoryStore(path, 10).list().map((item) => item.timestamp)).toEqual([7]);
  });
});
