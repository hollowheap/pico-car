import { appendFileSync, mkdirSync, mkdtempSync, renameSync, rmSync, rmdirSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, describe, expect, it, vi } from "vitest";
import { createHistoryStore, type TelemetryReceipt } from "./history.js";

const dirs: string[] = [];

function store(maxRecords = 2) {
  const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
  dirs.push(dir);
  return createHistoryStore(join(dir, "history.jsonl"), maxRecords);
}

function record(packet: string, value: number): TelemetryReceipt {
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
  it("preserves arrival order within a millisecond across both rings and compaction", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const history = createHistoryStore(path, 2, 1);
    for (let t = 1; t < 6; t += 1) {
      history.append(record("status", t));
    }
    history.append(record("event", 6));
    history.append(record("status", 6)); // First status of the boot in the event.
    const packets = (store: ReturnType<typeof createHistoryStore>) => store.list(2).map((item) => item.packet);
    expect(packets(history)).toEqual(["status", "event", "status"]);
    expect(packets(createHistoryStore(path, 2, 1))).toEqual(["status", "event", "status"]);
  });

  it("evicts by arrival after compacting and reopening across a clock step", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const history = createHistoryStore(path, 2, 1);
    for (const time of [1, 2, 3, 4, 5, 100, 90]) {
      history.append(record("status", time));
    }

    const reopened = createHistoryStore(path, 2, 1);
    expect(reopened.list().map((item) => item.timestamp)).toEqual([90, 100]);
    reopened.append(record("status", 91));
    expect(reopened.list().map((item) => item.timestamp)).toEqual([91, 90]);
  });

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

  it("assigns distinct identities to identical receipts and retains them through restart", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const history = createHistoryStore(path, 2);
    const first = history.append(record("status", 7));
    const second = history.append(record("status", 7));
    expect(first.receiptId).toBeTruthy();
    expect(second.receiptId).not.toBe(first.receiptId);
    expect(history.list()).toEqual([second, first]);
    const reopened = createHistoryStore(path, 2);
    expect(reopened.list()).toEqual([second, first]);
    const third = reopened.append(record("status", 7));
    expect(createHistoryStore(path, 2).list()).toEqual([third, second]);
  });

  it("keeps receipt order through a clock step and a restart", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const history = createHistoryStore(path, 3);
    const first = history.append(record("status", 20000));
    const second = history.append(record("status", 19900));
    const restarted = createHistoryStore(path, 3);
    const third = restarted.append(record("status", 20200));

    expect(first.receiptOrder).toBeLessThan(second.receiptOrder);
    expect(second.receiptOrder).toBeLessThan(third.receiptOrder);
    expect(second.timestamp).toBe(19900);
    expect(second.timelineMs).toBeGreaterThanOrEqual(first.timelineMs);
    expect(third.timelineMs).toBeGreaterThan(second.timelineMs);
    expect(restarted.list()).toEqual([third, second, first]);
  });

  it("retains and later saves receipts accepted while storage is unavailable", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const history = createHistoryStore(path, 2);
    const first = history.append(record("status", 1));
    renameSync(path, path + ".saved");
    mkdirSync(path);

    vi.useFakeTimers({ toFake: ["performance"] });
    const error = vi.spyOn(console, "error").mockImplementation(() => {});
    try {
      const second = history.append(record("status", 2));
      expect(history.list()).toEqual([second, first]);
      expect(history.storageStatus()).toBe("degraded");

      const third = history.append(record("status", 3));
      expect(history.list()).toEqual([third, second]);
      rmdirSync(path);
      renameSync(path + ".saved", path);
      vi.advanceTimersByTime(1000);
      const event = history.append(record("event", 4));
      expect(history.storageStatus()).toBe("ready");
      expect(createHistoryStore(path, 2).list()).toEqual([event, third, second]);
      expect(error).toHaveBeenCalledTimes(1);
    } finally {
      error.mockRestore();
      vi.useRealTimers();
    }
  });

  it("repairs a torn last line before appending the next record", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    createHistoryStore(path, 10).append(record("s", 7));
    appendFileSync(path, '{"timestamp":8,"topic":"car/PI', "utf8");

    const restarted = createHistoryStore(path, 10);
    expect(restarted.list().map((item) => item.timestamp)).toEqual([7]);
    restarted.append(record("event", 9));
    expect(createHistoryStore(path, 10).list().map((item) => item.timestamp)).toEqual([9, 7]);
  });

  it("preserves a valid last record missing its trailing newline", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const path = join(dir, "history.jsonl");
    const previous = record("status", 7);
    const next = record("event", 8);
    appendFileSync(path, JSON.stringify(previous), "utf8");
    const restarted = createHistoryStore(path, 10);
    const migrated = restarted.list()[0];
    expect(migrated).toMatchObject(previous);
    expect(migrated?.receiptId).toBeTruthy();
    const appended = restarted.append(next);
    expect(createHistoryStore(path, 10).list()).toEqual([appended, migrated]);
  });

  it("sorts list() by receiptOrder/timelineMs across wall-clock rollbacks and guards NaN limit", () => {
    const dir = mkdtempSync(join(tmpdir(), "bridge-history-"));
    dirs.push(dir);
    const history = createHistoryStore(join(dir, "history.jsonl"), 10);
    const first = history.append(record("status", 5000));
    const rolledBack = history.append(record("event", 2000));
    const after = history.append(record("status", 2100));
    expect(history.list().map((item) => item.receiptId)).toEqual([
      after.receiptId,
      rolledBack.receiptId,
      first.receiptId,
    ]);
    expect(history.list(Number.NaN)).toHaveLength(3);
  });
});
