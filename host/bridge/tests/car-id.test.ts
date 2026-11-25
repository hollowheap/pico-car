import { mkdtempSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import { CAR_MISMATCH_HOLD_MS, createCarPin, formatCarId, loadCarId, parseCarId, rememberCarId } from "../src/car-id.js";

const CAR = 0xc10abcf0;
const OTHER = 0xc10a0001;
const dirs: string[] = [];

afterEach(() => {
  for (const dir of dirs.splice(0)) rmSync(dir, { recursive: true, force: true });
});

function scratch(): string {
  const dir = mkdtempSync(join(tmpdir(), "bridge-car-id-"));
  dirs.push(dir);
  return dir;
}

describe("car id text", () => {
  it("round-trips eight hex digits", () => {
    expect(formatCarId(CAR)).toBe("c10abcf0");
    expect(formatCarId(0x1)).toBe("00000001");
    expect(parseCarId("C10ABCF0\n")).toBe(CAR);
    expect(parseCarId("")).toBeNull();
  });

  it("rejects text that is not eight hex digits", () => {
    expect(() => parseCarId("c10abc")).toThrow("eight hex digits");
    expect(() => parseCarId("c10abcf0ff")).toThrow("eight hex digits");
    expect(() => parseCarId("zzzzzzzz")).toThrow("eight hex digits");
  });

  it("prefers CAR_ID over the saved file and treats a missing file as unpinned", () => {
    const path = join(scratch(), "car-id");
    expect(loadCarId({}, path)).toBeNull();
    rememberCarId(path, CAR);
    expect(readFileSync(path, "utf8")).toBe("c10abcf0\n");
    expect(loadCarId({}, path)).toBe(CAR);
    expect(loadCarId({ CAR_ID: "00000007" }, path)).toBe(7);
  });
});

describe("car pin", () => {
  const heartbeat = (carId: number) => ({ name: "heartbeat", values: { car_id: carId } });
  const status = { name: "status", values: { speed_mms: 0 } };

  it("accepts the pinned car and refuses another one", () => {
    const pin = createCarPin(CAR, () => undefined);
    expect(pin.admit(heartbeat(CAR), 0)).toBe(true);
    expect(pin.admit(status, 100)).toBe(true);
    expect(pin.admit(heartbeat(OTHER), 1000)).toBe(false);
    expect(pin.state(1000)).toEqual({ expected: "c10abcf0", reporting: "c10a0001", mismatch: true, rejected: 1 });
  });

  it("drops all telemetry and refuses commands until the other car is quiet", () => {
    const pin = createCarPin(CAR, () => undefined);
    expect(pin.admit(heartbeat(OTHER), 1000)).toBe(false);
    expect(pin.blocked(1000)).toBe(true);
    expect(pin.admit(status, 1500)).toBe(false);
    expect(pin.admit(heartbeat(CAR), 2000)).toBe(false);
    expect(pin.blocked(1000 + CAR_MISMATCH_HOLD_MS)).toBe(true);
    expect(pin.admit(status, 1001 + CAR_MISMATCH_HOLD_MS)).toBe(true);
    expect(pin.blocked(1001 + CAR_MISMATCH_HOLD_MS)).toBe(false);
    expect(pin.state(1001 + CAR_MISMATCH_HOLD_MS).mismatch).toBe(false);
  });

  it("learns the first car it hears and keeps it", () => {
    const learned: number[] = [];
    const pin = createCarPin(null, (id) => learned.push(id));
    expect(pin.state(0)).toEqual({ expected: null, reporting: null, mismatch: false, rejected: 0 });
    expect(pin.admit(status, 0)).toBe(true);
    expect(pin.admit(heartbeat(CAR), 10)).toBe(true);
    expect(pin.admit(heartbeat(CAR), 1010)).toBe(true);
    expect(pin.admit(heartbeat(OTHER), 2010)).toBe(false);
    expect(learned).toEqual([CAR]);
    expect(pin.state(2010).expected).toBe("c10abcf0");
  });

  it("ignores a heartbeat without a car id", () => {
    const pin = createCarPin(CAR, () => undefined);
    expect(pin.admit({ name: "heartbeat", values: { uptime_s: 1 } }, 0)).toBe(true);
  });
});
