import { readFileSync, writeFileSync } from "node:fs";

/** Three missed 1 Hz heartbeats after the last one from a different car. */
export const CAR_MISMATCH_HOLD_MS = 3000;

export type CarPinState = {
  expected: string | null;
  reporting: string | null;
  mismatch: boolean;
  rejected: number;
};

export function formatCarId(id: number): string {
  return (id >>> 0).toString(16).padStart(8, "0");
}

/** Null for empty text; the car ID is the last four MAC bytes as eight hex digits. */
export function parseCarId(text: string): number | null {
  const trimmed = text.trim();
  if (trimmed === "") return null;
  if (!/^[0-9a-fA-F]{8}$/.test(trimmed)) throw new Error("CAR_ID must be eight hex digits");
  return Number.parseInt(trimmed, 16);
}

export function loadCarId(env: NodeJS.ProcessEnv, path: string): number | null {
  if (env.CAR_ID !== undefined) return parseCarId(env.CAR_ID);
  try {
    return parseCarId(readFileSync(path, "utf8"));
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code === "ENOENT") return null;
    throw error;
  }
}

export function rememberCarId(path: string, id: number): void {
  writeFileSync(path, `${formatCarId(id)}\n`, { mode: 0o600 });
}

type Packet = { name: string; values: Record<string, number> };

/**
 * Pins the car whose heartbeat carries the expected car_id. Only heartbeats
 * name the car, so a heartbeat from another car blocks every packet and
 * command until it has been quiet for CAR_MISMATCH_HOLD_MS.
 */
export function createCarPin(initial: number | null, learn: (id: number) => void) {
  let expected = initial;
  let reporting: number | null = null;
  let mismatchAt: number | null = null;
  let rejected = 0;

  const blocked = (atMs: number): boolean => mismatchAt !== null && atMs - mismatchAt <= CAR_MISMATCH_HOLD_MS;

  return {
    blocked,
    admit(packet: Packet, atMs: number): boolean {
      const carId = packet.name === "heartbeat" ? packet.values.car_id : undefined;
      if (carId !== undefined) {
        if (expected === null) {
          expected = carId;
          learn(carId);
        }
        reporting = carId;
        if (carId !== expected) mismatchAt = atMs;
      }
      const admitted = !blocked(atMs);
      if (!admitted) rejected += 1;
      return admitted;
    },
    state(atMs: number): CarPinState {
      return {
        expected: expected === null ? null : formatCarId(expected),
        reporting: reporting === null ? null : formatCarId(reporting),
        mismatch: blocked(atMs),
        rejected,
      };
    },
  };
}
