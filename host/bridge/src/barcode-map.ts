import { existsSync, readFileSync, renameSync, writeFileSync } from "node:fs";
import { COMMANDS, ENUMS, PACKETS } from "./console/schema.generated.js";

const ACTIONS = ["stop", "left", "right", "straight", "uturn"] as const;
export type BarcodeAction = typeof ACTIONS[number];
export type BarcodeMap = Record<string, BarcodeAction>;

export const DEFAULT_BARCODE_MAP: BarcodeMap = {
  H: "stop", L: "left", R: "right", S: "straight", U: "uturn", Z: "stop",
};

export function parseBarcodeMap(value: unknown): BarcodeMap {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Expected a letter-to-command map");
  const result: BarcodeMap = {};
  for (const [letter, action] of Object.entries(value)) {
    if (!/^[A-Z]$/.test(letter) || !ACTIONS.includes(action as BarcodeAction)) {
      throw new Error("Use uppercase A–Z with Stop, Left, Right, Straight or U-turn");
    }
    result[letter] = action as BarcodeAction;
  }
  return result;
}

export function loadBarcodeMap(path: string): BarcodeMap | null {
  return existsSync(path) ? parseBarcodeMap(JSON.parse(readFileSync(path, "utf8"))) : null;
}

export function saveBarcodeMap(path: string, map: BarcodeMap): void {
  const pending = `${path}.tmp`;
  writeFileSync(pending, `${JSON.stringify(map)}\n`, { mode: 0o600 });
  renameSync(pending, path);
}

export function barcodeMapBytes(map: BarcodeMap): Uint8Array {
  const length = PACKETS.barcode_map.wireSize - PACKETS.command.wireSize;
  const bytes = new Uint8Array(length).fill(ENUMS.commandConfig.unmapped);
  for (const [letter, action] of Object.entries(map)) {
    bytes[letter.charCodeAt(0) - 'A'.charCodeAt(0)] = COMMANDS[action];
  }
  return bytes;
}
