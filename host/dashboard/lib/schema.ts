import { BIT_FIELDS, ENUMS, PACKETS } from './schema.generated';

export type PacketName = keyof typeof PACKETS;
export type EnumName = keyof typeof ENUMS;
export type BitFieldName = keyof typeof BIT_FIELDS;
type FieldType = 'int8' | 'uint8' | 'int16' | 'uint16' | 'int32' | 'uint32';
type AnyField = { name: string; offset: number; type: FieldType; count?: number };
type AnyPacket = { wireSize: number; fields: readonly AnyField[] };

const FIELD_SIZE: Record<FieldType, number> = {
  int8: 1,
  uint8: 1,
  int16: 2,
  uint16: 2,
  int32: 4,
  uint32: 4,
};

function readField(view: DataView, type: FieldType, offset: number): number {
  switch (type) {
    case 'int8':
      return view.getInt8(offset);
    case 'uint8':
      return view.getUint8(offset);
    case 'int16':
      return view.getInt16(offset, false);
    case 'uint16':
      return view.getUint16(offset, false);
    case 'int32':
      return view.getInt32(offset, false);
    case 'uint32':
      return view.getUint32(offset, false);
  }
}

/**
 * Decode one packet to the flat shape the bridge sends. An array field
 * `slots` with count N becomes slots_0 .. slots_<N-1>.
 */
export function decodePacket(name: PacketName, payload: Uint8Array): Record<string, number> {
  const packet = PACKETS[name] as AnyPacket;
  if (payload.byteLength !== packet.wireSize) {
    throw new RangeError(`${name} packet is ${payload.byteLength} bytes, expected ${packet.wireSize}`);
  }
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const values: Record<string, number> = {};
  for (const field of packet.fields) {
    if (field.count === undefined) {
      values[field.name] = readField(view, field.type, field.offset);
      continue;
    }
    for (let index = 0; index < field.count; index += 1) {
      values[`${field.name}_${index}`] = readField(
        view,
        field.type,
        field.offset + index * FIELD_SIZE[field.type],
      );
    }
  }
  return values;
}

/** heading_ddeg value that means the heading is not valid yet. */
const HEADING_INVALID = 0xffff;

/** Convert heading_ddeg (0.1 degree) to degrees, or null when invalid. */
export function headingDegrees(ddeg: number | undefined): number | null {
  return ddeg === undefined || ddeg === HEADING_INVALID ? null : ddeg / 10;
}

function labelName(name: string): string {
  return name
    .replace(/([a-z0-9])([A-Z])/g, '$1 $2')
    .toUpperCase();
}

export function enumLabel(name: EnumName, value: number | undefined): string {
  if (value === undefined) {
    return '--';
  }
  const found = Object.entries(ENUMS[name]).find(([, code]) => code === value);
  if (found === undefined) {
    return `UNKNOWN ${value}`;
  }
  return labelName(found[0]);
}

export function bitLabels(
  name: BitFieldName,
  mask: number | undefined,
): string[] {
  if (mask === undefined || mask === 0) {
    return [];
  }
  return Object.entries(BIT_FIELDS[name])
    .filter(([label, bit]) => !/retired/i.test(label) && (mask & (1 << bit)) !== 0)
    .map(([label]) => labelName(label));
}

export function heartbeatAgeMs(nowMs: number, lastHeartbeatMs: number): number | null {
  if (!Number.isFinite(nowMs) || !Number.isFinite(lastHeartbeatMs)) return null;
  const age = nowMs - lastHeartbeatMs;
  return age >= 0 ? age : null;
}
