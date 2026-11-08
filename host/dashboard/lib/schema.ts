import { BIT_FIELDS, ENUMS, PACKETS } from './schema.generated';

export type PacketName = keyof typeof PACKETS;
export type EnumName = keyof typeof ENUMS;
export type BitFieldName = keyof typeof BIT_FIELDS;
export type StatusPacket = {
  speedMms: number;
  distMm: number;
  headingDeg: number;
  state: number;
  faults: number;
};
export type EventPacket = {
  eventId: number;
  eventType: number;
  p1: number;
  p2: number;
  reserved: number;
};
export type HeartbeatPacket = {
  uptimeS: number;
  drops: number;
  wifiRssi: number;
};

type AnyPacket = { wireSize: number; fields: readonly { name: string; offset: number }[] };

function layout(packet: PacketName): AnyPacket {
  return PACKETS[packet] as AnyPacket;
}

function offset(packet: PacketName, field: string): number {
  const found = layout(packet).fields.find((candidate) => candidate.name === field);
  if (!found) throw new Error(`missing ${packet}.${field}`);
  return found.offset;
}

function dataView(payload: Uint8Array, packet: PacketName): DataView {
  const expected = layout(packet).wireSize;
  if (payload.byteLength !== expected) {
    throw new RangeError(`${packet} packet is ${payload.byteLength} bytes, expected ${expected}`);
  }
  return new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
}

export function decodeStatus(payload: Uint8Array): StatusPacket {
  const view = dataView(payload, 'status');
  return {
    speedMms: view.getInt16(offset('status', 'speed_mms'), false),
    distMm: view.getUint16(offset('status', 'dist_mm'), false),
    headingDeg: view.getUint8(offset('status', 'heading_deg')),
    state: view.getUint8(offset('status', 'state')),
    faults: view.getUint16(offset('status', 'faults'), false),
  };
}

export function decodeEvent(payload: Uint8Array): EventPacket {
  const view = dataView(payload, 'event');
  return {
    eventId: view.getUint16(offset('event', 'event_id'), false),
    eventType: view.getUint8(offset('event', 'event_type')),
    p1: view.getInt16(offset('event', 'p1'), false),
    p2: view.getUint16(offset('event', 'p2'), false),
    reserved: view.getUint8(offset('event', 'reserved')),
  };
}

export function decodeHeartbeat(payload: Uint8Array): HeartbeatPacket {
  const view = dataView(payload, 'heartbeat');
  return {
    uptimeS: view.getUint16(offset('heartbeat', 'uptime_s'), false),
    drops: view.getUint8(offset('heartbeat', 'drops')),
    wifiRssi: view.getInt8(offset('heartbeat', 'wifi_rssi')),
  };
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
    .filter(([, bit]) => (mask & (1 << bit)) !== 0)
    .map(([label]) => labelName(label));
}

export function heartbeatAgeMs(nowMs: number, lastHeartbeatMs: number): number | null {
  if (!Number.isFinite(nowMs) || !Number.isFinite(lastHeartbeatMs)) return null;
  const age = nowMs - lastHeartbeatMs;
  return age >= 0 ? age : null;
}
