import type { LinkPacket } from '../../lib/link';

export const MISSION_STATES = ['INIT', 'READY', 'RUNNING', 'STOPPED'] as const;

export function missionName(state: number | undefined): string {
  if (state === undefined || state < 0 || state >= MISSION_STATES.length) {
    return '--';
  }
  return MISSION_STATES[state] ?? '--';
}

export function faultHex(mask: number | undefined): string {
  if (mask === undefined) {
    return '--';
  }
  return `0x${mask.toString(16).padStart(4, '0').toUpperCase()}`;
}

export function formatAge(ageMs: number | null): string {
  if (ageMs === null) {
    return '--';
  }
  if (ageMs < 1000) {
    return `${Math.max(0, Math.round(ageMs))} ms`;
  }
  return `${(ageMs / 1000).toFixed(1)} s`;
}

export function bearingName(headingDeg: number): string {
  const names = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  const index = Math.round(((headingDeg % 360) + 360) % 360 / 45) % 8;
  return names[index] ?? 'N';
}

export function packetSummary(packet: LinkPacket): string {
  const values = packet.values;
  if (packet.packet === 'status') {
    return `speed ${values.speed_mms ?? '--'} mm/s  dist ${values.dist_mm ?? '--'} mm  hdg ${values.heading_deg ?? '--'} deg`;
  }
  if (packet.packet === 'event') {
    return `id ${values.event_id ?? '--'}  type ${values.event_type ?? '--'}  p1 ${values.p1 ?? '--'}  p2 ${values.p2 ?? '--'}`;
  }
  if (packet.packet === 'heartbeat') {
    return `up ${values.uptime_s ?? '--'}s  drops ${values.drops ?? '--'}  rssi ${values.wifi_rssi ?? '--'} dBm`;
  }
  return packet.packet;
}

export function formatTime(timestamp: number): string {
  return new Date(timestamp).toLocaleTimeString([], { hour12: false });
}
