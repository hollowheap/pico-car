import type { LinkPacket } from '../../lib/link';
import { enumLabel } from '../../lib/schema';

export const MOTION_STATES = [
  'IDLE',
  'RUNNING',
  'REVERSING',
  'STOPPED',
  'FAULT',
] as const;

export function motionName(state: number | undefined): string {
  if (state === undefined || state < 0 || state >= MOTION_STATES.length) {
    return '--';
  }
  return MOTION_STATES[state] ?? '--';
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
  if (packet.packet === 'system') {
    const obstacle = enumLabel('obstacleState', values.obstacle_state);
    const hump = enumLabel('physicsState', values.physics_state);
    return (
      `obstacle ${obstacle}  dist ${values.obstacle_dist_mm ?? '--'} mm  ` +
      `hump ${hump}  pitch ${values.pitch_cdeg ?? '--'} cdg`
    );
  }
  return packet.packet;
}

export function formatTime(timestamp: number): string {
  return new Date(timestamp).toLocaleTimeString([], { hour12: false });
}
