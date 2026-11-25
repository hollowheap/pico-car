import type { LinkPacket } from '../../lib/link';
import { enumLabel, headingDegrees } from '../../lib/schema';

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
  const heading = headingDegrees(values.heading_ddeg);
  switch (packet.packet) {
    case 'status':
      return (
        `speed ${values.speed_mms ?? '--'} mm/s  odo ${values.odometer_mm ?? '--'} mm  ` +
        `hdg ${heading === null ? '--' : heading.toFixed(1)} deg  ` +
        `${enumLabel('runState', values.state)}  faults ${faultHex(values.faults)}`
      );
    case 'motion':
      return (
        `L ${values.left_mms ?? '--'}/${values.left_target_mms ?? '--'}  ` +
        `R ${values.right_mms ?? '--'}/${values.right_target_mms ?? '--'} mm/s  ` +
        `${enumLabel('motionState', values.motion_state)}  ${enumLabel('motionFault', values.motion_fault)}`
      );
    case 'physics':
      return (
        `pitch ${values.pitch_cdeg ?? '--'} cdeg  hump ${enumLabel('humpState', values.hump_state)}  ` +
        `${enumLabel('motionClass', values.motion_class)}  ${enumLabel('attitudeState', values.attitude_state)}`
      );
    case 'obstacle':
      return (
        `obstacle ${enumLabel('obstacleState', values.state)}  dist ${values.dist_mm ?? '--'} mm  ` +
        `cause ${enumLabel('obstacleCause', values.cause)}  echo ${enumLabel('echoClass', values.echo_class)}`
      );
    case 'diag':
      return (
        `line lost ${values.line_lost_count ?? '--'}  barcode ok ${values.barcode_ok ?? '--'} ` +
        `bad ${values.barcode_bad ?? '--'}  queue drops ${values.telem_queue_drops ?? '--'}`
      );
    case 'event':
      return (
        `id ${values.event_id ?? '--'}  ${enumLabel('eventType', values.event_type)}  ` +
        `${enumLabel('eventSource', values.source)}  p1 ${values.p1 ?? '--'}  p2 ${values.p2 ?? '--'}`
      );
    case 'heartbeat':
      return (
        `up ${values.uptime_s ?? '--'}s  drops ${values.drops ?? '--'}  ` +
        `rssi ${values.wifi_rssi ?? '--'} dBm  ${enumLabel('linkState', values.link_state)}`
      );
    case 'profile':
      return `req ${values.req_id ?? '--'}  closest ${values.closest_mm ?? '--'} mm  width ${values.width_mm ?? '--'} mm`;
    default:
      return packet.packet;
  }
}

export function formatTime(timestamp: number): string {
  return new Date(timestamp).toLocaleTimeString([], { hour12: false });
}

/** A value with its unit, or `--` when the packet has not arrived. */
export function withUnit(value: number | undefined, unit: string, digits = 0): string {
  return value === undefined ? '--' : `${value.toFixed(digits)} ${unit}`;
}

/** Odometer in millimetres as metres with two decimals. */
export function metres(mm: number | undefined): string {
  return mm === undefined ? '--' : `${(mm / 1000).toFixed(2)} m`;
}
