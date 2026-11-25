import type { LinkPacket } from './link';

export type CommandAction = 'start' | 'stop' | 'reset';

export type CommandRequest = {
  action: string;
  cmdId: number;
  runId: number;
  sentMs: number;
};

export function deviceSentMs(
  packets: LinkPacket[],
  nowMs: number,
): number | null {
  const heartbeat = packets.find((packet) => packet.packet === 'heartbeat');
  const uptime = heartbeat?.values.uptime_s;
  if (heartbeat === undefined || uptime === undefined) {
    return null;
  }
  const elapsed = Math.max(0, nowMs - heartbeat.timestamp);
  if (elapsed > 1500) {
    return null;
  }
  return (uptime * 1000 + elapsed) & 0xffff;
}

export async function postCommand(
  serviceUrl: string,
  request: CommandRequest,
): Promise<void> {
  const base = serviceUrl.replace(/\/+$/, '');
  const response = await fetch(`${base}/command`, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(request),
  });
  if (!response.ok) {
    const body = await response.text();
    throw new Error(body || `command failed: ${response.status}`);
  }
}
