import type { LinkPacket, LinkStatus } from './link';

/** The states every panel can show besides its data (loading, empty, stale, error). */
export type PanelStatus =
  | { kind: 'ready' }
  | { kind: 'loading'; message: string }
  | { kind: 'empty'; message: string }
  | { kind: 'stale'; message: string }
  | { kind: 'error'; message: string };

/** How old a packet of each kind can be before the panel reads stale (ms). */
export const STALE_AFTER_MS: Record<string, number> = {
  status: 1500,
  motion: 1500,
  physics: 1500,
  obstacle: 3500,
  diag: 3500,
  heartbeat: 3500,
};

const READY: PanelStatus = { kind: 'ready' };

/**
 * The state of a panel that shows one kind of packet. The profile packet comes
 * once per scan, so it never goes stale.
 */
export function panelStatus(
  link: LinkStatus,
  packet: LinkPacket | undefined,
  nowMs: number,
  kind: string,
): PanelStatus {
  if (packet === undefined) {
    return link === 'open'
      ? { kind: 'loading', message: `Waiting for the first ${kind} packet.` }
      : { kind: 'empty', message: 'No data yet. The link is offline.' };
  }
  const limit = STALE_AFTER_MS[kind];
  if (limit === undefined) return READY;
  const age = Math.max(0, nowMs - packet.timestamp);
  if (age > limit) {
    return { kind: 'stale', message: `No ${kind} packet for ${(age / 1000).toFixed(1)} s. Values are old.` };
  }
  return READY;
}
