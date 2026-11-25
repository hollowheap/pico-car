/** How old a packet of each kind can be before the panel reads stale (ms). */
export const STALE_AFTER_MS: Record<string, number> = {
  status: 1500,
  motion: 1500,
  physics: 1500,
  obstacle: 3500,
  diag: 3500,
  heartbeat: 3500,
};
