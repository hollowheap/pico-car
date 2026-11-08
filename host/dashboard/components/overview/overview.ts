import type { LinkSnapshot, LinkStatus } from '../../lib/link';
import { FAULT_BITS as GENERATED_FAULT_BITS } from '../../lib/schema.generated';

export type OverviewHealth = 'healthy' | 'fault' | 'stale' | 'offline';

export type Overview = {
  status: LinkStatus;
  health: OverviewHealth;
  statusAgeMs: number | null;
  faults: string[];
  speedMms: number | null;
  headingDeg: number | null;
};

const snakeCase = (value: string) =>
  value.replace(/[A-Z]/g, (letter) => `_${letter.toLowerCase()}`);

export const FAULT_BITS = Object.entries(GENERATED_FAULT_BITS).map(
  ([name, bit]) => ({ bit, name: snakeCase(name) }),
);

function faultNames(mask: number): string[] {
  return FAULT_BITS.filter((fault) => (mask & (1 << fault.bit)) !== 0).map(
    (fault) => fault.name,
  );
}

export function buildOverview(
  snapshot: LinkSnapshot,
  nowMs: number,
  staleAfterMs = 1500,
): Overview {
  const lastStatus = snapshot.lastStatus ?? null;
  const age =
    lastStatus === null ? null : Math.max(0, nowMs - lastStatus.timestamp);
  const statusPacket = lastStatus?.values ?? null;
  const faults = statusPacket === null ? [] : faultNames(statusPacket.faults ?? 0);

  let health: OverviewHealth = 'healthy';
  if (snapshot.status !== 'open') {
    health = 'offline';
  } else if (snapshot.bridge?.mqtt === 'disconnected') {
    health = 'offline';
  } else if (age === null || age > staleAfterMs) {
    health = 'stale';
  } else if (faults.length > 0) {
    health = 'fault';
  }

  return {
    status: snapshot.status,
    health,
    statusAgeMs: age,
    faults,
    speedMms: statusPacket?.speed_mms ?? null,
    headingDeg: statusPacket?.heading_deg ?? null,
  };
}
