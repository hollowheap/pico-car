import { describe, expect, it } from 'vitest';
import type { LinkSnapshot } from '../../lib/link';
import { buildOverview } from './overview';

function snapshot(overrides: Partial<LinkSnapshot> = {}): LinkSnapshot {
  return {
    status: 'open',
    lastPacketAt: 1000,
    lastPacket: {
      timestamp: 1000,
      packet: 'status',
      values: { speed_mms: 150, dist_mm: 1000, heading_deg: 90, state: 2, faults: 0 },
    },
    ...overrides,
  };
}

describe('overview', () => {
  it('reports healthy state, age, and live values', () => {
    const overview = buildOverview(snapshot(), 1200);
    expect(overview.health).toBe('healthy');
    expect(overview.heartbeatAgeMs).toBe(200);
    expect(overview.speedMms).toBe(150);
    expect(overview.headingDeg).toBe(90);
    expect(overview.faults).toEqual([]);
  });

  it('maps fault bits to names', () => {
    const overview = buildOverview(
      snapshot({
        lastPacket: {
          timestamp: 1000,
          packet: 'status',
          values: { speed_mms: 0, dist_mm: 0, heading_deg: 0, state: 2, faults: 0x14 },
        },
      }),
      1200,
    );
    expect(overview.health).toBe('fault');
    expect(overview.faults).toEqual(['motor_stall', 'line_lost']);
  });

  it('marks a stale heartbeat', () => {
    expect(buildOverview(snapshot(), 3000).health).toBe('stale');
  });

  it('marks a disconnected link offline', () => {
    expect(buildOverview(snapshot({ status: 'reconnecting' }), 1200).health).toBe('offline');
  });
});
