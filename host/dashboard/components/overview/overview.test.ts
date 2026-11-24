import { describe, expect, it } from 'vitest';
import type { LinkPacket, LinkSnapshot } from '../../lib/link';
import { buildOverview } from './overview';

const statusPacket: LinkPacket = {
  timestamp: 1000,
  packet: 'status',
  values: { speed_mms: 150, odometer_mm: 1000, heading_ddeg: 900, state: 2, faults: 0 },
};

function snapshot(overrides: Partial<LinkSnapshot> = {}): LinkSnapshot {
  return {
    status: 'open',
    lastPacketAt: 1000,
    lastPacket: statusPacket,
    lastStatus: statusPacket,
    ...overrides,
  };
}

describe('overview', () => {
  it('reports healthy state, age, and live values', () => {
    const overview = buildOverview(snapshot(), 1200);
    expect(overview.health).toBe('healthy');
    expect(overview.statusAgeMs).toBe(200);
    expect(overview.speedMms).toBe(150);
    expect(overview.headingDeg).toBe(90);
    expect(overview.faults).toEqual([]);
  });

  it('maps fault bits to names', () => {
    const faultStatus: LinkPacket = {
      timestamp: 1000,
      packet: 'status',
      values: { speed_mms: 0, odometer_mm: 0, heading_ddeg: 0, state: 2, faults: 0x14 },
    };
    const overview = buildOverview(
      snapshot({ lastPacket: faultStatus, lastStatus: faultStatus }),
      1200,
    );
    expect(overview.health).toBe('fault');
    expect(overview.faults).toEqual(['motor_stall', 'line_lost']);
  });

  it('marks a stale status', () => {
    expect(buildOverview(snapshot(), 3000).health).toBe('stale');
  });

  it('does not treat a heartbeat as a status', () => {
    const heartbeat: LinkPacket = {
      timestamp: 1000,
      packet: 'heartbeat',
      values: { uptime_s: 42, drops: 0, wifi_rssi: -55 },
    };
    const overview = buildOverview(
      snapshot({ lastPacket: heartbeat, lastStatus: null }),
      1200,
    );
    expect(overview.health).toBe('stale');
    expect(overview.statusAgeMs).toBeNull();
  });

  it('marks a disconnected link offline', () => {
    expect(buildOverview(snapshot({ status: 'reconnecting' }), 1200).health).toBe('offline');
  });

  it('marks a disconnected bridge offline', () => {
    const overview = buildOverview(
      snapshot({
        bridge: { mqtt: 'disconnected', packets: 0, lastPacketAt: null },
      }),
      1200,
    );
    expect(overview.health).toBe('offline');
  });

  it('compensates for browser-to-bridge clock skew when computing status freshness', () => {
    // Bridge clock is 5 s behind the browser clock: packet arrived at browser t=15000 with bridge timestamp 10000.
    const skewed = snapshot({
      lastPacketAt: 15_000,
      lastPacket: { ...statusPacket, timestamp: 10_000 },
      lastStatus: { ...statusPacket, timestamp: 10_000 },
    });
    const fresh = buildOverview(skewed, 15_200);
    expect(fresh.health).toBe('healthy');
    expect(fresh.statusAgeMs).toBe(200);

    // After 2 s without a newer status, it still turns stale despite clock skew.
    expect(buildOverview(skewed, 17_000).health).toBe('stale');

    // Bridge clock 5 s ahead of the browser clock also stays healthy then turns stale on time.
    const ahead = snapshot({
      lastPacketAt: 10_000,
      lastPacket: { ...statusPacket, timestamp: 15_000 },
      lastStatus: { ...statusPacket, timestamp: 15_000 },
    });
    expect(buildOverview(ahead, 10_200)).toMatchObject({ health: 'healthy', statusAgeMs: 200 });
    expect(buildOverview(ahead, 12_000).health).toBe('stale');
  });
});
