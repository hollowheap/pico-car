import { describe, expect, it } from 'vitest';
import { deviceSentMs } from './command';

const heartbeat = {
  timestamp: 1000,
  packet: 'heartbeat',
  values: { uptime_s: 10 },
};

describe('deviceSentMs', () => {
  it('adds the elapsed time to the heartbeat uptime', () => {
    expect(deviceSentMs([heartbeat], 1500)).toBe(10500);
  });

  it('wraps at 16 bits', () => {
    expect(
      deviceSentMs(
        [{ ...heartbeat, values: { uptime_s: 70 } }],
        1000,
      ),
    ).toBe((70000 + 0) & 0xffff);
  });

  it('returns null without a heartbeat', () => {
    expect(deviceSentMs([], 1000)).toBeNull();
  });

  it('rejects a heartbeat older than the command TTL', () => {
    expect(deviceSentMs([heartbeat], 2600)).toBeNull();
  });
});
