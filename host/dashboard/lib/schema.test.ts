import { describe, expect, it } from 'vitest';
import { decodeEvent, decodeHeartbeat, decodeStatus, heartbeatAgeMs } from './schema';

const bytes = (...values: number[]) => Uint8Array.from(values);

describe('wire schema', () => {
  it('decodes a big-endian status packet', () => {
    const packet = bytes(0xff, 0x6a, 0x03, 0xe8, 0x5a, 0x02, 0x01, 0x02);
    expect(decodeStatus(packet)).toEqual({
      speedMms: -150,
      distMm: 1000,
      headingDeg: 90,
      state: 2,
      faults: 0x0102,
    });
  });

  it('decodes an event packet with signed p1', () => {
    const packet = bytes(0x12, 0x34, 0x01, 0xff, 0xfe, 0x01, 0xf4, 0x00);
    expect(decodeEvent(packet)).toEqual({
      eventId: 0x1234,
      eventType: 1,
      p1: -2,
      p2: 500,
      reserved: 0,
    });
  });

  it('decodes a heartbeat packet with signed rssi', () => {
    const packet = bytes(0xff, 0xff, 0x03, 0xd6);
    expect(decodeHeartbeat(packet)).toEqual({
      uptimeS: 65535,
      drops: 3,
      wifiRssi: -42,
    });
  });

  it('rejects short or long payloads', () => {
    expect(() => decodeStatus(bytes(0x00))).toThrow(RangeError);
    expect(() => decodeHeartbeat(bytes(0x00, 0x01, 0x02, 0x03, 0x04))).toThrow(RangeError);
  });

  it('reports non-negative heartbeat age and rejects clock rollback', () => {
    expect(heartbeatAgeMs(1200, 1000)).toBe(200);
    expect(heartbeatAgeMs(900, 1000)).toBeNull();
  });
});
