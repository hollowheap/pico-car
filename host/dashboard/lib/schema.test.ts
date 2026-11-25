import { describe, expect, it } from 'vitest';
import {
  bitLabels,
  decodePacket,
  enumLabel,
  heartbeatAgeMs,
  type PacketName,
} from './schema';
import { COMMANDS, RESERVED_COMMANDS } from './schema.generated';
import vectors from '../../../protocol/vectors.json';

const hexBytes = (hex: string) =>
  Uint8Array.from((hex.match(/../g) ?? []).map((pair) => parseInt(pair, 16)));

function flatten(values: Record<string, number | number[] | undefined>): Record<string, number> {
  const flat: Record<string, number> = {};
  for (const [name, value] of Object.entries(values)) {
    if (Array.isArray(value)) {
      value.forEach((item, index) => {
        flat[`${name}_${index}`] = item;
      });
    } else if (value !== undefined) {
      flat[name] = value;
    }
  }
  return flat;
}

describe('wire schema', () => {
  it('decodes every shared vector', () => {
    for (const [name, vector] of Object.entries(vectors.vectors)) {
      expect(decodePacket(name as PacketName, hexBytes(vector.hex))).toEqual(flatten(vector.values));
    }
  });

  it('decodes the hug sequence byte of the obstacle frame', () => {
    const packet = new Uint8Array(24);
    packet[13] = 3;
    const decoded = decodePacket('obstacle', packet);
    expect(decoded.hug_seq).toBe(3);
    expect(decoded).not.toHaveProperty('reserved');
  });

  it('decodes a big-endian status packet', () => {
    const packet = Uint8Array.from([
      0xff, 0x6a, 0x00, 0x00, 0x03, 0xe8, 0x0e, 0x10, 0x02, 0x01, 0x01, 0x02, 0x00, 0x2a, 0x03, 0x00,
    ]);
    expect(decodePacket('status', packet)).toEqual({
      speed_mms: -150,
      odometer_mm: 1000,
      heading_ddeg: 3600,
      state: 2,
      mode: 1,
      faults: 0x0102,
      seq: 42,
      line_bits: 3,
      hold_mask: 0,
    });
  });

  it('rejects short or long payloads', () => {
    expect(() => decodePacket('status', Uint8Array.of(0x00))).toThrow(RangeError);
    expect(() => decodePacket('heartbeat', new Uint8Array(9))).toThrow(RangeError);
  });

  it('decodes pivot evidence carried by the current status layout', () => {
    const legacy = new Uint8Array(16);
    expect(decodePacket('status', legacy)).not.toHaveProperty('pivot_scale_pct');
    expect(decodePacket('status', Uint8Array.of(...legacy, 67))).toEqual({
      ...decodePacket('status', legacy), pivot_scale_pct: 67,
    });
  });

  it('reports non-negative heartbeat age and rejects clock rollback', () => {
    expect(heartbeatAgeMs(1200, 1000)).toBe(200);
    expect(heartbeatAgeMs(900, 1000)).toBeNull();
  });

  it('maps codes to protocol enum names', () => {
    expect(enumLabel('obstacleState', 3)).toBe('SCANNING');
    expect(enumLabel('obstacleCause', 10)).toBe('PLAN REJECTED');
    expect(enumLabel('echoClass', 2)).toBe('TOO CLOSE');
    expect(enumLabel('humpState', undefined)).toBe('--');
    expect(enumLabel('motionClass', 99)).toBe('UNKNOWN 99');
    expect(bitLabels('obstacleFaults', 0b1001)).toEqual(['NO ECHO', 'NO SAFE PATH']);
    expect(bitLabels('physicsFaults', 0)).toEqual([]);
    expect(enumLabel('obstacleCause', 15)).toBe('BYPASS GAVE UP');
    expect(enumLabel('hugState', 4)).toBe('LEG');
    expect(enumLabel('hugSide', 2)).toBe('NONE');
  });

  it('keeps opcode 10 reserved out of active commands', () => {
    expect(COMMANDS.start).toBe(1);
    expect(COMMANDS.start_continuous_floor).toBe(9);
    expect('start_stationary' in COMMANDS).toBe(false);
    expect(Object.values(COMMANDS)).not.toContain(10);
    expect(RESERVED_COMMANDS.start_stationary).toBe(10);
  });
});
