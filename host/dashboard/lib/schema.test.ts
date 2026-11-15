import { describe, expect, it } from 'vitest';
import {
  bitLabels,
  decodePacket,
  enumLabel,
  heartbeatAgeMs,
  obstacleCauseLabel,
  obstacleFaultBits,
  obstacleStopReason,
  type PacketName,
} from './schema';
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
  });

  it('reads the bypass give-up reason from bits 12 to 14 of the obstacle faults', () => {
    const faults = (2 << 12) | 0b1001;
    expect(obstacleStopReason(faults)).toBe(2);
    expect(obstacleStopReason(undefined)).toBe(0);
    expect(obstacleFaultBits(faults)).toBe(0b1001);
    expect(bitLabels('obstacleFaults', obstacleFaultBits(faults))).toEqual(['NO ECHO', 'NO SAFE PATH']);
  });

  it('names cause 15 with its reason, and other causes without one', () => {
    expect(obstacleCauseLabel(15, 1)).toBe('BYPASS GAVE UP: CIRCLED');
    expect(obstacleCauseLabel(15, 4)).toBe('BYPASS GAVE UP: ECHO LOST');
    expect(obstacleCauseLabel(15, 0)).toBe('BYPASS GAVE UP');
    expect(obstacleCauseLabel(15, 7)).toBe('BYPASS GAVE UP: UNKNOWN 7');
    expect(obstacleCauseLabel(7, 3)).toBe('COLLISION');
    expect(obstacleCauseLabel(undefined, 0)).toBe('--');
  });
});
