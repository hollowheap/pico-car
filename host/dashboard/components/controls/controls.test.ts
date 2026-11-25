import { describe, expect, it } from 'vitest';
import { confirmControl, encodeCommand } from './controls';

describe('controls', () => {
  it('requires confirmation', () => {
    expect(confirmControl('start', false, false)).toEqual({
      allowed: false,
      reason: 'confirmation required',
    });
  });

  it('rejects reset while the emergency latch is set', () => {
    expect(confirmControl('reset', true, true)).toEqual({
      allowed: false,
      reason: 'emergency latch is set',
    });
  });

  it('allows confirmed start and stop', () => {
    expect(confirmControl('start', true, false).allowed).toBe(true);
    expect(confirmControl('stop', true, true).allowed).toBe(true);
  });

  it('encodes the eight-byte command frame', () => {
    expect(Array.from(encodeCommand('start', 0x1234, 0x0001, 0x0102))).toEqual([
      0x12, 0x34, 0x00, 0x01, 0x01, 0x02, 0x01, 0x00,
    ]);
    expect(encodeCommand('reset', 1, 1, 1)[6]).toBe(0x02);
  });
});
