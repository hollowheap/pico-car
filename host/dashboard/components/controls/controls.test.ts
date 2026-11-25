import { describe, expect, it } from 'vitest';
import { confirmControl, needsConfirm } from './controls';

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

  it('sends an emergency stop on one press, and asks before anything else', () => {
    expect(needsConfirm('estop')).toBe(false);
    expect(confirmControl('estop', false, false).allowed).toBe(true);
    expect(confirmControl('estop', false, true).allowed).toBe(true);
    for (const action of ['start', 'stop', 'reset', 'left', 'right', 'uturn', 'straight', 'recalibrate'] as const) {
      expect(needsConfirm(action)).toBe(true);
    }
  });

  it('allows confirmed start and stop', () => {
    expect(confirmControl('start', true, false).allowed).toBe(true);
    expect(confirmControl('stop', true, true).allowed).toBe(true);
  });
});
