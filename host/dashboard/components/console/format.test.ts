import { describe, expect, it } from 'vitest';
import { motionName, packetSummary } from './format';

describe('motion state names', () => {
  it('matches the motion service codes', () => {
    expect(motionName(0)).toBe('IDLE');
    expect(motionName(1)).toBe('RUNNING');
    expect(motionName(2)).toBe('REVERSING');
    expect(motionName(3)).toBe('STOPPED');
    expect(motionName(4)).toBe('FAULT');
  });

  it('returns a placeholder outside the range', () => {
    expect(motionName(undefined)).toBe('--');
    expect(motionName(5)).toBe('--');
  });
});

describe('system packet summary', () => {
  it('uses the protocol enum names', () => {
    const summary = packetSummary({
      timestamp: 1,
      packet: 'system',
      values: {
        obstacle_state: 3,
        physics_state: 1,
        physics_motion: 5,
        obstacle_dist_mm: 420,
        pitch_cdeg: -125,
      },
    });
    expect(summary).toContain('obstacle SCANNING');
    expect(summary).toContain('hump CLIMB');
    expect(summary).toContain('dist 420 mm');
  });
});
