import { describe, expect, it } from 'vitest';
import { packetSummary } from './format';

describe('packet summaries', () => {
  it('names the obstacle packet codes', () => {
    const summary = packetSummary({
      timestamp: 1,
      packet: 'obstacle',
      values: { state: 3, cause: 10, echo_class: 2, dist_mm: 420 },
    });
    expect(summary).toContain('obstacle SCANNING');
    expect(summary).toContain('dist 420 mm');
    expect(summary).toContain('cause PLAN REJECTED');
    expect(summary).toContain('echo TOO CLOSE');
  });

  it('names the physics packet codes', () => {
    const summary = packetSummary({
      timestamp: 1,
      packet: 'physics',
      values: { pitch_cdeg: -125, hump_state: 1, motion_class: 5, attitude_state: 0 },
    });
    expect(summary).toContain('hump CLIMB');
    expect(summary).toContain('pitch -125 cdeg');
  });

  it('shows heading in degrees and hides an invalid heading', () => {
    const status = (heading_ddeg: number) =>
      packetSummary({
        timestamp: 1,
        packet: 'status',
        values: { speed_mms: 150, odometer_mm: 1000, heading_ddeg, state: 2, faults: 0 },
      });
    expect(status(905)).toContain('hdg 90.5 deg');
    expect(status(0xffff)).toContain('hdg -- deg');
  });
});
