import { describe, expect, it } from 'vitest';

import { scoreCourse } from '../../src/lib/course-calibration';
import type { LinkPacket } from '../../src/lib/link';

const status = (seq: number, odometerMm: number, headingDeg: number): LinkPacket => ({
  packet: 'status',
  timestamp: seq * 200,
  values: { seq, odometer_mm: odometerMm, heading_ddeg: headingDeg * 10, speed_mms: 150 },
});

const origin = { label: 'Start', seq: 1, xMm: 100, yMm: 200, headingDeg: 0 };

describe('course calibration', () => {
  it('aligns the start once and preserves the position and heading errors at later checkpoints', () => {
    const report = scoreCourse(
      [status(1, 0, 90), status(2, 30, 90), status(3, 60, 90)],
      [origin, { label: 'Straight', seq: 3, xMm: 106, yMm: 268, headingDeg: 2 }],
    );

    expect(report).toMatchObject({ scored: 1, missing: 0, positionRmseMm: 10, maxPositionErrorMm: 10 });
    expect(report.checkpoints[0]).toMatchObject({
      label: 'Straight', estimatedXMm: 100, estimatedYMm: 260, positionErrorMm: 10, headingErrorDeg: -2,
    });
  });

  it('does not score a position after a dropped interval or invent a missing checkpoint', () => {
    const report = scoreCourse(
      [status(1, 0, 0), status(2, 30, 0), status(5, 120, 0), status(6, 150, 0)],
      [
        origin,
        { label: 'Before gap', seq: 2, xMm: 100, yMm: 230 },
        { label: 'Missing', seq: 3, xMm: 100, yMm: 260 },
        { label: 'After gap', seq: 6, xMm: 100, yMm: 350 },
      ],
    );

    expect(report).toMatchObject({ scored: 1, missing: 2, maxPositionErrorMm: 0 });
    expect(report.checkpoints[1]).toMatchObject({ unavailable: 'status sequence missing or ambiguous' });
    expect(report.checkpoints[2]).toMatchObject({ unavailable: 'trace gap after origin' });
  });
});
