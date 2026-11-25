import { readFileSync } from 'node:fs';

import { scoreCourse, type CalibrationCheckpoint } from './course-calibration.js';
import type { LinkPacket } from './link.js';

function readHistory(path: string): LinkPacket[] {
  return readFileSync(path, 'utf8').split('\n').flatMap((line, index) => {
    if (!line.trim()) return [];
    const packet = JSON.parse(line) as Partial<LinkPacket> | null;
    if (!packet || typeof packet.packet !== 'string' || !Number.isFinite(packet.timestamp) ||
        !packet.values || typeof packet.values !== 'object' || Array.isArray(packet.values) ||
        Object.values(packet.values).some(value => !Number.isFinite(value))) {
      throw new Error(`Invalid history packet on line ${index + 1}.`);
    }
    return [packet as LinkPacket];
  });
}

function isCheckpoint(value: unknown): value is CalibrationCheckpoint {
  if (!value || typeof value !== 'object') return false;
  const point = value as Partial<CalibrationCheckpoint>;
  return typeof point.label === 'string' && point.label.trim().length > 0 &&
    typeof point.seq === 'number' && Number.isInteger(point.seq) && point.seq >= 0 && point.seq <= 65535 &&
    Number.isFinite(point.xMm) && Number.isFinite(point.yMm) &&
    (point.headingDeg === undefined ||
      (Number.isFinite(point.headingDeg) && point.headingDeg >= 0 && point.headingDeg < 360));
}

try {
  const [historyPath, checkpointsPath] = process.argv.slice(2);
  if (!historyPath || !checkpointsPath || process.argv.length !== 4) {
    throw new Error('Usage: node --import tsx src/console/score-course.ts HISTORY.jsonl CHECKPOINTS.json');
  }
  const measured: unknown = JSON.parse(readFileSync(checkpointsPath, 'utf8'));
  if (!Array.isArray(measured) || !measured.every(isCheckpoint)) {
    throw new Error('Checkpoints must supply label, status seq, xMm, yMm, and optional headingDeg (0–359.99).');
  }
  const report = scoreCourse(readHistory(historyPath), measured);
  process.stdout.write(JSON.stringify(report, null, 2) + '\n');
  if (report.missing > 0) process.exitCode = 1;
} catch (error) {
  process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
  process.exitCode = 1;
}
