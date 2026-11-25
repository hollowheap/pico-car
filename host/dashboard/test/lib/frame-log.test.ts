import { describe, expect, it } from 'vitest';

import { appendFrame, createFrameLog, FRAME_LOG_LIMIT, readFramePage, readFrameSlice } from '../../src/lib/frame-log';
import { replayHistory, type HistoryRecord } from '../../src/lib/history';
import type { LinkPacket } from '../../src/lib/link';

const frame = (timestamp: number, packet = 'status', topic = 'car/s', value = timestamp): HistoryRecord => ({
  timestamp, packet, topic, values: { seq: value },
});

describe('frame log', () => {
  it('merges history and live overlap once while retaining protocol records and topic identity', () => {
    const before = frame(1000);
    const boot = frame(1020, 'event', 'car/e');
    const retry = frame(1020, 'event', 'car/e');
    const thirdEvent = frame(1020, 'event', 'car/e');
    const anotherSource = frame(1020, 'event', 'other/e');
    const profile = frame(1030, 'profile', 'car/f');
    const liveOnly = frame(1040, 'diag', 'car/d');
    const buffered: LinkPacket[] = [boot, retry, thirdEvent, anotherSource, profile, liveOnly];
    const { frames } = replayHistory([profile, retry, boot, before], buffered);

    expect(frames).toEqual([before, boot, retry, thirdEvent, anotherSource, profile, liveOnly]);
    expect(readFramePage(createFrameLog(frames), null).rows.map(({ packet }) => packet))
      .toEqual([...frames].reverse());
  });

  it('holds an older page and its identities as live records arrive', () => {
    const seeded = createFrameLog(Array.from({ length: 25 }, (_, i) => frame(i)));
    const newest = readFramePage(seeded, null);
    const older = readFramePage(seeded, newest.start);
    const appended = appendFrame(appendFrame(seeded, frame(25)), frame(26));

    expect(readFramePage(appended, older.end).rows).toEqual(older.rows);
    expect(readFramePage(appended, null).rows[0]?.packet.timestamp).toBe(26);
  });

  it('uses a short oldest page without repeating records from the preceding page', () => {
    const log = createFrameLog(Array.from({ length: 9 }, (_, i) => frame(i)));
    const latest = readFramePage(log, null);
    const oldest = readFramePage(log, latest.older);
    expect(oldest.rows.map(({ id }) => id)).toEqual([0]);
  });

  it('matches a live overlap delivered after the history request completes', () => {
    const delayed = frame(1100, 'motion', 'car/m');
    const { takeSnapshotCopy } = replayHistory([delayed, frame(1000)], []);
    expect(takeSnapshotCopy({ ...delayed, topic: 'other/m' })).toBe(false);
    expect(takeSnapshotCopy(delayed)).toBe(true);
    expect(takeSnapshotCopy(delayed)).toBe(false);
    expect(takeSnapshotCopy(frame(1200))).toBe(false);
  });

  it('keeps exactly the newest retained records at and across a block boundary', () => {
    let log = createFrameLog(Array.from({ length: FRAME_LOG_LIMIT + 10 }, (_, i) => frame(i)));
    for (let i = 0; i < 135; i++) log = appendFrame(log, frame(FRAME_LOG_LIMIT + 10 + i));

    const latest = readFramePage(log, null);
    const oldest = readFramePage(log, 0);
    expect(latest.count).toBe(FRAME_LOG_LIMIT);
    expect(latest.rows[0]?.packet.timestamp).toBe(FRAME_LOG_LIMIT + 144);
    expect(oldest.rows.at(-1)?.packet.timestamp).toBe(145);
    expect(oldest.start).toBe(log.nextId - FRAME_LOG_LIMIT);
  });

  it('filters frame slices by packet name while preserving original frame ids', () => {
    const log = createFrameLog([
      frame(100, 'status'),
      frame(110, 'motion'),
      frame(120, 'event'),
      frame(130, 'motion'),
      frame(140, 'physics'),
    ]);
    const motion = readFrameSlice(log, 0, 10, 'motion');
    expect(motion.count).toBe(2);
    expect(motion.rows.map(({ id, packet }) => [id, packet.timestamp])).toEqual([[1, 110], [3, 130]]);
    expect(readFrameSlice(log, 0, 10, 'diag').count).toBe(0);
  });
});
