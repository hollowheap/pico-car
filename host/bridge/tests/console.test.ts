import { describe, expect, it } from 'vitest';
import { COMMANDS, ENUMS, FAULT_BITS } from '../src/console/schema.generated.js';
import { consoleUpdate } from '../src/console/update.js';
import { createConsoleView } from '../src/console/view.js';

const status = (seq: number, odometerMm: number, timestamp: number) => ({
  packet: 'status', timestamp, values: { seq, odometer_mm: odometerMm, speed_mms: 100, heading_ddeg: 0 },
});

const ack = (timestamp: number, action: number, id: number) => ({
  packet: 'event', timestamp, values: { event_id: id, event_type: ENUMS.eventType.cmdAccept,
    source: ENUMS.eventSource.supervisor, p1: action, p2: id },
});

describe('bridge console projection', () => {
  it('derives course, run metrics, faults and command outcomes from retained and live packets', () => {
    const view = createConsoleView([status(1, 0, 1000), status(2, 20, 1200)]);
    view.append(ack(1300, COMMANDS.left, 77));
    const latest = status(3, 40, 1400);
    view.append({ ...latest, values: { ...latest.values, faults: 1 << FAULT_BITS.lifted } });
    const snapshot = view.read();
    expect(snapshot.view.course.points).toEqual([{ x: 0, y: 0 }, { x: 0, y: 20 }, { x: 0, y: 40 }]);
    expect(snapshot.view.summary).toMatchObject({ distanceMm: 40, durationS: 0.4, averageMms: 100 });
    expect(snapshot.view.junction.armed).toMatchObject({ direction: 'Left', source: 'command' });
    expect(snapshot.view.replies[`${COMMANDS.left}:77`]).toMatchObject({ accepted: true });
    expect(snapshot.view.faults.some((tile) => tile.bit === FAULT_BITS.lifted && tile.state === 'set')).toBe(true);
    expect(snapshot.frameLog.nextId).toBe(4);
  });

  it('keeps receipt order and clears the course on the accepted Reset boundary', () => {
    const view = createConsoleView([{ ...status(1, 90, 1000), receiptOrder: 1 }]);
    const before = view.read();
    view.append({ ...ack(1000, COMMANDS.reset, 8), receiptOrder: 2, receiptId: 'reset' });
    const delta = consoleUpdate(view.read(), before);
    expect(delta).toMatchObject({ type: 'consoleUpdate',
      store: { resetReceiptId: 'reset' }, frames: { from: 0, revision: 1, nextId: 1 } });
    view.append({ ...status(2, 0, 900), timelineMs: 1000, receiptOrder: 3 });
    const after = view.read();
    expect(after.frameLog.blocks.flat().map(({ packet }) => packet)).toEqual(['event', 'status']);
    expect(after.view.course.samples.at(-1)?.odometerMm).toBe(0);
    expect(after.lastPacket).toMatchObject({ timestamp: 1000, receivedAt: 900 });
  });
});
