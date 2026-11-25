import { describe, expect, it } from 'vitest';
import { BIT_FIELDS, COMMANDS, ENUMS, FAULT_BITS } from '../src/console/schema.generated.js';
import { emptyStore, reduceStore } from '../src/console/telemetry-store.js';
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
  it('projects one bundled status through the motion, attitude and course views', () => {
    const base = status(65535, 20, 1000);
    const packet = {
      ...base,
      values: {
        ...base.values, device_sample_us: 0, line_bits: 3,
        left_mms: -40, right_mms: 60, left_travel_mm: -20, right_travel_mm: 60,
        physics_heading_ddeg: 2700, pitch_cdeg: 413, roll_cdeg: -140,
        flags: 1 << BIT_FIELDS.physicsFlags.calibrated, sample_flags: 0x3f,
        forward_dist_mm: 180, forward_age_ms: 15, forward_echo_class: 0,
      },
    };
    const first = reduceStore(emptyStore(), packet);
    expect(first.trail).toEqual([expect.objectContaining({
      lineBits: 3, headingDeg: 0, pitchDeg: 4.1, rollDeg: -1.4,
      wheels: { leftMms: -40, rightMms: 60 },
    })]);
    expect(first.wheelHistory).toEqual([expect.objectContaining({ at: 1000, seq: 255, left: -40, right: 60 })]);
    expect(first.pitchHistory).toEqual([{ at: 1000, pitchDeg: 4.13, humpActive: false }]);
    const snapshot = createConsoleView([packet]).read();
    expect(snapshot.store.latest).toMatchObject({
      status: { values: { left_travel_mm: -20, right_travel_mm: 60, forward_dist_mm: 180 } },
      motion: { packet: 'motion', timestamp: 1000, values: { left_mms: -40, right_mms: 60, seq: 255 } },
      physics: { packet: 'physics', timestamp: 1000, values: { heading_ddeg: 2700, pitch_cdeg: 413, seq: 255 } },
    });
    expect(snapshot.frameLog.blocks.flat()).toEqual([packet]);
    expect(reduceStore(first, { ...packet, timestamp: 1001 })).toBe(first);

    const late = reduceStore(first, {
      ...packet, timestamp: 1003,
      values: { ...packet.values, seq: 65534, pitch_cdeg: 760, left_mms: -33 },
    });
    expect(late.trail[0]).toMatchObject({ seq: 65534, pitchDeg: 7.6, wheels: { leftMms: -33, rightMms: 60 } });
    expect(late.latest).toBe(first.latest);
    expect(late.wheelHistory).toBe(first.wheelHistory);
    expect(late.pitchHistory).toBe(first.pitchHistory);

    const stale = reduceStore(first, {
      ...packet, timestamp: 1200,
      values: { ...packet.values, seq: 0, device_sample_us: 200000, sample_flags: 0 },
    });
    expect(stale.lastSeq).toBe(0);
    expect(stale.statusLost).toBe(0);
    expect(stale.trail.at(-1)?.wheels).toBeUndefined();
    expect(stale.trail.at(-1)?.pitchDeg).toBeUndefined();
    expect(stale.wheelHistory.at(-1)).toMatchObject({ at: 1200, seq: 0, left: undefined, right: undefined });
    expect(stale.pitchHistory).toBe(first.pitchHistory);
  });

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

  it('distinguishes the MQTT session timeout from a publish rejection', () => {
    const latest = status(1, 0, 1000);
    const { view } = createConsoleView([{ ...latest, values: { ...latest.values, faults: 0b11 } }]).read();

    expect(view.faults.slice(0, 2)).toEqual([
      { bit: 0, key: 'wifiDisconn', state: 'set', severity: 'warn',
        label: 'MQTT session down', hint: 'No MQTT session for at least 3 seconds.' },
      { bit: 1, key: 'mqttNack', state: 'set', severity: 'warn',
        label: 'MQTT rejected', hint: 'The broker refused a publish.' },
    ]);
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

  it('marks a recorded travel segment when the run began before the retained poses', () => {
    const start = { packet: 'event', timestamp: 1000, values: { event_id: 1,
      event_type: ENUMS.eventType.runStart, source: ENUMS.eventSource.supervisor } };
    const complete = createConsoleView([start, status(1, 0, 1200), status(2, 20, 1400)]).read();
    const partial = createConsoleView([start, status(50, 980, 11000), status(51, 1000, 11200)]).read();
    expect(complete.view.summary).toMatchObject({ scope: 'This run', partialTravel: false });
    expect(partial.view.summary).toMatchObject({ scope: 'This run', partialTravel: true, distanceMm: 20 });
  });
});
