import { describe, expect, it } from 'vitest';
import { BIT_FIELDS, COMMANDS, ENUMS, FAULT_BITS } from '../src/console/schema.generated.js';
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

  it('decodes rejoin events with signed lateral offset', () => {
    const view = createConsoleView([{
      packet: 'event',
      timestamp: 1000,
      values: { event_id: 1, event_type: ENUMS.eventType.rejoin, p1: -35, p2: 0xfff6 },
    }]);
    const row = view.read().store.events[0];
    expect(row?.text).toMatchObject({
      kind: 'obstacle',
      title: 'Rejoin check',
      detail: 'Heading change -3.5° since entry and lateral offset -10 mm from the entry line.',
    });
  });

  it('decodes build and stop_timing events', () => {
    const view = createConsoleView([
      {
        packet: 'event',
        timestamp: 1000,
        values: {
          event_id: 1,
          event_type: ENUMS.eventType.build,
          p1: ((ENUMS.buildFlag.pilot | ENUMS.buildFlag.dirty) << 12) | 0x0abc,
          p2: 0xdef0,
        },
      },
      {
        packet: 'event',
        timestamp: 1100,
        values: {
          event_id: 2,
          event_type: ENUMS.eventType.stopTiming,
          p1: 12,
          p2: 185,
        },
      },
    ]);
    const events = view.read().store.events;
    expect(events[0]?.text).toMatchObject({
      kind: 'system',
      title: 'Firmware build',
      detail: 'abcdef0 (pilot, dirty)',
    });
    expect(events[1]?.text).toMatchObject({
      kind: 'command',
      title: 'Stop timing',
      detail: 'duty 0 in 12 ms; wheels at rest in 185 ms',
    });
  });

  it('extracts hump height and forward sonar from status_bundle and physics packets', () => {
    const view = createConsoleView([{
      packet: 'status',
      timestamp: 1000,
      values: {
        seq: 1,
        odometer_mm: 100,
        speed_mms: 120,
        heading_ddeg: 0,
        device_sample_us: 1_000_000,
        sample_flags: 1 << BIT_FIELDS.statusSampleFlags.attitudeValid,
        hump_state: ENUMS.humpState.climb,
        hump_peak_mm: 19,
        flags: (1 << BIT_FIELDS.physicsFlags.calibrated) | (1 << BIT_FIELDS.physicsFlags.heightValid) |
          (1 << BIT_FIELDS.physicsFlags.humpActive),
        forward_dist_mm: 245,
        forward_echo_class: ENUMS.echoClass.valid,
        forward_age_ms: 40,
      },
    }]);
    const first = view.read();
    expect(first.view.summary.lastHump).toMatchObject({ mm: 19, state: 'measuring' });
    expect(first.store.obstaclePing).toMatchObject({
      timestamp: 960,
      values: { dist_mm: 245, echo_class: ENUMS.echoClass.valid, servo_deci: 0 },
    });

    view.append({
      packet: 'physics',
      timestamp: 1200,
      values: {
        hump_state: ENUMS.humpState.flat,
        hump_peak_mm: 22,
        flags: (1 << BIT_FIELDS.physicsFlags.calibrated) | (1 << BIT_FIELDS.physicsFlags.heightValid) |
          (1 << BIT_FIELDS.physicsFlags.finished),
      },
    });
    expect(view.read().view.summary.lastHump).toMatchObject({ mm: 22, state: 'measured' });
  });

  it('extracts bundled wheel travel and detailed course overlays', () => {
    const view = createConsoleView([{
      packet: 'status',
      timestamp: 1000,
      values: {
        seq: 1,
        odometer_mm: 100,
        speed_mms: 100,
        heading_ddeg: 0,
        line_bits: 0x07,
        device_sample_us: 1_000_000,
        sample_flags: (1 << BIT_FIELDS.statusSampleFlags.wheelFresh) |
          (1 << BIT_FIELDS.statusSampleFlags.leftTravelValid) |
          (1 << BIT_FIELDS.statusSampleFlags.rightTravelValid),
        left_travel_mm: 123,
        right_travel_mm: 125,
        wheel_age_us: 4200,
        forward_dist_mm: 220,
        forward_echo_class: ENUMS.echoClass.valid,
        forward_age_ms: 20,
      },
    }, {
      packet: 'status',
      timestamp: 1200,
      values: {
        seq: 2,
        odometer_mm: 120,
        speed_mms: 100,
        heading_ddeg: 0,
        line_bits: 0x07,
      },
    }]);
    const snap = view.read();
    expect(snap.store.latest.motion?.values).toMatchObject({
      left_travel_mm: 123,
      right_travel_mm: 125,
      wheel_age_us: 4200,
    });
    expect(snap.view.course.forwardReturn).toMatchObject({ rangeMm: 220 });
    expect(snap.view.drawing['north-up'].tapeRuns.length).toBeGreaterThan(0);
  });

  it('clears measurements on reboot', () => {
    const view = createConsoleView([
      { packet: 'heartbeat', timestamp: 1000, values: { run_id: 1, uptime_s: 10 } },
      { packet: 'status', timestamp: 1050, values: { seq: 1, odometer_mm: 100, speed_mms: 100, heading_ddeg: 0 } },
      { packet: 'event', timestamp: 1300, values: { event_id: 1, event_type: ENUMS.eventType.scanDone, p1: ENUMS.profileStatus.ok, p2: 88 } },
      { packet: 'event', timestamp: 1350, values: { event_id: 2, event_type: ENUMS.eventType.humpEnd, p1: 21, p2: 1 } },
    ]);
    expect(view.read().view.summary.obstacleWidth?.mm).toBe(88);
    expect(view.read().view.summary.lastHump?.mm).toBe(21);

    view.append({ packet: 'heartbeat', timestamp: 2000, values: { run_id: 2, uptime_s: 0 } });
    const afterReboot = view.read();
    expect(afterReboot.view.summary.obstacleWidth).toBeUndefined();
    expect(afterReboot.view.summary.lastHump).toBeUndefined();
  });

  it('gates bundled physics on attitudeValid and ignores out-of-order measurement events', () => {
    const view = createConsoleView([
      {
        packet: 'status',
        timestamp: 1000,
        values: {
          seq: 1,
          odometer_mm: 100,
          speed_mms: 100,
          heading_ddeg: 0,
          device_sample_us: 1_000_000,
          sample_flags: 1 << BIT_FIELDS.statusSampleFlags.attitudeValid,
          hump_peak_mm: 18,
          yaw_rate_cdps: 250,
          flags: 1 << BIT_FIELDS.physicsFlags.calibrated,
        },
      },
      {
        packet: 'status',
        timestamp: 1200,
        values: {
          seq: 2,
          odometer_mm: 120,
          speed_mms: 100,
          heading_ddeg: 0,
          device_sample_us: 1_200_000,
          sample_flags: 0,
          hump_peak_mm: 99,
          yaw_rate_cdps: 999,
          flags: 0,
        },
      },
      { packet: 'event', timestamp: 1500, values: { event_id: 1, event_type: ENUMS.eventType.scanDone, p1: ENUMS.profileStatus.ok, p2: 75 } },
      { packet: 'event', timestamp: 1400, values: { event_id: 2, event_type: ENUMS.eventType.obstacleSeen, p1: 200 } },
      { packet: 'event', timestamp: 1500, values: { event_id: 3, event_type: ENUMS.eventType.humpEnd, p1: 24, p2: 1 } },
      { packet: 'event', timestamp: 1450, values: { event_id: 4, event_type: ENUMS.eventType.humpStart } },
    ]);
    const snap = view.read();
    expect(snap.store.latest.physics?.values.yaw_rate_cdps).toBe(250);
    expect(snap.view.summary.obstacleWidth?.mm).toBe(75);
    expect(snap.view.summary.lastHump).toMatchObject({ mm: 24, state: 'measured' });
  });
});
