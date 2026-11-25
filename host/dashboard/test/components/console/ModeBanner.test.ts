import { expect, it, vi } from 'vitest';

import { projectedHoldMessage } from '../../../../bridge/src/console/hold';
import { emptyStore, reduceStore, type TelemetryStore } from '../../../../bridge/src/console/telemetry-store';
import type { LinkPacket } from '../../../src/lib/link';
import { FAULT_BITS } from '../../../src/lib/schema.generated';
import type { Overview } from '../../../src/components/overview/overview';
import { bannerMessage, currentHoldMessage } from '../../../src/components/console/ModeBanner';

vi.mock('react-native', () => ({
  StyleSheet: { create: (styles: unknown) => styles },
  Text: () => null,
  View: () => null,
}));
vi.mock('../../../src/lib/theme-provider', () => ({ useTheme: () => ({}) }));

const holdMessage = (store: TelemetryStore, now = Date.now()) => currentHoldMessage(projectedHoldMessage(store), now);

it('asks for START in the live lifted banner', () => {
  const overview: Overview = {
    status: 'open',
    health: 'fault',
    statusAgeMs: 0,
    faults: ['lifted'],
    speedMms: 0,
    headingDeg: null,
  };

  expect(bannerMessage(overview, 1 << FAULT_BITS.lifted)).toEqual({
    title: 'PICKUP DETECTED',
    action: 'Set down, then press Start.',
  });
});

it('clears a previous START refusal when the car receives the next command during calibration', () => {
  const packets: LinkPacket[] = [
    { packet: 'status', timestamp: 1000, values: { state: 3, mode: 0, speed_mms: 0, hold_mask: 68 } },
    { packet: 'event', timestamp: 1200, values: { event_type: 5, event_id: 8, p1: -28415, p2: 27534 } },
  ];
  const store = packets.reduce(reduceStore, emptyStore());
  expect(holdMessage(store)?.title).toBe('START REFUSED');
  expect(holdMessage(reduceStore(store, {
    packet: 'diag', timestamp: 1600, values: { cmd_last_id: 27535 },
  }))?.title).toBe('IMU CALIBRATION');
});

it('leaves staleness to the rail and labels a previously reported safety condition', () => {
  const overview: Overview = {
    status: 'open', health: 'stale', statusAgeMs: 2000, faults: [], speedMms: 0, headingDeg: null,
  };

  expect(bannerMessage(overview, 0)).toBeNull();
  expect(bannerMessage(overview, 1 << FAULT_BITS.estopLatched)?.title).toBe('LAST REPORTED EMERGENCY STOP');
  expect(bannerMessage(overview, 1 << FAULT_BITS.lifted)?.action).toBe('Waiting for fresh vehicle status.');
});

it('guides a righted car through the single Start action and keeps reboot guidance for E-stop', () => {
  const overview: Overview = {
    status: 'open', health: 'fault', statusAgeMs: 0, faults: [], speedMms: 0, headingDeg: null,
  };
  expect(bannerMessage(overview, 1 << FAULT_BITS.overturned)?.action).toBe('Set the car down upright and still, then press Start.');
  expect(bannerMessage(overview, 1 << FAULT_BITS.obstacleStopped)?.action).toBe('Clear the path, then press Start.');
  expect(bannerMessage(overview, 1 << FAULT_BITS.estopLatched)?.action).toMatch(/reboot/i);
});

it('explains a pilot hold on the status rail and clears it on a new decision or reboot', () => {
  const stopped = {
    packet: 'status', timestamp: 2000,
    values: { state: 1, mode: 0, speed_mms: 0, hold_mask: 0, odometer_mm: 999 },
  };
  const decision = { packet: 'event', timestamp: 1900, values: { event_type: 26, event_id: 1, p2: 0x010b } };
  const store = [decision, stopped].reduce(reduceStore, emptyStore());
  expect(holdMessage(store)).toEqual({
    title: 'COURSE CLEARANCE HOLD', action: 'The pilot surveys again after a rest. If the hold stays, clear the path, then send START.',
  });
  expect(holdMessage(reduceStore(store, {
    ...decision, timestamp: 2050, values: { ...decision.values, event_id: 2, p2: 0x010c },
  }))).toEqual({
    title: 'CHECKING COURSE CLEARANCE', action: 'Waiting for fresh forward clearance.',
  });
  expect(holdMessage(reduceStore(store, {
    ...decision, timestamp: 2100, values: { ...decision.values, event_id: 2, p2: 0x0300 },
  }))).toBeNull();
  expect(holdMessage(reduceStore(store, {
    ...decision, timestamp: 2100, values: { ...decision.values, event_type: 1 },
  }))).toBeNull();
  expect(holdMessage(reduceStore(store, {
    ...stopped, timestamp: 2100, values: { ...stopped.values, hold_mask: 4 },
  }))).toBeNull();
});

it('explains the stationary obstacle survey reported by boot 25568', () => {
  const packets: LinkPacket[] = [
    { packet: 'event', timestamp: 1000, values: { event_type: 4, event_id: 9, p1: 257, p2: 15007 } },
    { packet: 'event', timestamp: 3904, values: { event_type: 32, event_id: 18, p1: 99, odometer_mm: 153 } },
    { packet: 'event', timestamp: 3904, values: { event_type: 26, event_id: 19, p2: 268, odometer_mm: 153 } },
    { packet: 'profile', timestamp: 4919, values: { status: 4, req_id: 6, flags: 0, closest_mm: 93 } },
    { packet: 'status', timestamp: 60899, values: { state: 1, mode: 0, speed_mms: 0, hold_mask: 0, faults: 16, odometer_mm: 154 } },
  ];
  const store = packets.reduce(reduceStore, emptyStore());
  expect(holdMessage(store)?.title).toBe('OBSTACLE · DETOURS OFF');
  expect(holdMessage(store)?.action).toMatch(/Stop.*normal Start/);
  const next = reduceStore(store, { packet: 'event', timestamp: 60900,
    values: { event_type: 26, event_id: 33, p2: 268 } });
  expect(holdMessage(next)?.title).toBe('CHECKING COURSE CLEARANCE');
});

it('requires a reported set-down and current corroboration before prompting a pickup restart', () => {
  const status = { packet: 'status', timestamp: 2000,
    values: { state: 1, mode: 7, hold_mask: 16, faults: 0, speed_mms: 0 } };
  const physics = { packet: 'physics', timestamp: 2000,
    values: { flags: 8, attitude_state: 0, motion_class: 1, pitch_cdeg: 40, roll_cdeg: -15 } };
  const end = { packet: 'event', timestamp: 1950, values: { event_type: 57, source: 4, event_id: 1 } };
  const message = (packets: LinkPacket[], now = 2000) =>
    holdMessage(packets.reduce(reduceStore, emptyStore()), now);
  expect(message([end, status, physics])).toEqual({ title: 'UPRIGHT', action: 'Press Start to resume.' });
  expect(message([end, {...status, values: {...status.values, faults: 1 << FAULT_BITS.wifiDisconn}}, physics])?.title).toBe('UPRIGHT');
  expect(message([end, status, {...physics, timestamp: 4500}], 4500)?.title).not.toBe('UPRIGHT');
  for (const packets of [[status, physics], [end, status],
    [end, status, { ...physics, values: { ...physics.values, flags: 0 } }],
    [end, status, { ...physics, values: { ...physics.values, roll_cdeg: 2600 } }],
    [end, status, physics, { ...end, timestamp: 2001, values: { ...end.values, event_type: 56, event_id: 2 } }],
    [end, { ...status, values: { ...status.values, faults: 1 << FAULT_BITS.imuNack } }, physics]]) {
    expect(message(packets)?.title).not.toBe('UPRIGHT');
  }
  expect(message([end, status, physics], 4000)?.title).not.toBe('UPRIGHT');
  const projected = projectedHoldMessage([end, physics, { ...status, timestamp: 3000 }].reduce(reduceStore, emptyStore()));
  expect(currentHoldMessage(projected, 2999)?.title).toBe('PICKUP HOLD');
  expect(currentHoldMessage(projected, 3000)?.title).toBe('UPRIGHT');
  expect(currentHoldMessage(projected, 3500)?.title).toBe('UPRIGHT');
  expect(currentHoldMessage(projected, 3501)?.title).toBe('PICKUP HOLD');
});

it('reports a stopped car awaiting Start after set-down while the pickup fault stays active', () => {
  const lifted = 1 << FAULT_BITS.lifted;
  const overview: Overview = {
    status: 'open', health: 'fault', statusAgeMs: 0, faults: ['lifted'], speedMms: 0, headingDeg: null,
  };
  const frames: LinkPacket[] = [
    { packet: 'event', timestamp: 1950, values: { event_type: 57, source: 4, event_id: 1 } },
    { packet: 'status', timestamp: 2000, values: { state: 3, mode: 7, hold_mask: 20, faults: lifted, speed_mms: 0 } },
    { packet: 'physics', timestamp: 2000,
      values: { flags: 8, attitude_state: 0, motion_class: 1, pitch_cdeg: 40, roll_cdeg: -15 } },
  ];
  const store = frames.reduce(reduceStore, emptyStore());
  expect(bannerMessage(overview, lifted, holdMessage(store, 2000)))
    .toEqual({ title: 'STOPPED AFTER PICKUP', action: 'Press Start to resume.' });
  expect(bannerMessage(overview, lifted, holdMessage(store, 5000))?.title).toBe('PICKUP DETECTED');
});
