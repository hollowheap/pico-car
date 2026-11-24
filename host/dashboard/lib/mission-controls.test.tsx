/// <reference types="node" />

import { createRequire } from 'node:module';
import type { ReactElement } from 'react';
import { expect, it, vi } from 'vitest';

import RootLayout from '../app/_layout';
import { ControlsPanel } from '../components/controls/ControlsPanel';
import { FAULT_ESTOP_BIT } from './faults';
import type { LinkPacket } from './link';
import { emptyStore, reduceStore } from './telemetry-store';
import { themeFor } from './theme';

const { renderToStaticMarkup } = createRequire(import.meta.url)('react-dom/server') as {
  renderToStaticMarkup: (element: ReactElement) => string;
};
const state = vi.hoisted(() => ({ store: null as ReturnType<typeof emptyStore> | null }));

vi.mock('expo-router', () => ({
  DarkTheme: {}, DefaultTheme: {}, ThemeProvider: ({ children }: { children: ReactElement }) => children,
  usePathname: () => '/',
  Stack: () => <ControlsPanel card />,
}));
vi.mock('expo-status-bar', () => ({ StatusBar: () => null }));
vi.mock('react-native', () => ({
  Platform: { OS: 'web' }, StyleSheet: { create: (styles: unknown) => styles }, useWindowDimensions: () => ({ width: 1440 }),
  View: ({ children }: { children: ReactElement }) => <div>{children}</div>,
  Text: ({ children }: { children: ReactElement }) => <span>{children}</span>,
}));
vi.mock('react-native-safe-area-context', () => ({ useSafeAreaInsets: () => ({}) }));
vi.mock('../components/console/ConsoleNav', () => ({
  Brand: () => null, ThemeToggle: () => null,
}));
vi.mock('../components/console/ModeBanner', () => ({ ModeBanner: () => null, bannerMessage: () => null, reportedHoldMessage: () => null }));
vi.mock('../components/console/StatusRail', () => ({ StatusRail: () => null }));
vi.mock('../components/ui/button', () => ({
  Button: Object.assign(
    ({ children, disabled }: { children: ReactElement; disabled: boolean }) => <button disabled={disabled}>{children}</button>,
    { Text: ({ children }: { children: string }) => children },
  ),
}));
vi.mock('./config', () => ({ getLinkUrl: () => 'ws://localhost:5280/', getServiceUrl: () => 'http://localhost:5280' }));
vi.mock('./telemetry-context', () => ({
  TelemetryProvider: ({ children }: { children: ReactElement }) => children,
  useTelemetry: () => ({ store: state.store, snapshot: { status: 'open', bridge: { mqtt: 'connected' } } }),
}));
vi.mock('./theme-provider', () => ({
  ThemeProvider: ({ children }: { children: ReactElement }) => children,
  useTheme: () => themeFor('dark'),
}));
vi.mock('./useNow', () => ({ useNow: () => 2000 }));

const status = (timestamp: number, faults = 0, seq = 0, odometerMm = 100): LinkPacket => ({
  packet: 'status', timestamp, values: { seq, faults, odometer_mm: odometerMm, speed_mms: 0 },
});

function expectControls(store: ReturnType<typeof emptyStore>, missionDisabled: boolean, safetyDisabled = false) {
  state.store = store;
  const markup = renderToStaticMarkup(<RootLayout />);
  const attribute = missionDisabled ? ' disabled=""' : '';
  for (const label of ['Start', 'Left', 'Straight', 'Right', 'U-turn', 'Reset']) {
    expect(markup).toContain(`<button${attribute}>${label}</button>`);
  }
  const safetyAttribute = safetyDisabled ? ' disabled=""' : '';
  for (const label of ['Stop', 'E-stop']) expect(markup).toContain(`<button${safetyAttribute}>${label}</button>`);
  for (const label of ['Clear faults', 'Recalibrate IMU', 'Route']) expect(markup).not.toContain(`>${label}</button>`);
}

it('holds mission controls after a reboot until current status is available', () => {
  const store = [
    { packet: 'heartbeat', timestamp: 1995, values: { run_id: 7, uptime_s: 40 } },
    { packet: 'heartbeat', timestamp: 1996, values: { run_id: 8, uptime_s: 1 } },
    { packet: 'event', timestamp: 1996, values: { event_type: 1, event_id: 1 } },
    status(1997, 1 << FAULT_ESTOP_BIT),
  ].reduce(reduceStore, emptyStore());
  expectControls(store, true);

  const boot = { packet: 'event', timestamp: 1998, values: { event_type: 1, event_id: 1 } };
  const restarted = reduceStore(store, boot);
  expect(restarted.latest.status).toBeUndefined();
  expectControls(restarted, true, true);
  expectControls(reduceStore(restarted, status(1999)), false);
});

it('holds mission controls when status has aged past the console freshness limit', () => {
  expectControls(reduceStore(emptyStore(), status(500)), false);
  expectControls(reduceStore(emptyStore(), status(499)), true, true);
});

it.each([
  { current: 107, older: 100, lag: 0, odometer: 100 },
  { current: 107, older: 100, lag: 2, odometer: 9000 },
  { current: 107, older: 100, lag: 29, odometer: 100 },
  { current: 1, older: 65530, lag: 2, odometer: 100 },
])('keeps E-STOP set when status $older arrives after $current with $lag ms receipt skew',
  ({ current, older, lag, odometer }) => {
    const latched = status(1970, 1 << FAULT_ESTOP_BIT, current);
    const onlyCurrent = reduceStore(emptyStore(), latched);
    const store = reduceStore(onlyCurrent, status(1970 + lag, 0, older, odometer));

    expect(store.latest.status).toBe(latched);
    expect(store.trail.map((sample) => sample.seq)).toEqual([current]);
    expectControls(store, true);
  });

it('waits for post-boot status when an earlier receipt is delivered after the boot event', () => {
  const boot = { packet: 'event', timestamp: 1998, values: { event_type: 1, event_id: 1 } };
  const store = [status(1996, 1 << FAULT_ESTOP_BIT, 107), boot, status(1997, 0, 100)]
    .reduce(reduceStore, emptyStore());

  expect(store.latest.status).toBeUndefined();
  expectControls(store, true, true);
  expectControls(reduceStore(store, status(1998, 0, 1)), false);
});

it('does not make an expired status current again when its sequence is repeated', () => {
  const expired = status(499, 0, 107);
  const store = [expired, status(1999, 0, 107)].reduce(reduceStore, emptyStore());

  expect(store.latest.status).toBe(expired);
  expectControls(store, true, true);
});

it('shows signal, uptime and independently sampled missed ticks in the persistent navbar', () => {
  state.store = [
    status(2000),
    { packet: 'heartbeat', timestamp: 1900, values: { wifi_rssi: -55, uptime_s: 65 } },
    { packet: 'motion', timestamp: 1950, values: { overruns: 3 } },
    { packet: 'diag', timestamp: 1975, values: { physics_overruns: 0, telemetry_overruns: 12 } },
  ].reduce(reduceStore, emptyStore());
  const shell = renderToStaticMarkup(<RootLayout />);
  for (const text of ['Signal', '-55 dBm', 'Car uptime', '1m 5s', 'Missed ticks', 'Motion 3', 'Physics 0', 'Telemetry 12']) {
    expect(shell).toContain(text);
  }
});

it('leaves expired wheel counts unknown when the other tasks still report zero', () => {
  const packets: LinkPacket[] = [
    { packet: 'motion', timestamp: 100, values: { overruns: 99 } },
    { packet: 'diag', timestamp: 1999, values: { physics_overruns: 0, telemetry_overruns: 0 } },
  ];
  state.store = packets.reduce(reduceStore, emptyStore());
  expect(renderToStaticMarkup(<RootLayout />)).toContain('Motion --   Physics 0   Telemetry 0');
});

it('keeps the radio initialization sentinel out of the signal reading', () => {
  state.store = reduceStore(emptyStore(), {
    packet: 'heartbeat', timestamp: 1999, values: { wifi_rssi: -128, uptime_s: 0 },
  });
  const shell = renderToStaticMarkup(<RootLayout />);
  expect(shell).not.toContain('-128 dBm');
  expect(shell).toContain('0s');
});
