/// <reference types="node" />

import { createRequire } from 'node:module';
import type { ReactElement, ReactNode } from 'react';
import { expect, it, vi } from 'vitest';

import { createConsoleView } from '../../../../bridge/src/console/view';
import type { LinkPacket } from '../../../src/lib/link';
import { ENUMS } from '../../../src/lib/schema.generated';
import { themeFor } from '../../../src/lib/theme';
import { ObstaclePanel } from '../../../src/components/console/ObstaclePanel';

const { renderToStaticMarkup } = createRequire(import.meta.url)('react-dom/server') as {
  renderToStaticMarkup: (element: ReactElement) => string;
};

type Props = { children?: ReactNode; testID?: string; accessibilityLabel?: string };

vi.mock('react-native', () => ({
  StyleSheet: { create: (styles: unknown) => styles },
  View: ({ children, testID }: Props) => <div data-testid={testID}>{children}</div>,
  Text: ({ children, testID, accessibilityLabel }: Props) =>
    <span data-testid={testID} aria-label={accessibilityLabel}>{children}</span>,
}));
vi.mock('../../../src/lib/theme-provider', () => ({ useTheme: () => themeFor('dark') }));
vi.mock('../../../src/components/console/Panel', () => ({
  Panel: ({ children, right }: Props & { right?: ReactNode }) => <>{right}{children}</>, useDense: () => true,
}));
vi.mock('../../../src/components/console/StatusPill', () => ({
  StatusPill: ({ label, accessibilityLabel }: { label: string; accessibilityLabel?: string }) =>
    <span aria-label={accessibilityLabel}>{label}</span>,
}));
vi.mock('../../../src/components/console/ScanReadings', () => ({
  ScanReadings: ({ children }: Props) => <div data-testid="obstacle-sonar-view">{children}</div>,
}));

function renderHug(values?: LinkPacket['values'], profileValues?: LinkPacket['values']): string {
  const obstacle = values === undefined ? undefined : { packet: 'obstacle', timestamp: 100, values } as const;
  const profile = profileValues === undefined ? undefined : { packet: 'profile', timestamp: 100, values: profileValues } as const;
  const { view } = createConsoleView(profile ? [profile] : []).read();
  return renderToStaticMarkup(
    <ObstaclePanel scan={view.scan} obstacle={obstacle} ping={undefined} profile={profile} nowMs={100} />,
  );
}

it('renders the sonar view directly without an edges tab', () => {
  const markup = renderHug();
  expect(markup).toContain('data-testid="obstacle-sonar-view"');
  expect(markup).not.toContain('data-testid="obstacle-edges"');
});

it('shows the recorded probe direction only when the stationary pilot supplied it', () => {
  const sparse = { status: ENUMS.profileStatus.pilotSparse, side: 0, flags: 0 };
  expect(renderHug(undefined, sparse)).not.toContain('edge probe');
  expect(renderHug(undefined, { ...sparse, flags: 2 })).toContain('Left edge probe');
  expect(renderHug(undefined, { ...sparse, side: 1, flags: 2 })).toContain('Right edge probe');
  expect(renderHug(undefined, { ...sparse, status: ENUMS.profileStatus.ok, flags: 2 })).not.toContain('edge probe');
});

it('shows measured hug dimensions while active and hides idle hug status when empty', () => {
  const readings = { hug_side: ENUMS.hugSide.left, face_mm: 115, width_mm: 221, breadth_mm: 83 };
  const active = renderHug({ ...readings, hug_state: ENUMS.hugState.align });
  const partial = renderHug({ hug_state: ENUMS.hugState.contact, face_mm: 115, hug_side: ENUMS.hugSide.none });
  const idle = renderHug({ ...readings, hug_state: ENUMS.hugState.idle });
  for (const value of ['align', 'left', '115 mm', '221 mm', '83 mm']) expect(active).toContain(value);
  expect(partial).toContain('115 mm');
  expect(partial).not.toMatch(/Width|Depth/);
  expect(idle).not.toContain('221 mm');
});

it('shows the pilot phase while the legacy obstacle scanner monitors', () => {
  const fields = { state: ENUMS.obstacleState.monitor };
  const active = renderHug({ ...fields, hug_state: ENUMS.hugState.leg });
  expect(active).toContain('aria-label="Hug phase leg"');
  expect(active).not.toContain('>MONITOR<');
  expect(renderHug({ ...fields, hug_state: ENUMS.hugState.idle })).toContain('>MONITOR<');
});
