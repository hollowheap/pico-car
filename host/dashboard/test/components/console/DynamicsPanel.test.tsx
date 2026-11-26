/// <reference types="node" />

import { createRequire } from 'node:module';
import type { ReactElement, ReactNode } from 'react';
import { expect, it, vi } from 'vitest';

import { createConsoleView } from '../../../../bridge/src/console/view';
import { DynamicsPanel } from '../../../src/components/console/AttitudePanel';
import type { LinkPacket } from '../../../src/lib/link';
import { BIT_FIELDS, ENUMS } from '../../../src/lib/schema.generated';
import { themeFor } from '../../../src/lib/theme';

const { renderToStaticMarkup } = createRequire(import.meta.url)('react-dom/server') as {
  renderToStaticMarkup: (element: ReactElement) => string;
};

type Props = { children?: ReactNode; testID?: string };
vi.mock('react-native', () => ({
  StyleSheet: { create: (styles: unknown) => styles },
  View: ({ children }: Props) => <div>{children}</div>,
  Text: ({ children, testID }: Props) => <span data-testid={testID}>{children}</span>,
}));
vi.mock('react-native-svg', () => {
  const shape = () => null;
  return { default: shape, Circle: shape, Defs: shape, G: shape, Line: shape, LinearGradient: shape,
    Path: shape, Rect: shape, Stop: shape };
});
vi.mock('../../../src/lib/theme-provider', () => ({ useTheme: () => themeFor('dark') }));
vi.mock('../../../src/components/ui/button', () => ({ Button: () => null }));
vi.mock('../../../src/components/console/FrameLogPanel', () => ({ DecodedFrameFields: () => null }));
vi.mock('../../../src/components/console/Panel', () => ({ Panel: ({ children }: Props) => <>{children}</> }));

function event(id: number, type: number, p1 = 0, p2 = 0): LinkPacket {
  return { packet: 'event', timestamp: id * 30, values: { event_id: id, event_type: type, p1, p2 } };
}

function renderHeight(events: LinkPacket[], values: LinkPacket['values'] = {}): string {
  const physics = { packet: 'physics', timestamp: 200, values } as const;
  const { view } = createConsoleView(events).read();
  return renderToStaticMarkup(<DynamicsPanel physics={physics} summary={view.summary} />);
}

it('keeps a completed height visible until another hump starts', () => {
  const done = event(2, ENUMS.eventType.humpEnd, 18, 1);
  const events = [done, event(3, ENUMS.eventType.runStop), event(4, ENUMS.eventType.runStart)];
  const flat = { hump_state: ENUMS.humpState.flat };
  expect(renderHeight(events, flat)).toContain('18 mm');
  const next = [...events, event(5, ENUMS.eventType.humpStart)];
  expect(renderHeight(next, flat)).toContain('Measuring');
  expect(renderHeight([...next, event(6, ENUMS.eventType.humpEnd, 24, 1)], flat)).toContain('24 mm');
});

it('distinguishes a failed height from a later unvalidated peak estimate', () => {
  const events = [event(2, ENUMS.eventType.humpEnd, 2, 0)];
  const markup = renderHeight(events, { hump_state: ENUMS.humpState.flat, hump_peak_mm: 32,
    flags: 1 << BIT_FIELDS.physicsFlags.calibrated });
  expect(markup).toContain('Invalid');
  expect(markup).not.toContain('32 mm');
});
