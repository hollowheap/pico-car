/// <reference types="node" />

import { createRequire } from 'node:module';
import type { ReactElement, ReactNode } from 'react';
import { expect, it, vi } from 'vitest';

import { createConsoleView } from '../../../../bridge/src/console/view';
import type { LinkPacket } from '../../../src/lib/link';
import { themeFor } from '../../../src/lib/theme';
import { LineBarcodePanel } from '../../../src/components/console/LineBarcodePanel';

const { renderToStaticMarkup } = createRequire(import.meta.url)('react-dom/server') as {
  renderToStaticMarkup: (element: ReactElement) => string;
};
type Props = { children?: ReactNode; testID?: string };
vi.mock('react-native', () => ({
  StyleSheet: { create: (styles: unknown) => styles },
  View: ({ children, testID }: Props) => <div data-testid={testID}>{children}</div>,
  Text: ({ children, testID }: Props) => <span data-testid={testID}>{children}</span>,
  Pressable: ({ children }: Props) => <button>{children}</button>,
  Modal: ({ children, visible }: Props & { visible: boolean }) => visible ? <div>{children}</div> : null,
}));
vi.mock('react-native-svg', () => ({ default: () => null, Path: () => null }));
vi.mock('../../../src/lib/theme-provider', () => ({ useTheme: () => themeFor('dark') }));
vi.mock('../../../src/components/console/PanelIcon', () => ({ PanelIcon: () => null }));
vi.mock('../../../src/components/console/SectionPicker', () => ({ SectionPicker: () => null }));

function readings(diagValues?: LinkPacket['values'], hasStatus = true, lineBits?: number) {
  const event = (id: number, type: number, p1 = 0, p2 = 0): LinkPacket => ({
    packet: 'event', timestamp: id * 100, values: { event_id: id, event_type: type, p1, p2 },
  });
  const projection = createConsoleView([event(1, 0x10, 65, 5), event(2, 0x11), event(3, 0x18)]);
  const diag = diagValues && { packet: 'diag', timestamp: 600, values: diagValues };
  const statusPacket = lineBits === undefined ? undefined : { packet: 'status', timestamp: 600, values: { line_bits: lineBits } };
  if (diag !== undefined) projection.append(diag);
  if (statusPacket !== undefined) projection.append(statusPacket);
  const { store, view } = projection.read();
  return renderToStaticMarkup(<LineBarcodePanel statusPacket={store.latest.status} diag={store.latest.diag}
    rates={store.diagRates} lineSignals={store.lineSignals} nowMs={600} summary={view.summary}
    barcode={view.barcode} junction={view.junction} longestRecoveryMm={view.longestRecoveryMm}
    status={hasStatus ? { kind: 'ready' } : { kind: 'loading', message: 'Waiting for status' }} />);
}

it('displays white floor at raw GP26 LOW and dark tape at HIGH', () => {
  expect(readings(undefined, true, 0)).toContain('>Light<');
  expect(readings(undefined, true, 4)).toContain('>Dark<');
});

it('uses late-join firmware counts once, and retains observed counts for missing fields', () => {
  const markup = readings({ barcode_ok: 34, barcode_bad: 2, line_lost_count: 6 });
  expect(markup).toContain('Decoded 34, read issues 2');
  expect(markup).toContain('6 line losses');
  expect(markup.match(/Since boot/g)).toHaveLength(1);
  expect(markup).toContain('Recorded: 1 no action<');
  const partial = readings({ barcode_ok: 34 });
  expect(partial).toContain('Recorded: 1 no action, 1 read issue, 1 line loss<');
  expect(partial).not.toContain('1 decode');
});

it('presents retained line events even when the first status packet is still pending', () => {
  expect(readings(undefined, false)).toContain('Recorded: 1 decode, 1 no action, 1 read issue, 1 line loss<');
});
