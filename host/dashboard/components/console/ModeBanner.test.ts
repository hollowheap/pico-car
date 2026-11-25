import { expect, it, vi } from 'vitest';

import { FAULT_BITS } from '../../lib/schema.generated';
import type { Overview } from '../overview/overview';
import { bannerMessage } from './ModeBanner';

vi.mock('react-native', () => ({
  StyleSheet: { create: (styles: unknown) => styles },
  Text: () => null,
  View: () => null,
}));
vi.mock('../../lib/theme-provider', () => ({ useTheme: () => ({}) }));

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
    title: 'LIFTED',
    action: 'The wheels are held. Set the car down, then send START.',
  });
});
