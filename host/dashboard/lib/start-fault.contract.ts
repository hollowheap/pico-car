import { expect, it } from 'vitest';

import { describeEvent } from './events';
import { faultTiles } from './faults';
import { FAULT_BITS } from './schema.generated';

it('explains a START refused because of a hardware connection', () => {
  const refusals = Array.from({ length: 16 }, (_, reason) =>
    describeEvent(0x05, (reason << 12) | (1 << 8) | 1, 73));
  const refusal = refusals.find((event) => /hardware.*fault|connection.*fault/i.test(event.detail));

  expect(refusal).toBeDefined();
  expect(refusal?.title).toBe('Command refused');
  expect(refusal?.detail).toContain('command 73');
});

it('names the silent sonar and the unresponsive IMU by their current fault bits', () => {
  const active = (1 << FAULT_BITS.sonarNoEcho) | (1 << FAULT_BITS.imuNack);
  const oldEchoFault = 1 << FAULT_BITS.echoLine;
  const tiles = faultTiles(active, active | oldEchoFault);

  expect(tiles.filter((tile) => tile.state === 'set').map((tile) => tile.label))
    .toEqual(['IMU not answering', 'Sonar silent']);
  expect(tiles.find((tile) => tile.bit === FAULT_BITS.echoLine)?.state).toBe('was-set');
});
