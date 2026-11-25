import { expect, it } from 'vitest';

import { describeEvent } from './events';
import { faultTiles } from './faults';
import { ENUMS, FAULT_BITS } from './schema.generated';

it.each([73, 0])('explains a START refused because of a hardware connection for command %i', (cmdId) => {
  // Signed wire value 0xA101: hardware fault, telemetry source, START.
  const refusal = describeEvent(ENUMS.eventType.cmdReject, -24319, cmdId);

  expect(refusal.title).toBe('Command refused');
  expect(refusal.detail).toBe(`start from telemetry: hardware connection fault (command ${cmdId})`);
});

it('omits the command ID when a barcode START is refused', () => {
  const refusal = describeEvent(ENUMS.eventType.cmdReject, 0x5201, 0);

  expect(refusal.detail).toBe('start from barcode: source not allowed');
});

it('names the silent sonar and the unresponsive IMU by their current fault bits', () => {
  const active = (1 << FAULT_BITS.sonarNoEcho) | (1 << FAULT_BITS.imuNack);
  const oldEchoFault = 1 << FAULT_BITS.echoLine;
  const tiles = faultTiles(active, active | oldEchoFault);

  expect(tiles.filter((tile) => tile.state === 'set').map((tile) => tile.label))
    .toEqual(['IMU not answering', 'Sonar silent']);
  expect(tiles.find((tile) => tile.bit === FAULT_BITS.echoLine)?.state).toBe('was-set');
});
