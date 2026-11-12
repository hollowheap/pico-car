import { FAULT_BITS } from './schema.generated';

/**
 * How a set fault reads on the dashboard. `critical` faults hold or stop the
 * car. `warn` faults need a look but the car still runs. Colour is never the
 * only cue: every tile also carries a state word and a severity word.
 */
export type Severity = 'critical' | 'warn';

export type FaultMeta = {
  label: string;
  severity: Severity;
  hint: string;
};

/** Keyed by bit number. A test fails when the schema adds a bit without a row. */
export const FAULT_META: Record<number, FaultMeta> = {
  0: { label: 'Wi-Fi down', severity: 'warn', hint: 'The car lost its access point.' },
  1: { label: 'MQTT rejected', severity: 'warn', hint: 'The broker refused a publish.' },
  2: { label: 'Motor stalled', severity: 'critical', hint: 'A wheel is told to turn and has no edges.' },
  3: { label: 'Encoder fault', severity: 'critical', hint: 'An encoder is dead, or the two plugs are swapped.' },
  4: { label: 'Line lost', severity: 'warn', hint: 'The car weaves at reduced speed until a sensor sees the line.' },
  5: { label: 'IMU not answering', severity: 'warn', hint: 'The I2C reads of the accelerometer fail.' },
  6: { label: 'Stale wheel speed', severity: 'critical', hint: 'The wheel feedback is too old.' },
  7: { label: 'Tilt out of range', severity: 'warn', hint: 'The tilt reading is not valid.' },
  8: { label: 'Sonar silent', severity: 'critical', hint: 'No echo from three pings in a row.' },
  9: { label: 'Emergency stop', severity: 'critical', hint: 'Latched until the car reboots.' },
  10: { label: 'Sonar echo line', severity: 'critical', hint: 'The echo line is stuck or noisy.' },
  11: { label: 'Car lifted', severity: 'critical', hint: 'The wheels are held. Set the car down, then START.' },
  12: { label: 'Car overturned', severity: 'critical', hint: 'Set the car upright, then RESET and START.' },
  13: { label: 'Stopped by obstacle', severity: 'warn', hint: 'The obstacle task stopped the car. RESET, then START.' },
  14: { label: 'Task stalled', severity: 'critical', hint: 'A task missed its heartbeat, or the car does not move.' },
  15: { label: 'Abnormal reset', severity: 'warn', hint: 'The last reset was not a power-up.' },
};

/** The emergency-stop bit, for the confirmation of the START command. */
export const FAULT_ESTOP_BIT = FAULT_BITS.estopLatched;

export type TileState = 'ok' | 'set' | 'was-set' | 'unknown';

export type FaultTile = {
  bit: number;
  key: string;
  label: string;
  hint: string;
  severity: Severity;
  state: TileState;
};

const bitSet = (word: number, bit: number): boolean => (word & (1 << bit)) !== 0;

/**
 * The 16 tiles in bit order. `level` is the status packet word (set now).
 * `sticky` is the diag packet word (set since the last RESET). A missing word
 * leaves the tiles `unknown`, never `ok`.
 */
export function faultTiles(level: number | undefined, sticky: number | undefined): FaultTile[] {
  return Object.entries(FAULT_BITS)
    .map(([key, bit]) => {
      const meta = FAULT_META[bit] ?? { label: key, severity: 'warn' as const, hint: '' };
      let state: TileState = 'unknown';
      if (level !== undefined) {
        if (bitSet(level, bit)) {
          state = 'set';
        } else if (sticky !== undefined && bitSet(sticky, bit)) {
          state = 'was-set';
        } else {
          state = 'ok';
        }
      }
      return { bit, key, ...meta, state };
    })
    .sort((a, b) => a.bit - b.bit);
}

export type FaultSummary = { set: number; critical: number; wasSet: number };

export function summarizeFaults(tiles: FaultTile[]): FaultSummary {
  return {
    set: tiles.filter((tile) => tile.state === 'set').length,
    critical: tiles.filter((tile) => tile.state === 'set' && tile.severity === 'critical').length,
    wasSet: tiles.filter((tile) => tile.state === 'was-set').length,
  };
}

export const STATE_WORD: Record<TileState, string> = {
  ok: 'OK',
  set: 'SET',
  'was-set': 'WAS SET',
  unknown: 'NO DATA',
};
