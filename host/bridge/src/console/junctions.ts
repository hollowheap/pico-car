import { BIT_FIELDS, COMMANDS, ENUMS } from './schema.generated.js';
import type { EventRow } from './telemetry-store.js';

export type JunctionDirection = 'Left' | 'Right' | 'U-turn' | 'Straight';
export type JunctionStripeShape = 'T / cross' | 'Left branch' | 'Right branch' | 'Stripe';

export type JunctionMarker = {
  timestamp: number;
  direction: JunctionDirection | JunctionStripeShape;
  outcome: 'active' | 'done' | 'aborted' | 'stripe' | 'noBranch' | 'fallbackExit';
};

export type JunctionState = {
  armed: {
    direction: JunctionDirection;
    source: 'barcode' | 'command';
    letter?: string;
    timestamp: number;
    probeNote?: 'no branch' | 'fallback exit';
  } | null;
  active: { direction: JunctionDirection; timestamp: number } | null;
  last: { direction: JunctionDirection; outcome: 'done' | 'aborted'; durationMs: number; timestamp: number } | null;
  lastStripe: { shape: JunctionStripeShape; corroborated: boolean; timestamp: number } | null;
  markers: JunctionMarker[];
};

const TURN_DIRECTION: Partial<Record<number, JunctionDirection>> = {
  1: 'Left',
  2: 'Right',
  3: 'U-turn',
  4: 'Straight',
};

const BARCODE_DIRECTION: Partial<Record<number, JunctionDirection>> = {
  [ENUMS.barcodeAction.left]: 'Left',
  [ENUMS.barcodeAction.right]: 'Right',
  [ENUMS.barcodeAction.uturn]: 'U-turn',
  [ENUMS.barcodeAction.straight]: 'Straight',
};

const COMMAND_DIRECTION: Partial<Record<number, JunctionDirection>> = {
  [COMMANDS.left]: 'Left',
  [COMMANDS.right]: 'Right',
  [COMMANDS.uturn]: 'U-turn',
  [COMMANDS.straight]: 'Straight',
};

export function deriveJunctionState(events: readonly EventRow[], runStartMs: number | null): JunctionState {
  const since = runStartMs ?? -Infinity;
  const state: JunctionState = {
    armed: null,
    active: null,
    last: null,
    lastStripe: null,
    markers: [],
  };

  for (const event of events) {
    if (event.timestamp < since) continue;
    if (event.name === 'barcode') {
      const direction = BARCODE_DIRECTION[event.p2 & 0x7];
      if (direction) {
        const code = event.p1 & 0x7f;
        const letter = code > 0 ? String.fromCharCode(code) : undefined;
        state.armed = { direction, source: 'barcode', letter, timestamp: event.timestamp };
      }
    } else if (event.name === 'cmdAccept') {
      const direction = COMMAND_DIRECTION[event.p1 & 0xff];
      if (direction) {
        state.armed = { direction, source: 'command', timestamp: event.timestamp };
      }
    } else if (event.name === 'junction') {
      if (event.p1 === ENUMS.junctionKind.stripe) {
        const left = (event.p2 & (1 << BIT_FIELDS.junctionStripe.left)) !== 0;
        const right = (event.p2 & (1 << BIT_FIELDS.junctionStripe.right)) !== 0;
        const corroborated = (event.p2 & (1 << BIT_FIELDS.junctionStripe.corroborated)) !== 0;
        const shape: JunctionStripeShape = left && right ? 'T / cross' : left ? 'Left branch' : right ? 'Right branch' : 'Stripe';
        state.lastStripe = { shape, corroborated, timestamp: event.timestamp };
        state.markers.push({ timestamp: event.timestamp, direction: shape, outcome: 'stripe' });
      } else if (event.p1 === ENUMS.junctionKind.probe) {
        const direction = TURN_DIRECTION[(event.p2 >> 8) & 0xff] ?? state.armed?.direction;
        const outcome = event.p2 & 0xff;
        if (direction) {
          if (outcome === ENUMS.junctionProbe.noBranch || outcome === ENUMS.junctionProbe.fallbackExit) {
            const note = outcome === ENUMS.junctionProbe.noBranch ? 'no branch' : 'fallback exit';
            // The firmware keeps the junction instruction pending after noBranch or fallbackExit.
            state.armed = state.armed !== null
              ? { ...state.armed, direction, probeNote: note }
              : { direction, source: 'barcode', timestamp: event.timestamp, probeNote: note };
            state.markers.push({
              timestamp: event.timestamp,
              direction,
              outcome: outcome === ENUMS.junctionProbe.noBranch ? 'noBranch' : 'fallbackExit',
            });
          } else if (outcome === ENUMS.junctionProbe.aborted) {
            state.active = null;
            state.markers.push({ timestamp: event.timestamp, direction, outcome: 'aborted' });
          }
        }
      }
    } else if (event.name === 'turnStart') {
      const direction = TURN_DIRECTION[event.p1];
      if (direction) {
        state.armed = null;
        state.active = { direction, timestamp: event.timestamp };
        state.markers.push({ timestamp: event.timestamp, direction, outcome: 'active' });
      }
    } else if (event.name === 'turnDone' || event.name === 'turnAbort') {
      const direction = TURN_DIRECTION[event.p1] ?? state.active?.direction;
      const outcome = event.name === 'turnDone' ? 'done' : 'aborted';
      state.active = null;
      if (direction) {
        state.last = { direction, outcome, durationMs: event.p2, timestamp: event.timestamp };
        const latestMarker = state.markers.at(-1);
        if (latestMarker && latestMarker.outcome === 'active' && latestMarker.direction === direction) {
          latestMarker.outcome = outcome;
        } else {
          state.markers.push({ timestamp: event.timestamp, direction, outcome });
        }
      }
    }
  }

  return state;
}
