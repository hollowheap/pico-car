import { COMMANDS, ENUMS } from './schema.generated';
import { FAULT_META } from './faults';
import { enumLabel } from './schema';

/** What the event row draws, and what the course trace uses for its mark. */
export type EventKind =
  | 'system'
  | 'command'
  | 'barcode'
  | 'turn'
  | 'line'
  | 'obstacle'
  | 'hump'
  | 'attitude'
  | 'fault'
  | 'link';

export type EventText = { title: string; detail: string; kind: EventKind };

const SOURCES = ['', 'telemetry', 'barcode', 'button'];
const TURNS = ['', 'Turn left', 'Turn right', 'U-turn', 'Straight'];
const SIDES = ['none', 'left', 'right'];
const REJECT = ['', 'not running', 'busy', 'latched', 'lifted', 'source not allowed', 'unknown command', 'on a hump', 'stalled', 'lean unknown'];
const BARCODE_BAD = ['', 'bad element', 'parity', 'delimiter', 'speed out of range', 'same code twice'];
const PLAN_STATUS = ['ok', 'clear', 'no safe path', 'failed'];
const RUN_STATES = ['boot', 'running', 'stopped', 'held', 'latched'];
const STALLS: Record<number, string> = {
  0x80: 'the car does not move',
  0x81: 'line sensors never change',
  0x82: 'wheels answer crosswise (encoder plugs)',
};
const PILOT_BEHAVIOURS = ['none', 'safety', 'hug', 'follow line'];
const PILOT_REASONS = [
  'none',
  'stale wheel feedback',
  'emergency stop',
  'line lost',
  'stuck',
  'encoder',
  'sonar',
  'held',
  'hug move timeout',
  'sonar (latched)',
  'stale wheel feedback (latched)',
];
const TASKS = ['motion', 'line', 'obstacle', 'physics', 'telemetry', 'connection', 'barcode'];
const BOOT_CHECKS: [number, string][] = [[1, 'IMU'], [2, 'sonar'], [4, 'sonar beam blocked'], [8, 'IMU not calibrated']];

const sentence = (text: string) => text.charAt(0).toUpperCase() + text.slice(1).toLowerCase();
const word = (list: string[], index: number, fallback = String(index)) => list[index] ?? fallback;
const eventKey = Object.fromEntries(
  Object.entries(ENUMS.eventType).map(([name, code]) => [code, name]),
) as Record<number, string>;
const commandName = (opcode: number) => {
  const found = Object.entries(COMMANDS).find(([, code]) => code === opcode);
  return found === undefined ? `command ${opcode}` : found[0];
};
/** "opcode from source" out of a cmd_accept or cmd_reject p1. */
const commandWho = (p1: number) => `${commandName(p1 & 0xff)} from ${word(SOURCES, (p1 >> 8) & 0xf)}`;
const commandId = (cmdId: number) => (cmdId === 0 ? '' : ` (command ${cmdId})`);

/** The schema name of an event type, `unknown` for a code the schema lacks. */
export function eventName(type: number): string {
  return eventKey[type] ?? 'unknown';
}

/**
 * Title and detail of one event. The layouts of p1 and p2 are in the comments
 * of protocol/telemetry.yaml. An unknown type reads `Event 12 / type 3`.
 */
export function describeEvent(type: number, p1: number, p2: number): EventText {
  switch (eventName(type)) {
    case 'boot': {
      const failed = BOOT_CHECKS.filter(([bit]) => (p2 & bit) !== 0).map(([, name]) => name);
      return {
        kind: 'system',
        title: 'Boot',
        detail: `reset: ${enumLabel('resetReason', p1).toLowerCase()}; self-test ${failed.length === 0 ? 'passed' : `failed: ${failed.join(', ')}`}`,
      };
    }
    case 'runStart':
      return { kind: 'system', title: 'Run started', detail: '' };
    case 'runStop':
      return { kind: 'system', title: 'Run stopped', detail: `state: ${word(RUN_STATES, p1)}` };
    // p1 = reason << 12 | source << 8 | opcode (reason 0 when accepted),
    // p2 = cmd_id of a telemetry command, 0 from a barcode or the button.
    case 'cmdAccept':
      return { kind: 'command', title: 'Command accepted', detail: `${commandWho(p1)}${commandId(p2)}` };
    case 'cmdReject': {
      const reason = (p1 >> 12) & 0xf;
      return {
        kind: 'command',
        title: 'Command refused',
        detail: `${commandWho(p1)}: ${word(REJECT, reason, 'reason ' + reason)}${commandId(p2)}`,
      };
    }
    case 'barcode': {
      const letter = String.fromCharCode(p1 & 0x7f);
      const action = enumLabel('barcodeAction', p2 & 0x7);
      const reverse = (p2 & 0x8) !== 0 ? ', read in reverse' : '';
      return { kind: 'barcode', title: `Barcode ${letter}`, detail: `${sentence(action)}${reverse}; narrow bar ${(p2 >> 4) * 4} µm` };
    }
    case 'barcodeBad':
      return { kind: 'barcode', title: 'Barcode refused', detail: word(BARCODE_BAD, p1, 'reason ' + p1) };
    case 'barcodeBlock':
      return { kind: 'barcode', title: 'Block on the track', detail: `black for ${p1} mm; not a barcode` };
    case 'turnStart':
      return { kind: 'turn', title: `${word(TURNS, p1)} started`, detail: '' };
    case 'turnDone':
      return { kind: 'turn', title: `${word(TURNS, p1)} done`, detail: `${p2} ms` };
    case 'turnAbort':
      return { kind: 'turn', title: `${word(TURNS, p1)} aborted`, detail: `after ${p2} ms` };
    case 'lineLost':
      // p2 = 0 when the search starts; 0x8000 | n for retry n (the car backed up and searches
      // again); the search turn in degrees when the car stops.
      if (p2 === 0) {
        return { kind: 'line', title: 'Line lost', detail: `last seen on the ${word(SIDES, p1)}; the car turns on the spot to find it` };
      }
      if ((p2 & 0x8000) !== 0) {
        return { kind: 'line', title: `Line search retry ${p2 & 0x7fff}`, detail: `backed up and searching again; last seen on the ${word(SIDES, p1)}` };
      }
      return { kind: 'line', title: 'Line search stopped', detail: `no line in a ${p2} degree turn each way; last seen on the ${word(SIDES, p1)}` };
    case 'lineFound':
      return { kind: 'line', title: 'Line found', detail: `after ${p2} mm; last seen on the ${word(SIDES, p1)}` };
    case 'obstacleSeen':
      return { kind: 'obstacle', title: 'Obstacle seen', detail: `${p1} mm ahead` };
    case 'scanDone':
      return { kind: 'obstacle', title: 'Scan done', detail: `plan ${word(PLAN_STATUS, p1)}; width ${p2} mm` };
    case 'bypassStart':
      return { kind: 'obstacle', title: 'Bypass started', detail: `hugging the ${enumLabel('hugSide', p1).toLowerCase()} side` };
    case 'bypassEnd':
      return { kind: 'obstacle', title: 'Bypass ended', detail: `back on the line; width ${p1 === 0 ? 'unknown' : `${p1} mm`}, breadth ${p2 === 0 ? 'unknown' : `${p2} mm`}` };
    case 'obstacleStop':
      return {
        kind: 'obstacle',
        title: 'Obstacle stop',
        detail: `cause ${enumLabel('obstacleCause', p1).toLowerCase()}; echo ${enumLabel('echoClass', p2).toLowerCase()}`,
      };
    case 'humpStart':
      return { kind: 'hump', title: 'Hump started', detail: '' };
    case 'humpEnd':
      return { kind: 'hump', title: 'Hump ended', detail: p2 === 1 ? `peak ${p1} mm` : 'height not valid' };
    case 'liftStart':
      return { kind: 'attitude', title: 'Car lifted', detail: 'wheels held' };
    case 'liftEnd':
      return { kind: 'attitude', title: 'Car set down', detail: 'START releases the hold' };
    case 'overturn':
      return { kind: 'attitude', title: 'Car overturned', detail: 'RESET after it is upright' };
    case 'faultSet':
      return { kind: 'fault', title: 'Fault set', detail: FAULT_META[p1]?.label ?? `bit ${p1}` };
    case 'faultClear':
      return { kind: 'fault', title: 'Fault cleared', detail: FAULT_META[p1]?.label ?? `bit ${p1}` };
    case 'taskStall':
      return {
        kind: 'fault',
        title: 'Stall',
        detail: STALLS[p1] ?? `${word(TASKS, p1, 'task ' + p1)} task missed its heartbeat`,
      };
    // p1 = speed mm/s (signed), p2 = behaviour << 8 | reason.
    case 'pilotDecision': {
      const behaviour = (p2 >> 8) & 0xff;
      const reason = p2 & 0xff;
      const speed = ((p1 & 0xffff) << 16) >> 16;
      return {
        kind: 'system',
        title: 'Pilot decision',
        detail: `${word(PILOT_BEHAVIOURS, behaviour, 'behaviour ' + behaviour)}; reason ${word(PILOT_REASONS, reason, String(reason))}; ${speed} mm/s`,
      };
    }
    case 'linkUp':
      return { kind: 'link', title: 'Link up', detail: '' };
    case 'linkDown':
      return { kind: 'link', title: 'Link down', detail: '' };
    default:
      return { kind: 'system', title: `Event ${type}`, detail: `type ${type}, p1 ${p1}, p2 ${p2}` };
  }
}
