import { BIT_FIELDS, COMMANDS, ENUMS, RESERVED_COMMANDS } from './schema.generated.js';
import { FAULT_META } from './faults.js';
import { barcodePhaseLabel } from './line-signals.js';
import { bitLabels, enumLabel } from './schema.js';

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
const TELEMETRY_SOURCE = 1;
const TURNS = ['', 'Turn left', 'Turn right', 'U-turn', 'Straight'];
const SIDES = ['none', 'left', 'right'];
const REJECT = ['', 'not running', 'busy', 'latched', 'lifted', 'source not allowed', 'unknown command', 'on a hump', 'stalled', 'IMU attitude unavailable', 'hardware connection fault'];
const REJECT_NO_ATTITUDE = 9;
const BARCODE_BAD = ['', 'Unexpected stripe width or bar/space order.',
  'Parity check failed.', 'Delimiter missing or invalid.', 'Speed outside the read range.', 'Duplicate code suppressed.',
  'Symbol outside the A–Z route alphabet.', 'Barcode sensor transitions were lost during the read.'];
const PLAN_STATUS = ['ok', 'clear', 'no safe path', 'failed'];
export const SCAN_WIDTH_LIMIT_MM = 1023;
const RUN_STATES = ['boot', 'running', 'stopped', 'held', 'latched'];
const STALLS: Record<number, string> = {
  0x80: 'the car does not move',
  0x81: 'line sensors never change',
  0x82: 'wheels answer crosswise (encoder plugs)',
};
const PILOT_BEHAVIOURS = ['none', 'safety', 'hug', 'follow line', 'rejoin search'];
export const PILOT_REASON = {
  stale: 1,
  lineLost: 3,
  encoder: 5,
  sonar: 6,
  held: 7,
  hugTimeout: 8,
  sonarLatched: 9,
  staleLatched: 10,
  courseClearance: 11,
  courseRecovery: 12,
  rejoinSearch: 13,
  rejoinStop: 14,
  blindCrawl: 15,
} as const;
export const pilotReason = (p2: number) => p2 & 0xff;
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
  'course clearance hold',
  'checking course clearance',
  'searching for line after detour',
  'no line after detour',
  'crawling without sonar',
];
const TASKS = ['motion', 'line', 'obstacle', 'physics', 'telemetry', 'connection', 'barcode'];
const BOOT_CHECKS: [number, string][] = [[1, 'IMU'], [2, 'sonar'], [4, 'sonar beam blocked'], [8, 'IMU not calibrated']];

const sentence = (text: string) => text.charAt(0).toUpperCase() + text.slice(1).toLowerCase();
const word = (list: string[], index: number, fallback = String(index)) => list[index] ?? fallback;
export const barcodeHasNoAction = (action: number) => action === ENUMS.barcodeAction.rejected || action === ENUMS.barcodeAction.none;
export const barcodeActionLabel = (action: number | undefined) => action === undefined ? 'Action unavailable'
  : barcodeHasNoAction(action) ? 'No driving action' : sentence(enumLabel('barcodeAction', action));

export function scanWidthLabel(widthMm: number | undefined): string {
  if (widthMm === undefined || widthMm <= 0) return 'Unmeasured';
  // Extras uses this code when an edge is open or the width reaches the cap.
  return widthMm === SCAN_WIDTH_LIMIT_MM ? 'Out of range' : `${widthMm} mm`;
}

const eventKey = Object.fromEntries(
  Object.entries(ENUMS.eventType).map(([name, code]) => [code, name]),
) as Record<number, string>;
const COMMAND_LABELS = [...Object.entries(COMMANDS), ...Object.entries(RESERVED_COMMANDS)];
const commandName = (opcode: number) => {
  const found = COMMAND_LABELS.find(([, code]) => code === opcode);
  return found === undefined ? `command ${opcode}` : found[0].replaceAll('_', ' ');
};
const commandSource = (p1: number) => (p1 >> 8) & 0xf;
export function isCurrentStartAttitudeRefusal(type: number, p1: number, commandId: number, lastReceivedId?: number) {
  const opcode = p1 & 0xff;
  if (type !== ENUMS.eventType.cmdReject || (opcode !== COMMANDS.start && opcode !== COMMANDS.start_continuous_floor && opcode !== RESERVED_COMMANDS.start_stationary) || ((p1 >> 12) & 0xf) !== REJECT_NO_ATTITUDE) return false;
  if (commandSource(p1) !== TELEMETRY_SOURCE || lastReceivedId === undefined) return true;
  // A new START may be calibrating before the supervisor reports its outcome.
  const ahead = (lastReceivedId - commandId) & 0xffff;
  return ahead === 0 || ahead >= 0x8000;
}
/** "opcode from source" out of a cmd_accept or cmd_reject p1. */
const commandWho = (p1: number) => `${commandName(p1 & 0xff)} from ${word(SOURCES, commandSource(p1))}`;
const commandId = (p1: number, cmdId: number) => (
  commandSource(p1) === TELEMETRY_SOURCE ? ` (command ${cmdId})` : ''
);

function attitudeDetail(action: string, tiltCdeg: number, accelerationMg: number): string {
  if (tiltCdeg === 0 && accelerationMg === 0) return action;
  const tilt = tiltCdeg >= 0 && tiltCdeg <= 18_000 ? `${tiltCdeg / 100}°` : 'unavailable';
  return `${action}, tilt ${tilt}, acceleration ${accelerationMg} mg`;
}

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
      if (p2 === ENUMS.runStopReason.startupUnverified) {
        return { kind: 'system', title: 'Startup motion unverified',
          detail: 'Set the car down with clear space ahead, then press Start.' };
      }
      return { kind: 'system', title: 'Run stopped', detail: `state: ${word(RUN_STATES, p1)}` };
    // p1 = reason << 12 | source << 8 | opcode (reason 0 when accepted),
    // p2 = cmd_id of a telemetry command, 0 from a barcode or the button.
    case 'cmdAccept':
      return { kind: 'command', title: 'Command accepted', detail: `${commandWho(p1)}${commandId(p1, p2)}` };
    case 'cmdReject': {
      const reason = (p1 >> 12) & 0xf;
      return {
        kind: 'command',
        title: 'Command refused',
        detail: `${commandWho(p1)}: ${word(REJECT, reason, 'reason ' + reason)}${commandId(p1, p2)}`,
      };
    }
    case 'motionState': {
      const beforeCode = (p1 >> 8) & 0xff;
      const afterCode = p1 & 0xff;
      const before = enumLabel('motionState', beforeCode).toLowerCase();
      const after = enumLabel('motionState', afterCode).toLowerCase();
      const fault = p2 >> 8;
      const holds = bitLabels('holdMask', p2 & 0xff);
      const detail = [`${before} → ${after}`];
      if (beforeCode === ENUMS.motionState.fault && afterCode === ENUMS.motionState.idle && fault === ENUMS.motionFault.none) {
        detail.push('cleared in place');
      }
      if (fault !== ENUMS.motionFault.none) detail.push(`fault ${enumLabel('motionFault', fault).toLowerCase()}`);
      if (holds.length > 0) detail.push(`hold ${holds.join(', ').toLowerCase()}`);
      return { kind: 'system', title: 'Motion state changed', detail: detail.join(', ') };
    }
    case 'barcode': {
      const letter = String.fromCharCode(p1 & 0x7f);
      const action = p2 & 0x7;
      const decoded = barcodeHasNoAction(action) ? 'Decoded, no driving action' : barcodeActionLabel(action);
      const reverse = (p2 & 0x8) !== 0 ? ', read in reverse' : '';
      return { kind: 'barcode', title: `Barcode ${letter}`, detail: `${decoded}${reverse}, narrow bar ${(p2 >> 4) * 4} µm` };
    }
    case 'barcodeBad': {
      const explanation = word(BARCODE_BAD, p1, `Reason ${p1}`);
      const element = p2 !== 0 ? ` Estimated last element width ${p2} µm.`
        : p1 === 1 ? ' The failed stripe is unavailable.' : '';
      return { kind: 'barcode', title: p1 === 5 ? 'Barcode ignored' : 'Barcode read failed',
        detail: explanation + element };
    }
    case 'barcodeBlock':
      return { kind: 'barcode', title: 'Long dark barcode reading', detail: `Barcode IR saw a dark surface for ${p1} mm.` };
    case 'barcodePhase': {
      const phase = barcodePhaseLabel(p1);
      const reverse = (p2 & 1) !== 0;
      const reject = (p2 >> 8) & 0xff;
      const reason = reject === 0 ? '' : word(BARCODE_BAD, reject, `Reason ${reject}`);
      return {
        kind: 'barcode', title: phase === undefined ? 'Barcode phase' : `Barcode ${phase.toLowerCase()}`,
        detail: [reverse ? 'Read in reverse' : '', reason].filter(Boolean).join(', '),
      };
    }
    case 'turnStart':
      return { kind: 'turn', title: `${word(TURNS, p1)} started`, detail: '' };
    case 'turnDone':
      return { kind: 'turn', title: `${word(TURNS, p1)} done`, detail: `${p2} ms` };
    case 'turnAbort':
      return { kind: 'turn', title: `${word(TURNS, p1)} aborted`, detail: `after ${p2} ms` };
    case 'junction': {
      if (p1 === ENUMS.junctionKind.stripe) {
        const left = (p2 & (1 << BIT_FIELDS.junctionStripe.left)) !== 0;
        const right = (p2 & (1 << BIT_FIELDS.junctionStripe.right)) !== 0;
        const corroborated = (p2 & (1 << BIT_FIELDS.junctionStripe.corroborated)) !== 0;
        const title = left && right
          ? 'Junction crossing'
          : left
            ? 'Left branch stripe'
            : right
              ? 'Right branch stripe'
              : 'Junction stripe';
        const detectors = left && right
          ? 'Both detectors crossed transverse tape'
          : left
            ? 'Left detector crossed tape (side branch or bend)'
            : right
              ? 'Right detector crossed tape (side branch or bend)'
              : 'Transverse tape measured';
        return {
          kind: 'turn',
          title,
          detail: corroborated ? `${detectors}, corroborated.` : `${detectors}.`,
        };
      }
      if (p1 === ENUMS.junctionKind.probe) {
        const requested = word(TURNS, (p2 >> 8) & 0xff, `Turn ${(p2 >> 8) & 0xff}`);
        const outcome = p2 & 0xff;
        if (outcome === ENUMS.junctionProbe.noBranch) {
          return { kind: 'turn', title: `${requested} probe: no branch`, detail: 'Requested exit not found; instruction stays pending.' };
        }
        if (outcome === ENUMS.junctionProbe.fallbackExit) {
          return { kind: 'turn', title: `${requested} probe: fallback exit`, detail: 'Following another exit; instruction stays pending.' };
        }
        if (outcome === ENUMS.junctionProbe.aborted) {
          return { kind: 'turn', title: `${requested} probe aborted`, detail: 'Cancelled, interrupted, or timed out.' };
        }
        return { kind: 'turn', title: `${requested} probe`, detail: `outcome ${outcome}` };
      }
      return { kind: 'turn', title: 'Junction event', detail: `kind ${p1}, p2 ${p2}` };
    }
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
      if (p1 === 4) {
        return { kind: 'obstacle', title: 'Stationary scan complete', detail: 'Course clearance is unverified.' };
      }
      return { kind: 'obstacle', title: 'Scan done', detail: `plan ${word(PLAN_STATUS, p1)}; width ${scanWidthLabel(p2).toLowerCase()}` };
    case 'bypassStart':
      return { kind: 'obstacle', title: 'Bypass started', detail: `hugging the ${enumLabel('hugSide', p1).toLowerCase()} side` };
    case 'bypassEnd':
      return { kind: 'obstacle', title: 'Bypass ended', detail: `back on the line; width ${p1 === 0 ? 'unknown' : `${p1} mm`}, breadth ${p2 === 0 ? 'unknown' : `${p2} mm`}` };
    case 'rejoin': {
      const offsetMm = (p2 << 16) >> 16;
      return {
        kind: 'obstacle',
        title: 'Rejoin check',
        detail: `Heading change ${p1 / 10}° since entry and lateral offset ${offsetMm} mm from the entry line.`,
      };
    }
    case 'obstacleStop':
      if (p1 === ENUMS.obstacleCause.external) {
        return { kind: 'command', title: 'External stop', detail: 'Navigation stopped by an external request.' };
      }
      return {
        kind: 'obstacle',
        title: 'Obstacle stop',
        detail: `cause ${enumLabel('obstacleCause', p1).toLowerCase()}; echo ${enumLabel('echoClass', p2).toLowerCase()}`,
      };
    case 'sonarAim':
      return {
        kind: 'obstacle',
        title: 'Sonar aim',
        detail: p1 === 0 ? 'Commanded 0° (center)' : `Commanded ${Math.abs(p1) / 10}° ${p1 > 0 ? 'left' : 'right'}`,
      };
    case 'humpStart':
      return { kind: 'hump', title: 'Hump started', detail: '' };
    case 'humpEnd':
      return { kind: 'hump', title: 'Hump ended', detail: p2 === 1 ? `height ${p1} mm` : `invalid height (raw ${p1} mm)` };
    case 'liftStart':
      return { kind: 'attitude', title: 'Car lifted', detail: attitudeDetail('pickup detected', p1, p2) };
    case 'liftEnd':
      return { kind: 'attitude', title: 'Car set down', detail: attitudeDetail('START releases the hold', p1, p2) };
    case 'overturn':
      return { kind: 'attitude', title: 'Car overturned', detail: attitudeDetail('Set down upright and still before START', p1, p2) };
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
      const reason = pilotReason(p2);
      const speed = ((p1 & 0xffff) << 16) >> 16;
      if (reason === PILOT_REASON.courseClearance) {
        return {
          kind: 'obstacle',
          title: 'Course clearance hold',
          detail: 'The pilot stopped before the detour because course clearance is unverified.',
        };
      }
      if (reason === PILOT_REASON.courseRecovery) {
        return {
          kind: 'obstacle',
          title: 'Checking course clearance',
          detail: 'The pilot is holding for fresh forward clearance.',
        };
      }
      if (reason === PILOT_REASON.rejoinSearch) {
        return { kind: 'obstacle', title: 'Searching for tape behind obstacle',
          detail: 'The car reached the original line position by odometry and is checking it with the IR sensors.' };
      }
      if (reason === PILOT_REASON.blindCrawl) {
        return { kind: 'obstacle', title: 'Crawling without sonar',
          detail: 'The sonar stays silent. The car follows the line slowly and treats wheel load or a stall as contact.' };
      }
      if (reason === PILOT_REASON.rejoinStop) {
        return { kind: 'obstacle', title: 'Rejoin search stopped',
          detail: 'The wheels stopped before the car confirmed the tape.' };
      }
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
    case 'stopTiming':
      return {
        kind: 'command',
        title: 'Stop timing',
        detail: `duty 0 in ${p1} ms; ${p2 === 65535 ? 'wheels did not rest' : `wheels at rest in ${p2} ms`}`,
      };
    case 'build': {
      const flags = (p1 >>> 12) & 0xf;
      const hash = (((p1 & 0x0fff) << 16) | (p2 & 0xffff)) >>> 0;
      const unknown = (flags & ENUMS.buildFlag.unknown) !== 0;
      const tags = [
        (flags & ENUMS.buildFlag.pilot) !== 0 ? 'pilot' : null,
        (flags & ENUMS.buildFlag.watchdog) !== 0 ? 'watchdog' : null,
        (flags & ENUMS.buildFlag.dirty) !== 0 ? 'dirty' : null,
      ].filter(Boolean);
      const rev = unknown ? 'unknown rev' : hash.toString(16).padStart(7, '0');
      return {
        kind: 'system',
        title: 'Firmware build',
        detail: tags.length > 0 ? `${rev} (${tags.join(', ')})` : rev,
      };
    }
    default:
      return { kind: 'system', title: `Event ${type}`, detail: `type ${type}, p1 ${p1}, p2 ${p2}` };
  }
}
