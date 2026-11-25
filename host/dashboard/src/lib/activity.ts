import { PILOT_REASON, pilotReason } from './events';
import { BIT_FIELDS, ENUMS, FAULT_BITS } from './schema.generated';
import type { EventRow } from './telemetry-store';

export type ActivityGroup = { event: EventRow; members: EventRow[] };
const liftMask = 1 << BIT_FIELDS.holdMask.lift;
const source = ENUMS.eventSource;

function pickupRecord(e: EventRow) {
  switch (e.name) {
    case 'faultSet': return e.source === source.telemetry && e.p1 === FAULT_BITS.lifted;
    case 'liftStart': return e.source === source.physics;
    case 'runStop': return e.source === source.supervisor && e.p1 === 3;
    case 'pilotDecision': return e.source === source.line && e.p1 === 0 && (e.p2 >> 8) === 1 && pilotReason(e.p2) === PILOT_REASON.held;
    case 'motionState': return e.source === source.motion && (e.p1 & 255) === ENUMS.motionState.idle && e.p2 === liftMask;
    case 'obstacleStop': return e.source === source.obstacle && e.p1 === ENUMS.obstacleCause.external;
    default: return false;
  }
}

export function groupActivity(events: readonly EventRow[]): ActivityGroup[] {
  const result: ActivityGroup[] = [];
  for (let i = 0; i < events.length;) {
    const first = events[i]!;
    const members = [first];
    if (pickupRecord(first)) {
      const kinds = new Set([first.name]);
      for (let j = i + 1; j < events.length; j++) {
        const next = events[j]!;
        if (next.timestamp - first.timestamp > 250 || next.eventId !== ((events[j - 1]!.eventId + 1) & 65535) ||
            kinds.has(next.name) || !pickupRecord(next)) break;
        kinds.add(next.name);
        members.push(next);
      }
    }
    const pickup = members.find(e => e.name === 'liftStart');
    if (pickup && members.some(e => e.name === 'motionState')) result.push({ event: pickup, members });
    else for (const event of members) result.push({ event, members: [event] });
    i += members.length;
  }
  return result;
}
