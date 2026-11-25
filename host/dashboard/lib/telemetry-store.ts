import { describeEvent, eventName, type EventText } from './events';
import type { LinkPacket } from './link';
import { headingDegrees } from './schema';

/** 20 minutes of status frames at 5 Hz. */
export const TRAIL_MAX = 6000;
export const EVENTS_MAX = 500;
export const SPEEDS_MAX = 150;
/** A jump in the odometer larger than this is a reset, not travel (mm). */
const ODOMETER_JUMP_MM = 5000;
/** A gap in `seq` larger than this is a reboot, not loss. */
const SEQ_JUMP = 1000;
const SEQ_WRAP = 65536;

export type TrailPoint = {
  timestamp: number;
  odometerMm: number;
  /** Compass heading in degrees, null while invalid. */
  headingDeg: number | null;
  speedMms: number;
};

export type EventRow = {
  eventId: number;
  timestamp: number;
  type: number;
  name: string;
  source: number;
  p1: number;
  p2: number;
  odometerMm: number;
  text: EventText;
};

export type TelemetryStore = {
  /** The newest packet of each kind, by packet name. */
  latest: Record<string, LinkPacket>;
  trail: TrailPoint[];
  /** Oldest first. */
  events: EventRow[];
  /** Newest last. */
  speeds: number[];
  statusReceived: number;
  statusLost: number;
  lastSeq: number | null;
  firstTimestamp: number | null;
};

export function emptyStore(): TelemetryStore {
  return {
    latest: {},
    trail: [],
    events: [],
    speeds: [],
    statusReceived: 0,
    statusLost: 0,
    lastSeq: null,
    firstTimestamp: null,
  };
}

const cap = <T>(list: T[], max: number): T[] => (list.length > max ? list.slice(list.length - max) : list);

function addStatus(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const v = packet.values;
  const seq = v.seq;
  let lost = store.statusLost;
  if (store.lastSeq !== null && seq !== undefined) {
    const gap = (seq - store.lastSeq - 1 + SEQ_WRAP) % SEQ_WRAP;
    if (gap > 0 && gap < SEQ_JUMP) lost += gap;
  }
  let trail = store.trail;
  if (v.odometer_mm !== undefined) {
    const last = trail[trail.length - 1];
    // A smaller odometer, or a huge jump, starts a new trail: the car rebooted.
    const restarted = last !== undefined &&
      (v.odometer_mm < last.odometerMm || v.odometer_mm - last.odometerMm > ODOMETER_JUMP_MM);
    const point: TrailPoint = {
      timestamp: packet.timestamp,
      odometerMm: v.odometer_mm,
      headingDeg: headingDegrees(v.heading_ddeg),
      speedMms: v.speed_mms ?? 0,
    };
    trail = cap(restarted ? [point] : [...trail, point], TRAIL_MAX);
  }
  return {
    ...store,
    trail,
    speeds: cap([...store.speeds, v.speed_mms ?? 0], SPEEDS_MAX),
    statusReceived: store.statusReceived + 1,
    statusLost: lost,
    lastSeq: seq ?? store.lastSeq,
  };
}

function addEvent(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const v = packet.values;
  const type = v.event_type ?? 0;
  const eventId = v.event_id ?? 0;
  const recent = store.events.slice(-20);
  // The event queue is QoS 1, so the broker can deliver one twice.
  if (recent.some((row) => row.eventId === eventId && row.type === type && row.timestamp === packet.timestamp)) {
    return store;
  }
  const row: EventRow = {
    eventId,
    timestamp: packet.timestamp,
    type,
    name: eventName(type),
    source: v.source ?? 0,
    p1: v.p1 ?? 0,
    p2: v.p2 ?? 0,
    odometerMm: v.odometer_mm ?? 0,
    text: describeEvent(type, v.p1 ?? 0, v.p2 ?? 0),
  };
  // A boot event means a new run: the old trail no longer joins the new one.
  const fresh = row.name === 'boot';
  return {
    ...store,
    trail: fresh ? [] : store.trail,
    events: cap(fresh ? [row] : [...store.events, row], EVENTS_MAX),
  };
}

/** Fold one packet into the store. Returns the same object for a duplicate event. */
export function reduceStore(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  let next: TelemetryStore = {
    ...store,
    latest: { ...store.latest, [packet.packet]: packet },
    firstTimestamp: store.firstTimestamp ?? packet.timestamp,
  };
  if (packet.packet === 'status') next = addStatus(next, packet);
  if (packet.packet === 'event') next = addEvent(next, packet);
  return next;
}

/** Percent of status frames lost, from gaps in `seq`. null before 2 frames. */
export function statusLossPercent(store: TelemetryStore): number | null {
  const total = store.statusReceived + store.statusLost;
  if (store.statusReceived < 2 || total === 0) return null;
  return (store.statusLost / total) * 100;
}
