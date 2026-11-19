import { describeEvent, eventName, type EventText } from './events';
import type { LinkPacket } from './link';
import { headingDegrees } from './schema';

/** 20 minutes of status frames at 5 Hz. */
export const TRAIL_MAX = 6000;
export const EVENTS_MAX = 500;
export const SPEEDS_MAX = 150;
/** A jump up in the odometer larger than this is a reset, not travel (mm). */
const ODOMETER_JUMP_MM = 5000;
/**
 * A drop in the odometer larger than this is a reset, not a reverse (mm). The
 * longest reverse is the hug back-off, the standoff of 250 mm; the line search
 * reverses 150 mm and the pilot back-off 40 mm. This is twice the longest.
 */
const ODOMETER_DROP_MM = 500;
/** A gap in `seq` larger than this is a reboot, not loss. */
const SEQ_JUMP = 1000;
const SEQ_WRAP = 65536;
const HUG_IDLE = 0;
const HUG_CONTACT = 1;

export type TrailPoint = {
  timestamp: number;
  odometerMm: number;
  /** Compass heading in degrees, null while invalid. */
  headingDeg: number | null;
  speedMms: number;
};

/** The obstacle a hug found: the heading of the contact and the odometer where its face first showed. */
export type ObstacleStamp = {
  /** Packet time of the trail point at `odometerMm`; the odometer falls in a reverse, so it alone can name two places. */
  timestamp: number;
  odometerMm: number;
  headingDeg: number | null;
  /** Axle to near face, the first non-zero face of the hug; 0 until seen, and the odometer is stamped again when it is. */
  faceMm: number;
  widthMm: number;
  breadthMm: number;
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
  /** Packet time of the last restart; events before it belong to the run before. Null until one. */
  trailStart: number | null;
  /** Kept after the hug ends, until the trail restarts. */
  obstacle: ObstacleStamp | null;
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
    trailStart: null,
    obstacle: null,
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
  let obstacle = store.obstacle;
  let trailStart = store.trailStart;
  if (v.odometer_mm !== undefined) {
    const last = trail[trail.length - 1];
    // The odometer falls in a reverse and a pivot; only a drop no reverse reaches, or a huge jump, is a reboot.
    const restarted = last !== undefined &&
      (last.odometerMm - v.odometer_mm > ODOMETER_DROP_MM || v.odometer_mm - last.odometerMm > ODOMETER_JUMP_MM);
    const point: TrailPoint = {
      timestamp: packet.timestamp,
      odometerMm: v.odometer_mm,
      headingDeg: headingDegrees(v.heading_ddeg),
      speedMms: v.speed_mms ?? 0,
    };
    trail = cap(restarted ? [point] : [...trail, point], TRAIL_MAX);
    if (restarted) {
      obstacle = null;
      trailStart = packet.timestamp;
    }
  }
  return {
    ...store,
    trail,
    trailStart,
    obstacle,
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
    obstacle: fresh ? null : store.obstacle,
    events: cap(fresh ? [row] : [...store.events, row], EVENTS_MAX),
  };
}

/**
 * A hug is new when it leaves IDLE or returns to CONTACT from a later state (an
 * IDLE frame can be lost). The heading is the one of the contact; the odometer is
 * stamped again where the face first shows, because a near first contact or a stall
 * backs off before it reads a face. The pose is the newest status point at or
 * before the obstacle frame (up to 200 ms old). The size follows the frames while
 * the hug runs; an IDLE frame (all 0) keeps the last box. A hug whose start was not
 * seen (no trail point yet, or the trail restarted in it) stamps nothing until the
 * next one.
 */
function addObstacle(store: TelemetryStore, packet: LinkPacket, previous: number | undefined): TelemetryStore {
  const v = packet.values;
  const state = v.hug_state ?? HUG_IDLE;
  if (state === HUG_IDLE) return store;
  const last = store.trail[store.trail.length - 1];
  if (last === undefined) return store;
  const face = v.face_mm ?? 0;
  const widthMm = v.width_mm ?? 0;
  const breadthMm = v.breadth_mm ?? 0;
  if (previous === HUG_IDLE || (state === HUG_CONTACT && previous !== HUG_CONTACT)) {
    return {
      ...store,
      obstacle: { timestamp: last.timestamp, odometerMm: last.odometerMm, headingDeg: lastHeading(store.trail), faceMm: face, widthMm, breadthMm },
    };
  }
  const o = store.obstacle;
  if (o === null) return store;
  const faceMm = o.faceMm === 0 ? face : o.faceMm;
  const next = { faceMm, widthMm: widthMm || o.widthMm, breadthMm: breadthMm || o.breadthMm };
  if (next.faceMm === o.faceMm && next.widthMm === o.widthMm && next.breadthMm === o.breadthMm) return store;
  const seen = o.faceMm === 0 && faceMm !== 0;
  return {
    ...store,
    obstacle: {
      timestamp: seen ? last.timestamp : o.timestamp,
      odometerMm: seen ? last.odometerMm : o.odometerMm,
      headingDeg: o.headingDeg,
      ...next,
    },
  };
}

function lastHeading(trail: TrailPoint[]): number | null {
  for (let i = trail.length - 1; i >= 0; i -= 1) {
    const heading = trail[i]?.headingDeg ?? null;
    if (heading !== null) return heading;
  }
  return null;
}

/**
 * A new `run_id`, or an uptime that falls, is a reboot even when the boot event was
 * lost and the odometer restarted too low for the drop rule.
 */
function addHeartbeat(store: TelemetryStore, packet: LinkPacket, previous: LinkPacket | undefined): TelemetryStore {
  if (previous === undefined) return store;
  const runId = packet.values.run_id;
  const uptime = packet.values.uptime_s;
  const rebooted =
    (runId !== undefined && previous.values.run_id !== undefined && runId !== previous.values.run_id) ||
    (uptime !== undefined && previous.values.uptime_s !== undefined && uptime < previous.values.uptime_s);
  // An odometer restart since the last heartbeat already began this run's trail.
  const restartedSince = store.trailStart !== null && store.trailStart > previous.timestamp;
  return rebooted && !restartedSince ? { ...store, trail: [], trailStart: packet.timestamp, obstacle: null } : store;
}

/** Fold one packet into the store. Returns the same object for a duplicate event. */
export function reduceStore(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const previousHug = store.latest.obstacle?.values.hug_state;
  const previousHeartbeat = store.latest.heartbeat;
  let next: TelemetryStore = {
    ...store,
    latest: { ...store.latest, [packet.packet]: packet },
    firstTimestamp: store.firstTimestamp ?? packet.timestamp,
  };
  if (packet.packet === 'obstacle') next = addObstacle(next, packet, previousHug);
  if (packet.packet === 'status') next = addStatus(next, packet);
  if (packet.packet === 'event') next = addEvent(next, packet);
  if (packet.packet === 'heartbeat') next = addHeartbeat(next, packet, previousHeartbeat);
  return next;
}

/** Percent of status frames lost, from gaps in `seq`. null before 2 frames. */
export function statusLossPercent(store: TelemetryStore): number | null {
  const total = store.statusReceived + store.statusLost;
  if (store.statusReceived < 2 || total === 0) return null;
  return (store.statusLost / total) * 100;
}
