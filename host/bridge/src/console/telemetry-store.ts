import { describeEvent, eventName, type EventText } from './events.js';
import { recordWheels, type WheelSample } from './wheel-history.js';
import { compactTrail } from './course-retention.js';
import { LINE_SIGNALS_MAX, type LineSignalSample } from './line-signals.js';
import type { LinkPacket } from './link.js';
import { odometerDiscontinuous } from './odometer.js';
import { hasSpeedFault, headingDegrees, pivotScalePercent } from './schema.js';
import { profileSurfaces, stationaryContact, type Survey } from './scan.js';
import { BIT_FIELDS, COMMANDS, ENUMS, FAULT_BITS } from './schema.generated.js';

/** Keep up to 20 minutes at full resolution, then simplify the older geometry. */
export const TRAIL_MAX = 6000;
export const EVENTS_MAX = 500;
/** Larger gaps are ambiguous and excluded from frame-loss statistics. */
const SEQ_JUMP = 1000;
const SEQ_WRAP = 65536;
const MOTION_SEQ_WRAP = 256;
// Five 5 Hz passes. Receipt time only guards pairing across low-byte reuse.
const MOTION_PAIR_PASSES = 5;
const MOTION_PAIR_AGE_MS = 1000;
// MQTT packets in one bridge receipt batch can arrive a few milliseconds apart.
const STATUS_BATCH_SKEW_MS = 5;
// The supervisor emits BOOT after its 2 s self-test, and the connection
// publishes its heartbeat each second before draining queued events.
const BOOT_EVENT_PAIR_MS = 3000;
export const HEARTBEAT_TIMER_WRAP_MS = 2 ** 32;
const MS_PER_SECOND = 1000;
const HUG_IDLE = 0;
const HUG_CONTACT = 1;
// Bound the range of raw near echoes displayed on the course trace.
const NEAR_RETURN_PLOT_MM = 250;
const SONAR_POSE_MAX_AGE_MS = 400;
const HUG_POSE_MAX_AGE_MS = 200;
// Obstacle reports arrive at 2 Hz. A long silence can conceal a different scan.
const SCAN_REPORT_MAX_AGE_MS = 1500;
const USEC_PER_SEC = 1_000_000;
const MAX_DIAG_INTERVAL_US = 30 * USEC_PER_SEC;
const DIAG_TIMER_WRAP_MS = 2 ** 32 / 1000;
const MIN_DIAG_WRAP_GAP_MS = DIAG_TIMER_WRAP_MS - MAX_DIAG_INTERVAL_US / 1000;
// The 5 ms IR sample can report two edges. Allow for read skew around the timer sample.
const MAX_SENSOR_RATE_HZ = 500;
const DIAG_RATE_FIELDS = {
  line_edges_per_s: 'line_edges_total',
  line_raw_edges_per_s: 'line_raw_edges_total',
  echo_valid: 'echo_total_0',
  echo_open: 'echo_total_1',
  echo_too_close: 'echo_total_2',
  echo_no_echo: 'echo_total_3',
  echo_glitch: 'echo_total_4',
  echo_stuck: 'echo_total_5',
} as const;

export type DiagRates = Partial<Record<keyof typeof DIAG_RATE_FIELDS, number>>;

export type TrailPoint = {
  timestamp: number;
  odometerMm: number;
  /** Compass heading in degrees, null while invalid. */
  headingDeg: number | null;
  /** Signed measured speed in mm/s, null when the packet omits it. */
  speedMms: number | null;
  /** Invalid feedback can appear as zero in a status packet. */
  speedFault?: true;
  lineBits?: number;
  pitchDeg?: number;
  rollDeg?: number;
  humpState?: number;
  seq?: number;
  wheels?: { leftMms: number; rightMms: number };
  /** Measured body yaw as a percentage of encoder yaw during a pivot. */
  pivotScalePct?: number;
  /** Recorded surfaces in the axle frame of this sample. */
  surveys?: Survey[];
  /** World pose retained when earlier raw samples are simplified. */
  checkpoint?: {
    x: number;
    y: number;
    headingDeg: number | null;
    reference: 'compass' | 'relative' | 'unavailable';
    hasGap: boolean;
  };
};

type PendingWheels = { seq: number; timestamp: number; wheels: NonNullable<TrailPoint['wheels']> };

/** The obstacle a hug found: the heading of the contact and the odometer where its face first showed. */
export type ObstacleStamp = {
  /** The exact trail sample, since different positions can share a timestamp and odometer. */
  sample: TrailPoint;
  headingSample?: TrailPoint;
  /** The encoder yaw is relative to contact and positive left; its receipt must be paired with the course pose. */
  bodyYaw?: { timestamp: number; deci: number };
  faceTime?: number;
  headingDeg: number | null;
  /** Axle to near face, the first non-zero face of the hug; 0 until seen, and the odometer is stamped again when it is. */
  faceMm: number;
  widthMm: number;
  breadthMm: number;
};

export type ForwardReturnStamp = {
  rangeMm: number;
  pose: TrailPoint | null;
};

export type EventRow = {
  eventId: number;
  timestamp: number;
  receivedAt?: number;
  type: number;
  name: string;
  source: number;
  p1: number;
  p2: number;
  odometerMm: number;
  text: EventText;
};

export type PitchSample = { at: number; pitchDeg: number; humpActive: boolean };
export const PITCH_HISTORY_MAX = 60;

export type TelemetryStore = {
  /** The newest packet of each kind, by packet name. */
  latest: Record<string, LinkPacket>;
  /** First bridge receipt of the sonar measurement in the current obstacle packet. */
  obstaclePing: LinkPacket | undefined;
  scanStart: TrailPoint | null;
  diagRates: DiagRates;
  /** Recent accepted pin snapshots, including when the car is stationary. */
  lineSignals: LineSignalSample[];
  barcodeEdges: LinkPacket[];
  pitchHistory: PitchSample[];
  trail: TrailPoint[];
  pendingWheels: PendingWheels[];
  /** Packet time of the last restart; events before it belong to the run before. Null until one. */
  trailStart: number | null;
  resetAt: number | null;
  resetReceiptId?: string;
  resetOrder?: number;
  /** Receipt of this boot's startup event. The heartbeat can establish the boot first. */
  bootEventAt: number | null;
  /** Kept after the hug ends, until the trail restarts. */
  obstacle: ObstacleStamp | null;
  /** The most recent near, centered return in the recorded run. */
  forwardReturn: ForwardReturnStamp | null;
  /** Oldest first. */
  events: EventRow[];
  wheelHistory: WheelSample[];
  statusReceived: number;
  statusLost: number;
  lastSeq: number | null;
  firstTimestamp: number | null;
};

export function emptyStore(): TelemetryStore {
  return {
    latest: {},
    obstaclePing: undefined,
    scanStart: null,
    diagRates: {},
    lineSignals: [],
    barcodeEdges: [],
    pitchHistory: [],
    trail: [],
    pendingWheels: [],
    trailStart: null,
    resetAt: null,
    bootEventAt: null,
    obstacle: null,
    forwardReturn: null,
    events: [],
    wheelHistory: [],
    statusReceived: 0,
    statusLost: 0,
    lastSeq: null,
    firstTimestamp: null,
  };
}

const cap = <T>(list: T[], max: number): T[] => (list.length > max ? list.slice(list.length - max) : list);

function restartReadings(store: TelemetryStore, first: LinkPacket): TelemetryStore {
  return {
    ...store,
    latest: { [first.packet]: first },
    obstaclePing: undefined,
    scanStart: null,
    diagRates: {},
    lineSignals: [],
    barcodeEdges: [],
    pitchHistory: [],
    trail: [],
    pendingWheels: [],
    trailStart: first.timestamp,
    bootEventAt: null,
    obstacle: null,
    forwardReturn: null,
    wheelHistory: [],
    lastSeq: null,
  };
}

const sequenceGap = (newer: number, older: number, wrap = SEQ_WRAP) => (newer - older + wrap) % wrap;
const sameMotionPass = (seq: number, timestamp: number, pending: PendingWheels) =>
  seq % MOTION_SEQ_WRAP === pending.seq && Math.abs(timestamp - pending.timestamp) <= MOTION_PAIR_AGE_MS;

function replaceTrailSample(store: TelemetryStore, index: number, point: TrailPoint): TelemetryStore {
  const sample = store.trail[index]!;
  const trail = store.trail.slice();
  trail[index] = point;
  const o = store.obstacle;
  const p = store.forwardReturn;
  return {
    ...store, trail,
    scanStart: store.scanStart === sample ? point : store.scanStart,
    obstacle: o !== null && (o.sample === sample || o.headingSample === sample) ? {
      ...o, sample: o.sample === sample ? point : o.sample,
      headingSample: o.headingSample === sample ? point : o.headingSample,
    } : o,
    forwardReturn: p?.pose === sample ? { ...p, pose: point } : p,
  };
}

function addMotion(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const { seq, left_mms: leftMms, right_mms: rightMms, motion_fault: fault } = packet.values;
  if (seq === undefined || !Number.isInteger(seq) || seq < 0 || seq >= MOTION_SEQ_WRAP ||
      leftMms === undefined || rightMms === undefined || !Number.isFinite(leftMms) || !Number.isFinite(rightMms) ||
      (fault !== undefined && fault !== 0)) return store;

  const pending = { seq, timestamp: packet.timestamp, wheels: { leftMms, rightMms } };
  const start = Math.max(0, store.trail.length - MOTION_PAIR_PASSES);
  const offset = store.trail.slice(start).findLastIndex((point) =>
    point.seq !== undefined && sameMotionPass(point.seq, point.timestamp, pending));
  const i = offset < 0 ? -1 : start + offset;
  const sample = store.trail[i];
  if (sample !== undefined) {
    if (sample.wheels !== undefined) return store;
    return replaceTrailSample(store, i, { ...sample, wheels: pending.wheels });
  }
  // Allow motion before status for nearby passes, including the low-byte wrap.
  const ahead = store.lastSeq === null ? 1 : sequenceGap(seq, store.lastSeq % MOTION_SEQ_WRAP, MOTION_SEQ_WRAP);
  const delayedBatch = ahead >= MOTION_SEQ_WRAP - MOTION_PAIR_PASSES + 1 &&
    Math.abs(packet.timestamp - (store.trail.at(-1)?.timestamp ?? -Infinity)) <= STATUS_BATCH_SKEW_MS;
  if (ahead === 0 || (ahead > MOTION_PAIR_PASSES && !delayedBatch)) return store;
  const fresh = store.pendingWheels.filter((p) => Math.abs(p.timestamp - pending.timestamp) <= MOTION_PAIR_AGE_MS);
  if (fresh.some((p) => p.seq === pending.seq)) return store;
  return { ...store, pendingWheels: cap([...fresh, pending], MOTION_PAIR_PASSES) };
}

function addStatus(current: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const v = packet.values;
  const seq = v.seq;
  const latest = current.latest.status;
  const latestSeq = latest?.values.seq;
  const last = current.trail.at(-1);
  const receiptDelta = packet.timestamp - (latest?.timestamp ?? -Infinity);
  if (receiptDelta < -STATUS_BATCH_SKEW_MS) return current;
  let insertAt = current.trail.length;
  if (seq !== undefined && latestSeq !== undefined) {
    const back = sequenceGap(latestSeq, seq);
    if (back === 0) return current;
    if (back > 0 && back <= SEQ_WRAP / 2) {
      if (receiptDelta > STATUS_BATCH_SKEW_MS || back >= MOTION_PAIR_PASSES || last?.seq !== latestSeq ||
          v.odometer_mm === undefined || odometerDiscontinuous(v.odometer_mm, last.odometerMm)) return current;
      // Keep sequence order inside a batch and leave existing stamped samples intact.
      while (insertAt > 0) {
        const before = current.trail[insertAt - 1]!;
        if (Math.abs(before.timestamp - packet.timestamp) > STATUS_BATCH_SKEW_MS || before.seq === undefined) break;
        const distance = sequenceGap(before.seq, seq);
        if (distance === 0) return current;
        if (distance >= MOTION_PAIR_PASSES) break;
        insertAt -= 1;
      }
    }
  }
  const reordered = insertAt < current.trail.length;
  if (!reordered && receiptDelta < 0) return current;
  let lost = current.statusLost;
  if (reordered && insertAt > 0 && seq !== undefined) {
    const before = current.trail[insertAt - 1]?.seq;
    const after = current.trail[insertAt]?.seq;
    if (before !== undefined && after !== undefined) {
      const interval = sequenceGap(after, before);
      const repaired = sequenceGap(seq, before);
      if (repaired > 0 && repaired < interval && interval <= SEQ_JUMP) lost = Math.max(0, lost - 1);
    }
  } else if (!reordered && current.lastSeq !== null && seq !== undefined) {
    const gap = sequenceGap(seq, current.lastSeq) - 1;
    if (gap > 0 && gap < SEQ_JUMP) lost += gap;
  }
  let trail = current.trail;
  let pendingWheels = current.pendingWheels;
  if (v.odometer_mm !== undefined) {
    const phys = current.latest.physics;
    const physFresh = phys !== undefined && Math.abs(packet.timestamp - phys.timestamp) <= 1000;
    const physCal = physFresh && ((phys.values.flags ?? 0) & (1 << BIT_FIELDS.physicsFlags.calibrated)) !== 0;
    const pitchDeg = physCal && phys.values.pitch_cdeg !== undefined ? Math.round(phys.values.pitch_cdeg / 10) / 10 : undefined;
    const rollDeg = physCal && phys.values.roll_cdeg !== undefined ? Math.round(phys.values.roll_cdeg / 10) / 10 : undefined;
    const humpState = physFresh ? phys.values.hump_state : undefined;
    const pivotScalePct = pivotScalePercent(v.pivot_scale_pct);
    const point: TrailPoint = {
      timestamp: reordered ? current.trail[insertAt]!.timestamp : packet.timestamp,
      odometerMm: v.odometer_mm,
      headingDeg: headingDegrees(v.heading_ddeg),
      speedMms: hasSpeedFault(v.faults) ? null : v.speed_mms ?? null,
      ...(hasSpeedFault(v.faults) && { speedFault: true }),
      ...(v.line_bits !== undefined && { lineBits: v.line_bits }),
      ...(pitchDeg !== undefined && { pitchDeg }),
      ...(rollDeg !== undefined && { rollDeg }),
      ...(humpState !== undefined && { humpState }),
      ...(pivotScalePct !== undefined && { pivotScalePct }),
      seq,
    };
    const pending = seq === undefined ? undefined : pendingWheels.find((p) => sameMotionPass(seq, packet.timestamp, p));
    if (pending !== undefined) {
      point.wheels = pending.wheels;
      pendingWheels = pendingWheels.filter((p) => p !== pending);
    }
    trail = reordered ? [...trail.slice(0, insertAt), point, ...trail.slice(insertAt)] : [...trail, point];
  }
  let scanStart = current.scanStart;
  // The 2 Hz scan notice can land between the final deceleration reading and
  // the first stationary 5 Hz pose. Confirm the axle before a body pivot starts.
  if (!reordered && scanStart !== null && scanStart.speedMms !== 0) {
    const point = trail.at(-1);
    const headingChange = point?.headingDeg == null || scanStart.headingDeg === null ? Infinity
      : Math.abs(((point.headingDeg - scanStart.headingDeg + 540) % 360) - 180);
    scanStart = point?.speedMms === 0 && point.timestamp - scanStart.timestamp <= 400 &&
      point.odometerMm === scanStart.odometerMm && headingChange <= 2 ? point : null;
  }
  // A late batch member belongs before the next accepted sensor sample.
  // The signal window is independent of the compacted odometry trail.
  const signals = current.lineSignals;
  let signalAt = signals.length;
  if (reordered && seq !== undefined) {
    while (signalAt > 0) {
      const sample = signals[signalAt - 1]!;
      if (sample.seq === undefined || Math.abs(sample.timestamp - packet.timestamp) > STATUS_BATCH_SKEW_MS ||
          sequenceGap(sample.seq, seq) >= MOTION_PAIR_PASSES) break;
      signalAt--;
    }
  }
  const signal = { timestamp: signals[signalAt]?.timestamp ?? packet.timestamp, seq, bits: v.line_bits ?? null };
  const next = {
    ...current,
    latest: reordered ? current.latest : { ...current.latest, status: packet },
    firstTimestamp: current.firstTimestamp ?? packet.timestamp,
    lineSignals: cap(reordered ? [...signals.slice(0, signalAt), signal, ...signals.slice(signalAt)]
      : [...signals, signal], LINE_SIGNALS_MAX),
    trail,
    scanStart,
    pendingWheels,
    statusReceived: current.statusReceived + 1,
    statusLost: lost,
    lastSeq: reordered ? current.lastSeq : seq ?? current.lastSeq,
  };
  // A CONTACT and the first status of its pass can be drained in either order.
  const contact = current.latest.obstacle;
  const anchored = next.obstacle === null && contact?.values.hug_state === HUG_CONTACT && contact.values.body_deci !== undefined &&
      packet.timestamp >= contact.timestamp && packet.timestamp - contact.timestamp <= STATUS_BATCH_SKEW_MS
    ? addObstacle(next, contact, undefined, false) : next;
  return trail.length > TRAIL_MAX ? compactTrail(anchored) : anchored;
}

export function acceptedReset(packet: LinkPacket): boolean {
  const {event_type, source, p1} = packet.values;
  return packet.packet === 'event' && event_type === ENUMS.eventType.cmdAccept &&
    source === ENUMS.eventSource.supervisor && p1 !== undefined && (p1 & 0xff) === COMMANDS.reset;
}

export function precedesReset(packet: LinkPacket, timestamp: number | null, order?: number): boolean {
  return order !== undefined && packet.receiptOrder !== undefined
    ? packet.receiptOrder < order : packet.timestamp < (timestamp ?? -Infinity);
}

function isCurrentReset(store: TelemetryStore, packet: LinkPacket): boolean {
  return store.resetAt === packet.timestamp && store.resetReceiptId === packet.receiptId &&
    store.resetOrder === packet.receiptOrder;
}

function addEvent(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const v = packet.values;
  const type = v.event_type ?? 0;
  const eventId = v.event_id ?? 0;
  const recent = store.events.slice(-20);
  // History and the live feed can carry the same bridge receipt.
  if ((!acceptedReset(packet) || isCurrentReset(store, packet)) &&
      recent.some((row) => row.eventId === eventId && row.type === type && row.timestamp === packet.timestamp)) {
    return store;
  }
  const row: EventRow = {
    eventId,
    timestamp: packet.timestamp,
    ...(packet.receivedAt !== undefined && { receivedAt: packet.receivedAt }),
    type,
    name: eventName(type),
    source: v.source ?? 0,
    p1: v.p1 ?? 0,
    p2: v.p2 ?? 0,
    odometerMm: v.odometer_mm ?? 0,
    text: describeEvent(type, v.p1 ?? 0, v.p2 ?? 0),
  };
  if (acceptedReset(packet)) {
    // Replaying the acknowledgement at a persisted reset boundary may follow
    // packets with the same bridge timestamp. Those already belong to this session.
    if (isCurrentReset(store, packet)) return { ...store, events: [row, ...store.events] };
    const heartbeat = store.latest.heartbeat;
    return { ...emptyStore(), latest: { ...(heartbeat && { heartbeat }), event: packet }, events: [row],
      resetAt: packet.timestamp, resetReceiptId: packet.receiptId, resetOrder: packet.receiptOrder,
      trailStart: packet.timestamp, firstTimestamp: packet.timestamp };
  }
  if (row.name !== 'boot') return { ...store, events: cap([...store.events, row], EVENTS_MAX) };
  // Firmware publishes its heartbeat before draining queued events. Readings
  // after a heartbeat-confirmed reboot already belong to that boot.
  const heartbeatReason = store.latest.heartbeat?.values.reset_reason;
  if (store.trailStart !== null && store.bootEventAt === null
      && packet.timestamp - store.trailStart <= BOOT_EVENT_PAIR_MS
      && (heartbeatReason === undefined || v.p1 === undefined || heartbeatReason === v.p1)) {
    const sinceBoot = store.trailStart;
    return { ...store, bootEventAt: packet.timestamp,
      events: cap([...store.events.filter((event) => event.timestamp >= sinceBoot), row], EVENTS_MAX) };
  }
  return { ...restartReadings(store, packet), bootEventAt: packet.timestamp, events: [row] };
}

function addDiag(store: TelemetryStore, current: LinkPacket, previous: LinkPacket | undefined): TelemetryStore {
  const sample = current.values.sensor_sample_us;
  const oldSample = previous?.values.sensor_sample_us;
  const fullWrapPossible = previous !== undefined && current.timestamp - previous.timestamp >= DIAG_TIMER_WRAP_MS;
  if (sample !== undefined && sample === oldSample && !fullWrapPossible) return store;

  // The timer and totals are unsigned 32-bit. Use the device interval so a
  // delayed batch or a lost frame does not distort the readings.
  const interval = sample === undefined || oldSample === undefined ? 0 : (sample - oldSample) >>> 0;
  const usable = !fullWrapPossible && interval > 0 && interval <= MAX_DIAG_INTERVAL_US;
  const diagRates: DiagRates = {};
  for (const rate of Object.keys(DIAG_RATE_FIELDS) as (keyof DiagRates)[]) {
    if (sample === undefined) {
      diagRates[rate] = current.values[rate];
    } else if (usable) {
      const field = DIAG_RATE_FIELDS[rate];
      const now = current.values[field];
      const before = previous?.values[field];
      if (now !== undefined && before !== undefined) {
        const count = (now - before) >>> 0;
        const ratePerSecond = count * USEC_PER_SEC / interval;
        if (ratePerSecond <= MAX_SENSOR_RATE_HZ) diagRates[rate] = Math.round(ratePerSecond);
      }
    }
  }
  return { ...store, diagRates };
}

function sonarPoseIndex(trail: TrailPoint[], timestamp: number, maxAgeMs: number): number {
  const i = trail.findLastIndex((point) => point.timestamp <= timestamp);
  const age = timestamp - (trail[i]?.timestamp ?? -Infinity);
  return age <= maxAgeMs ? i : -1;
}

function recentSonarPoseValues(packet: LinkPacket | undefined, timestamp: number): LinkPacket['values'] | undefined {
  return packet !== undefined && packet.timestamp <= timestamp && timestamp - packet.timestamp <= SONAR_POSE_MAX_AGE_MS
    ? packet.values : undefined;
}

function airborneAt(store: TelemetryStore, timestamp: number): boolean {
  const physics = recentSonarPoseValues(store.latest.physics, timestamp);
  if (physics && ((physics.flags ?? 0) & (1 << BIT_FIELDS.physicsFlags.calibrated)) !== 0 &&
      ((physics.physics_faults ?? 0) & (1 << BIT_FIELDS.physicsFaults.i2c)) === 0) {
    const attitude = physics.attitude_state;
    if (attitude === ENUMS.attitudeState.lifted || attitude === ENUMS.attitudeState.overturned) return true;
    // The supervisor can keep the lift hold latched after the IMU sees set-down.
    if (attitude === ENUMS.attitudeState.ok) return false;
  }
  const status = recentSonarPoseValues(store.latest.status, timestamp);
  return status?.mode === ENUMS.vehicleMode.lifted || status?.mode === ENUMS.vehicleMode.overturned ||
    ((status?.faults ?? 0) & ((1 << FAULT_BITS.lifted) | (1 << FAULT_BITS.overturned))) !== 0;
}

function observeScanStart(store: TelemetryStore, packet: LinkPacket, previous?: LinkPacket): TrailPoint | null {
  const state = packet.values.state;
  const states = ENUMS.obstacleState;
  if (!previous || packet.timestamp - previous.timestamp > SCAN_REPORT_MAX_AGE_MS) return null;
  if (state === states.plan || (state === states.scanning && previous.values.state === states.scanning)) {
    if (store.scanStart?.speedMms !== 0 && (state !== states.scanning || packet.values.body_deci !== 0)) return null;
    return store.scanStart;
  }
  if (state !== states.scanning || packet.values.body_deci !== 0 ||
      (previous?.values.state !== states.waitStop && previous?.values.state !== states.monitor)) return null;
  const sample = store.trail[sonarPoseIndex(store.trail, packet.timestamp, HUG_POSE_MAX_AGE_MS)];
  return sample?.speedMms == null ? null : sample;
}

function recordProfile(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  if (airborneAt(store, packet.timestamp)) return { ...store, scanStart: null };
  const contact = stationaryContact(packet.values);
  const sample = contact ? store.trail[sonarPoseIndex(store.trail, packet.timestamp, HUG_POSE_MAX_AGE_MS)] : store.scanStart;
  if (contact && (!sample || sample.speedFault || (sample.wheels &&
      (sample.wheels.leftMms !== 0 || sample.wheels.rightMms !== 0)))) return store;
  if (!sample || packet.timestamp < sample.timestamp) return store;
  if (sample.speedMms !== 0) return { ...store, scanStart: null };
  const i = store.trail.indexOf(sample);
  const age = packet.timestamp - (store.latest.obstacle?.timestamp ?? -Infinity);
  if (i < 0 || (!contact && (age < 0 || age > SCAN_REPORT_MAX_AGE_MS)) ||
      store.trail.slice(i + 1).some((p) => p.timestamp <= packet.timestamp && Math.abs(p.odometerMm - sample.odometerMm) > 3)) {
    return { ...store, scanStart: null };
  }
  const surfaces = profileSurfaces(packet.values);
  if (!surfaces.length) return { ...store, scanStart: null };
  const survey = { timestamp: packet.timestamp, request: packet.values.req_id ?? -1, surfaces };
  const next = replaceTrailSample(store, i, { ...sample, surveys: [...sample.surveys ?? [], survey] });
  return { ...next, scanStart: null };
}

function observeObstaclePing(previous: LinkPacket | undefined, packet: LinkPacket): LinkPacket | undefined {
  const { seq, dist_mm: range, echo_class: echo, servo_deci: angle } = packet.values;
  if (range === undefined || !Number.isFinite(range) || range < 0 ||
      angle === undefined || !Number.isFinite(angle) || echo === undefined ||
      !Number.isInteger(echo) || echo < ENUMS.echoClass.valid || echo > ENUMS.echoClass.stuck ||
      (range === 0 && echo === ENUMS.echoClass.valid)) return undefined;
  // The empty firmware snapshot has the same shape as NO_ECHO at sequence wrap.
  if (previous === undefined && seq === 0 && range === 0 && angle === 0 && echo === ENUMS.echoClass.noEcho) return undefined;
  const before = previous?.values;
  // Requested and free-running pings use separate sequence spaces. Legacy
  // records without a sequence can prove new data only through a changed value.
  const sameSequence = seq === undefined || before?.seq === undefined || seq === before.seq;
  return sameSequence && before?.dist_mm === range && before.echo_class === echo && before.servo_deci === angle
    ? previous : packet;
}

function addForwardReturn(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  const { dist_mm: rangeMm, echo_class: echo, servo_deci: angle, state } = packet.values;
  if (echo !== ENUMS.echoClass.valid || angle !== 0 || !Number.isFinite(rangeMm) ||
      rangeMm === undefined || rangeMm <= 0 || rangeMm > NEAR_RETURN_PLOT_MM) return store;
  // Legacy scan frames can carry a ranged echo while the pilot's hug is idle.
  if (state !== undefined && state !== ENUMS.obstacleState.stopped && state !== ENUMS.obstacleState.monitor) return store;
  const pose = store.trail[sonarPoseIndex(store.trail, packet.timestamp, SONAR_POSE_MAX_AGE_MS)] ?? null;
  return { ...store, forwardReturn: { rangeMm, pose } };
}

/**
 * A hug is new when it leaves IDLE or returns to CONTACT from a later state (an
 * IDLE frame can be lost). Store the contact-relative body yaw separately from
 * the first face reading. The car can pivot before the first CONTACT receipt
 * and back off before measuring the axle-to-face distance. Their receipt times
 * let the course builder locate each observation between adjacent status poses.
 * Later receipts add measured dimensions. IDLE keeps the last measured box.
 */
function addObstacle(store: TelemetryStore, packet: LinkPacket, previous: number | undefined, newPing: boolean): TelemetryStore {
  const v = packet.values;
  const state = v.hug_state ?? HUG_IDLE;
  if (state === HUG_IDLE) return newPing ? addForwardReturn(store, packet) : store;
  const face = v.face_mm ?? 0;
  const widthMm = v.width_mm ?? 0;
  const breadthMm = v.breadth_mm ?? 0;
  if (previous === HUG_IDLE || (state === HUG_CONTACT && previous !== HUG_CONTACT)) {
    const deci = v.body_deci;
    if (deci !== undefined && (!Number.isInteger(deci) || Math.abs(deci) > 1800)) return { ...store, obstacle: null };
    const bodyYaw = deci === undefined ? undefined : { timestamp: packet.timestamp, deci };
    const i = hugPoseIndex(store.trail, packet.timestamp, bodyYaw !== undefined);
    const sample = store.trail[i];
    if (sample === undefined) return { ...store, obstacle: null };
    const headingSample = bodyYaw === undefined ? contactHeadingSample(store.trail, i, packet.timestamp) : sample;
    return {
      ...store,
      obstacle: { sample, headingSample, headingDeg: headingSample?.headingDeg ?? null, bodyYaw,
        faceTime: bodyYaw !== undefined && face > 0 ? packet.timestamp : undefined, faceMm: face, widthMm, breadthMm },
    };
  }
  const o = store.obstacle;
  if (o === null) return store;
  const faceMm = o.faceMm === 0 ? face : o.faceMm;
  const next = { faceMm, widthMm: widthMm || o.widthMm, breadthMm: breadthMm || o.breadthMm };
  if (next.faceMm === o.faceMm && next.widthMm === o.widthMm && next.breadthMm === o.breadthMm) return store;
  const seen = o.faceMm === 0 && faceMm !== 0;
  const sample = seen ? store.trail[hugPoseIndex(store.trail, packet.timestamp, o.bodyYaw !== undefined)] : o.sample;
  if (sample === undefined || sample.timestamp < o.sample.timestamp) return { ...store, obstacle: null };
  return {
    ...store,
    obstacle: { ...o, sample, faceTime: seen && o.bodyYaw !== undefined ? packet.timestamp : o.faceTime, ...next },
  };
}

function hugPoseIndex(trail: TrailPoint[], timestamp: number, paired: boolean): number {
  const before = sonarPoseIndex(trail, timestamp, HUG_POSE_MAX_AGE_MS);
  if (before >= 0 || !paired) return before;
  return trail.findIndex((p) => p.timestamp > timestamp && p.timestamp - timestamp <= STATUS_BATCH_SKEW_MS);
}

function contactHeadingSample(trail: TrailPoint[], poseIndex: number, timestamp: number): TrailPoint | undefined {
  for (let i = poseIndex; i >= 0; i -= 1) {
    const sample = trail[i];
    if (sample === undefined || timestamp - sample.timestamp > HUG_POSE_MAX_AGE_MS) break;
    if (sample.timestamp <= timestamp && sample.headingDeg !== null) return sample;
  }
  return undefined;
}

/** The heartbeat identifies a new firmware boot even when its boot event was lost. */
function addHeartbeat(store: TelemetryStore, packet: LinkPacket, previous: LinkPacket | undefined): TelemetryStore {
  if (previous === undefined) return store;
  const runId = packet.values.run_id;
  const uptime = packet.values.uptime_s;
  const previousUptime = previous.values.uptime_s;
  // Firmware reports whole seconds from a wrapping 32-bit millisecond clock.
  // A rollback is evidence of a reboot when even the earliest wrap is too late.
  const rollback = uptime !== undefined && previousUptime !== undefined && uptime < previousUptime &&
    packet.timestamp - previous.timestamp < HEARTBEAT_TIMER_WRAP_MS - (previousUptime - uptime + 1) * MS_PER_SECOND;
  const rebooted =
    (runId !== undefined && previous.values.run_id !== undefined && runId !== previous.values.run_id) || rollback;
  return rebooted ? restartReadings(store, packet) : store;
}

/** Fold one packet into the store. Returns the same object for a duplicate event. */
export function reduceStore(store: TelemetryStore, packet: LinkPacket): TelemetryStore {
  if (acceptedReset(packet) && isCurrentReset(store, packet) &&
      store.events.some(e => e.timestamp === packet.timestamp && e.eventId === packet.values.event_id)) return store;
  if (precedesReset(packet, store.resetAt, store.resetOrder)) return store;
  if (packet.timestamp < (store.trailStart ?? -Infinity)) return store;
  if (packet.packet === 'event' && packet.values.event_type === ENUMS.eventType.boot &&
      packet.timestamp <= (store.bootEventAt ?? -Infinity)) return store;
  if (packet.packet === 'status') return addStatus(store, packet);
  if (packet.packet === 'profile' && packet.timestamp < (store.latest.profile?.timestamp ?? -Infinity)) return store;
  const previousObstacle = store.latest.obstacle;
  const previousHeartbeat = store.latest.heartbeat;
  if (packet.packet === 'heartbeat' && previousHeartbeat !== undefined && packet.timestamp < previousHeartbeat.timestamp) return store;
  const previousDiag = store.latest.diag;
  if (packet.packet === 'obstacle' && previousObstacle !== undefined && packet.timestamp < previousObstacle.timestamp) return store;
  if (packet.packet === 'diag' && previousDiag !== undefined) {
    const sample = packet.values.sensor_sample_us;
    const oldSample = previousDiag.values.sensor_sample_us;
    const backwards = sample === undefined || oldSample === undefined ? 0 : (oldSample - sample) >>> 0;
    // A small backwards device step is delayed data unless enough wall time
    // passed for an almost complete timer wrap.
    if (backwards > 0 && backwards <= MAX_DIAG_INTERVAL_US &&
        packet.timestamp - previousDiag.timestamp < MIN_DIAG_WRAP_GAP_MS) return store;
  }
  let next: TelemetryStore = {
    ...store,
    latest: { ...store.latest, [packet.packet]: packet },
    firstTimestamp: store.firstTimestamp ?? packet.timestamp,
  };
  if (packet.packet === 'obstacle') {
    next.obstaclePing = observeObstaclePing(store.obstaclePing, packet);
    if (airborneAt(store, packet.timestamp)) {
      next.scanStart = null;
    } else {
      next.scanStart = observeScanStart(store, packet, previousObstacle);
      const newPing = next.obstaclePing !== undefined && next.obstaclePing !== store.obstaclePing;
      next = addObstacle(next, packet, previousObstacle?.values.hug_state, newPing);
    }
  }
  if (packet.packet === 'barcode_sample') {
    const previous = store.barcodeEdges.at(-1);
    if (previous && (previous.timestamp > packet.timestamp ||
        (previous.values.edge_us === packet.values.edge_us && previous.values.seq === packet.values.seq))) return store;
    next.barcodeEdges = cap([...store.barcodeEdges, packet], 2048);
  }
  if (packet.packet === 'profile') next = recordProfile(next, packet);
  if (packet.packet === 'motion') {
    next.wheelHistory = recordWheels(store.wheelHistory, packet);
    if (packet.timestamp < (store.latest.motion?.timestamp ?? -Infinity)) next.latest.motion = store.latest.motion!;
    next = addMotion(next, packet);
  }
  if (packet.packet === 'event') next = addEvent(next, packet);
  if (packet.packet === 'heartbeat') next = addHeartbeat(next, packet, previousHeartbeat);
  if (packet.packet === 'diag') next = addDiag(next, packet, previousDiag);
  if (packet.packet === 'physics') {
    const pitchCdeg = packet.values.pitch_cdeg;
    const calibrated = ((packet.values.flags ?? 0) & 8) !== 0;
    if (calibrated && pitchCdeg !== undefined && Number.isFinite(pitchCdeg)) {
      const humpActive = (packet.values.hump_state ?? 0) !== 0;
      const lastPitch = store.pitchHistory.at(-1);
      if (!lastPitch || packet.timestamp >= lastPitch.at) {
        next.pitchHistory = cap([...store.pitchHistory, { at: packet.timestamp, pitchDeg: pitchCdeg / 100, humpActive }], PITCH_HISTORY_MAX);
      }
    }
  }
  return next;
}

/** Percent of status frames lost, from gaps in `seq`. null before 2 frames. */
export function statusLossPercent(store: TelemetryStore): number | null {
  const total = store.statusReceived + store.statusLost;
  if (store.statusReceived < 2 || total === 0) return null;
  return (store.statusLost / total) * 100;
}
