import { compactTrail } from './course-retention.js';
import type { LinkPacket } from './link.js';
import { STALE_AFTER_MS } from './freshness.js';
import { SLOT_COUNT, type Survey } from './scan.js';
import { pivotScalePercent } from './schema.js';
import { HEARTBEAT_TIMER_WRAP_MS, TRAIL_MAX, emptyStore, acceptedReset, precedesReset, reduceStore, type ObstacleStamp, type TelemetryStore, type TrailPoint } from './telemetry-store.js';

export type StoragePort = {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
};
type SavedObstacle = Omit<ObstacleStamp, 'sample' | 'headingSample'> & { sample: number; headingSample?: number };
type Saved = { scope: string; runId: number; uptime: number; time: number; start: number | null;
  trail: TrailPoint[]; obstacle?: SavedObstacle };
type ResetBoundary = { timestamp: number; receiptId?: string; receiptOrder?: number };
const KEY = 'pico-car.course.v1';
const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const word = (value: unknown, max: number): value is number => finite(value) && Number.isInteger(value) && value >= 0 && value <= max;

function cachedObstacle(saved: Saved): ObstacleStamp | null {
  const o = saved.obstacle;
  const last = saved.trail.length - 1;
  if (!o || !word(o.sample, last) || (o.headingSample !== undefined && !word(o.headingSample, last)) ||
      !word(o.faceMm, 4095) || o.faceMm === 0 || !word(o.widthMm, 4095) || !word(o.breadthMm, 4095) ||
      (o.headingDeg !== null && !finite(o.headingDeg)) || (o.faceTime !== undefined && !finite(o.faceTime)) ||
      (o.bodyYaw !== undefined && (!finite(o.bodyYaw?.timestamp) || !finite(o.bodyYaw.deci) || !word(Math.abs(o.bodyYaw.deci), 1800)))) return null;
  return { ...o, sample: saved.trail[o.sample]!, headingSample: o.headingSample === undefined ? undefined : saved.trail[o.headingSample] };
}

function validSurvey(survey: Survey): boolean {
  return finite(survey?.timestamp) && (survey.request === -1 || word(survey.request, 255)) &&
    Array.isArray(survey.surfaces) && survey.surfaces.length > 0 && survey.surfaces.length <= SLOT_COUNT &&
    survey.surfaces.every((face) => word(face?.firstSlot, SLOT_COUNT - 1) && Array.isArray(face.points) &&
      face.points.length >= 3 && face.firstSlot + face.points.length <= SLOT_COUNT &&
      typeof face.boundedStart === 'boolean' && typeof face.boundedEnd === 'boolean' &&
      face.points.every((point) => finite(point?.x) && finite(point.y)));
}

function validPoint(point: TrailPoint): boolean {
  const pose = point?.checkpoint;
  return point != null && finite(point.timestamp) && finite(point.odometerMm) &&
    (point.headingDeg === null || finite(point.headingDeg)) && (point.speedMms === null || finite(point.speedMms)) &&
    (point.speedFault === undefined || point.speedFault === true) &&
    (point.seq === undefined || word(point.seq, 65535)) &&
    (point.wheels === undefined || (finite(point.wheels?.leftMms) && finite(point.wheels?.rightMms))) &&
    (point.pivotScalePct === undefined || pivotScalePercent(point.pivotScalePct) !== undefined) &&
    (point.surveys === undefined || (Array.isArray(point.surveys) && point.surveys.every(validSurvey))) &&
    (pose === undefined || (finite(pose?.x) && finite(pose.y) && (pose.headingDeg === null || finite(pose.headingDeg)) &&
      ['compass', 'relative', 'unavailable'].includes(pose.reference) && typeof pose.hasGap === 'boolean'));
}

function read(scope: string, storage: StoragePort | undefined): Saved | null {
  const raw = storage?.getItem(KEY);
  if (!raw || raw.length > 3_000_000) return null;
  const record: Saved = JSON.parse(raw);
  if (record?.scope !== scope || !word(record.runId, 65535) || record.runId === 0 || !word(record.uptime, 0xffffffff) ||
      !finite(record.time) || (record.start !== null && !finite(record.start)) ||
      !Array.isArray(record.trail) || !record.trail.length || record.trail.length > TRAIL_MAX ||
      !record.trail.every((p, i) => validPoint(p) && (!i || p.timestamp >= record.trail[i - 1]!.timestamp))) return null;
  return record;
}

function order(a: TrailPoint, b: TrailPoint): number {
  if (a.timestamp !== b.timestamp) return a.timestamp - b.timestamp;
  // Multiple status frames can share a bridge timestamp, also at sequence wrap.
  if (a.seq !== undefined && b.seq !== undefined) {
    const distance = (a.seq - b.seq + 65536) % 65536;
    return distance < 32768 ? distance : distance - 65536;
  }
  return 0;
}

const sameReading = (a: TrailPoint, b: TrailPoint) =>
  a.odometerMm === b.odometerMm && a.headingDeg === b.headingDeg && a.speedMms === b.speedMms && a.speedFault === b.speedFault;

function replayOrder(a: TrailPoint, replay: TrailPoint[], index: number): number {
  const b = replay[index]!;
  const bySequence = order(a, b);
  if (bySequence !== 0 || (a.seq !== undefined && b.seq !== undefined) || sameReading(a, b)) return bySequence;
  // Legacy bridge batches have no sequence. Align on a shared observation
  // so distinct poses with the same receipt time both survive the reload.
  for (let i = index + 1; i < replay.length && replay[i]!.timestamp === a.timestamp; i++) {
    if (sameReading(a, replay[i]!)) return 1;
  }
  return -1;
}

function join(saved: Saved, store: TelemetryStore): TelemetryStore {
  const fresh = store.trail;
  const old = saved.trail;
  const trail: TrailPoint[] = [];
  const replacements = new Map<TrailPoint, TrailPoint>();
  const cached = cachedObstacle(saved);
  const cachedAnchor = (p: TrailPoint) => cached !== null && (cached.sample === p || cached.headingSample === p);
  let i = 0;
  let j = 0;
  // If replay reaches further back, its first pose defines the world origin.
  const originOrder = fresh.length === 0 ? -1 : replayOrder(old[0]!, fresh, 0);
  while (i < old.length || j < fresh.length) {
    const a = old[i];
    const b = fresh[j];
    const cmp = a === undefined ? 1 : b === undefined ? -1 : replayOrder(a, fresh, j);
    if (cmp < 0 && a !== undefined) {
      const point = originOrder > 0 && a.checkpoint ? { ...a, checkpoint: undefined } : a;
      trail.push(point);
      if (a !== point && cachedAnchor(a)) replacements.set(a, point);
      i++;
    } else if (cmp > 0 && b !== undefined) {
      const sample = originOrder < 0 && b.checkpoint ? { ...b, checkpoint: undefined } : b;
      trail.push(sample);
      if (sample !== b) replacements.set(b, sample);
      j++;
    } else if (a !== undefined && b !== undefined) {
      const same = sameReading(a, b);
      const replayCheckpoint = originOrder >= 0 ? b.checkpoint : undefined;
      const checkpoint = originOrder <= 0 && same ? a.checkpoint ?? replayCheckpoint : replayCheckpoint;
      const wheels = b.wheels ?? (same ? a.wheels : undefined);
      const oldSurveys = same ? a.surveys?.filter((oldSurvey) =>
        !b.surveys?.some((s) => s.timestamp === oldSurvey.timestamp && s.request === oldSurvey.request)) : undefined;
      const surveys = oldSurveys?.length ? [...oldSurveys, ...b.surveys ?? []] : b.surveys;
      const matched = checkpoint !== b.checkpoint || wheels !== b.wheels || surveys !== b.surveys
        ? { ...b, checkpoint, wheels, surveys } : b;
      trail.push(matched);
      if (matched !== b) replacements.set(b, matched);
      if (same && matched !== a && cachedAnchor(a)) replacements.set(a, matched);
      i++;
      j++;
    }
  }
  const replace = (p: TrailPoint) => replacements.get(p) ?? p;
  let o = store.obstacle;
  const contactTime = (stamp: ObstacleStamp) => stamp.bodyYaw?.timestamp ?? stamp.headingSample?.timestamp ?? stamp.sample.timestamp;
  // An active hug with no located start must not borrow the previous obstacle. An idle report keeps the last estimate.
  if (cached && (o === null ? !store.latest.obstacle?.values.hug_state : contactTime(cached) > contactTime(o)) &&
      trail.includes(replace(cached.sample)) && (!cached.headingSample || trail.includes(replace(cached.headingSample)))) o = cached;
  const ping = store.forwardReturn;
  let merged: TelemetryStore = { ...store, trail, trailStart: store.trailStart ?? saved.start,
    scanStart: store.scanStart === null ? null : replace(store.scanStart),
    obstacle: o === null ? null : { ...o, sample: replace(o.sample),
      headingSample: o.headingSample === undefined ? undefined : replace(o.headingSample) },
    forwardReturn: ping?.pose == null ? ping : { ...ping, pose: replace(ping.pose) } };
  while (merged.trail.length > TRAIL_MAX) merged = compactTrail(merged);
  return merged;
}

/** Restore geometry only after this connection identifies the firmware boot. */
export function courseSession(scope: string, storage?: StoragePort) {
  let saved: Saved | null = null;
  const resetKey = 'pico-car.reset.' + scope;
  let cutoff: ResetBoundary | undefined;
  try {
    saved = read(scope, storage);
    const boundary: number | ResetBoundary | null = JSON.parse(storage?.getItem(resetKey) ?? 'null');
    if (typeof boundary === 'number' && word(boundary, Number.MAX_SAFE_INTEGER) && boundary > 0) {
      cutoff = { timestamp: boundary };
    } else if (boundary && typeof boundary === 'object' && word(boundary.timestamp, Number.MAX_SAFE_INTEGER)) {
      cutoff = boundary;
    }
  } catch { /* The browser can disable storage or evict an incomplete snapshot. */ }
  return {
    reduce(store: TelemetryStore, frame: LinkPacket): TelemetryStore {
      if (cutoff !== undefined && store.resetAt === null) {
        store = { ...emptyStore(), resetAt: cutoff.timestamp, trailStart: cutoff.timestamp,
          resetOrder: cutoff.receiptOrder, resetReceiptId: cutoff.receiptOrder === undefined ? undefined : cutoff.receiptId };
      }
      if (cutoff !== undefined && precedesReset(frame, cutoff.timestamp, cutoff.receiptOrder)) {
        // A replay can still supply the boot identity that survived the soft reset.
        if (frame.packet === 'heartbeat' && frame.timestamp > (store.latest.heartbeat?.timestamp ?? -Infinity)) {
          return { ...store, latest: { ...store.latest, heartbeat: frame } };
        }
        return store;
      }
      if (acceptedReset(frame) && frame.timestamp >= (store.trailStart ?? 0) &&
          (cutoff?.timestamp !== frame.timestamp || cutoff?.receiptId !== frame.receiptId ||
            cutoff?.receiptOrder !== frame.receiptOrder)) {
        cutoff = { timestamp: frame.timestamp, receiptId: frame.receiptId, receiptOrder: frame.receiptOrder };
        if (saved && saved.trail[0]!.timestamp <= cutoff.timestamp) saved = null;
        try { storage?.setItem(resetKey, JSON.stringify(cutoff)); } catch { /* Storage is optional. */ }
      }
      // A short replay may start with old status followed by the new boot's first
      // heartbeat. Supply only the boot comparison; the incoming heartbeat replaces it.
      if (saved !== null && frame.packet === 'heartbeat' && frame.timestamp >= saved.time &&
          store.latest.heartbeat === undefined && store.trailStart === null) {
        const previous = { packet: 'heartbeat', timestamp: saved.time, values: { run_id: saved.runId, uptime_s: saved.uptime } };
        return reduceStore({ ...store, latest: { ...store.latest, heartbeat: previous } }, frame);
      }
      return reduceStore(store, frame);
    },
    restore(store: TelemetryStore, now = Date.now()): TelemetryStore {
      if (saved === null) return store;
      const hb = store.latest.heartbeat;
      const limit = STALE_AFTER_MS.heartbeat!;
      if (hb === undefined || now - (hb.receivedAt ?? hb.timestamp) > limit || hb.timestamp < saved.time ||
          !word(hb.values.run_id, 65535) || !word(hb.values.uptime_s, 0xffffffff)) return store;
      const pending = saved;
      const elapsed = hb.timestamp - pending.time;
      const reportedMs = (hb.values.uptime_s - pending.uptime) * 1000;
      const wraps = Math.max(0, Math.round((elapsed - reportedMs) / HEARTBEAT_TIMER_WRAP_MS));
      const deviceMs = reportedMs + wraps * HEARTBEAT_TIMER_WRAP_MS;
      // The 16-bit boot ID can repeat. Elapsed bridge and device time must also agree.
      if (hb.values.run_id !== pending.runId ||
          (reportedMs < 0 && elapsed < HEARTBEAT_TIMER_WRAP_MS + reportedMs - 1000) ||
          Math.abs(deviceMs - elapsed) > limit + elapsed * 0.001 ||
          (store.trailStart !== null && store.trailStart > pending.trail[0]!.timestamp)) {
        saved = null;
        return store;
      }
      const latest = store.trail.at(-1);
      if (!latest || order(latest, pending.trail.at(-1)!) < 0) return store;
      saved = null;
      return join(pending, store);
    },
    save(store: TelemetryStore) {
      try {
        const hb = store.latest.heartbeat;
        if (!storage) return;
        if (!hb || !store.trail.length || !word(hb.values.run_id, 65535) || !hb.values.run_id || !word(hb.values.uptime_s, 0xffffffff)) {
          if (store.trailStart !== null) storage.removeItem(KEY);
          return;
        }
        if (saved !== null) return; // A second reload can happen before boot confirmation.
        const record: Saved = { scope, runId: hb.values.run_id, uptime: hb.values.uptime_s,
          time: hb.timestamp, start: store.trailStart, trail: store.trail };
        const obstacle = store.obstacle;
        if (obstacle !== null && obstacle.faceMm > 0) {
          const sample = store.trail.indexOf(obstacle.sample);
          const headingSample = obstacle.headingSample === undefined ? undefined : store.trail.indexOf(obstacle.headingSample);
          if (sample >= 0 && (headingSample === undefined || headingSample >= 0)) {
            record.obstacle = { ...obstacle, sample, headingSample };
          }
        }
        storage.setItem(KEY, JSON.stringify(record));
      } catch { /* Storage quota and privacy settings must not interrupt telemetry. */ }
    },
  };
}
