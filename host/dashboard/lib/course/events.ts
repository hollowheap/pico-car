import type { EventKind } from '../events';
import { BIT_FIELDS, ENUMS } from '../schema.generated';
import type { EventRow, TrailPoint } from '../telemetry-store';
import { RAD_PER_DEG, SPEED_GAP_MS, WHEEL_TRACK_MM, sampleSpeed, type CourseHump, type CourseJunction, type CourseMarker, type CoursePoint } from './geometry';

export function eventKey(event: Pick<EventRow, 'eventId' | 'timestamp' | 'type'>): string {
  return `${event.eventId}:${event.timestamp}:${event.type}`;
}

/** Events that do not belong on the trace (they say nothing about the place). */
const NOT_ON_MAP: EventKind[] = ['command', 'link', 'system'];

/** The first index whose timestamp is at or after `timestamp`; `trail.length` when none is. */
function firstAtOrAfter(trail: TrailPoint[], timestamp: number): number {
  let low = 0;
  let high = trail.length;
  while (low < high) {
    const mid = (low + high) >> 1;
    if ((trail[mid]?.timestamp ?? Infinity) < timestamp) low = mid + 1;
    else high = mid;
  }
  return low;
}

/**
 * The odometer falls in a reverse, so one value is reached on several passes. A
 * packet's timestamp says which: take the segment nearest in trail order that spans the
 * value; no segment spanning it means no place. An event older than the trail
 * lands on the first segment that spans the value.
 */
type LocatedEvent = CoursePoint & { speedMms: number | null };

function pointAtPacket(trail: TrailPoint[], points: CoursePoint[], timestamp: number, odometerMm: number): LocatedEvent | null {
  if (trail.length === 0) return null;
  if (trail.length === 1) {
    const sample = trail[0];
    const point = points[0];
    return point !== undefined && sample?.headingDeg != null && sample.odometerMm === odometerMm
      ? { ...point, speedMms: eventSpeed(sample, timestamp) }
      : null;
  }
  const start = Math.min(Math.max(firstAtOrAfter(trail, timestamp), 1), trail.length - 1);
  // An odometer can repeat on either side of a data gap. Only match passes
  // from the interval in which the event was received.
  const preceding = trail[start - 1]!;
  if (timestamp === preceding.timestamp && odometerMm === preceding.odometerMm && preceding.headingDeg !== null) {
    return { ...points[start - 1]!, speedMms: eventSpeed(preceding, timestamp) };
  }
  if (trail[start]!.headingDeg === null) return null;
  let left = start;
  let right = start;
  while (left > 0 || right < trail.length) {
    for (const k of left === right ? [left] : [left, right]) {
      const at = pointInSegment(trail, points, k, timestamp, odometerMm);
      if (at !== null) return at;
    }
    left = left > 1 && trail[left - 1]!.headingDeg !== null ? left - 1 : 0;
    right = right + 1 < trail.length && trail[right + 1]!.headingDeg !== null ? right + 1 : trail.length;
  }
  return null;
}

function eventSpeed(sample: TrailPoint | undefined, timestamp: number): number | null {
  return sample !== undefined && Math.abs(sample.timestamp - timestamp) <= SPEED_GAP_MS ? sampleSpeed(sample) : null;
}

function pointInSegment(trail: TrailPoint[], points: CoursePoint[], k: number, timestamp: number, odometerMm: number): LocatedEvent | null {
  const a = trail[k - 1];
  const b = trail[k];
  const pa = points[k - 1];
  const pb = points[k];
  if (a === undefined || b === undefined || pa === undefined || pb === undefined) return null;
  if (odometerMm < Math.min(a.odometerMm, b.odometerMm) || odometerMm > Math.max(a.odometerMm, b.odometerMm)) return null;
  const span = b.odometerMm - a.odometerMm;
  const t = span === 0 ? 0 : (odometerMm - a.odometerMm) / span;
  // An exact endpoint can supply a measured value. An interpolated map point
  // between two odometer samples has no recorded speed of its own.
  const sample = span === 0
    ? (Math.abs(timestamp - a.timestamp) < Math.abs(timestamp - b.timestamp) ? a : b)
    : t === 0 ? a : t === 1 ? b : undefined;
  return {
    x: pa.x + (pb.x - pa.x) * t,
    y: pa.y + (pb.y - pa.y) * t,
    speedMms: eventSpeed(sample, timestamp),
  };
}

const RAMP_DEFAULT_LENGTH_MM = 200;
const RAMP_WIDTH_MM = 160;
const HUMP_MIN_PEAK_MM = 8;
const HUMP_MIN_SPAN_MM = 40;
const HUMP_ROLL_DEADBAND_DEG = 2.5;
const HUMP_ROLL_FULL_DEG = 12;

export function placeCourseEvents(
  samples: TrailPoint[],
  points: CoursePoint[],
  events: EventRow[],
  trailStart: number | null,
  minOdometer: number,
  maxOdometer: number,
) {
  const junctions: CourseJunction[] = [];
  const TURN_DIR: Partial<Record<number, CourseJunction['direction']>> = {
    1: 'left',
    2: 'right',
    3: 'uturn',
    4: 'straight',
  };
  const headingAt = (timestamp: number): number | null => {
    for (let k = samples.length - 1; k >= 0; k -= 1) {
      const s = samples[k]!;
      if (s.timestamp <= timestamp && s.headingDeg !== null) return s.headingDeg;
    }
    return null;
  };
  const markers: CourseMarker[] = [];
  for (const event of events) {
    if (NOT_ON_MAP.includes(event.text.kind)) continue;
    if (trailStart !== null && event.timestamp < trailStart) continue;
    if (event.odometerMm < minOdometer || event.odometerMm > maxOdometer) continue;
    const at = pointAtPacket(samples, points, event.timestamp, event.odometerMm);
    if (at !== null) {
      markers.push({ ...at, kind: event.text.kind, title: event.text.title, eventId: event.eventId, odometerMm: event.odometerMm, event });
      if (event.name === 'junction' && event.p1 === ENUMS.junctionKind.stripe) {
        const left = (event.p2 & (1 << BIT_FIELDS.junctionStripe.left)) !== 0;
        const right = (event.p2 & (1 << BIT_FIELDS.junctionStripe.right)) !== 0;
        const corroborated = (event.p2 & (1 << BIT_FIELDS.junctionStripe.corroborated)) !== 0;
        const headingDeg = headingAt(event.timestamp);
        if (headingDeg !== null && (left || right)) {
          const direction: CourseJunction['direction'] = left && right ? 'cross' : left ? 'left' : 'right';
          junctions.push({ x: at.x, y: at.y, headingDeg, direction, corroborated, timestamp: event.timestamp, eventId: event.eventId });
        }
      } else if (event.name === 'turnStart') {
        const direction = TURN_DIR[event.p1];
        const headingDeg = headingAt(event.timestamp);
        const nearStripe = junctions.some((j) =>
          Math.abs(j.timestamp - event.timestamp) <= 2000 && Math.hypot(j.x - at.x, j.y - at.y) <= 60);
        if (direction && headingDeg !== null && !nearStripe) {
          junctions.push({ x: at.x, y: at.y, headingDeg, direction, timestamp: event.timestamp, eventId: event.eventId });
        }
      }
    }
  }

  // Project ONLY firmware-flagged humps/ramps meeting the noise threshold, offset laterally
  // relative to the wheels based on roll (positive roll = left down -> right wheel on ramp).
  const humps: CourseHump[] = [];
  const humpEndEvents = events.filter((e) => e.name === 'humpEnd' && (trailStart === null || e.timestamp >= trailStart));
  const usedEnds = new Set<number>();
  let runStart = -1;
  for (let i = 0; i <= samples.length; i += 1) {
    const s = samples[i];
    const inHump = s !== undefined && s.humpState !== undefined && s.humpState !== ENUMS.humpState.flat;
    if (inHump && runStart === -1) {
      runStart = i;
    } else if (!inHump && runStart !== -1) {
      const runEnd = i - 1;
      const pStart = points[runStart]!;
      const pEnd = points[runEnd]!;
      const sStart = samples[runStart]!;
      const sEnd = samples[runEnd]!;
      const midIdx = Math.floor((runStart + runEnd) / 2);
      const headingDeg = samples[midIdx]?.headingDeg ?? sStart.headingDeg ?? sEnd.headingDeg;
      if (headingDeg !== null) {
        const spanMm = Math.hypot(pEnd.x - pStart.x, pEnd.y - pStart.y);
        const matchIdx = humpEndEvents.findIndex((e, idx) => !usedEnds.has(idx) && e.timestamp >= sStart.timestamp - 500 && e.timestamp <= sEnd.timestamp + 1500);
        let peakMm: number | null = null;
        if (matchIdx !== -1) {
          usedEnds.add(matchIdx);
          const ev = humpEndEvents[matchIdx]!;
          peakMm = ev.p2 === 1 && ev.p1 > 0 ? ev.p1 : null;
        }
        const meetsThreshold = (peakMm !== null && peakMm >= HUMP_MIN_PEAK_MM) ||
          (runEnd - runStart >= 1 && spanMm >= HUMP_MIN_SPAN_MM);
        if (meetsThreshold) {
          let rollSum = 0;
          let rollCount = 0;
          for (let k = runStart; k <= runEnd; k += 1) {
            const r = samples[k]?.rollDeg;
            if (r !== undefined) {
              rollSum += r;
              rollCount += 1;
            }
          }
          const meanRoll = rollCount > 0 ? rollSum / rollCount : 0;
          const lateralMm = Math.abs(meanRoll) < HUMP_ROLL_DEADBAND_DEG
            ? 0
            : Math.max(-1, Math.min(1, meanRoll / HUMP_ROLL_FULL_DEG)) * (WHEEL_TRACK_MM / 2);
          const rad = headingDeg * RAD_PER_DEG;
          const rx = Math.cos(rad);
          const ry = -Math.sin(rad);
          humps.push({
            x: (pStart.x + pEnd.x) / 2 + lateralMm * rx,
            y: (pStart.y + pEnd.y) / 2 + lateralMm * ry,
            headingDeg,
            lengthMm: Math.max(160, Math.min(280, Math.round(spanMm) || RAMP_DEFAULT_LENGTH_MM)),
            widthMm: RAMP_WIDTH_MM,
            peakMm,
            timestamp: sEnd.timestamp,
          });
        }
      }
      runStart = -1;
    }
  }
  for (let idx = 0; idx < humpEndEvents.length; idx += 1) {
    if (usedEnds.has(idx)) continue;
    const ev = humpEndEvents[idx]!;
    if (ev.p2 !== 1 || ev.p1 < HUMP_MIN_PEAK_MM) continue;
    const at = pointAtPacket(samples, points, ev.timestamp, ev.odometerMm);
    const sampleIdx = Math.min(Math.max(firstAtOrAfter(samples, ev.timestamp), 0), samples.length - 1);
    const sample = samples[sampleIdx];
    const headingDeg = sample?.headingDeg ?? null;
    if (at !== null && headingDeg !== null) {
      const rad = headingDeg * RAD_PER_DEG;
      const roll = sample?.rollDeg ?? 0;
      const lateralMm = Math.abs(roll) < HUMP_ROLL_DEADBAND_DEG
        ? 0
        : Math.max(-1, Math.min(1, roll / HUMP_ROLL_FULL_DEG)) * (WHEEL_TRACK_MM / 2);
      humps.push({
        x: at.x - (RAMP_DEFAULT_LENGTH_MM / 2) * Math.sin(rad) + lateralMm * Math.cos(rad),
        y: at.y - (RAMP_DEFAULT_LENGTH_MM / 2) * Math.cos(rad) - lateralMm * Math.sin(rad),
        headingDeg,
        lengthMm: RAMP_DEFAULT_LENGTH_MM,
        widthMm: RAMP_WIDTH_MM,
        peakMm: ev.p1,
        timestamp: ev.timestamp,
      });
    }
  }

  return { markers, junctions, humps };
}
