import { groupActivity } from './activity.js';
import { commandReplies } from './command-replies.js';
import { buildCourse, courseTapeRuns, courseTraceSegments, latestCourseSpeed, rotateToCarUp, type Course } from './course.js';
import { courseSession, type StoragePort } from './course-session.js';
import { barcodeActionLabel } from './events.js';
import { faultTiles } from './faults.js';
import { appendFrame, createFrameLog, type FrameLog } from './frame-log.js';
import { projectedHoldMessage } from './hold.js';
import { deriveJunctionState } from './junctions.js';
import { barcodePhaseCode, barcodePhaseLabel } from './line-signals.js';
import { packetOnTimeline, type LinkPacket } from './link.js';
import { longestLineRecovery, summarizeRun } from './run-summary.js';
import { scanPoints, stationaryContact } from './scan.js';
import { emptyStore, precedesReset, statusLossPercent, type TelemetryStore } from './telemetry-store.js';

export type ConsoleStore = Pick<TelemetryStore, 'latest' | 'obstaclePing' | 'diagRates' | 'lineSignals' |
  'barcodeEdges' | 'pitchHistory' | 'trailStart' | 'resetAt' | 'resetReceiptId' | 'events' | 'wheelHistory'>;

function barcodeView({ latest, events, trailStart }: TelemetryStore) {
  const { status, diag } = latest;
  const d = diag?.values;
  const eventDecode = events.findLast((event) => event.name === 'barcode' && event.timestamp >= (trailStart ?? -Infinity));
  const eventIsLatest = eventDecode !== undefined && (diag === undefined || d?.last_barcode === undefined ||
    eventDecode.timestamp >= diag.timestamp);
  const code = eventIsLatest ? eventDecode.p1 & 0x7f : d?.last_barcode;
  const letter = code === undefined || code === 0 ? null : String.fromCharCode(code);
  const action = eventIsLatest ? eventDecode.p2 & 0x7 : d?.last_barcode_action;
  const lifecycle = events.filter((event) => event.name === 'barcodePhase' && event.timestamp >= (trailStart ?? -Infinity) &&
    barcodePhaseLabel(event.p1) !== undefined).sort((a, b) => a.timestamp - b.timestamp).slice(-4);
  const latestPhase = lifecycle.at(-1);
  const snapshotCode = barcodePhaseCode(status?.values.line_bits);
  const phaseCode = latestPhase !== undefined &&
    (latestPhase.timestamp >= (status?.timestamp ?? -Infinity) || barcodePhaseLabel(snapshotCode) === undefined)
    ? latestPhase.p1 : snapshotCode;
  const phaseReport = latestPhase?.p1 === phaseCode ? latestPhase : undefined;
  return { code, letter, actionLabel: barcodeActionLabel(action), phase: barcodePhaseLabel(phaseCode), phaseReport,
    earlierPhases: phaseReport === undefined ? lifecycle : lifecycle.slice(0, -1) };
}

function consoleStore(s: TelemetryStore): ConsoleStore {
  const { latest, obstaclePing, diagRates, lineSignals, barcodeEdges, pitchHistory, trailStart, resetAt, resetReceiptId,
    events, wheelHistory } = s;
  return { latest, obstaclePing, diagRates, lineSignals, barcodeEdges, pitchHistory, trailStart, resetAt, resetReceiptId,
    events, wheelHistory };
}

function projectCourse(s: TelemetryStore) {
  const course = buildCourse(s.trail, s.events, s.obstacle, s.trailStart, s.forwardReturn);
  const strokes = courseTraceSegments(course);
  const tapeRuns = courseTapeRuns(course);
  const car = course.points.at(-1);
  const heading = course.samples.at(-1)?.headingDeg;
  const known = course.positionKnown && car !== undefined && heading != null;
  const carUp = known
    ? strokes.map((stroke) => ({ ...stroke, points: stroke.points.map((point) => rotateToCarUp(point, car, heading)) }))
    : strokes;
  const carUpTapeRuns = known
    ? tapeRuns.map((run) => run.map((point) => rotateToCarUp(point, car, heading)))
    : tapeRuns;
  return { course: { ...course, samples: course.samples.slice(-1) },
    drawing: { 'north-up': { strokes, tapeRuns }, 'car-up': { strokes: carUp, tapeRuns: carUpTapeRuns } } };
}

export type CourseDrawing = ReturnType<typeof projectCourse>['drawing'];

export type ConsoleView = {
  course: Course;
  drawing: CourseDrawing;
  pendingPose: string | null;
  summary: ReturnType<typeof summarizeRun>;
  speedMms: number | null;
  lossPercent: number | null;
  groups: ReturnType<typeof groupActivity>;
  replies: ReturnType<typeof commandReplies>;
  junction: ReturnType<typeof deriveJunctionState>;
  longestRecoveryMm: number | null;
  faults: ReturnType<typeof faultTiles>;
  scan: { readings: ReturnType<typeof scanPoints>; stationaryContact: boolean };
  hold: ReturnType<typeof projectedHoldMessage>;
  barcode: ReturnType<typeof barcodeView>;
};

export type ConsoleSnapshot = {
  store: ConsoleStore;
  view: ConsoleView;
  frameLog: FrameLog;
  lastPacket: LinkPacket | null;
};

export function createConsoleView(records: readonly LinkPacket[] = [], storage?: StoragePort) {
  const session = courseSession('bridge', storage);
  let store = emptyStore();
  let frameLog = createFrameLog();
  let lastPacket: LinkPacket | null = null;
  let previous: TelemetryStore | undefined;
  let snapshot: ConsoleSnapshot | undefined;

  const append = (input: LinkPacket) => {
    const packet = packetOnTimeline(input);
    const before = store;
    store = session.reduce(store, packet);
    if (store.resetAt !== before.resetAt || store.resetReceiptId !== before.resetReceiptId || store.resetOrder !== before.resetOrder) {
      frameLog = createFrameLog([], frameLog.revision + 1);
    }
    if (!precedesReset(packet, store.resetAt, store.resetOrder)) frameLog = appendFrame(frameLog, packet);
    lastPacket = packet;
  };
  for (const record of records) append(record);
  store = session.restore(store);

  return {
    append(packet: LinkPacket) {
      append(packet);
      store = session.restore(store);
    },
    save: () => session.save(store),
    read(): ConsoleSnapshot {
      if (snapshot && store === previous && frameLog === snapshot.frameLog && lastPacket === snapshot.lastPacket) return snapshot;
      const old = snapshot?.view;
      const hasCourseChanges = old === undefined || store.trail !== previous?.trail || store.events !== previous?.events ||
        store.trailStart !== previous?.trailStart || store.obstacle !== previous?.obstacle ||
        store.forwardReturn !== previous?.forwardReturn;
      const course = hasCourseChanges ? projectCourse(store) : old;
      const eventChanges = old === undefined || store.events !== previous?.events || store.trailStart !== previous?.trailStart;
      const profile = store.latest.profile;
      const pose = store.trail.at(-1);
      const view: ConsoleView = {
        course: course.course,
        drawing: course.drawing,
        pendingPose: !course.course.positionKnown && pose?.headingDeg === null && pose.seq !== undefined && pose.wheels === undefined
          ? `${store.trailStart ?? ''}/${pose.seq}/${pose.timestamp}` : null,
        summary: hasCourseChanges || store.measurements !== previous?.measurements
          ? { ...summarizeRun(store.trail, store.events, store.trailStart), ...store.measurements } : old.summary,
        speedMms: latestCourseSpeed(store.latest.status, pose),
        lossPercent: statusLossPercent(store),
        groups: eventChanges ? groupActivity(store.events) : old.groups,
        replies: eventChanges ? commandReplies(store.events) : old.replies,
        junction: eventChanges ? deriveJunctionState(store.events, store.trailStart) : old.junction,
        longestRecoveryMm: eventChanges ? longestLineRecovery(store.events, store.trailStart) : old.longestRecoveryMm,
        faults: old !== undefined && store.latest.status?.values.faults === previous?.latest.status?.values.faults &&
          store.latest.diag?.values.sticky_faults === previous?.latest.diag?.values.sticky_faults
          ? old.faults : faultTiles(store.latest.status?.values.faults, store.latest.diag?.values.sticky_faults),
        scan: old !== undefined && profile === previous?.latest.profile ? old.scan : {
          readings: profile === undefined ? [] : scanPoints(profile.values).filter((point) => point.measured),
          stationaryContact: profile !== undefined && stationaryContact(profile.values),
        },
        hold: projectedHoldMessage(store),
        barcode: eventChanges || store.latest.diag !== previous?.latest.diag || store.latest.status !== previous?.latest.status
          ? barcodeView(store) : old.barcode,
      };
      snapshot = { store: snapshot && previous === store ? snapshot.store : consoleStore(store), view, frameLog, lastPacket };
      previous = store;
      return snapshot;
    },
  };
}
