import type { ConsoleUpdate, FrameLogUpdate } from '../../../bridge/src/console/update.js';
import type { ConsoleSnapshot, ConsoleView } from '../../../bridge/src/console/view.js';
import { FRAME_LOG_BLOCK_SIZE, FRAME_LOG_LIMIT, type FrameLog } from './frame-log';

export type { ConsoleSnapshot, ConsoleStore, ConsoleView, CourseDrawing } from '../../../bridge/src/console/view.js';
export type { ConsoleUpdate } from '../../../bridge/src/console/update.js';

const view: ConsoleView = {
  course: { points: [], samples: [], hasGap: false, positionKnown: false, headingReference: 'unavailable',
    bootTimestamp: null, markers: [], junctions: [], humps: [], obstacle: null, surfaces: [], forwardReturn: null,
    bounds: { minX: 0, maxX: 0, minY: 0, maxY: 0 } },
  drawing: { 'north-up': { strokes: [], tapeRuns: [] }, 'car-up': { strokes: [], tapeRuns: [] } },
  pendingPose: null,
  summary: { scope: 'Recorded', hasTravel: false, partialTravel: false, distanceMm: 0, durationS: 0, averageMms: 0,
    turns: 0, barcodesOk: 0, barcodesNoAction: 0, barcodesBad: 0, obstacles: 0, bypasses: 0,
    humps: 0, tallestHumpMm: 0, lifts: 0, lineLosses: 0, faults: 0, faultBits: [], stops: 0 },
  speedMms: null,
  lossPercent: null,
  groups: [],
  replies: {},
  junction: { armed: null, active: null, last: null, lastStripe: null, markers: [] },
  longestRecoveryMm: null,
  faults: [],
  scan: { readings: [], stationaryContact: false },
  hold: null,
  barcode: { code: undefined, letter: null, actionLabel: 'Action unavailable', phase: undefined,
    phaseReport: undefined, earlierPhases: [] },
};

export const EMPTY_CONSOLE: ConsoleSnapshot = {
  store: { latest: {}, obstaclePing: undefined, diagRates: {}, lineSignals: [], barcodeEdges: [], pitchHistory: [],
    trailStart: null, resetAt: null, events: [], wheelHistory: [] },
  view,
  frameLog: { blocks: [], nextId: 0, revision: 0 },
  lastPacket: null,
};

const firstBlock = (log: Pick<FrameLog, 'nextId'>) => Math.floor(Math.max(0, log.nextId - FRAME_LOG_LIMIT) / FRAME_LOG_BLOCK_SIZE);

function resumeFrames(log: FrameLog, update: FrameLogUpdate): FrameLog | null {
  const first = firstBlock(update);
  const before = firstBlock(log);
  const unchanged = update.revision === log.revision ? log.blocks.slice(first - before, update.from - before) : [];
  if (first + unchanged.length !== update.from) return null;
  return { nextId: update.nextId, revision: update.revision, blocks: [...unchanged, ...update.blocks] };
}

export function applyConsoleUpdate(previous: ConsoleSnapshot | null, update: ConsoleUpdate): ConsoleSnapshot | null {
  if (update.type === 'console') return update.data;
  if (previous === null) return null;
  const frameLog = update.frames === undefined ? previous.frameLog : resumeFrames(previous.frameLog, update.frames);
  if (frameLog === null) return null;
  const store = { ...previous.store, ...update.store };
  for (const field of update.removed) {
    if (field === 'obstaclePing' || field === 'resetReceiptId') delete store[field];
  }
  return { store, view: { ...previous.view, ...update.view }, lastPacket: update.lastPacket, frameLog };
}
