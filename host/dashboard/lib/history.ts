import { emptyStore, reduceStore, type TelemetryStore } from './telemetry-store';
import { packetOnTimeline, type LinkClient, type LinkPacket, type LinkStatus } from './link';

export type HistoryRecord = LinkPacket & { topic: string };

type FetchLike = (
  input: string,
  init?: { signal: AbortSignal },
) => Promise<{ ok: boolean; status: number; json: () => Promise<unknown> }>;

export const HISTORY_CATCHUP_LIMIT = 10000;
const HISTORY_TIMEOUT_MS = 8000;
const HISTORY_RETRY_MS = 2000;

function isRecord(value: unknown): value is HistoryRecord {
  if (typeof value !== 'object' || value === null) {
    return false;
  }
  const record = value as Partial<HistoryRecord>;
  return (
    Number.isFinite(record.timestamp) &&
    typeof record.topic === 'string' &&
    typeof record.packet === 'string' &&
    typeof record.values === 'object' &&
    record.values !== null
  );
}

export async function fetchHistory(
  serviceUrl: string,
  limit = 100,
  fetchImpl: FetchLike = fetch,
  signal?: AbortSignal,
): Promise<HistoryRecord[]> {
  const base = serviceUrl.replace(/\/+$/, '');
  const response = await fetchImpl(`${base}/history?limit=${limit}`, signal ? { signal } : undefined);
  if (!response.ok) {
    throw new Error(`history request failed: ${response.status}`);
  }
  const payload = await response.json();
  if (!Array.isArray(payload)) {
    throw new Error('history response is not an array');
  }
  return payload.filter(isRecord);
}

function compareReceipts(a: LinkPacket, b: LinkPacket): number {
  return a.receiptOrder !== undefined && b.receiptOrder !== undefined
    ? a.receiptOrder - b.receiptOrder : a.timestamp - b.timestamp;
}

function packetCopies(records: readonly LinkPacket[]) {
  const overlap = new Map<number, LinkPacket[]>();
  for (const frame of records) {
    const time = frame.receivedAt ?? frame.timestamp;
    const copies = overlap.get(time);
    if (copies === undefined) overlap.set(time, [frame]);
    else copies.push(frame);
  }
  const take = (frame: LinkPacket) => {
    const time = frame.receivedAt ?? frame.timestamp;
    const copies = overlap.get(time);
    if (copies === undefined) return false;
    const exact = frame.receiptId === undefined ? -1 : copies.findIndex(entry => entry.receiptId === frame.receiptId);
    const fields = Object.keys(frame.values);
    const match = exact >= 0 ? exact : copies.findIndex((entry) =>
      (frame.receiptId === undefined || entry.receiptId === undefined) && entry.packet === frame.packet &&
      (frame.topic === undefined || entry.topic === undefined || frame.topic === entry.topic) &&
      fields.length === Object.keys(entry.values).length &&
      fields.every((field) => frame.values[field] === entry.values[field]));
    if (match < 0) return false;
    copies.splice(match, 1);
    if (copies.length === 0) overlap.delete(time);
    return true;
  };
  return { take, clear: () => overlap.clear() };
}

/** Replay the bridge snapshot and live frames together, oldest receipt first. */
export function replayHistory(records: HistoryRecord[], buffered: LinkPacket[] = []) {
  const packets: LinkPacket[] = records.map(packetOnTimeline).reverse();
  const overlap = packetCopies(packets);
  const newest = packets.reduce<LinkPacket | undefined>((last, frame) =>
    last === undefined || compareReceipts(frame, last) > 0 ? frame : last, undefined);
  // The bridge records a packet before broadcasting it. Match one live copy
  // per stored receipt so identical records still appear in the frame log.
  for (const input of buffered) {
    const frame = packetOnTimeline(input);
    if (!overlap.take(frame)) packets.push(frame);
  }
  return {
    frames: packets.sort(compareReceipts),
    takeSnapshotCopy(frame: LinkPacket): boolean {
      // The WebSocket can deliver an overlapping frame after HTTP completes.
      // A newer receipt on the ordered stream finishes the overlap window.
      if (newest !== undefined && compareReceipts(frame, newest) > 0) overlap.clear();
      return overlap.take(frame);
    },
  };
}

/** Fill gaps from the bridge while keeping the already reduced course in memory. */
export function syncHistory(
  client: Pick<LinkClient, 'onState' | 'onPacket'>,
  read: (signal: AbortSignal) => Promise<HistoryRecord[]>,
  apply: (frames: LinkPacket[], provisional?: boolean) => void,
): () => void {
  let pending: AbortController | null = null;
  let retry: ReturnType<typeof setTimeout> | undefined;
  let buffer: LinkPacket[] = [];
  let latest: LinkPacket | undefined;
  let boundary: LinkPacket[] = [];
  let takeSnapshotCopy: ((frame: LinkPacket) => boolean) | undefined;
  let status: LinkStatus = 'disconnected';
  let recovering = false;

  const commit = (frames: LinkPacket[], replacePreview = false) => {
    for (const frame of frames) {
      if (latest === undefined || compareReceipts(frame, latest) >= 0) {
        if (frame.timestamp !== latest?.timestamp) boundary = [];
        latest = frame;
        boundary.push(frame);
      }
    }
    if (frames.length > 0 || replacePreview) apply(frames);
  };

  const refresh = async () => {
    clearTimeout(retry);
    const request = new AbortController();
    pending?.abort();
    pending = request;
    const timer = setTimeout(() => request.abort(), HISTORY_TIMEOUT_MS);
    let records: HistoryRecord[];
    try {
      records = await read(request.signal);
    } catch (error) {
      if (pending !== request) return;
      console.warn('The dashboard could not recover bridge history:', error);
      pending = null;
      if (!recovering && buffer.length > 0) apply(buffer, true);
      // Keep the last confirmed point until a later request can fill the gap.
      recovering = true;
      if (status === 'open') retry = setTimeout(() => void refresh(), HISTORY_RETRY_MS);
      return;
    } finally {
      clearTimeout(timer);
    }
    if (pending !== request) return;
    const replay = replayHistory(records.filter(frame =>
      latest === undefined || compareReceipts(packetOnTimeline(frame), latest) >= 0), buffer);
    const seen = packetCopies(boundary);
    const replacePreview = recovering;
    recovering = false;
    takeSnapshotCopy = replay.takeSnapshotCopy;
    pending = null;
    buffer = [];
    commit(replay.frames.filter(frame => !seen.take(frame)), replacePreview);
  };

  const unsubscribeState = client.onState((snapshot) => {
    const opened = snapshot.status === 'open' && status !== 'open';
    status = snapshot.status;
    if (opened) void refresh();
    else if (status !== 'open') clearTimeout(retry);
  });
  const unsubscribePacket = client.onPacket((frame) => {
    if (takeSnapshotCopy?.(frame)) return;
    if (pending === null && !recovering) commit([frame]);
    else {
      buffer.push(frame);
      if (buffer.length > HISTORY_CATCHUP_LIMIT) buffer.shift();
      if (recovering) apply([frame], true);
    }
  });
  // History is also available when the WebSocket cannot open.
  void refresh();
  return () => {
    unsubscribeState();
    unsubscribePacket();
    clearTimeout(retry);
    const request = pending;
    pending = null;
    request?.abort();
  };
}

export function storeFromHistory(records: HistoryRecord[], buffered: LinkPacket[] = []): TelemetryStore {
  return replayHistory(records, buffered).frames.reduce(reduceStore, emptyStore());
}
