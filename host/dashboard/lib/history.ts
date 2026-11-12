import { emptyStore, reduceStore, type TelemetryStore } from './telemetry-store';

export type HistoryRecord = {
  timestamp: number;
  topic: string;
  packet: string;
  values: Record<string, number>;
};

type FetchLike = (
  input: string,
) => Promise<{ ok: boolean; status: number; json: () => Promise<unknown> }>;

function isRecord(value: unknown): value is HistoryRecord {
  if (typeof value !== 'object' || value === null) {
    return false;
  }
  const record = value as Partial<HistoryRecord>;
  return (
    typeof record.timestamp === 'number' &&
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
): Promise<HistoryRecord[]> {
  const base = serviceUrl.replace(/\/+$/, '');
  const response = await fetchImpl(`${base}/history?limit=${limit}`);
  if (!response.ok) {
    throw new Error(`history request failed: ${response.status}`);
  }
  const payload = await response.json();
  if (!Array.isArray(payload)) {
    throw new Error('history response is not an array');
  }
  return payload.filter(isRecord);
}

/**
 * Rebuild the store from the records the bridge keeps. The bridge lists the
 * newest first, so the records go in reverse.
 */
export function storeFromHistory(records: HistoryRecord[]): TelemetryStore {
  return [...records]
    .reverse()
    .reduce(
      (store, record) =>
        reduceStore(store, { timestamp: record.timestamp, packet: record.packet, values: record.values }),
      emptyStore(),
    );
}
