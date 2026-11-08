import type { HistoryRecord } from '../../lib/history';

export type HistoryRow = {
  timestamp: number;
  packet: string;
  detail: string;
};

export type HistorySummary = {
  total: number;
  counts: Record<string, number>;
  rows: HistoryRow[];
};

function detail(record: HistoryRecord): string {
  const parts = Object.entries(record.values).map(
    ([key, value]) => `${key}=${value}`,
  );
  return parts.length === 0 ? record.topic : parts.join(' ');
}

export function summarizeHistory(
  records: HistoryRecord[],
  limit = 20,
): HistorySummary {
  const counts: Record<string, number> = {};
  for (const record of records) {
    counts[record.packet] = (counts[record.packet] ?? 0) + 1;
  }

  return {
    total: records.length,
    counts,
    rows: records.slice(0, Math.max(limit, 0)).map((record) => ({
      timestamp: record.timestamp,
      packet: record.packet,
      detail: detail(record),
    })),
  };
}
