import { describe, expect, it } from 'vitest';
import { fetchHistory, type HistoryRecord } from '../../lib/history';
import { summarizeHistory } from './history';

const records: HistoryRecord[] = [
  { timestamp: 3, topic: 'car/SE01/h', packet: 'heartbeat', values: { uptime_s: 10 } },
  { timestamp: 2, topic: 'car/SE01/s', packet: 'status', values: { speed_mms: 150 } },
  { timestamp: 1, topic: 'car/SE01/s', packet: 'status', values: { speed_mms: 100 } },
];

describe('history view', () => {
  it('summarizes counts and newest rows', () => {
    const summary = summarizeHistory(records, 2);
    expect(summary.total).toBe(3);
    expect(summary.counts).toEqual({ heartbeat: 1, status: 2 });
    expect(summary.rows).toHaveLength(2);
    expect(summary.rows[0]?.packet).toBe('heartbeat');
    expect(summary.rows[1]?.detail).toBe('speed_mms=150');
  });

  it('fetches and filters history records', async () => {
    const fetched = await fetchHistory('http://lan', 10, async () => ({
      ok: true,
      status: 200,
      json: async () => [...records, { bogus: true }],
    }));
    expect(fetched).toHaveLength(3);
  });

  it('throws on a failed history request', async () => {
    await expect(
      fetchHistory('http://lan', 10, async () => ({
        ok: false,
        status: 503,
        json: async () => [],
      })),
    ).rejects.toThrow('history request failed: 503');
  });
});
