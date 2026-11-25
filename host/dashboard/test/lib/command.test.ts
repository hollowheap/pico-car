import { describe, expect, it, vi } from 'vitest';

import { postCommand } from '../../src/lib/command';
import { COMMANDS, EVENTS } from '../../src/lib/schema.generated';
import { emptyStore, reduceStore } from '../../src/lib/telemetry-store';
import { correlateCommandReply } from '../../src/lib/useCommand';

describe('command transport and ACK correlation', () => {
  it('returns cmdId from POST /command and aborts on timeout', async () => {
    const okFetch = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({ ok: true, cmdId: 42 }),
    });
    vi.stubGlobal('fetch', okFetch);
    try {
      await expect(postCommand('http://bridge.local', { action: 'start' })).resolves.toEqual({ cmdId: 42 });

      const hangingFetch = vi.fn((_url: string, init?: RequestInit) => new Promise((_resolve, reject) => {
        init?.signal?.addEventListener('abort', () => reject(Object.assign(new Error('Aborted'), { name: 'AbortError' })));
      }));
      vi.stubGlobal('fetch', hangingFetch);
      await expect(postCommand('http://bridge.local', { action: 'start' }, 20)).rejects.toThrow(/timed out/i);
    } finally {
      vi.unstubAllGlobals();
    }
  });

  it('correlates cmd_accept and cmd_reject events by cmdId', () => {
    const pending = { action: 'start' as const, label: 'start', text: 'start sent to bridge.', error: false, resetAt: null, cmdId: 77 };
    const otherAccept = [
      { packet: 'event', timestamp: 100, values: { event_id: 1, event_type: EVENTS.cmdAccept, p1: (1 << 8) | COMMANDS.start, p2: 76 } },
    ].reduce(reduceStore, emptyStore()).events;
    expect(correlateCommandReply(pending, otherAccept)).toEqual({ text: 'start sent to bridge.', error: false });

    const matchedAccept = [
      { packet: 'event', timestamp: 200, values: { event_id: 2, event_type: EVENTS.cmdAccept, p1: (1 << 8) | COMMANDS.start, p2: 77 } },
    ].reduce(reduceStore, emptyStore()).events;
    expect(correlateCommandReply(pending, matchedAccept)).toEqual({ text: 'start accepted by car.', error: false });

    const matchedReject = [
      { packet: 'event', timestamp: 300, values: { event_id: 3, event_type: EVENTS.cmdReject, p1: (2 << 12) | (1 << 8) | COMMANDS.start, p2: 77 } },
    ].reduce(reduceStore, emptyStore()).events;
    const rejected = correlateCommandReply(pending, matchedReject);
    expect(rejected.error).toBe(true);
    expect(rejected.text).toMatch(/refused/i);
  });
});
