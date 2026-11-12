import { describe, expect, it } from 'vitest';

import { panelStatus } from './panel-state';

const packet = (timestamp: number) => ({ timestamp, packet: 'status', values: {} });

describe('panelStatus', () => {
  it('waits while the link is open and nothing arrived', () => {
    expect(panelStatus('open', undefined, 1000, 'status').kind).toBe('loading');
  });
  it('is empty while the link is down and nothing arrived', () => {
    expect(panelStatus('disconnected', undefined, 1000, 'status').kind).toBe('empty');
  });
  it('goes stale after the limit of the packet kind', () => {
    expect(panelStatus('open', packet(0), 1000, 'status').kind).toBe('ready');
    expect(panelStatus('open', packet(0), 2000, 'status').kind).toBe('stale');
    expect(panelStatus('open', packet(0), 3000, 'obstacle').kind).toBe('ready');
  });
  it('never marks a profile stale', () => {
    expect(panelStatus('open', packet(0), 999999, 'profile').kind).toBe('ready');
  });
});
