import { expect, it } from 'vitest';
import { createConsoleView } from '../../../bridge/src/console/view.js';
import { consoleUpdate } from '../../../bridge/src/console/update.js';
import { applyConsoleUpdate, type ConsoleUpdate } from '../../src/lib/console-view';

const wire = (update: ConsoleUpdate): ConsoleUpdate => JSON.parse(JSON.stringify(update));

it('resumes retained frame blocks after skipped updates and accepts a fresh snapshot on reconnect', () => {
  const frames = Array.from({ length: 5102 }, (_, index) => ({
    packet: 'diag', timestamp: 1000 + index, values: { stack_words: index },
  }));
  const console = createConsoleView(frames.slice(0, 5000));
  const before = console.read();
  const initial = applyConsoleUpdate(null, wire(consoleUpdate(before)));
  for (const frame of frames.slice(5000)) console.append(frame);
  const after = console.read();
  const update = wire(consoleUpdate(after, before));
  expect(applyConsoleUpdate(null, update)).toBeNull();
  expect(applyConsoleUpdate(initial, update)).toEqual(applyConsoleUpdate(null, wire(consoleUpdate(after))));
  expect(after.frameLog.nextId).toBe(5102);
  expect(after.frameLog.blocks[0]?.[0]?.values.stack_words).toBe(100);
});
