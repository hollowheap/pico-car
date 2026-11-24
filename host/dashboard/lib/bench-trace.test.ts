import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { parseBenchTrace } from './bench-trace';

const trace = readFileSync(new URL('../../../tests/fixtures/bench-trace.jsonl', import.meta.url), 'utf8');

describe('bench trace import', () => {
  it('keeps measured angles separate and leaves unmatched measurements unavailable', () => {
    const parsed = parseBenchTrace(trace.replace('{"kind":"end"', [
      '{"kind":"reference","trial":"another-trial","cue":"wing","source":"other gauge","angleDeci":500}',
      '{"kind":"end"',
    ].join('\n')));

    expect(parsed.header).toMatchObject({ build: 'abc123', fixture: 'servo', trial: 'center-return' });
    expect(parsed.samples).toMatchObject([
      { seq: 0, cue: 'zero', timeUs: 100, pwmUs: 1494, requestedDeci: 0, effectiveDeci: 0, measurement: { angleDeci: 7, source: 'angle gauge' } },
      { seq: 1, cue: 'wing', timeUs: 200, pwmUs: 1382, requestedDeci: 200, effectiveDeci: 200, measurement: null },
    ]);
    expect(parsed).toMatchObject({ dropped: 0, missing: 0, incomplete: false });
  });

  it('reports incomplete capture and rejects an unsupported schema', () => {
    expect(parseBenchTrace(trace.replace('"dropped":0', '"dropped":3'))).toMatchObject({ dropped: 3, incomplete: true });
    expect(parseBenchTrace(trace.replace('"seq":1', '"seq":3'))).toMatchObject({ dropped: 0, missing: 2, incomplete: true });
    expect(() => parseBenchTrace(trace.replace('"version":1', '"version":99'))).toThrow(/version/i);
  });

  it('accepts the size boundary and rejects larger UTF-8 traces before parsing', () => {
    const limit = 128 * 1024;
    const boundary = trace.padEnd(limit, ' ');
    const multibyte = trace.replace('angle gauge', 'é'.repeat(limit / 2));

    expect(parseBenchTrace(boundary).samples).toHaveLength(2);
    expect(() => parseBenchTrace('!' + boundary)).toThrow(/128 KiB/i);
    expect(multibyte.length).toBeLessThan(limit);
    expect(() => parseBenchTrace(multibyte)).toThrow(/128 KiB/i);
  });

  it('preserves scheduler, trigger and reply fields from the device export', () => {
    const exported = [
      '{"kind":"header","version":1,"build":"abc1234+dirty","fixture":"bench-timing","trial":"timing-000b7e3c"}',
      '{"kind":"scheduler","seq":0,"timeUs":29,"marker":"release"}',
      '{"kind":"scheduler","seq":1,"timeUs":35,"marker":"capture"}',
      '{"kind":"scheduler","seq":2,"timeUs":36,"marker":"complete"}',
      '{"kind":"scheduler","seq":3,"timeUs":36,"marker":"contention","cue":"lock"}',
      '{"kind":"trigger","seq":4,"timeUs":40,"requestedDeci":-350,"effectiveDeci":-300}',
      '{"kind":"reply","seq":5,"timeUs":4294967295,"requestedDeci":-350,"effectiveDeci":-300,"echo":"valid","rangeMm":120}',
      '{"kind":"end","dropped":0}',
    ].join('\r\n');

    expect(parseBenchTrace(exported).samples).toMatchObject([
      { kind: 'scheduler', timeUs: 29, marker: 'release', measurement: null },
      { kind: 'scheduler', timeUs: 35, marker: 'capture', measurement: null },
      { kind: 'scheduler', timeUs: 36, marker: 'complete', measurement: null },
      { kind: 'scheduler', timeUs: 36, marker: 'contention', cue: 'lock', measurement: null },
      { kind: 'trigger', timeUs: 40, requestedDeci: -350, effectiveDeci: -300, measurement: null },
      { kind: 'reply', timeUs: 4294967295, echo: 'valid', rangeMm: 120, measurement: null },
    ]);
  });

  it.each([
    ['interrupted export', trace.replace('{"kind":"end","dropped":0}', '')],
    ['data past the trailer', trace + '{"kind":"end","dropped":0}'],
    ['non-integer timestamp', trace.replace('"timeUs":200', '"timeUs":200.5')],
    ['reversed device time', trace.replace('"timeUs":200', '"timeUs":99')],
    ['reused sequence', trace.replace('"seq":1', '"seq":0')],
    ['competing reference', trace.replace('{"kind":"end"', [
      '{"kind":"reference","trial":"center-return","cue":"zero","source":"second gauge","angleDeci":19}',
      '{"kind":"end"',
    ].join('\n'))],
  ])('rejects %s', (_name, damaged) => {
    expect(() => parseBenchTrace(damaged)).toThrow();
  });
});
