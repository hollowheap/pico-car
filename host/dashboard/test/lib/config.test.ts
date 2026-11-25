import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { getLinkUrl, getServiceUrl } from '../../src/lib/config';

const originalService = process.env.EXPO_PUBLIC_SERVICE_URL;
const originalLink = process.env.EXPO_PUBLIC_LINK_URL;
const originalInsecure = process.env.EXPO_PUBLIC_ALLOW_INSECURE;

beforeEach(() => {
  delete process.env.EXPO_PUBLIC_ALLOW_INSECURE;
});

afterEach(() => {
  vi.unstubAllGlobals();
  if (originalService === undefined) delete process.env.EXPO_PUBLIC_SERVICE_URL;
  else process.env.EXPO_PUBLIC_SERVICE_URL = originalService;
  if (originalLink === undefined) delete process.env.EXPO_PUBLIC_LINK_URL;
  else process.env.EXPO_PUBLIC_LINK_URL = originalLink;
  if (originalInsecure === undefined) delete process.env.EXPO_PUBLIC_ALLOW_INSECURE;
  else process.env.EXPO_PUBLIC_ALLOW_INSECURE = originalInsecure;
});

describe('config', () => {
  it.each([
    ['http://127.0.0.1:5281/course', 'http://127.0.0.1:5281', 'ws://127.0.0.1:5281'],
    ['https://station.example/systems', 'https://station.example', 'wss://station.example'],
  ])('uses the web page origin for history, commands and telemetry at %s', (href, service, link) => {
    delete process.env.EXPO_PUBLIC_SERVICE_URL;
    delete process.env.EXPO_PUBLIC_LINK_URL;
    vi.stubGlobal('document', { location: new URL(href) });
    expect(getServiceUrl()).toBe(service);
    expect(getLinkUrl()).toBe(link);
    process.env.EXPO_PUBLIC_LINK_URL = 'broken';
    expect(getLinkUrl()).toBeNull();
  });

  it('requires configured endpoints when there is no browser document', () => {
    delete process.env.EXPO_PUBLIC_SERVICE_URL;
    delete process.env.EXPO_PUBLIC_LINK_URL;
    expect(getServiceUrl()).toBeNull();
    expect(getLinkUrl()).toBeNull();
  });

  it('accepts HTTPS and local HTTP only', () => {
    process.env.EXPO_PUBLIC_SERVICE_URL = 'https://car.lan:8443';
    expect(getServiceUrl()).toBe('https://car.lan:8443');
    process.env.EXPO_PUBLIC_SERVICE_URL = 'http://localhost:8080/';
    expect(getServiceUrl()).toBe('http://localhost:8080');
    process.env.EXPO_PUBLIC_SERVICE_URL = 'http://192.168.1.10:8080';
    expect(getServiceUrl()).toBeNull();

    process.env.EXPO_PUBLIC_ALLOW_INSECURE = '1';
    expect(getServiceUrl()).toBe('http://192.168.1.10:8080');
  });

  it('accepts WSS and local WS only', () => {
    process.env.EXPO_PUBLIC_LINK_URL = 'wss://car.lan/ws';
    expect(getLinkUrl()).toBe('wss://car.lan/ws');
    process.env.EXPO_PUBLIC_LINK_URL = 'ws://127.0.0.1:8124';
    expect(getLinkUrl()).toBe('ws://127.0.0.1:8124');
    process.env.EXPO_PUBLIC_LINK_URL = 'ws://192.168.1.10:8124';
    expect(getLinkUrl()).toBeNull();

    process.env.EXPO_PUBLIC_ALLOW_INSECURE = '1';
    expect(getLinkUrl()).toBe('ws://192.168.1.10:8124');
  });

  it.each([
    ['missing authority', 'https://', 'wss://'],
    ['missing slashes', 'https:car.lan', 'wss:car.lan'],
    ['path in place of authority', 'https:///car.lan', 'wss:///car.lan'],
    ['malformed host', 'https://car lan', 'wss://car lan'],
    ['invalid port', 'https://car.lan:65536', 'wss://car.lan:65536'],
    ['invalid IPv6 address', 'https://[::1', 'wss://[::1'],
    ['credentials', 'https://user@car.lan', 'wss://user@car.lan'],
    ['fragment', 'https://car.lan#', 'wss://car.lan#'],
  ])('rejects %s for both endpoints', (_label, service, link) => {
    process.env.EXPO_PUBLIC_SERVICE_URL = service;
    process.env.EXPO_PUBLIC_LINK_URL = link;
    expect(getServiceUrl()).toBeNull();
    expect(getLinkUrl()).toBeNull();
  });

  it('keeps the local IPv6 exception and validates even with insecure opt-in', () => {
    process.env.EXPO_PUBLIC_SERVICE_URL = 'http://[::1]:8080/';
    process.env.EXPO_PUBLIC_LINK_URL = 'ws://[::1]:8124/';
    expect(getServiceUrl()).toBe('http://[::1]:8080');
    expect(getLinkUrl()).toBe('ws://[::1]:8124');

    process.env.EXPO_PUBLIC_ALLOW_INSECURE = '1';
    process.env.EXPO_PUBLIC_SERVICE_URL = 'http://';
    process.env.EXPO_PUBLIC_LINK_URL = 'ws://car.lan:bad';
    expect(getServiceUrl()).toBeNull();
    expect(getLinkUrl()).toBeNull();
  });

  it('preserves a socket query and rejects queries on the service base', () => {
    process.env.EXPO_PUBLIC_SERVICE_URL = 'https://car.lan/api?view=all';
    process.env.EXPO_PUBLIC_LINK_URL = 'WSS://car.lan/ws?view=test/';
    expect(getServiceUrl()).toBeNull();
    expect(getLinkUrl()).toBe('WSS://car.lan/ws?view=test/');
  });
});
