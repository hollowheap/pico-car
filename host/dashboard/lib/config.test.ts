import { afterEach, describe, expect, it } from 'vitest';
import { getLinkUrl, getServiceUrl } from './config';

const originalService = process.env.EXPO_PUBLIC_SERVICE_URL;
const originalLink = process.env.EXPO_PUBLIC_LINK_URL;
const originalInsecure = process.env.EXPO_PUBLIC_ALLOW_INSECURE;

afterEach(() => {
  if (originalService === undefined) delete process.env.EXPO_PUBLIC_SERVICE_URL;
  else process.env.EXPO_PUBLIC_SERVICE_URL = originalService;
  if (originalLink === undefined) delete process.env.EXPO_PUBLIC_LINK_URL;
  else process.env.EXPO_PUBLIC_LINK_URL = originalLink;
  if (originalInsecure === undefined) delete process.env.EXPO_PUBLIC_ALLOW_INSECURE;
  else process.env.EXPO_PUBLIC_ALLOW_INSECURE = originalInsecure;
});

describe('config', () => {
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
  });
});
