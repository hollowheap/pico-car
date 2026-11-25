const AUTHORITY = /^[a-z]+:\/\/([^/?#\\]+)/i;
const LOCAL_HOST = /^(?:localhost|127\.0\.0\.1|\[::1\])(?::\d+)?$/i;

export const SERVICE_CONFIG_ERROR =
  'Set EXPO_PUBLIC_SERVICE_URL to a complete HTTPS base URL. HTTP is allowed for localhost or with EXPO_PUBLIC_ALLOW_INSECURE=1.';
export const LINK_CONFIG_ERROR =
  'Set EXPO_PUBLIC_LINK_URL to a complete WSS URL. WS is allowed for localhost or with EXPO_PUBLIC_ALLOW_INSECURE=1.';

function allowInsecure(): boolean {
  return process.env.EXPO_PUBLIC_ALLOW_INSECURE === '1';
}

function readUrl(
  value: string | undefined,
  secure: string,
  local: string,
): string | null {
  if (!value || /[\s\\#]/.test(value) || (secure === 'https:' && value.includes('?'))) {
    return null;
  }

  // URL repairs forms such as https:///host. Require the authority as written.
  const authority = AUTHORITY.exec(value)?.[1];
  if (!authority || authority.includes('@')) {
    return null;
  }

  let url: URL;
  try {
    url = new URL(value);
  } catch {
    return null;
  }
  if (!url.hostname || (url.protocol !== secure && url.protocol !== local)) {
    return null;
  }
  if (url.protocol === local && !allowInsecure() && !LOCAL_HOST.test(authority)) {
    return null;
  }

  return value.includes('?') ? value : value.replace(/\/+$/, '');
}

export function getServiceUrl(): string | null {
  const origin = typeof document === 'undefined' ? undefined : document.location?.origin;
  return readUrl(process.env.EXPO_PUBLIC_SERVICE_URL ?? origin, 'https:', 'http:');
}

export function getLinkUrl(): string | null {
  const origin = typeof document === 'undefined' ? undefined : document.location?.origin;
  return readUrl(process.env.EXPO_PUBLIC_LINK_URL ?? origin?.replace(/^http/, 'ws'), 'wss:', 'ws:');
}
