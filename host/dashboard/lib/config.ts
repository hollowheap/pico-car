const LOCAL_HOST = /^\/\/(localhost|127\.0\.0\.1|\[::1\])(?::\d+)?(\/|$)/i;

export const SERVICE_CONFIG_ERROR =
  'Set EXPO_PUBLIC_SERVICE_URL to HTTPS. HTTP is allowed for localhost or with EXPO_PUBLIC_ALLOW_INSECURE=1.';
export const LINK_CONFIG_ERROR =
  'Set EXPO_PUBLIC_LINK_URL to WSS. WS is allowed for localhost or with EXPO_PUBLIC_ALLOW_INSECURE=1.';

function allowInsecure(): boolean {
  return process.env.EXPO_PUBLIC_ALLOW_INSECURE === '1';
}

function readUrl(
  value: string | undefined,
  secure: string,
  local: string,
): string | null {
  if (!value) {
    return null;
  }
  const lower = value.toLowerCase();
  if (lower.startsWith(secure)) {
    return value.replace(/\/+$/, '');
  }
  if (
    lower.startsWith(local) &&
    (allowInsecure() || LOCAL_HOST.test(value.slice(local.length)))
  ) {
    return value.replace(/\/+$/, '');
  }
  return null;
}

export function getServiceUrl(): string | null {
  return readUrl(process.env.EXPO_PUBLIC_SERVICE_URL, 'https:', 'http:');
}

export function getLinkUrl(): string | null {
  return readUrl(process.env.EXPO_PUBLIC_LINK_URL, 'wss:', 'ws:');
}
