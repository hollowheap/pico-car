import { COMMANDS } from './schema.generated';

export type CommandAction = keyof typeof COMMANDS;

export type CommandRequest = {
  action: CommandAction;
};

export const COMMAND_TIMEOUT_MS = 5000;

export type CommandResponse = {
  cmdId?: number;
};

export async function postCommand(
  serviceUrl: string,
  request: CommandRequest,
  timeoutMs = COMMAND_TIMEOUT_MS,
): Promise<CommandResponse> {
  const base = serviceUrl.replace(/\/+$/, '');
  const controller = typeof AbortController === 'function' ? new AbortController() : undefined;
  const timer = controller ? setTimeout(() => controller.abort(), timeoutMs) : undefined;
  try {
    const response = await fetch(`${base}/command`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify(request),
      ...(controller ? { signal: controller.signal } : {}),
    });
    if (!response.ok) {
      const body = await response.text();
      throw new Error(body || `command failed: ${response.status}`);
    }
    try {
      const data = await response.json() as { cmdId?: unknown } | null;
      return typeof data?.cmdId === 'number' ? { cmdId: data.cmdId } : {};
    } catch {
      return {};
    }
  } catch (error) {
    if ((error as Error)?.name === 'AbortError') {
      throw new Error(`command timed out after ${Math.round(timeoutMs / 1000)} s`);
    }
    throw error;
  } finally {
    if (timer !== undefined) clearTimeout(timer);
  }
}
