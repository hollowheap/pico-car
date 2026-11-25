import { COMMANDS } from './schema.generated';

export type CommandAction = keyof typeof COMMANDS;

export type CommandRequest = {
  action: CommandAction;
};

export async function postCommand(
  serviceUrl: string,
  request: CommandRequest,
): Promise<void> {
  const base = serviceUrl.replace(/\/+$/, '');
  const response = await fetch(`${base}/command`, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(request),
  });
  if (!response.ok) {
    const body = await response.text();
    throw new Error(body || `command failed: ${response.status}`);
  }
}
