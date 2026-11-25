import { useRef, useState } from 'react';

import { postCommand, type CommandAction } from './command';
import { SERVICE_CONFIG_ERROR } from './config';
import { COMMANDS } from './schema.generated';
import type { EventRow } from './telemetry-store';

export type PendingCommandReply = {
  action?: CommandAction;
  label?: string;
  text: string;
  error: boolean;
  resetAt: number | null;
  cmdId?: number;
};

export function correlateCommandReply(
  reply: PendingCommandReply,
  events: readonly EventRow[],
): { text: string; error: boolean } {
  if (reply.cmdId === undefined || reply.error) {
    return { text: reply.text, error: reply.error };
  }
  const opcode = reply.action === undefined ? undefined : COMMANDS[reply.action];
  const label = reply.label ?? reply.action ?? 'command';
  for (let i = events.length - 1; i >= 0; i -= 1) {
    const event = events[i]!;
    if (event.name !== 'cmdAccept' && event.name !== 'cmdReject') continue;
    if (event.p2 !== reply.cmdId) continue;
    if (opcode !== undefined && (event.p1 & 0xff) !== opcode) continue;
    if (event.name === 'cmdAccept') {
      return { text: `${label} accepted by car.`, error: false };
    }
    const detail = event.text.detail ? `: ${event.text.detail}` : '.';
    return { text: `${label} refused by car${detail}`, error: true };
  }
  return { text: reply.text, error: reply.error };
}

export function useCommand(
  serviceUrl: string | null,
  resetAt: number | null,
  events: readonly EventRow[] = [],
) {
  const [reply, setReply] = useState<PendingCommandReply | null>(null);
  const latestRequest = useRef(0);

  const send = (action: CommandAction) => {
    const label = action === 'start_stationary' ? 'Stop-at-obstacles start' : action.replaceAll('_', ' ');
    const request = ++latestRequest.current;
    const showReply = (text: string, error: boolean, cmdId?: number) => {
      if (request === latestRequest.current) setReply({ action, label, text, error, resetAt, cmdId });
    };
    if (serviceUrl === null) {
      showReply(SERVICE_CONFIG_ERROR, true);
      return;
    }
    showReply(`Sending ${label}...`, false);
    // The bridge numbers the command and adds the car's run ID; it refuses
    // the command without a fresh heartbeat.
    void postCommand(serviceUrl, { action })
      .then(({ cmdId }) => showReply(`${label} sent to bridge.`, false, cmdId))
      .catch((reason: Error) => showReply(reason.message, true));
  };

  const current = reply?.resetAt === resetAt ? correlateCommandReply(reply, events) : null;
  return { feedback: current?.text ?? null, feedbackError: current?.error ?? false, send };
}
