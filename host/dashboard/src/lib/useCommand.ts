import { useRef, useState } from 'react';

import { postCommand, type CommandAction } from './command';
import { SERVICE_CONFIG_ERROR } from './config';
import { COMMANDS } from './schema.generated';
import type { ConsoleView } from './console-view';

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
  replies: ConsoleView['replies'],
): { text: string; error: boolean } {
  if (reply.cmdId === undefined || reply.error) {
    return { text: reply.text, error: reply.error };
  }
  const opcode = reply.action === undefined ? undefined : COMMANDS[reply.action];
  const outcome = replies[opcode === undefined ? `${reply.cmdId}` : `${opcode}:${reply.cmdId}`];
  if (outcome === undefined) return { text: reply.text, error: reply.error };
  const label = reply.label ?? reply.action ?? 'command';
  if (outcome.accepted) return { text: `${label} accepted by car.`, error: false };
  const detail = outcome.detail ? `: ${outcome.detail}` : '.';
  return { text: `${label} refused by car${detail}`, error: true };
}

export function useCommand(
  serviceUrl: string | null,
  resetAt: number | null,
  replies: ConsoleView['replies'] = {},
) {
  const [reply, setReply] = useState<PendingCommandReply | null>(null);
  const latestRequest = useRef(0);

  const send = (action: CommandAction) => {
    const label = action.replaceAll('_', ' ');
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

  const current = reply?.resetAt === resetAt ? correlateCommandReply(reply, replies) : null;
  return { feedback: current?.text ?? null, feedbackError: current?.error ?? false, send };
}
