import type { EventRow } from './telemetry-store.js';

type CommandOutcome = { accepted: boolean; detail: string };
export type CommandReplies = Record<string, CommandOutcome | undefined>;

export function commandReplies(events: readonly EventRow[]): CommandReplies {
  const replies: CommandReplies = {};
  for (const event of events) {
    if (event.name !== 'cmdAccept' && event.name !== 'cmdReject') continue;
    const outcome = { accepted: event.name === 'cmdAccept', detail: event.text.detail };
    replies[`${event.p2}`] = outcome;
    replies[`${event.p1 & 0xff}:${event.p2}`] = outcome;
  }
  return replies;
}
