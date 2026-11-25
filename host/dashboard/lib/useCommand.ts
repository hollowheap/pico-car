import { useState } from 'react';

import type { LinkPacket } from './link';
import {
  deviceSentMs,
  postCommand,
  type CommandAction,
} from './command';
import { SERVICE_CONFIG_ERROR } from './config';

export function useCommand(
  serviceUrl: string | null,
  packets: LinkPacket[],
) {
  const [feedback, setFeedback] = useState<string | null>(null);
  const [feedbackError, setFeedbackError] = useState(false);

  const send = (action: CommandAction) => {
    if (serviceUrl === null) {
      setFeedback(SERVICE_CONFIG_ERROR);
      setFeedbackError(true);
      return;
    }
    const sentMs = deviceSentMs(packets, Date.now());
    if (sentMs === null) {
      setFeedback('No fresh heartbeat. Command not sent.');
      setFeedbackError(true);
      return;
    }
    setFeedback(`Sending ${action}...`);
    setFeedbackError(false);
    const request = {
      action,
      cmdId: Date.now() & 0xffff,
      runId: 0,
      sentMs,
    };
    void postCommand(serviceUrl, request)
      .then(() => setFeedback(`${action} sent to bridge.`))
      .catch((reason: Error) => {
        setFeedback(reason.message);
        setFeedbackError(true);
      });
  };

  return { feedback, feedbackError, send };
}
