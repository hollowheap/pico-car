import { useState } from 'react';

import { postCommand, type CommandAction } from './command';
import { SERVICE_CONFIG_ERROR } from './config';

export function useCommand(serviceUrl: string | null) {
  const [feedback, setFeedback] = useState<string | null>(null);
  const [feedbackError, setFeedbackError] = useState(false);

  const send = (action: CommandAction) => {
    if (serviceUrl === null) {
      setFeedback(SERVICE_CONFIG_ERROR);
      setFeedbackError(true);
      return;
    }
    setFeedback(`Sending ${action}...`);
    setFeedbackError(false);
    // The bridge numbers the command and adds the car's run ID; it refuses
    // the command without a fresh heartbeat.
    void postCommand(serviceUrl, { action })
      .then(() => setFeedback(`${action} sent to bridge.`))
      .catch((reason: Error) => {
        setFeedback(reason.message);
        setFeedbackError(true);
      });
  };

  return { feedback, feedbackError, send };
}
