import { createContext, useContext, type ReactNode } from 'react';

import { getLinkUrl, getServiceUrl } from './config';
import { emptyStore } from './telemetry-store';
import { useLink, type LinkState } from './useLink';

const initial: LinkState = {
  snapshot: {
    status: 'disconnected',
    lastPacketAt: null,
    lastPacket: null,
    bridge: { mqtt: 'unknown', packets: 0, lastPacketAt: null },
  },
  packets: [],
  store: emptyStore(),
};

const TelemetryContext = createContext<LinkState>(initial);

/**
 * One link and one store for every screen, so the trail and the events survive
 * a switch between Live, Systems and Run.
 */
export function TelemetryProvider({ children }: { children: ReactNode }) {
  const state = useLink(getLinkUrl() ?? undefined, getServiceUrl() ?? undefined);
  return <TelemetryContext.Provider value={state}>{children}</TelemetryContext.Provider>;
}

export function useTelemetry(): LinkState {
  return useContext(TelemetryContext);
}
