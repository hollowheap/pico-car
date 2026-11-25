export type LinkPacket = {
  timestamp: number;
  packet: string;
  topic?: string;
  receiptId?: string;
  receiptOrder?: number;
  timelineMs?: number;
  receivedAt?: number;
  values: Record<string, number>;
};

export function packetOnTimeline(packet: LinkPacket): LinkPacket {
  const { timelineMs, timestamp } = packet;
  return timelineMs === undefined || !Number.isFinite(timelineMs) || timelineMs === timestamp ? packet :
    { ...packet, timestamp: timelineMs, receivedAt: timestamp };
}
