import { describe, expect, it } from "vitest";
import { encodeLivePacket, encodeLiveStatus } from "./live.js";

describe("live packets", () => {
  it("keeps the history receipt ID in the dashboard message envelope", () => {
    expect(
      JSON.parse(
        encodeLivePacket({
          timestamp: 10,
          topic: "car/s",
          packet: "status",
          receiptId: "first-receipt",
          receiptOrder: 20,
          timelineMs: 12,
          values: { speed_mms: 150 },
        }),
      ),
    ).toEqual({
      type: "packet",
      timestamp: 10,
      topic: "car/s",
      packet: "status",
      receiptId: "first-receipt",
      receiptOrder: 20,
      timelineMs: 12,
      values: { speed_mms: 150 },
    });
  });

  it("wraps the bridge status in the dashboard message envelope", () => {
    expect(
      JSON.parse(
        encodeLiveStatus({
          mqtt: "connected",
          packets: 3,
          lastPacketAt: 10,
          historyStorage: "degraded",
        }),
      ),
    ).toEqual({
      type: "status",
      mqtt: "connected",
      packets: 3,
      lastPacketAt: 10,
      historyStorage: "degraded",
    });
  });
});
