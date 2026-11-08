import { describe, expect, it } from "vitest";
import { encodeLivePacket, encodeLiveStatus } from "./live.js";

describe("live packets", () => {
  it("wraps decoded packets in the dashboard message envelope", () => {
    expect(
      JSON.parse(
        encodeLivePacket({
          timestamp: 10,
          packet: "status",
          values: { speed_mms: 150 },
        }),
      ),
    ).toEqual({
      type: "packet",
      timestamp: 10,
      packet: "status",
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
        }),
      ),
    ).toEqual({
      type: "status",
      mqtt: "connected",
      packets: 3,
      lastPacketAt: 10,
    });
  });
});
