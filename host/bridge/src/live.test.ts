import { describe, expect, it } from "vitest";
import { encodeLivePacket } from "./live.js";

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
});
