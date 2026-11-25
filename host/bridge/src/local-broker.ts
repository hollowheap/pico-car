import { spawn, type ChildProcess } from "node:child_process";
import { once } from "node:events";
import { connect } from "node:net";
import { fileURLToPath } from "node:url";

/** Leave an existing listener under its owner's control. */
export async function startLocalBroker(): Promise<ChildProcess | undefined> {
  const listening = await new Promise<boolean>((resolve, reject) => {
    const socket = connect(1883, "127.0.0.1");
    socket.once("connect", () => {
      socket.destroy();
      resolve(true);
    });
    socket.once("error", (error: NodeJS.ErrnoException) => {
      if (error.code === "ECONNREFUSED") resolve(false);
      else reject(error);
    });
  });
  if (listening) return;

  const config = fileURLToPath(new URL("../../mosquitto.conf", import.meta.url));
  const broker = spawn("mosquitto", ["-c", config], { stdio: "inherit" });
  try {
    await once(broker, "spawn");
  } catch (error) {
    throw new Error("Could not start Mosquitto. Install it or set MQTT_URL to a running broker.", { cause: error });
  }
  return broker;
}
