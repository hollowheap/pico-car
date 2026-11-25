#!/usr/bin/env python3
"""Ground-station CLI for the pico-car MQTT telemetry and commands."""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path

try:
    from .mqtt import MQTTError, MqttClient
    from .schema import (
        decode_packet,
        encode_command,
        load_schema,
        packet_name_for_topic,
    )
except ImportError:  # direct execution: python3 host/cli/cli.py
    from mqtt import MQTTError, MqttClient
    from schema import (
        decode_packet,
        encode_command,
        load_schema,
        packet_name_for_topic,
    )

DEFAULT_SCHEMA = Path(__file__).resolve().parents[2] / "protocol" / "telemetry.yaml"


def _topic_prefix(value: str) -> str:
    return value.rstrip("/")


def local_host() -> str:
    """Best-effort IPv4 address of this host's default route."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("1.1.1.1", 80))
            return sock.getsockname()[0]
    except OSError:
        return "localhost"


def cmd_monitor(args, schema) -> int:
    prefix = _topic_prefix(args.prefix)

    def on_message(topic: str, payload: bytes) -> None:
        name = packet_name_for_topic(schema, topic)
        if name is None:
            return
        try:
            packet = decode_packet(schema, name, payload)
        except ValueError as exc:
            print(f"{topic}: {exc}", file=sys.stderr)
            return
        values = " ".join(
            f"{key}={value}" for key, value in packet["values"].items()
        )
        stamp = time.strftime("%H:%M:%S")
        print(f"{stamp} {name:9s} {values}")

    client = MqttClient(
        args.host,
        args.port,
        f"pico-car-cli-{os.getpid()}",
        args.username,
        args.password,
        args.keepalive,
        on_message,
    )
    try:
        client.run_forever(f"{prefix}/#")
    except KeyboardInterrupt:
        pass
    return 0


def _device_sent_ms(client, schema, prefix: str, timeout: float = 2.0):
    """Approximate the device's 16-bit millisecond clock from a heartbeat."""
    heartbeat = {}

    def on_message(topic: str, payload: bytes) -> None:
        if packet_name_for_topic(schema, topic) != "heartbeat":
            return
        try:
            values = decode_packet(schema, "heartbeat", payload)["values"]
        except ValueError:
            return
        heartbeat["uptime_s"] = values["uptime_s"]
        heartbeat["received"] = time.monotonic()

    client.on_message = on_message
    client.subscribe(f"{prefix}/{schema['topics']['heartbeat']}", qos=0)
    deadline = time.monotonic() + timeout
    while "received" not in heartbeat and time.monotonic() < deadline:
        client.loop(0.1)
    if "received" not in heartbeat:
        return None
    elapsed_ms = (time.monotonic() - heartbeat["received"]) * 1000.0
    return int(heartbeat["uptime_s"] * 1000 + elapsed_ms) & 0xFFFF


def cmd_command(args, schema) -> int:
    if args.name not in schema["commands"]:
        choices = ", ".join(sorted(schema["commands"]))
        print(f"unknown command {args.name!r}; choose: {choices}",
              file=sys.stderr)
        return 2

    prefix = _topic_prefix(args.prefix)
    cmd_id = args.cmd_id
    if cmd_id is None:
        cmd_id = int(time.time() * 1000) & 0xFFFF

    client = MqttClient(
        args.host,
        args.port,
        f"pico-car-cli-{os.getpid()}",
        args.username,
        args.password,
        args.keepalive,
    )
    topic = f"{prefix}/{schema['topics']['command']}"
    try:
        client.connect()
        sent_ms = args.sent_ms
        if sent_ms is None:
            sent_ms = _device_sent_ms(client, schema, prefix)
            if sent_ms is None:
                print("no heartbeat received; pass --sent-ms", file=sys.stderr)
                return 1
        payload = encode_command(schema, args.name, cmd_id, args.run_id,
                                 sent_ms)
        packet_id = client.publish(topic, payload, qos=1)
        if not client.wait_for_puback(packet_id):
            print("command not acknowledged", file=sys.stderr)
            return 1
        print(f"sent {args.name} cmd_id={cmd_id} run_id={args.run_id} "
              f"sent_ms={sent_ms}")
        return 0
    except (OSError, MQTTError) as exc:
        print(f"mqtt error: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="pico-car ground-station CLI (direct MQTT)"
    )
    parser.add_argument(
        "--host", default=None, help="broker host (default: this host)"
    )
    parser.add_argument("--port", type=int, default=1883, help="broker port")
    parser.add_argument("--username", default=None)
    parser.add_argument("--password", default=None)
    parser.add_argument("--prefix", default="car/PICO-CAR", help="topic prefix")
    parser.add_argument("--keepalive", type=int, default=30)
    parser.add_argument(
        "--schema",
        default=str(DEFAULT_SCHEMA),
        help="path to protocol/telemetry.yaml",
    )

    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("monitor", help="decode live telemetry")

    command = sub.add_parser("command", help="send a validated command")
    command.add_argument("name", help="command name from the protocol schema")
    command.add_argument("--run-id", type=int, default=0)
    command.add_argument("--cmd-id", type=int, default=None)
    command.add_argument(
        "--sent-ms",
        type=int,
        default=None,
        help="override the heartbeat-synchronized 16-bit timestamp",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.host is None:
        args.host = local_host()
    schema = load_schema(args.schema)

    if args.action == "monitor":
        return cmd_monitor(args, schema)
    if args.action == "command":
        return cmd_command(args, schema)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
