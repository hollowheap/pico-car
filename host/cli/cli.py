#!/usr/bin/env python3
"""Ground-station CLI for the pico-car bridge HTTP API."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

DEFAULT_URL = "http://127.0.0.1:5281"
POLL_S = 0.5
HTTP_TIMEOUT_S = 10


def _request(url: str, body: dict | None = None):
    data = None if body is None else json.dumps(body).encode()
    headers = {} if data is None else {"content-type": "application/json"}
    request = urllib.request.Request(url, data=data, headers=headers)
    with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_S) as response:
        return json.load(response)


def _error_text(exc: Exception) -> str:
    if isinstance(exc, urllib.error.HTTPError):
        try:
            detail = json.load(exc).get("error")
        except (ValueError, AttributeError):
            detail = None
        return f"bridge returned {exc.code}" + (f": {detail}" if detail else "")
    if isinstance(exc, urllib.error.URLError):
        return f"bridge unreachable: {exc.reason}"
    return f"bridge error: {exc}"


def _history(base: str, limit: int) -> list[dict]:
    return _request(f"{base}/history?limit={limit}")


def _print_row(row: dict) -> None:
    values = " ".join(f"{key}={value}" for key, value in row["values"].items())
    stamp = time.strftime("%H:%M:%S", time.localtime(row["timestamp"] / 1000))
    print(f"{stamp} {row['packet']:9s} {values}", flush=True)


def cmd_monitor(args) -> int:
    base = args.url.rstrip("/")
    try:
        rows = _history(base, 1)
        last = max((row["receiptOrder"] for row in rows), default=0)
        while True:
            time.sleep(POLL_S)
            try:
                rows = _history(base, args.limit)
            except (urllib.error.URLError, TimeoutError, ConnectionError) as exc:
                print(_error_text(exc), file=sys.stderr)
                continue
            fresh = sorted(
                (row for row in rows if row["receiptOrder"] > last),
                key=lambda row: row["receiptOrder"],
            )
            for row in fresh:
                _print_row(row)
            if fresh:
                last = fresh[-1]["receiptOrder"]
    except KeyboardInterrupt:
        return 0
    except (urllib.error.URLError, OSError, ValueError, KeyError) as exc:
        print(_error_text(exc), file=sys.stderr)
        return 1


def cmd_command(args) -> int:
    base = args.url.rstrip("/")
    try:
        reply = _request(f"{base}/command", {"action": args.name})
    except (urllib.error.URLError, OSError, ValueError) as exc:
        print(_error_text(exc), file=sys.stderr)
        return 1
    print(f"sent {args.name} cmd_id={reply.get('cmdId')}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="pico-car ground-station CLI (bridge HTTP API)"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get("BRIDGE_URL", DEFAULT_URL),
        help=f"bridge base URL (env BRIDGE_URL, default {DEFAULT_URL})",
    )

    sub = parser.add_subparsers(dest="action", required=True)
    monitor = sub.add_parser("monitor", help="print live telemetry")
    monitor.add_argument(
        "--limit",
        type=int,
        default=200,
        help="history rows fetched per poll",
    )

    command = sub.add_parser("command", help="send a command through the bridge")
    command.add_argument("name", help="command name from the protocol schema")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.action == "monitor":
        return cmd_monitor(args)
    if args.action == "command":
        return cmd_command(args)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
