"""Schema-driven decode/encode for the pico-car wire protocol.

The protocol YAML is the single source of truth.  This module reads only the
subset of YAML used by ``protocol/telemetry.yaml`` so the CLI has no third
party dependency.
"""

from __future__ import annotations

import struct
from pathlib import Path

_FIELD_TYPES = {
    "int8": (">b", 1),
    "uint8": (">B", 1),
    "int16": (">h", 2),
    "uint16": (">H", 2),
    "int32": (">i", 4),
    "uint32": (">I", 4),
}


class SchemaError(ValueError):
    """Raised when the protocol schema is malformed."""


def _scalar(value: str):
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    try:
        return int(value, 0)
    except ValueError:
        return value


def _strip_comment(line: str) -> str:
    return line.split("#", 1)[0].rstrip()


def _inline_mapping(body: str) -> dict:
    fields = {}
    for part in body.strip().strip("{}").split(","):
        key, _, value = part.partition(":")
        if not key or not _:
            raise SchemaError(f"bad inline mapping: {part!r}")
        fields[key.strip()] = _scalar(value)
    return fields


def load_schema(path: str | Path) -> dict:
    """Load topics, packets, and commands from the protocol YAML."""
    schema = {"topics": {}, "packets": {}, "commands": {}}
    section = None
    packet = None
    in_fields = False

    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = _strip_comment(raw)
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        text = line.strip()

        if indent == 0:
            section = text[:-1] if text.endswith(":") else None
            packet = None
            in_fields = False
            continue

        if section == "topics":
            key, _, value = text.partition(":")
            schema["topics"][key.strip()] = _scalar(value)
        elif section == "commands":
            key, _, value = text.partition(":")
            schema["commands"][key.strip()] = _scalar(value)
        elif section == "packets":
            if indent == 2 and text.endswith(":"):
                packet = text[:-1].strip()
                schema["packets"][packet] = {"wire_size": 0, "fields": []}
                in_fields = False
            elif packet is None:
                continue
            elif indent == 4 and text.startswith("wire_size:"):
                schema["packets"][packet]["wire_size"] = int(
                    text.split(":", 1)[1], 0
                )
            elif indent == 4 and text == "fields:":
                in_fields = True
            elif in_fields and text.startswith("-"):
                schema["packets"][packet]["fields"].append(
                    _inline_mapping(text[1:])
                )

    _validate(schema)
    return schema


def _validate(schema: dict) -> None:
    if not schema["topics"] or not schema["packets"]:
        raise SchemaError("schema needs topics and packets")
    for name, packet in schema["packets"].items():
        if not packet["fields"] or packet["wire_size"] <= 0:
            raise SchemaError(f"packet {name} has no fields or wire size")
        for field in packet["fields"]:
            if field.get("type") not in _FIELD_TYPES:
                raise SchemaError(
                    f"{name}.{field.get('name')} has unknown type"
                )
            if "offset" not in field or "name" not in field:
                raise SchemaError(f"{name} has an incomplete field")
            count = field.get("count", 1)
            size = _FIELD_TYPES[field["type"]][1]
            if int(field["offset"]) + size * int(count) > packet["wire_size"]:
                raise SchemaError(f"{name}.{field['name']} overruns the packet")


def packet_name_for_topic(schema: dict, topic: str) -> str | None:
    for name, suffix in schema["topics"].items():
        if name == "command":
            continue
        if topic.endswith("/" + str(suffix)):
            return name
    return None


def decode_packet(schema: dict, name: str, payload: bytes) -> dict:
    try:
        packet = schema["packets"][name]
    except KeyError as exc:
        raise SchemaError(f"unknown packet: {name}") from exc
    if len(payload) != packet["wire_size"]:
        raise SchemaError(
            f"{name} is {len(payload)} bytes, expected {packet['wire_size']}"
        )

    values = {}
    for field in packet["fields"]:
        fmt, size = _FIELD_TYPES[field["type"]]
        offset = int(field["offset"])
        if "count" in field:
            values[field["name"]] = [
                struct.unpack_from(fmt, payload, offset + index * size)[0]
                for index in range(int(field["count"]))
            ]
        else:
            values[field["name"]] = struct.unpack_from(fmt, payload, offset)[0]
    return {"name": name, "values": values}


def encode_command(
    schema: dict, name: str, cmd_id: int, run_id: int, sent_ms: int
) -> bytes:
    try:
        opcode = schema["commands"][name]
    except KeyError as exc:
        raise SchemaError(f"unknown command: {name}") from exc
    return struct.pack(
        ">HHHBB",
        cmd_id & 0xFFFF,
        run_id & 0xFFFF,
        sent_ms & 0xFFFF,
        int(opcode) & 0xFF,
        0,
    )
