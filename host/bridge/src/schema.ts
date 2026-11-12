import { readFileSync } from "node:fs";
import { parse } from "yaml";

type FieldType = "int8" | "uint8" | "int16" | "uint16" | "int32" | "uint32";

type Field = {
  name: string;
  offset: number;
  type: FieldType;
  count?: number;
};

type Packet = {
  wire_size: number;
  fields: Field[];
};

export type ProtocolSchema = {
  topics: Record<string, string>;
  packets: Record<string, Packet>;
  commands: Record<string, number>;
};

export type DecodedPacket = {
  name: string;
  values: Record<string, number>;
};

export function loadProtocolSchema(path: string): ProtocolSchema {
  const document = parse(readFileSync(path, "utf8")) as ProtocolSchema;
  if (!document.topics || !document.packets || !document.commands) {
    throw new Error(`invalid protocol schema: ${path}`);
  }
  return document;
}

function readField(view: DataView, field: Field): number {
  switch (field.type) {
    case "int8":
      return view.getInt8(field.offset);
    case "uint8":
      return view.getUint8(field.offset);
    case "int16":
      return view.getInt16(field.offset, false);
    case "uint16":
      return view.getUint16(field.offset, false);
    case "int32":
      return view.getInt32(field.offset, false);
    case "uint32":
      return view.getUint32(field.offset, false);
  }
}

const FIELD_SIZE: Record<FieldType, number> = {
  int8: 1,
  uint8: 1,
  int16: 2,
  uint16: 2,
  int32: 4,
  uint32: 4,
};

export function decodePacket(
  schema: ProtocolSchema,
  name: string,
  payload: Uint8Array,
): DecodedPacket {
  const packet = schema.packets[name];
  if (!packet) {
    throw new Error(`unknown packet: ${name}`);
  }
  if (payload.byteLength !== packet.wire_size) {
    throw new RangeError(
      `${name} packet is ${payload.byteLength} bytes, expected ${packet.wire_size}`,
    );
  }

  const view = new DataView(
    payload.buffer,
    payload.byteOffset,
    payload.byteLength,
  );
  // Dashboard values are a flat Record<string, number>, so an array field
  // `slots` with count N becomes slots_0 .. slots_<N-1>.
  const values: Record<string, number> = {};
  for (const field of packet.fields) {
    if (field.count === undefined) {
      values[field.name] = readField(view, field);
      continue;
    }
    for (let index = 0; index < field.count; index += 1) {
      values[`${field.name}_${index}`] = readField(view, {
        ...field,
        offset: field.offset + index * FIELD_SIZE[field.type],
      });
    }
  }
  return { name, values };
}

export function packetNameForTopic(
  schema: ProtocolSchema,
  topic: string,
): string | undefined {
  for (const [name, suffix] of Object.entries(schema.topics)) {
    if (name === "command") {
      continue;
    }
    if (topic.endsWith(`/${suffix}`)) {
      return name;
    }
  }
  return undefined;
}
