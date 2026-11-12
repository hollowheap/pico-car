import { readFileSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

type FieldType = 'int8' | 'uint8' | 'int16' | 'uint16' | 'int32' | 'uint32';

interface Field {
  name: string;
  offset: number;
  type: FieldType;
  enum?: string;
  count?: number;
}

interface Packet {
  wire_size: number;
  fields: Field[];
}

type EnumDefinition =
  | { values: Record<string, number> }
  | { bits: Record<string, number> };

interface Schema {
  schema: number;
  byte_order: string;
  topics: Record<string, string>;
  packets: Record<string, Packet>;
  commands: Record<string, number>;
  events: Record<string, number>;
  fault_bits: { name: string; bit: number }[];
  enums?: Record<string, EnumDefinition>;
}

const require = createRequire(new URL('../host/dashboard/package.json', import.meta.url));
const { parse } = require('yaml') as typeof import('yaml');

const TYPE_SIZE: Record<FieldType, number> = {
  int8: 1,
  uint8: 1,
  int16: 2,
  uint16: 2,
  int32: 4,
  uint32: 4,
};

const sourceUrl = new URL('./telemetry.yaml', import.meta.url);
const tsUrl = new URL('../host/dashboard/lib/schema.generated.ts', import.meta.url);
const cUrl = new URL('../include/telemetry/wire_schema.h', import.meta.url);
const vectorsUrl = new URL('./vectors.json', import.meta.url);
const testHeaderUrl = new URL('../tests/telemetry_vectors.h', import.meta.url);
const schema = parse(readFileSync(sourceUrl, 'utf8')) as Schema;

if (schema.schema !== 2) throw new Error('unsupported schema version');
if (schema.byte_order !== 'big') throw new Error('only big-endian schemas are supported');

const fieldBytes = (field: Field) => TYPE_SIZE[field.type] * (field.count ?? 1);

for (const [name, packet] of Object.entries(schema.packets)) {
  const fields = [...packet.fields].sort((a, b) => a.offset - b.offset);
  let cursor = 0;
  for (const field of fields) {
    if (!(field.type in TYPE_SIZE)) throw new Error(`${name}.${field.name}: bad type ${field.type}`);
    if (field.offset !== cursor) throw new Error(`${name}.${field.name}: non-contiguous offset`);
    if (field.count !== undefined && (!Number.isInteger(field.count) || field.count < 1)) {
      throw new Error(`${name}.${field.name}: bad count`);
    }
    cursor += fieldBytes(field);
  }
  if (cursor !== packet.wire_size) throw new Error(`${name}: field bytes do not match wire_size`);
}

const topicChars = Object.values(schema.topics);
if (new Set(topicChars).size !== topicChars.length) throw new Error('duplicate topic character');
for (const name of Object.keys(schema.packets)) {
  if (!(name in schema.topics)) throw new Error(`packet ${name} has no topic`);
}

const enums: Record<string, EnumDefinition> = { ...(schema.enums ?? {}) };
// `event_type` is the `events` map, so the two never drift apart.
enums.event_type = { values: schema.events };
for (const [name, packet] of Object.entries(schema.packets)) {
  for (const field of packet.fields) {
    if (field.enum !== undefined && !(field.enum in enums)) {
      throw new Error(`${name}.${field.name}: unknown enum ${field.enum}`);
    }
  }
}
for (const [name, definition] of Object.entries(enums)) {
  const codes = Object.values('values' in definition ? definition.values : definition.bits);
  if (new Set(codes).size !== codes.length) throw new Error(`enum ${name}: duplicate code`);
}

const bits = schema.fault_bits.map((fault) => fault.bit);
if (new Set(bits).size !== bits.length) throw new Error('duplicate fault bit');
if (bits.some((bit) => bit < 0 || bit > 15)) throw new Error('fault bit outside 0..15');
const eventCodes = Object.values(schema.events);
if (new Set(eventCodes).size !== eventCodes.length) throw new Error('duplicate event code');
if (eventCodes.some((code) => code < 1 || code > 255)) throw new Error('event code outside 1..255');

const camel = (name: string) => name.replace(/_([a-z])/g, (_, c: string) => c.toUpperCase());
const upper = (name: string) => name.toUpperCase();

// ---------------------------------------------------------------- TypeScript

const packetLines = Object.entries(schema.packets)
  .map(([name, packet]) => {
    const fields = packet.fields
      .map((field) => {
        const count = field.count === undefined ? '' : `, count: ${field.count}`;
        return `        { name: '${field.name}', offset: ${field.offset}, type: '${field.type}'${count} }`;
      })
      .join(',\n');
    return `  ${name}: {\n    wireSize: ${packet.wire_size},\n    fields: [\n${fields},\n    ],\n  },`;
  })
  .join('\n');

const commands = Object.entries(schema.commands)
  .map(([name, value]) => `  ${name}: ${value},`)
  .join('\n');
const topics = Object.entries(schema.topics)
  .map(([name, value]) => `  ${name}: '${value}',`)
  .join('\n');
const faults = schema.fault_bits
  .map((fault) => `  ${camel(fault.name)}: ${fault.bit},`)
  .join('\n');
const events = Object.entries(schema.events)
  .map(([name, value]) => `  ${camel(name)}: ${value},`)
  .join('\n');
const enumValues = Object.entries(enums)
  .map(([name, definition]) => {
    if (!('values' in definition)) return null;
    const values = Object.entries(definition.values)
      .map(([key, value]) => `    ${camel(key)}: ${value},`)
      .join('\n');
    return `  ${camel(name)}: {\n${values}\n  },`;
  })
  .filter((line): line is string => line !== null)
  .join('\n');
const enumBits = Object.entries(enums)
  .map(([name, definition]) => {
    if (!('bits' in definition)) return null;
    const lines = Object.entries(definition.bits)
      .map(([key, bit]) => `    ${camel(key)}: ${bit},`)
      .join('\n');
    return `  ${camel(name)}: {\n${lines}\n  },`;
  })
  .filter((line): line is string => line !== null)
  .join('\n');

writeFileSync(
  tsUrl,
  `// Generated by protocol/generate.ts from protocol/telemetry.yaml. Do not edit.

export const SCHEMA_VERSION = ${schema.schema} as const;
export const WIRE_BYTE_ORDER = '${schema.byte_order}' as const;

export const TOPICS = {
${topics}
} as const;

export const PACKETS = {
${packetLines}
} as const;

export const COMMANDS = {
${commands}
} as const;

export const EVENTS = {
${events}
} as const;

export const FAULT_BITS = {
${faults}
} as const;

export const ENUMS = {
${enumValues}
} as const;

export const BIT_FIELDS = {
${enumBits}
} as const;
`,
);

// -------------------------------------------------------------------- C header

const cLines: string[] = [];
cLines.push('/* Generated by protocol/generate.ts from protocol/telemetry.yaml. Do not edit. */');
cLines.push('');
cLines.push('#ifndef TELEMETRY_WIRE_SCHEMA_H');
cLines.push('#define TELEMETRY_WIRE_SCHEMA_H');
cLines.push('');
cLines.push(`#define TELEMETRY_SCHEMA_VERSION ${schema.schema}u`);
cLines.push('');
cLines.push('/* Topics: the last MQTT topic character, and the first byte of a queued frame. */');
for (const [name, ch] of Object.entries(schema.topics)) {
  cLines.push(`#define TELEMETRY_TOPIC_${upper(name)} '${ch}'`);
}
for (const [name, packet] of Object.entries(schema.packets)) {
  cLines.push('');
  cLines.push(`/* ${name} */`);
  cLines.push(`#define TELEMETRY_${upper(name)}_WIRE_SIZE ${packet.wire_size}u`);
  for (const field of packet.fields) {
    cLines.push(`#define TELEMETRY_${upper(name)}_OFF_${upper(field.name)} ${field.offset}u`);
    if (field.count !== undefined) {
      cLines.push(`#define TELEMETRY_${upper(name)}_COUNT_${upper(field.name)} ${field.count}u`);
    }
  }
}
cLines.push('');
cLines.push('/* Event types. */');
for (const [name, code] of Object.entries(schema.events)) {
  cLines.push(`#define TELEMETRY_EVT_${upper(name)} 0x${code.toString(16).toUpperCase().padStart(2, '0')}u`);
}
cLines.push('');
cLines.push('/* Fault register bit positions. */');
for (const fault of schema.fault_bits) {
  cLines.push(`#define TELEMETRY_FAULT_BIT_${upper(fault.name)} ${fault.bit}u`);
}
for (const [name, definition] of Object.entries(enums)) {
  cLines.push('');
  cLines.push(`/* ${name} */`);
  if ('values' in definition) {
    for (const [key, value] of Object.entries(definition.values)) {
      cLines.push(`#define TELEMETRY_${upper(name)}_${upper(key)} ${value}u`);
    }
  } else {
    for (const [key, bit] of Object.entries(definition.bits)) {
      cLines.push(`#define TELEMETRY_${upper(name)}_BIT_${upper(key)} ${bit}u`);
    }
  }
}
cLines.push('');
cLines.push('#endif /* TELEMETRY_WIRE_SCHEMA_H */');
cLines.push('');
writeFileSync(cUrl, cLines.join('\n'));

// --------------------------------------------------------------------- vectors

// Deterministic sample values that use the full range of every type, so a wrong
// offset, width or sign shows up in the bytes.
let seed = 0x2545f491;
const next = () => {
  seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
  return seed;
};
const bitsOf = (type: FieldType) => TYPE_SIZE[type] * 8;
const sample = (type: FieldType): number => {
  const width = bitsOf(type);
  const raw = width === 32 ? next() : next() >>> (32 - width);
  const unsigned = raw === 0 ? 1 : raw;
  if (type.startsWith('uint')) return unsigned;
  const sign = 2 ** (width - 1);
  return unsigned >= sign ? unsigned - 2 ** width : unsigned;
};

type Value = number | number[];
const vectors: Record<string, { values: Record<string, Value>; hex: string }> = {};

for (const [name, packet] of Object.entries(schema.packets)) {
  const bytes = new Uint8Array(packet.wire_size);
  const view = new DataView(bytes.buffer);
  const values: Record<string, Value> = {};
  for (const field of packet.fields) {
    const write = (value: number, offset: number) => {
      switch (field.type) {
        case 'int8':
          view.setInt8(offset, value);
          break;
        case 'uint8':
          view.setUint8(offset, value);
          break;
        case 'int16':
          view.setInt16(offset, value, false);
          break;
        case 'uint16':
          view.setUint16(offset, value, false);
          break;
        case 'int32':
          view.setInt32(offset, value, false);
          break;
        case 'uint32':
          view.setUint32(offset, value, false);
          break;
      }
    };
    if (field.count === undefined) {
      // A reserved byte is always written as zero.
      const value = field.name === 'reserved' ? 0 : sample(field.type);
      write(value, field.offset);
      values[field.name] = value;
    } else {
      const list: number[] = [];
      for (let index = 0; index < field.count; index += 1) {
        const value = sample(field.type);
        write(value, field.offset + index * TYPE_SIZE[field.type]);
        list.push(value);
      }
      values[field.name] = list;
    }
  }
  const hex = Array.from(bytes, (byte) => byte.toString(16).padStart(2, '0')).join('');
  vectors[name] = { values, hex };
}

writeFileSync(
  vectorsUrl,
  `${JSON.stringify({ schema: schema.schema, vectors }, null, 2)}\n`,
);

const cType = (type: FieldType) => `${type}_t`;
const hLines: string[] = [];
hLines.push('/* Generated by protocol/generate.ts from protocol/telemetry.yaml. Do not edit. */');
hLines.push('/* Known-good bytes for every packet, and the field values that produce them. */');
hLines.push('');
hLines.push('#ifndef TELEMETRY_VECTORS_H');
hLines.push('#define TELEMETRY_VECTORS_H');
hLines.push('');
hLines.push('#include <stdint.h>');
for (const [name, vector] of Object.entries(vectors)) {
  const packet = schema.packets[name];
  hLines.push('');
  const body = vector.hex.match(/../g)!.map((pair) => `0x${pair}u`).join(', ');
  hLines.push(`static const uint8_t TELEMETRY_VECTOR_${upper(name)}[${packet.wire_size}] = { ${body} };`);
  for (const field of packet.fields) {
    const value = vector.values[field.name];
    if (Array.isArray(value)) {
      const suffix = field.type.startsWith('uint') ? 'u' : '';
      const list = value.map((v) => `${v}${suffix}`).join(', ');
      hLines.push(
        `static const ${cType(field.type)} TELEMETRY_VEC_${upper(name)}_${upper(field.name)}[${field.count}] = { ${list} };`,
      );
    } else {
      const suffix = field.type.startsWith('uint') ? 'u' : '';
      hLines.push(`#define TELEMETRY_VEC_${upper(name)}_${upper(field.name)} (${value}${suffix})`);
    }
  }
}
hLines.push('');
hLines.push('#endif /* TELEMETRY_VECTORS_H */');
hLines.push('');
writeFileSync(testHeaderUrl, hLines.join('\n'));

process.stdout.write(
  `generated ${fileURLToPath(tsUrl)}\n` +
    `generated ${fileURLToPath(cUrl)}\n` +
    `generated ${fileURLToPath(vectorsUrl)}\n` +
    `generated ${fileURLToPath(testHeaderUrl)}\n`,
);
