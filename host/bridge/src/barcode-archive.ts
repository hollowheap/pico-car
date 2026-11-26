import { DatabaseSync } from "node:sqlite";
import { isDeepStrictEqual } from "node:util";
import type { TelemetryReceipt } from "./history.js";
import { decodePacket, type ProtocolSchema } from "./schema.js";

type BarcodeSample = { recordId: number; values: Record<string, number> };
export type BarcodeBatch = {
  carId: number;
  bootId: number;
  overflow: number;
  samples: BarcodeSample[];
};

const RECORD_ID_BYTES = 4;

export function decodeBarcodeBatch(schema: ProtocolSchema, payload: Buffer): BarcodeBatch {
  const layout = schema.packets.barcode_batch;
  const { values } = decodePacket(schema, "barcode_batch", payload);
  const samplesField = layout.fields.find((field) => field.name === "samples");
  const slotSize = RECORD_ID_BYTES + schema.packets.barcode_sample.wire_size;
  const count = values.sample_count;
  if (!samplesField?.count || count < 1 || count * slotSize > samplesField.count ||
      !values.car_id || !values.boot_id) {
    throw new RangeError("invalid retained barcode batch");
  }

  const samples: BarcodeSample[] = [];
  let previous = 0;
  for (let i = 0; i < count; i += 1) {
    const offset = samplesField.offset + i * slotSize;
    const recordId = payload.readUInt32BE(offset);
    if (recordId <= previous) throw new RangeError("barcode record IDs must increase within a batch");
    const edge = payload.subarray(offset + RECORD_ID_BYTES, offset + slotSize);
    samples.push({ recordId, values: decodePacket(schema, "barcode_sample", edge).values });
    previous = recordId;
  }
  return { carId: values.car_id, bootId: values.boot_id, overflow: values.overflow, samples };
}

export function archivedBarcodeValues(carId: number, bootId: number, sample: BarcodeSample): Record<string, number> {
  return { ...sample.values, car_id: carId, boot_id: bootId, record_id: sample.recordId };
}

export function barcodeReceipt(schema: ProtocolSchema, batch: BarcodeBatch): Buffer {
  const layout = schema.packets.barcode_receipt;
  const values = {
    car_id: batch.carId,
    boot_id: batch.bootId,
    last_record_id: batch.samples[batch.samples.length - 1].recordId,
  };
  const payload = Buffer.alloc(layout.wire_size);
  for (const field of layout.fields) {
    payload.writeUInt32BE(values[field.name as keyof typeof values], field.offset);
  }
  return payload;
}

function openArchive(path: string): DatabaseSync {
  const db = new DatabaseSync(path);
  try {
    // The car retires a batch after our receipt. Sync the rollback journal's directory at commit.
    db.exec(`
      PRAGMA synchronous = EXTRA;
      PRAGMA busy_timeout = 25;
      CREATE TABLE IF NOT EXISTS barcode_sample (
        car_id INTEGER NOT NULL, boot_id INTEGER NOT NULL, record_id INTEGER NOT NULL,
        received_at INTEGER NOT NULL, values_json TEXT NOT NULL,
        PRIMARY KEY (car_id, boot_id, record_id)
      ) WITHOUT ROWID;
      CREATE INDEX IF NOT EXISTS barcode_sample_by_receipt ON barcode_sample (received_at);
      CREATE TABLE IF NOT EXISTS barcode_boot (
        car_id INTEGER NOT NULL, boot_id INTEGER NOT NULL, record_count INTEGER NOT NULL,
        last_record INTEGER NOT NULL, overflow INTEGER NOT NULL,
        PRIMARY KEY (car_id, boot_id)
      ) WITHOUT ROWID;
    `);
    return db;
  } catch (error) {
    db.close();
    throw error;
  }
}

/** The receipt may be sent only after this transaction returns successfully. */
export function createBarcodeArchive(path: string) {
  let db: DatabaseSync | undefined;
  let writable = true;
  const database = () => db ??= openArchive(path);

  return {
    append(batch: BarcodeBatch, receivedAt: number): BarcodeSample[] {
      try {
        const archive = database();
        const insert = archive.prepare("INSERT OR IGNORE INTO barcode_sample VALUES (?, ?, ?, ?, ?)");
        const lookup = archive.prepare(
          "SELECT values_json FROM barcode_sample WHERE car_id = ? AND boot_id = ? AND record_id = ?",
        );
        const note = archive.prepare(`
          INSERT INTO barcode_boot VALUES (?, ?, ?, ?, ?)
          ON CONFLICT(car_id, boot_id) DO UPDATE SET
            record_count = barcode_boot.record_count + excluded.record_count,
            last_record = max(barcode_boot.last_record, excluded.last_record),
            overflow = max(barcode_boot.overflow, excluded.overflow)
        `);
        const added: BarcodeSample[] = [];
        archive.exec("BEGIN IMMEDIATE");
        for (const sample of batch.samples) {
          const values = JSON.stringify(sample.values);
          const key = [batch.carId, batch.bootId, sample.recordId] as const;
          if (Number(insert.run(...key, receivedAt, values).changes) !== 0) {
            added.push(sample);
          } else {
            const stored = lookup.get(...key)?.values_json;
            if (typeof stored !== "string" || !isDeepStrictEqual(JSON.parse(stored), sample.values)) {
              throw new Error("conflicting barcode record for a stored ID");
            }
          }
        }
        note.run(batch.carId, batch.bootId, added.length, batch.samples[batch.samples.length - 1].recordId, batch.overflow);
        archive.exec("COMMIT");
        writable = true;
        return added;
      } catch (error) {
        if (db?.isTransaction) db.exec("ROLLBACK");
        writable = false;
        throw error;
      }
    },
    replay(carId: number, bootId: number, after: number, limit: number) {
      return database().prepare(`
        SELECT record_id, received_at, values_json FROM barcode_sample
        WHERE car_id = ? AND boot_id = ? AND record_id > ? ORDER BY record_id LIMIT ?
      `).all(carId, bootId, after, limit).map((row) => ({
        recordId: Number(row.record_id),
        receivedAt: Number(row.received_at),
        values: JSON.parse(String(row.values_json)) as BarcodeSample["values"],
      }));
    },
    recent(topic: string, limit: number): TelemetryReceipt[] {
      if (!Number.isSafeInteger(limit) || limit <= 0) throw new RangeError("invalid barcode history limit");

      return database().prepare(`
        SELECT car_id, boot_id, record_id, received_at, values_json FROM barcode_sample
        ORDER BY received_at DESC, car_id DESC, boot_id DESC, record_id DESC LIMIT ?
      `).all(limit).reverse().map((row) => ({
        packet: "barcode_sample",
        topic,
        timestamp: Number(row.received_at),
        values: archivedBarcodeValues(Number(row.car_id), Number(row.boot_id), {
          recordId: Number(row.record_id),
          values: JSON.parse(String(row.values_json)) as BarcodeSample["values"],
        }),
      }));
    },
    health() {
      try {
        const totals = database().prepare(`
          SELECT coalesce(sum(record_count), 0) AS samples,
                 coalesce(sum(last_record - record_count), 0) AS gaps,
                 coalesce(sum(overflow), 0) AS overflow FROM barcode_boot
        `).get();
        return { storage: writable ? "ready" : "degraded", ...totals };
      } catch {
        return { storage: "degraded" };
      }
    },
    close() {
      db?.close();
      db = undefined;
    },
  };
}

export type BarcodeArchive = ReturnType<typeof createBarcodeArchive>;
