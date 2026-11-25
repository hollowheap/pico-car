import { readFileSync, renameSync, writeFileSync } from 'node:fs';
import type { StoragePort } from './course-session.js';

/** A separate optional checkpoint retains course geometry when the JSONL compacts. */
export function courseStorage(historyPath: string): StoragePort {
  const path = `${historyPath}.course`;
  let records: Record<string, string> = {};
  try {
    const text = readFileSync(path, 'utf8');
    if (text.length <= 4_000_000) {
      const saved: unknown = JSON.parse(text);
      if (saved && typeof saved === 'object' && !Array.isArray(saved)) records = saved as Record<string, string>;
    }
  } catch { /* An absent or damaged checkpoint is covered by history replay. */ }
  let persisted: string | undefined;
  const write = () => {
    const data = JSON.stringify(records);
    if (data === persisted) return;
    const temporary = `${path}.tmp`;
    writeFileSync(temporary, data, { mode: 0o600 });
    renameSync(temporary, path);
    persisted = data;
  };
  return {
    getItem: (key) => typeof records[key] === 'string' ? records[key] : null,
    setItem(key, value) {
      records[key] = value;
      write();
    },
    removeItem(key) {
      if (!Object.hasOwn(records, key)) return;
      delete records[key];
      write();
    },
  };
}
