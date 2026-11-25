import { BIT_FIELDS, ENUMS } from './schema.generated';

export type EnumName = keyof typeof ENUMS;

export type BitFieldName = keyof typeof BIT_FIELDS;

const HEADING_INVALID = 0xffff;

/** Convert heading_ddeg (0.1 degree) to degrees, or null when invalid. */
export function headingDegrees(ddeg: number | undefined): number | null {
  return ddeg === undefined || ddeg === HEADING_INVALID ? null : ddeg / 10;
}

function labelName(name: string): string {
  return name.replace(/([a-z0-9])([A-Z])/g, '$1 $2').toUpperCase();
}

export function enumLabel(name: EnumName, value: number | undefined): string {
  if (value === undefined) {
    return '--';
  }
  const found = Object.entries(ENUMS[name]).find(([, code]) => code === value);
  if (found === undefined) {
    return `UNKNOWN ${value}`;
  }
  return labelName(found[0]);
}

export function bitLabels(
  name: BitFieldName,
  mask: number | undefined,
): string[] {
  if (mask === undefined || mask === 0) {
    return [];
  }
  return Object.entries(BIT_FIELDS[name])
    .filter(([label, bit]) => !/retired/i.test(label) && (mask & (1 << bit)) !== 0)
    .map(([label]) => labelName(label));
}

export function heartbeatAgeMs(nowMs: number, lastHeartbeatMs: number): number | null {
  if (!Number.isFinite(nowMs) || !Number.isFinite(lastHeartbeatMs)) return null;
  const age = nowMs - lastHeartbeatMs;
  return age >= 0 ? age : null;
}
