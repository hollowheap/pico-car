import type { CommandAction } from '../../lib/command';

export type ControlAction = CommandAction;

export type ControlDecision = {
  allowed: boolean;
  reason?: string;
};

const OPCODES: Record<ControlAction, number> = {
  stop: 0x00,
  start: 0x01,
  reset: 0x02,
};

export function confirmControl(
  action: ControlAction,
  confirmed: boolean,
  emergencyLatched: boolean,
): ControlDecision {
  if (!confirmed) {
    return { allowed: false, reason: 'confirmation required' };
  }
  if (action === 'reset' && emergencyLatched) {
    return { allowed: false, reason: 'emergency latch is set' };
  }
  return { allowed: true };
}

export function encodeCommand(
  action: ControlAction,
  cmdId: number,
  runId: number,
  sentMs: number,
): Uint8Array {
  const frame = new Uint8Array(8);
  const view = new DataView(frame.buffer);
  view.setUint16(0, cmdId, false);
  view.setUint16(2, runId, false);
  view.setUint16(4, sentMs, false);
  view.setUint8(6, OPCODES[action]);
  view.setUint8(7, 0);
  return frame;
}
