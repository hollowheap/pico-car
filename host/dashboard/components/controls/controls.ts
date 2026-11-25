import type { CommandAction } from '../../lib/command';

export type ControlAction = CommandAction;

export type ControlDecision = {
  allowed: boolean;
  reason?: string;
};

/** An emergency stop must act on one press; every other command is confirmed first. */
export function needsConfirm(action: ControlAction): boolean {
  return action !== 'estop';
}

export function confirmControl(
  action: ControlAction,
  confirmed: boolean,
  emergencyLatched: boolean,
): ControlDecision {
  if (!confirmed && needsConfirm(action)) {
    return { allowed: false, reason: 'confirmation required' };
  }
  if (action === 'reset' && emergencyLatched) {
    return { allowed: false, reason: 'emergency latch is set' };
  }
  return { allowed: true };
}
