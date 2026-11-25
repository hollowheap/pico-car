import { useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';

import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';
import { confirmControl, needsConfirm, type ControlAction } from './controls';

type Props = {
  emergencyLatched: boolean;
  onAction: (action: ControlAction) => void;
  title?: string;
  disabled?: boolean;
  disabledReason?: string;
  feedback?: string | null;
  feedbackError?: boolean;
};

/** What the confirmation says. The wire name `reset` clears faults: it never moves the car. */
const ACTION_WORDS: Record<string, string> = {
  start: 'start',
  stop: 'stop',
  estop: 'emergency stop',
  reset: 'clear faults (then START)',
  left: 'turn left 90°',
  right: 'turn right 90°',
  uturn: 'U-turn',
  straight: 'go straight across the junction',
  recalibrate: 'recalibrate the IMU (the car stops for about 1 s; keep it level and still)',
};

export function ControlsPanel({
  emergencyLatched,
  onAction,
  title = 'Controls',
  disabled = false,
  disabledReason = 'Link offline. Commands are disabled.',
  feedback = null,
  feedbackError = false,
}: Props) {
  const [pending, setPending] = useState<ControlAction | null>(null);
  const [error, setError] = useState<string | null>(null);
  const theme = useTheme();

  const request = (action: ControlAction) => {
    setError(null);
    if (!needsConfirm(action)) {
      // E-stop: no confirmation step, the command goes out on this press.
      setPending(null);
      onAction(action);
      return;
    }
    setPending(action);
  };

  const confirm = () => {
    if (pending === null) {
      return;
    }
    const decision = confirmControl(pending, true, emergencyLatched);
    if (!decision.allowed) {
      setError(decision.reason ?? 'action rejected');
      return;
    }
    onAction(pending);
    setPending(null);
  };

  // One status line after the button groups, so it never pushes the page down.
  const message =
    error ?? (emergencyLatched ? 'Emergency latch engaged. Reset requires a power cycle.' : null) ?? (disabled ? disabledReason : null) ?? feedback;
  const messageAlert = error !== null || emergencyLatched || (feedbackError && message === feedback);
  const status =
    message === null ? null : (
      <Text
        accessibilityLiveRegion="polite"
        style={[theme.typography.caption, styles.message, { color: messageAlert ? theme.colors.danger : theme.colors.muted }]}
      >
        {message}
      </Text>
    );

  return (
    <View
      style={[
        styles.card,
        {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.card,
          paddingHorizontal: theme.spacing.lg,
          paddingVertical: theme.spacing.md,
        },
      ]}
    >
      <Text style={[theme.typography.headingSm, { color: theme.colors.text }]}>{title}</Text>
      {pending === null ? (
        <View style={styles.row}>
          <View style={styles.group}>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              onPress={() => request('start')}
            >
              <Button.Text>Start</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled}
              variant="secondary"
              onPress={() => request('stop')}
            >
              <Button.Text>Stop</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled}
              variant="danger"
              onPress={() => request('estop')}
            >
              <Button.Text>E-stop</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              variant="secondary"
              onPress={() => request('reset')}
            >
              <Button.Text>Clear faults</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              variant="secondary"
              onPress={() => request('recalibrate')}
            >
              <Button.Text>Recalibrate IMU</Button.Text>
            </Button>
          </View>
          <View style={styles.group}>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              variant="secondary"
              onPress={() => request('left')}
            >
              <Button.Text>Left 90°</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              variant="secondary"
              onPress={() => request('straight')}
            >
              <Button.Text>Straight</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              variant="secondary"
              onPress={() => request('right')}
            >
              <Button.Text>Right 90°</Button.Text>
            </Button>
            <Button
              compact
              disabled={disabled || emergencyLatched}
              variant="secondary"
              onPress={() => request('uturn')}
            >
              <Button.Text>U-turn</Button.Text>
            </Button>
          </View>
          {status}
        </View>
      ) : (
        <View style={styles.confirmRow}>
          <Button compact disabled={disabled} onPress={confirm}>
            <Button.Text>Confirm</Button.Text>
          </Button>
          {/* Cancel stays usable when the link drops: the user can always back out. */}
          <Button compact variant="secondary" onPress={() => setPending(null)}>
            <Button.Text>Cancel</Button.Text>
          </Button>
          <Text style={[theme.typography.body, styles.confirmText, { color: theme.colors.text }]}>
            Confirm {ACTION_WORDS[pending] ?? pending}?
          </Text>
          {status}
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  card: { alignItems: 'center', borderWidth: 1, flexDirection: 'row', flexWrap: 'wrap', gap: 16 },
  row: { alignItems: 'center', flex: 1, flexDirection: 'row', flexWrap: 'wrap', gap: 20, minWidth: 280 },
  group: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 8, maxWidth: '100%' },
  confirmRow: { alignItems: 'center', flex: 1, flexDirection: 'row', flexWrap: 'wrap', gap: 8, minWidth: 280 },
  message: { flexBasis: 160, flexGrow: 1, flexShrink: 1 },
  confirmText: { flexShrink: 1, fontWeight: '600' },
});
