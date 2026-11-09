import { useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';

import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';
import { confirmControl, type ControlAction } from './controls';

type Props = {
  emergencyLatched: boolean;
  onAction: (action: ControlAction) => void;
  title?: string;
  disabled?: boolean;
  disabledReason?: string;
  feedback?: string | null;
  feedbackError?: boolean;
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
    setPending(action);
    setError(null);
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

  return (
    <View
      style={[
        styles.card,
        {
          backgroundColor: theme.colors.card,
          borderColor: theme.colors.border,
          borderRadius: theme.radii.card,
          padding: theme.spacing.lg,
        },
      ]}
    >
      <Text
        style={[theme.typography.headingSm, { color: theme.colors.text }]}
      >
        {title}
      </Text>
      {disabled ? (
        <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
          {disabledReason}
        </Text>
      ) : null}
      {emergencyLatched ? (
        <Text
          style={[
            theme.typography.body,
            {
              backgroundColor: theme.colors.dangerSurface,
              color: theme.colors.danger,
            },
          ]}
        >
          Emergency latch engaged. Reset requires a power cycle.
        </Text>
      ) : null}
      {feedback === null ? null : (
        <Text
          style={[
            theme.typography.body,
            {
              color: feedbackError
                ? theme.colors.danger
                : theme.colors.muted,
            },
          ]}
        >
          {feedback}
        </Text>
      )}
      {error === null ? null : (
        <Text
          style={[
            theme.typography.body,
            {
              backgroundColor: theme.colors.dangerSurface,
              color: theme.colors.danger,
            },
          ]}
        >
          {error}
        </Text>
      )}
      {pending === null ? (
        <View style={styles.row}>
          <Button
            disabled={disabled || emergencyLatched}
            onPress={() => request('start')}
          >
            <Button.Text>Start</Button.Text>
          </Button>
          <Button
            disabled={disabled}
            variant="secondary"
            onPress={() => request('stop')}
          >
            <Button.Text>Stop</Button.Text>
          </Button>
          <Button
            disabled={disabled}
            variant="secondary"
            onPress={() => request('estop')}
          >
            <Button.Text>E-stop</Button.Text>
          </Button>
          <Button
            disabled={disabled || emergencyLatched}
            variant="secondary"
            onPress={() => request('reset')}
          >
            <Button.Text>Reset</Button.Text>
          </Button>
          <Button
            disabled={disabled || emergencyLatched}
            variant="secondary"
            onPress={() => request('left')}
          >
            <Button.Text>Left 90°</Button.Text>
          </Button>
          <Button
            disabled={disabled || emergencyLatched}
            variant="secondary"
            onPress={() => request('right')}
          >
            <Button.Text>Right 90°</Button.Text>
          </Button>
          <Button
            disabled={disabled || emergencyLatched}
            variant="secondary"
            onPress={() => request('uturn')}
          >
            <Button.Text>U-turn</Button.Text>
          </Button>
        </View>
      ) : (
        <View style={styles.confirmRow}>
          <View style={styles.confirmActions}>
            <Button
              disabled={disabled}
              variant="secondary"
              onPress={() => setPending(null)}
            >
              <Button.Text>Cancel</Button.Text>
            </Button>
            <Button disabled={disabled} onPress={confirm}>
              <Button.Text>Confirm</Button.Text>
            </Button>
          </View>
          <Text
            style={[
              theme.typography.body,
              styles.confirmText,
              { color: theme.colors.text },
            ]}
          >
            Confirm {pending}?
          </Text>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  card: { gap: 8, borderWidth: 1 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap' },
  confirmRow: {
    flexDirection: 'row',
    alignItems: 'center',
    flexWrap: 'wrap',
    gap: 8,
  },
  confirmText: {
    flexShrink: 1,
    marginLeft: 'auto',
    textAlign: 'right',
  },
  confirmActions: {
    flexDirection: 'row',
    gap: 8,
  },
});
