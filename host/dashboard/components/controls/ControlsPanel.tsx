import { createContext, useContext, useEffect, useRef, useState, type ComponentRef, type ReactNode } from 'react';
import { Animated, BackHandler, Easing, Platform, Pressable, ScrollView, StyleSheet, Switch, Text, View, type ViewStyle } from 'react-native';

import { focusElement } from '../../lib/focus';
import { useTheme } from '../../lib/theme-provider';
import { useReducedMotion } from '../../lib/useReducedMotion';
import { Button } from '../ui/button';
import { needsConfirm, type ControlAction } from './controls';

type ConsoleAction = Exclude<ControlAction, 'recalibrate' | 'start_continuous_floor' | 'start_stationary'>;
type FocusTrigger = () => boolean;
type Confirmation = { action: ConsoleAction; focusTrigger: FocusTrigger };

type ProviderProps = {
  children: ReactNode;
  emergencyLatched: boolean;
  onAction: (action: ConsoleAction | 'start_stationary') => void;
  disabled: boolean;
  missionAvailable: boolean;
  safetyAvailable: boolean;
  feedback?: string | null;
  feedbackError?: boolean;
};

type ControlSession = {
  canRequest: (action: ConsoleAction) => boolean;
  request: (action: ConsoleAction, focusTrigger: FocusTrigger) => void;
  popup: Confirmation | null;
  cancel: () => void;
  confirm: (stopAtObstacles: boolean) => void;
  toolbar: { current: ComponentRef<typeof View> | null };
  feedback: string | null;
  feedbackError: boolean;
};

const ControlContext = createContext<ControlSession | null>(null);

const ACTION_WORDS: Record<ConsoleAction, string> = {
  start: 'start',
  reset: 'reset',
  stop: 'stop',
  estop: 'emergency stop',
  left: 'turn left 90°',
  right: 'turn right 90°',
  uturn: 'U-turn',
  straight: 'go straight across the junction',
};

const ROUTES = [
  { action: 'left', label: 'Left' },
  { action: 'straight', label: 'Straight' },
  { action: 'right', label: 'Right' },
  { action: 'uturn', label: 'U-turn' },
] as const;

const focusControl = (control: ComponentRef<typeof View> | null): boolean => focusElement(control);

export function ControlsProvider({ children, emergencyLatched, onAction, disabled, missionAvailable, safetyAvailable, feedback = null, feedbackError = false }: ProviderProps) {
  const toolbar = useRef<ComponentRef<typeof View>>(null);
  const [popup, setPopup] = useState<Confirmation | null>(null);
  const canRequest = (action: ConsoleAction) => !disabled && safetyAvailable &&
    (action === 'stop' || action === 'estop' || (missionAvailable && !emergencyLatched));
  const active = popup !== null && canRequest(popup.action) ? popup : null;

  useEffect(() => {
    if (popup === null || active !== null) return;
    setPopup(null);
    requestAnimationFrame(() => focusControl(toolbar.current));
  }, [popup, active]);

  const cancel = () => {
    setPopup(null);
    if (popup !== null) requestAnimationFrame(() => {
      if (!popup.focusTrigger()) focusControl(toolbar.current);
    });
  };
  const request = (action: ConsoleAction, focusTrigger: FocusTrigger) => {
    if (!canRequest(action)) return;
    if (!needsConfirm(action)) {
      onAction(action);
      if (popup !== null) cancel();
      return;
    }
    setPopup({ action, focusTrigger });
  };
  const confirm = (stopAtObstacles: boolean) => {
    if (active === null || !canRequest(active.action)) return;
    onAction(active.action === 'start' && stopAtObstacles ? 'start_stationary' : active.action);
    cancel();
  };

  return (
    <ControlContext.Provider value={{ canRequest, request, popup: active, cancel, confirm, toolbar, feedback, feedbackError }}>
      {children}
    </ControlContext.Provider>
  );
}

function useControls() {
  const controls = useContext(ControlContext);
  if (controls === null) throw new Error('Commands require the shared console controls.');
  return controls;
}

function ControlButton({ action, label, narrow = false, flex = false, variant = 'secondary' }: {
  action: ConsoleAction;
  label: string;
  narrow?: boolean;
  flex?: boolean;
  variant?: 'primary' | 'secondary' | 'danger';
}) {
  const { canRequest, request } = useControls();
  const trigger = useRef<ComponentRef<typeof View>>(null);
  const theme = useTheme();
  return (
    <Button compact aria-haspopup={needsConfirm(action) ? 'dialog' : undefined} disabled={!canRequest(action)}
      onPress={() => request(action, () => focusControl(trigger.current))} ref={trigger}
      style={[narrow ? styles.narrowButton : undefined, flex ? styles.gridButton : undefined]} variant={variant}>
      <Button.Text style={narrow || flex ? theme.typography.label : undefined}>{label}</Button.Text>
    </Button>
  );
}

export function ControlsPanel({ feedback: feedbackProp, feedbackError: feedbackErrorProp, inline = true, card = false }: {
  feedback?: string | null;
  feedbackError?: boolean;
  inline?: boolean;
  card?: boolean;
}) {
  const theme = useTheme();
  const controls = useControls();
  const feedback = feedbackProp !== undefined ? feedbackProp : controls.feedback;
  const feedbackError = feedbackErrorProp !== undefined ? feedbackErrorProp : controls.feedbackError;
  return (
    <View style={styles.panel}>
      {card ? (
        <View accessibilityLabel="Vehicle commands" ref={controls.toolbar} role="toolbar" style={styles.cardGrid} tabIndex={-1}>
          <View style={styles.cardRow}>
            <ControlButton action="start" label="Start" flex variant="primary" />
            <ControlButton action="reset" label="Reset" flex />
            <ControlButton action="stop" label="Stop" flex />
            <ControlButton action="estop" label="E-stop" flex variant="danger" />
          </View>
          <View style={styles.cardRow}>
            {ROUTES.map((command) => <ControlButton key={command.action} {...command} flex />)}
          </View>
        </View>
      ) : (
        <View accessibilityLabel="Vehicle commands" ref={controls.toolbar} role="toolbar" style={styles.toolbar} tabIndex={-1}>
          <ControlButton action="start" label="Start" narrow={!inline} variant="primary" />
          {ROUTES.map((command) => <ControlButton key={command.action} {...command} narrow={!inline} />)}
          <ControlButton action="reset" label="Reset" narrow={!inline} />
          <ControlButton action="stop" label="Stop" narrow={!inline} />
          <ControlButton action="estop" label="E-stop" narrow={!inline} variant="danger" />
        </View>
      )}
      {feedback === null ? null : (
        <Text accessibilityLiveRegion="polite" style={[theme.typography.caption, { color: feedbackError ? theme.colors.danger : theme.colors.text }]}>
          {feedback}
        </Text>
      )}
    </View>
  );
}

export function CommandWorkspace({ children }: { children: ReactNode }) {
  const theme = useTheme();
  const { popup, cancel } = useControls();
  const webInert = Platform.OS === 'web' && popup !== null ? { inert: true } : {};
  return (
    <View role="main" style={styles.workspace}>
      <View {...webInert} accessibilityElementsHidden={popup !== null}
        importantForAccessibility={popup === null ? 'auto' : 'no-hide-descendants'} style={styles.workspace}>
        {children}
      </View>
      {popup !== null && (
        <View
          style={[
            styles.overlay,
            Platform.OS === 'web' ? (styles.overlayWeb as object) : null,
            { backgroundColor: theme.colors.scrim },
          ]}
          testID="command-backdrop"
        >
          <Pressable accessible={false} focusable={false} tabIndex={-1} onPress={cancel} style={StyleSheet.absoluteFill} />
          <CommandDialog key={popup.action} popup={popup} />
        </View>
      )}
    </View>
  );
}

function CommandDialog({ popup }: { popup: Confirmation }) {
  const theme = useTheme();
  const { cancel, confirm } = useControls();
  const [stopAtObstacles, setStopAtObstacles] = useState(false);
  const cancelButton = useRef<ComponentRef<typeof View>>(null);
  const reduced = useReducedMotion();
  const progress = useRef(new Animated.Value(reduced ? 1 : 0)).current;
  const title = `Confirm ${ACTION_WORDS[popup.action]}`;

  useEffect(() => {
    const frame = requestAnimationFrame(() => focusControl(cancelButton.current));
    return () => cancelAnimationFrame(frame);
  }, []);
  useEffect(() => {
    if (Platform.OS === 'web') {
      const onEscape = (event: KeyboardEvent) => {
        if (event.key !== 'Escape') return;
        event.preventDefault();
        event.stopImmediatePropagation();
        cancel();
      };
      window.addEventListener('keydown', onEscape, true);
      return () => window.removeEventListener('keydown', onEscape, true);
    }
    const back = BackHandler.addEventListener('hardwareBackPress', () => { cancel(); return true; });
    return () => back.remove();
  }, [cancel]);
  useEffect(() => {
    if (reduced) {
      progress.setValue(1);
      return;
    }
    progress.setValue(0);
    const animation = Animated.timing(progress, {
      toValue: 1, duration: 160, easing: Easing.out(Easing.exp), useNativeDriver: Platform.OS !== 'web',
    });
    animation.start();
    return () => animation.stop();
  }, [progress, reduced]);

  return (
    <Animated.View accessibilityLabel={title} role="dialog" testID="command-dialog"
      style={[styles.dialog, { backgroundColor: theme.colors.card, borderColor: theme.colors.border, borderRadius: theme.radii.card },
        { opacity: progress, transform: [{ translateY: progress.interpolate({ inputRange: [0, 1], outputRange: [8, 0] }) }] }]}>
      <ScrollView keyboardShouldPersistTaps="handled" contentContainerStyle={styles.dialogContent} bounces={false}>
        <Text accessibilityRole="header" style={[theme.typography.subheading, { color: theme.colors.text }]}>{title}</Text>
        {popup.action === 'reset' && <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
          Reset stops the car and clears its route. The dashboard starts a new trace when the car accepts. Press Start to drive again.
        </Text>}
        {popup.action === 'start' ? (
          <>
            <Text style={[theme.typography.body, { color: theme.colors.muted }]}>Start clears resolved faults and calibrates the IMU on level ground. The car can then move.</Text>
            <View style={styles.floorChoice}>
              <Pressable accessible={false} focusable={false} tabIndex={-1} style={styles.floorText}
                onPress={() => setStopAtObstacles((selected) => !selected)}>
                <Text style={[theme.typography.label, { color: theme.colors.text }]}>Stop at obstacles</Text>
                <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>{stopAtObstacles
                    ? 'On. The car stops and surveys the obstacle. Select this if the floor may have drop-offs.'
                    : 'Off. The car drives around obstacles on continuous floor.'}</Text>
              </Pressable>
              <Switch accessibilityLabel="Stop at obstacles" value={stopAtObstacles} onValueChange={setStopAtObstacles} />
            </View>
          </>
        ) : null}
        <View style={styles.buttons}>
          <Button compact onPress={cancel} ref={cancelButton} variant="secondary"><Button.Text>Cancel</Button.Text></Button>
          <Button compact onPress={() => confirm(stopAtObstacles)}><Button.Text>Confirm</Button.Text></Button>
        </View>
      </ScrollView>
    </Animated.View>
  );
}

const styles = StyleSheet.create({
  panel: { gap: 4, minWidth: 0 },
  toolbar: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 6 },
  cardGrid: { gap: 4 },
  cardRow: { flexDirection: 'row', gap: 5 },
  gridButton: { flex: 1, minWidth: 0, paddingHorizontal: 6, minHeight: 32 },
  narrowButton: { paddingHorizontal: 12 },
  floorChoice: { alignItems: 'center', flexDirection: 'row', gap: 12 },
  floorText: { flex: 1, gap: 2 },
  buttons: { alignItems: 'center', flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  workspace: { flex: 1, minHeight: 0 },
  overlay: { ...StyleSheet.absoluteFill, alignItems: 'center', justifyContent: 'center', padding: 12, zIndex: 1000 },
  overlayWeb: { position: 'fixed' as 'absolute', top: 0, right: 0, bottom: 0, left: 0, backdropFilter: 'blur(3px)' } as ViewStyle,
  dialog: { borderWidth: 1, maxHeight: '100%', maxWidth: 440, width: '100%', zIndex: 1 },
  dialogContent: { gap: 12, padding: 16 },
});
