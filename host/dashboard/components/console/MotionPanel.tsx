import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { enumLabel } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import { withUnit } from './format';
import { Panel } from './Panel';
import { Sparkline } from './Sparkline';

type Props = {
  graphHeight?: number;
  motion: LinkPacket | undefined;
  /** Status speeds in mm/s, newest last (about 30 s). */
  speeds: number[];
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** The bars stop at this speed (mm/s). The follower caps the wheel targets near 163. */
const BAR_FULL_MMS = 300;
/** Two targets this close count as one straight run, so a speed difference is slip. */
const STRAIGHT_BAND_MMS = 10;

type WheelProps = { name: string; speed: number | undefined; target: number | undefined; duty: number | undefined };

function Wheel({ name, speed, target, duty }: WheelProps) {
  const theme = useTheme();
  const fill = speed === undefined ? 0 : Math.min(1, Math.abs(speed) / BAR_FULL_MMS);
  const mark = target === undefined ? null : Math.min(1, Math.abs(target) / BAR_FULL_MMS);

  return (
    <View style={styles.wheel}>
      <Text style={[theme.typography.label, styles.name, { color: theme.colors.muted }]}>{name}</Text>
      <View
        accessibilityLabel={`${name} wheel ${withUnit(speed, 'mm/s')}, target ${withUnit(target, 'mm/s')}`}
        accessibilityRole="progressbar"
        style={[styles.bar, { backgroundColor: theme.colors.elevated, borderRadius: 6 }]}
      >
        <View style={[styles.fill, { backgroundColor: theme.colors.text, width: `${fill * 100}%` }]} />
        {mark !== null ? (
          <View style={[styles.target, { backgroundColor: theme.colors.accent, left: `${mark * 100}%` }]} />
        ) : null}
      </View>
      <Text style={[theme.typography.body, styles.value, { color: theme.colors.text }]}>
        {speed ?? '--'} / {target ?? '--'}
        <Text style={[theme.typography.caption, { color: theme.colors.muted }]}> mm/s · duty {duty === undefined ? '--' : `${Math.round(duty / 10)} %`}</Text>
      </Text>
    </View>
  );
}

/**
 * Left and right wheel speed against the target, the duty the controller
 * applied, and the speed trace. A speed difference on a straight run is slip.
 */
export function MotionPanel({ graphHeight = 40, motion, speeds, status, style }: Props) {
  const theme = useTheme();
  const v = motion?.values;
  const straight =
    v?.left_target_mms !== undefined && v.right_target_mms !== undefined &&
    Math.abs(v.left_target_mms - v.right_target_mms) <= STRAIGHT_BAND_MMS && Math.abs(v.left_target_mms) > STRAIGHT_BAND_MMS;
  const slip = straight && v?.left_mms !== undefined && v.right_mms !== undefined ? Math.abs(v.left_mms - v.right_mms) : null;
  const fault = v === undefined ? undefined : enumLabel('motionFault', v.motion_fault);
  const faulty = v !== undefined && v.motion_fault !== 0;

  return (
    <Panel
      right={
        <Text style={[theme.typography.label, { color: faulty ? theme.colors.danger : theme.colors.muted }]}>
          {v === undefined ? '--' : `${enumLabel('motionState', v.motion_state)}${faulty ? ` · ${fault}` : ''}`}
        </Text>
      }
      embedded
      showPlaceholder
      status={status}
      style={style}
      testID="motion-panel"
      title="Motion"
    >
      <View style={styles.wheels}>
        <Wheel duty={v?.left_duty_pm} name="LEFT" speed={v?.left_mms} target={v?.left_target_mms} />
        <Wheel duty={v?.right_duty_pm} name="RIGHT" speed={v?.right_mms} target={v?.right_target_mms} />
      </View>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        Bar: speed. Blue mark: target.{' '}
        {slip === null ? 'Slip shows on a straight run.' : `Wheel difference on this straight run: ${slip} mm/s.`}{' '}
        Missed control ticks: {v?.overruns ?? '--'}.
      </Text>
      <View style={styles.trace}>
        <View style={styles.traceHead}>
          <Text style={[theme.typography.label, { color: theme.colors.muted }]}>RECENT SPEED</Text>
          <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
            now {speeds[speeds.length - 1] ?? '--'} mm/s
          </Text>
        </View>
        <Sparkline
          color={theme.colors.accent}
          height={graphHeight}
          label={`Speed trace, ${speeds.length} samples, now ${speeds[speeds.length - 1] ?? '--'} mm/s`}
          range={[0, BAR_FULL_MMS]}
          values={speeds}
        />
      </View>
    </Panel>
  );
}

const styles = StyleSheet.create({
  wheels: { flexDirection: 'row', gap: 15 },
  wheel: { flex: 1, gap: 4, minWidth: 0 },
  name: { letterSpacing: 1 },
  bar: { height: 8, position: 'relative' },
  fill: { borderRadius: 6, bottom: 0, left: 0, position: 'absolute', top: 0 },
  target: { bottom: 0, position: 'absolute', top: 0, width: 3 },
  value: { fontVariant: ['tabular-nums'] },
  trace: { gap: 4, marginTop: 'auto' },
  traceHead: { alignItems: 'center', flexDirection: 'row', justifyContent: 'space-between' },
});
