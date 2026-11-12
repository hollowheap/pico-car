import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { enumLabel, headingDegrees } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import { bearingName } from './format';
import { Compass } from './Compass';
import { LevelGauge } from './LevelGauge';
import { Panel } from './Panel';

type Props = {
  /** Status packet: the heading and the mode of the car. */
  statusPacket: LinkPacket | undefined;
  physics: LinkPacket | undefined;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** The tilt that counts as a lift, as in src/physics/attitude.c. */
const TILT_ENVELOPE_DEG = 20;
const CDEG = 100;

const signed = (value: number | null, digits = 1) =>
  value === null ? '--' : `${value >= 0 ? '+' : ''}${value.toFixed(digits)}°`;

/**
 * Heading compass, pitch and roll gauges, and the lift state. The heading is
 * the magnetometer heading and it is not valid until the IMU has data.
 */
export function AttitudePanel({ statusPacket, physics, status, style }: Props) {
  const theme = useTheme();
  const heading = headingDegrees(physics?.values.heading_ddeg ?? statusPacket?.values.heading_ddeg);
  const pitch = physics?.values.pitch_cdeg === undefined ? null : physics.values.pitch_cdeg / CDEG;
  const roll = physics?.values.roll_cdeg === undefined ? null : physics.values.roll_cdeg / CDEG;
  const yawRate = physics?.values.yaw_rate_cdps === undefined ? 0 : physics.values.yaw_rate_cdps / CDEG;
  const attitude = physics?.values.attitude_state;
  const alert = attitude === 2 || attitude === 3;

  return (
    <Panel
      right={
        <Text style={[theme.typography.label, { color: alert ? theme.colors.danger : theme.colors.muted }]}>
          {attitude === undefined ? '--' : `ATTITUDE ${enumLabel('attitudeState', attitude)}`}
        </Text>
      }
      status={status}
      style={style}
      testID="attitude-panel"
      title="Attitude"
    >
      <View style={styles.readout}>
        <Text style={[theme.typography.headingLg, { color: theme.colors.text }]}>
          {heading === null ? '--' : heading.toFixed(0)}
          <Text style={[theme.typography.subheading, { color: theme.colors.muted }]}> ° heading</Text>
        </Text>
        <Text style={[theme.typography.label, { color: heading === null ? theme.colors.muted : theme.colors.text }]}>
          {heading === null ? 'NO HEADING' : bearingName(heading)}
        </Text>
      </View>
      <View style={styles.compass}>
        <Compass available={heading !== null} headingDeg={heading ?? 0} yawRateDps={yawRate} />
      </View>
      <LevelGauge envelopeDeg={TILT_ENVELOPE_DEG} pitchDeg={pitch} rollDeg={roll} />
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        pitch {signed(pitch)} · roll {signed(roll)} · turn {signed(physics ? yawRate : null, 0)}/s
      </Text>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        Hump {physics === undefined ? '--' : enumLabel('humpState', physics.values.hump_state).toLowerCase()}, peak{' '}
        {physics?.values.hump_peak_mm ?? '--'} mm. Motion{' '}
        {physics === undefined ? '--' : enumLabel('motionClass', physics.values.motion_class).toLowerCase()}. Ticks at ±{TILT_ENVELOPE_DEG}° mark the lift envelope.
      </Text>
    </Panel>
  );
}

const styles = StyleSheet.create({
  readout: { alignItems: 'baseline', flexDirection: 'row', gap: 8, justifyContent: 'space-between' },
  compass: { alignItems: 'center' },
});
