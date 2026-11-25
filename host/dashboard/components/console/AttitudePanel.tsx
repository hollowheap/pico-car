import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import { useTheme } from '../../lib/theme-provider';
import { bearingName } from './format';
import { Compass } from './Compass';
import { Panel } from './Panel';

type Props = {
  status: LinkPacket | undefined;
  system: LinkPacket | undefined;
  style?: StyleProp<ViewStyle>;
};

export function AttitudePanel({ status, system, style }: Props) {
  const theme = useTheme();
  const heading = status?.values.heading_deg ?? 0;
  const hasHeading = status?.values.heading_deg !== undefined;
  const pitchCdeg = system?.values.pitch_cdeg;
  const pitch =
    pitchCdeg === undefined ? '--' : (pitchCdeg / 100).toFixed(2);

  return (
    <Panel
      style={style}
      title="Attitude"
      right={
        <Text style={[theme.typography.label, { color: theme.colors.muted }]}>
          IMU
        </Text>
      }
    >
      <View style={styles.readout}>
        <Text style={[theme.typography.headingLg, { color: theme.colors.text }]}>
          {hasHeading ? heading.toFixed(1) : '--'}
          <Text style={[theme.typography.subheading, { color: theme.colors.muted }]}>
            {' '}
            deg yaw
          </Text>
        </Text>
        <Text
          style={[
            theme.typography.label,
            {
              color: hasHeading ? theme.colors.accent : theme.colors.muted,
            },
          ]}
        >
          {hasHeading ? bearingName(heading) : 'NO DATA'}
        </Text>
      </View>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        pitch {pitch} deg
      </Text>
      <Compass available={hasHeading} headingDeg={heading} />
    </Panel>
  );
}

const styles = StyleSheet.create({
  readout: {
    flexDirection: 'row',
    alignItems: 'baseline',
    justifyContent: 'space-between',
    gap: 8,
  },
});
