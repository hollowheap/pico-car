import { useContext } from 'react';
import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { scanPoints } from '../../lib/scan';
import { bitLabels, enumLabel } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import { formatTime, sizeMm } from './format';
import { Panel, PanelFill, useDense } from './Panel';
import { StatusPill } from './StatusPill';
import { ScanPlot } from './ScanPlot';

type Props = {
  obstacle: LinkPacket | undefined;
  profile: LinkPacket | undefined;
  status?: PanelStatus;
  style?: StyleProp<ViewStyle>;
};

/** OBSTACLE_BEAM_HALF_DECI and OBSTACLE_DETECT_MM of obstacle_config.h. */
const BEAM_HALF_DEG = 9.5;
const DETECT_MM = 250;
const DECI = 10;

/** Whole degrees without a minus sign on zero. */
const degrees = (deci: number) => `${Math.round(deci / DECI) || 0}°`;

/**
 * Sonar state, the latest scan as a polar plot, and the hug card (state, side, face
 * distance, width, breadth). The scan keeps showing after the car drives on.
 */
export function ObstaclePanel({ obstacle, profile, status, style }: Props) {
  const theme = useTheme();
  const o = obstacle?.values;
  const servoDeci = o?.servo_deci;
  const bodyDeci = o?.body_deci;
  const faults = o === undefined ? [] : bitLabels('obstacleFaults', o.faults);
  const cause = o === undefined ? '--' : enumLabel('obstacleCause', o.cause);
  const stopped = o !== undefined && o.state === 0 && o.cause !== 0;
  const dense = useDense();
  const fill = useContext(PanelFill);
  const text = dense ? theme.typography.caption : theme.typography.body;

  return (
    <Panel
      right={
        <StatusPill
          label={o === undefined ? 'NO DATA' : `${enumLabel('obstacleState', o.state)}${stopped ? ` · ${enumLabel('obstacleCause', o.cause)}` : ''}`}
          tone={o === undefined ? 'muted' : stopped ? 'danger' : o.state === 1 ? 'neutral' : 'active'}
        />
      }
      icon="sonar"
      stretch
      status={status}
      style={style}
      testID="obstacle-panel"
      title="Obstacle"
    >
      <View style={fill ? styles.plotFill : null}>
        <ScanPlot
          beamHalfDeg={BEAM_HALF_DEG}
          detectMm={DETECT_MM}
          height={fill ? '100%' : dense ? 200 : undefined}
          points={profile === undefined ? [] : scanPoints(profile.values)}
          servoDeg={servoDeci === undefined ? null : servoDeci / DECI}
        />
      </View>
      <Text style={[theme.typography.caption, dense ? styles.hidden : null, { color: theme.colors.muted }]}>
        Slots every 5°, rings every 100 mm, dashed ring at {DETECT_MM} mm. Dot: echo. Ring on the edge: open space. Cross: bad echo.
      </Text>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        {profile === undefined ? 'No scan yet.' : `Scan from ${formatTime(profile.timestamp)}: closest ${profile.values.closest_mm} mm, width ${profile.values.width_mm} mm.`}
      </Text>
      <Text style={[text, { color: theme.colors.text }]}>
        Range {o?.dist_mm ?? '--'} mm · echo {o === undefined ? '--' : enumLabel('echoClass', o.echo_class).toLowerCase()} · servo{' '}
        {servoDeci === undefined ? '--' : degrees(servoDeci)} · body {bodyDeci === undefined ? '--' : degrees(bodyDeci)}
      </Text>
      <View style={[styles.plan, { borderColor: theme.colors.border }]}>
        <Text style={[theme.typography.label, { color: theme.colors.muted }]}>HUG</Text>
        <Text style={[text, { color: theme.colors.text }]}>
          {o === undefined
            ? '--'
            : `${enumLabel('hugState', o.hug_state).toLowerCase()}, ${enumLabel('hugSide', o.hug_side).toLowerCase()} side, face ${sizeMm(o.face_mm)}, width ${sizeMm(o.width_mm)}, breadth ${sizeMm(o.breadth_mm)}`}
        </Text>
      </View>
      {stopped ? (
        // The pill has one line; a long cause goes here, where the text wraps.
        <Text style={[text, { color: theme.colors.danger }]}>Stop cause: {cause.toLowerCase()}</Text>
      ) : null}
      <Text style={[theme.typography.caption, { color: faults.length > 0 ? theme.colors.danger : theme.colors.muted }]}>
        Sonar faults: {faults.length === 0 ? 'none' : faults.join(', ').toLowerCase()}
      </Text>
    </Panel>
  );
}

const styles = StyleSheet.create({
  plan: { borderTopWidth: 1, gap: 2, paddingTop: 16 },
  hidden: { display: 'none' },
  plotFill: { flex: 1, minHeight: 90 },
});
