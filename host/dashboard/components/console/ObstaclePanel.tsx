import { StyleSheet, Text, View, type StyleProp, type ViewStyle } from 'react-native';

import type { LinkPacket } from '../../lib/link';
import type { PanelStatus } from '../../lib/panel-state';
import { scanPoints } from '../../lib/scan';
import { bitLabels, enumLabel } from '../../lib/schema';
import { useTheme } from '../../lib/theme-provider';
import { formatTime } from './format';
import { Panel } from './Panel';
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
const PLAN_WORDS = ['ok', 'clear', 'no safe path', 'failed'];

/** Whole degrees without a minus sign on zero. */
const degrees = (deci: number) => `${Math.round(deci / DECI) || 0}°`;

/**
 * Sonar state, the latest scan as a polar plot, and the plan card (side, width,
 * room needed, start angle). The scan keeps showing after the car drives on.
 */
export function ObstaclePanel({ obstacle, profile, status, style }: Props) {
  const theme = useTheme();
  const o = obstacle?.values;
  const servoDeci = o?.servo_deci;
  const bodyDeci = o?.body_deci;
  const planStatus = o?.plan_status ?? 0;
  const faults = o === undefined ? [] : bitLabels('obstacleFaults', o.faults);
  const cause = o === undefined ? '--' : enumLabel('obstacleCause', o.cause);
  const stopped = o !== undefined && o.state === 0 && o.cause !== 0;

  return (
    <Panel
      right={
        <Text style={[theme.typography.label, { color: stopped ? theme.colors.danger : theme.colors.muted }]}>
          {o === undefined ? '--' : enumLabel('obstacleState', o.state)}
          {stopped ? ` · ${cause}` : ''}
        </Text>
      }
      status={status}
      style={style}
      testID="obstacle-panel"
      title="Obstacle"
    >
      <ScanPlot
        beamHalfDeg={BEAM_HALF_DEG}
        detectMm={DETECT_MM}
        points={profile === undefined ? [] : scanPoints(profile.values)}
        servoDeg={servoDeci === undefined ? null : servoDeci / DECI}
      />
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        Slots every 5°, rings every 100 mm, dashed ring at {DETECT_MM} mm. Dot: echo. Ring on the edge: open space. Cross: bad echo.
      </Text>
      <Text style={[theme.typography.caption, { color: theme.colors.muted }]}>
        {profile === undefined ? 'No scan yet.' : `Scan from ${formatTime(profile.timestamp)}: closest ${profile.values.closest_mm} mm, width ${profile.values.width_mm} mm.`}
      </Text>
      <Text style={[theme.typography.body, { color: theme.colors.text }]}>
        Range {o?.dist_mm ?? '--'} mm · echo {o === undefined ? '--' : enumLabel('echoClass', o.echo_class).toLowerCase()} · servo{' '}
        {servoDeci === undefined ? '--' : degrees(servoDeci)} · body {bodyDeci === undefined ? '--' : degrees(bodyDeci)}
      </Text>
      <View style={[styles.plan, { borderColor: theme.colors.border }]}>
        <Text style={[theme.typography.label, { color: theme.colors.muted }]}>PLAN</Text>
        <Text style={[theme.typography.body, { color: theme.colors.text }]}>
          {o === undefined
            ? '--'
            : `${PLAN_WORDS[planStatus] ?? planStatus}, pass on the ${o.plan_side === 0 ? 'left' : 'right'}, width ${o.plan_width_mm} mm, needs ${o.plan_need_mm} mm, start ${o.plan_start_deg}°, rescans ${o.rescans}`}
        </Text>
      </View>
      <Text style={[theme.typography.caption, { color: faults.length > 0 ? theme.colors.danger : theme.colors.muted }]}>
        Sonar faults: {faults.length === 0 ? 'none' : faults.join(', ').toLowerCase()}
      </Text>
    </Panel>
  );
}

const styles = StyleSheet.create({
  plan: { borderTopWidth: 1, gap: 2, paddingTop: 8 },
});
