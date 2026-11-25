import { StyleSheet, Text, View } from 'react-native';

import type { Overview } from '../overview/overview';
import { isCurrentStartAttitudeRefusal, PILOT_REASON, pilotReason } from '../../lib/events';
import { STALE_AFTER_MS } from '../../lib/panel-state';
import { BIT_FIELDS, ENUMS, FAULT_BITS } from '../../lib/schema.generated';
import type { TelemetryStore } from '../../lib/telemetry-store';
import { useTheme } from '../../lib/theme-provider';

const BIT_ESTOP = 9;
const BIT_LIFTED = 11;
const BIT_OVERTURNED = 12;
const BIT_OBSTACLE_STOPPED = 13;
const BIT_TASK_STALLED = 14;
const PICKUP_STOP_TITLE = 'STOPPED AFTER PICKUP';

export type BannerMessage = { title: string; action: string };

// Link loss and the line-loss report can coexist with a healthy, stationary car.
const ADVISORY_FAULTS = (1 << FAULT_BITS.wifiDisconn) | (1 << FAULT_BITS.mqttNack) | (1 << FAULT_BITS.lineLost);

const PILOT_HOLDS: Partial<Record<number, BannerMessage>> = {
  [PILOT_REASON.stale]: { title: 'WHEEL FEEDBACK STALE', action: 'Waiting for fresh wheel feedback.' },
  [PILOT_REASON.lineLost]: { title: 'LINE SEARCH EXHAUSTED', action: 'Put the car back on the line.' },
  [PILOT_REASON.encoder]: { title: 'PILOT ENCODER STOP', action: 'The pilot retries after a rest. If the stop stays, check the wheels and encoders, then press Start.' },
  [PILOT_REASON.sonar]: { title: 'SONAR STOP', action: 'Waiting for usable sonar readings.' },
  [PILOT_REASON.hugTimeout]: { title: 'DETOUR TIMED OUT', action: 'The pilot surveys again after a rest. If the stop stays, check the path, then press Start.' },
  [PILOT_REASON.sonarLatched]: { title: 'SONAR STOP LATCHED', action: 'On a continuous floor the car crawls after a rest. Check sonar readings and wiring.' },
  [PILOT_REASON.staleLatched]: { title: 'WHEEL FEEDBACK STOP LATCHED', action: 'Check encoder feedback and wiring.' },
  [PILOT_REASON.courseClearance]: { title: 'COURSE CLEARANCE HOLD', action: 'The pilot surveys again after a rest. If the hold stays, clear the path, then send START.' },
  [PILOT_REASON.courseRecovery]: { title: 'CHECKING COURSE CLEARANCE', action: 'Waiting for fresh forward clearance.' },
  [PILOT_REASON.rejoinStop]: { title: 'REJOIN SEARCH STOPPED', action: 'Check the tape and clearance before pressing Start.' },
};

/** Use the current hold mask and the matching on-car event to explain a stop. */
export function reportedHoldMessage({ events, latest, trailStart }: TelemetryStore, nowMs = Date.now()): BannerMessage | null {
  const packet = latest.status;
  const status = packet?.values;
  if (((status?.hold_mask ?? 0) & (1 << BIT_FIELDS.holdMask.calibrate)) !== 0) {
    for (let i = events.length - 1; i >= 0; i--) {
      const event = events[i]!;
      if (trailStart !== null && event.timestamp < trailStart) break;
      if (event.type !== ENUMS.eventType.cmdReject && event.type !== ENUMS.eventType.cmdAccept) continue;
      if (isCurrentStartAttitudeRefusal(event.type, event.p1, event.p2, latest.diag?.values.cmd_last_id)) {
        return { title: 'START REFUSED', action: 'Keep the car level and still until the IMU calibrates, then press Start.' };
      }
      break;
    }
    return { title: 'IMU CALIBRATION', action: 'Keep the car level and still.' };
  }
  const liftHold = 1 << BIT_FIELDS.holdMask.lift;
  if (((status?.hold_mask ?? 0) & liftHold) !== 0) {
    const pickupStop = status?.state === ENUMS.runState.stopped &&
      status.hold_mask === (liftHold | (1 << BIT_FIELDS.holdMask.operator)) &&
      ((status.faults ?? 0) & (1 << BIT_LIFTED)) !== 0;
    const allowedFaults = ADVISORY_FAULTS | (pickupStop ? (1 << BIT_LIFTED) : 0);
    const sample = latest.physics;
    const lastPoseEvent = events.findLast(event => event.timestamp >= (trailStart ?? 0) &&
      [ENUMS.eventType.liftStart, ENUMS.eventType.liftEnd, ENUMS.eventType.overturn,
       ENUMS.eventType.runStart, ENUMS.eventType.boot].some(type => event.type === type));
    // A lift-end is the firmware's settled-pose decision. It can arrive ahead
    // of status and physics, or persist in history after the pose changes.
    const values = sample?.values;
    if (lastPoseEvent?.type === ENUMS.eventType.liftEnd && lastPoseEvent.source === ENUMS.eventSource.physics &&
        sample && values && sample.timestamp >= lastPoseEvent.timestamp &&
        packet && packet.timestamp >= lastPoseEvent.timestamp && packet.timestamp <= nowMs &&
        nowMs - packet.timestamp <= STALE_AFTER_MS.status! && sample.timestamp <= nowMs &&
        nowMs - sample.timestamp <= STALE_AFTER_MS.physics! && status?.speed_mms === 0 &&
        (pickupStop || status.hold_mask === liftHold) && status.faults !== undefined && (status.faults & ~allowedFaults) === 0 &&
        ((values.flags ?? 0) & (1 << BIT_FIELDS.physicsFlags.calibrated)) !== 0 &&
        values.attitude_state === ENUMS.attitudeState.ok && values.motion_class === ENUMS.motionClass.stationary &&
        Math.hypot(values.pitch_cdeg ?? Infinity, values.roll_cdeg ?? Infinity) < 500) {
      return { title: pickupStop ? PICKUP_STOP_TITLE : 'UPRIGHT', action: 'Press Start to resume.' };
    }
    return { title: 'PICKUP HOLD', action: 'Keep the car upright and still. Check the active faults.' };
  }
  if (status?.speed_mms !== 0 || status.hold_mask !== 0 ||
      (status.mode !== ENUMS.vehicleMode.stopped && status.mode !== ENUMS.vehicleMode.hold &&
       status.mode !== ENUMS.vehicleMode.search)) return null;
  for (let i = events.length - 1; i >= 0; i -= 1) {
    const event = events[i]!;
    if (trailStart !== null && event.timestamp < trailStart) break;
    if (event.type !== ENUMS.eventType.pilotDecision) continue;
    const reason = pilotReason(event.p2);
    const profile = latest.profile;
    if (reason === PILOT_REASON.courseRecovery && profile?.values.status === ENUMS.profileStatus.pilotSparse &&
        profile.timestamp >= event.timestamp) {
      return { title: 'OBSTACLE · DETOURS OFF', action: 'On clear, continuous floor, press Stop, then use normal Start.' };
    }
    return PILOT_HOLDS[reason] ?? null;
  }
  return null;
}

/** The one message that must not scroll away, with what to do. Null for none. */
export function bannerMessage(overview: Overview, faults: number | undefined, holdMessage: BannerMessage | null = null): BannerMessage | null {
  if (overview.health === 'offline') {
    return { title: 'OFFLINE', action: 'Telemetry link unavailable. The displayed readings are from the last update. Commands are off.' };
  }
  const report = modeReport(faults, holdMessage);
  return report !== null && overview.health === 'stale'
    ? { title: `LAST REPORTED ${report.title}`, action: 'Waiting for fresh vehicle status.' }
    : report;
}

function modeReport(faults: number | undefined, holdMessage: BannerMessage | null): BannerMessage | null {
  const set = (bit: number) => faults !== undefined && (faults & (1 << bit)) !== 0;
  if (set(BIT_ESTOP)) {
    return { title: 'EMERGENCY STOP', action: 'The stop is latched until the car reboots. Switch the car off and on.' };
  }
  if (set(BIT_OVERTURNED)) {
    return { title: 'OVERTURNED', action: 'Set the car down upright and still, then press Start.' };
  }
  if (set(BIT_LIFTED)) {
    if (holdMessage?.title === PICKUP_STOP_TITLE) return holdMessage;
    return { title: 'PICKUP DETECTED', action: 'Set down, then press Start.' };
  }
  if (set(BIT_TASK_STALLED)) {
    return { title: 'TASK STALLED', action: 'Check wheel motion and the missed ticks in the navbar.' };
  }
  if (set(BIT_OBSTACLE_STOPPED)) {
    return { title: 'STOPPED BY OBSTACLE', action: 'Clear the path, then press Start.' };
  }
  return holdMessage;
}

/** Persistent recovery guidance for connection issues and reported safety conditions. */
export function ModeBanner({ message, offline, compact = false }: { message: BannerMessage | null; offline: boolean; compact?: boolean }) {
  const theme = useTheme();
  if (message === null) return null;

  return (
    <View
      accessibilityLiveRegion="assertive"
      accessibilityRole="alert"
      style={[
        styles.banner,
        compact ? styles.bannerCompact : null,
      ]}
      testID="mode-banner"
    >
      <Text numberOfLines={compact ? 1 : undefined}
        style={[theme.typography.label, styles.noticeText, { color: offline ? theme.colors.muted : message.title === 'UPRIGHT' ? theme.colors.text : theme.colors.danger }]}>{message.title}</Text>
      <Text numberOfLines={compact ? 2 : undefined} style={[compact ? theme.typography.caption : theme.typography.body, styles.noticeText, { color: theme.colors.text }]}>
        {message.action}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  noticeText: { textAlign: 'right' },
  // Plain text at the end of the rail: a box here is a card inside a card.
  banner: { flexBasis: 320, flexGrow: 1, gap: 2, maxWidth: 420 },
  // In the wide header the notice must not take width from the metrics.
  bannerCompact: { flexBasis: 'auto', flexGrow: 0, flexShrink: 1, maxWidth: '100%' },
});
