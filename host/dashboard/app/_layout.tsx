import { DarkTheme, DefaultTheme, Stack, ThemeProvider as NavigationTheme, usePathname } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import { useEffect } from 'react';
import { Platform, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { Brand, ThemeToggle } from '../components/console/ConsoleNav';
import { bannerMessage, ModeBanner, reportedHoldMessage } from '../components/console/ModeBanner';
import { NavbarReadings } from '../components/console/NavbarReadings';
import { StatusRail } from '../components/console/StatusRail';
import { CommandWorkspace, ControlsProvider } from '../components/controls/ControlsPanel';
import { buildOverview, type OverviewHealth } from '../components/overview/overview';
import { getLinkUrl, getServiceUrl, LINK_CONFIG_ERROR, SERVICE_CONFIG_ERROR } from '../lib/config';
import { consoleFrame } from '../lib/console-frame';
import { FAULT_ESTOP_BIT } from '../lib/faults';
import { GLOBAL_CSS } from '../lib/global-css';
import { snapshotClockOffset } from '../lib/link';
import { TelemetryProvider, useTelemetry } from '../lib/telemetry-context';
import { ThemeProvider, useTheme } from '../lib/theme-provider';
import { useCommand } from '../lib/useCommand';
import { useNow } from '../lib/useNow';

const hasCurrentStatus = (health: OverviewHealth) => health === 'healthy' || health === 'fault';

function RootNavigator() {
  const theme = useTheme();
  const { width, height } = useWindowDimensions();
  const pathname = usePathname();
  const insets = useSafeAreaInsets();
  const { snapshot, store } = useTelemetry();
  const serviceUrl = getServiceUrl();
  const { feedback, feedbackError, send } = useCommand(serviceUrl, store.resetAt, store.events);
  const nowMs = useNow();
  const wide = width >= 960;
  const frame = consoleFrame(width, height);
  const latest = store.latest;
  const timelineNow = nowMs + snapshotClockOffset(snapshot, latest.status);
  const vehicle = { ...snapshot, lastStatus: latest.status };
  const overview = buildOverview(vehicle, nowMs);
  const readings = <NavbarReadings latest={latest} connected={overview.health !== 'offline'} nowMs={timelineNow} />;
  const faults = latest.status?.values.faults;
  const notice = bannerMessage(overview, faults, reportedHoldMessage(store, timelineNow));
  const emergencyLatched = faults !== undefined && (faults & (1 << FAULT_ESTOP_BIT)) !== 0;
  const commandsDisabled = snapshot.status !== 'open' || snapshot.bridge?.mqtt === 'disconnected' || serviceUrl === null;
  const safetyAvailable = hasCurrentStatus(overview.health);
  const missionAvailable = pathname === '/' && faults !== undefined && safetyAvailable;

  const base = theme.mode === 'dark' ? DarkTheme : DefaultTheme;
  const navigation = {
    ...base,
    colors: { ...base.colors, background: theme.colors.canvas, card: theme.colors.card, border: theme.colors.border },
  };

  return (
    <NavigationTheme value={navigation}>
      <StatusBar style={theme.mode === 'dark' ? 'light' : 'dark'} />
      <ControlsProvider
        disabled={commandsDisabled}
        emergencyLatched={emergencyLatched}
        feedback={feedback}
        feedbackError={feedbackError}
        missionAvailable={missionAvailable}
        safetyAvailable={safetyAvailable}
        onAction={(action) => {
          // A backgrounded tab can resume before its freshness timer has run.
          if (hasCurrentStatus(buildOverview(vehicle, Date.now()).health)) send(action);
        }}
      >
        <View style={[styles.shell, {
          backgroundColor: theme.colors.canvas,
          paddingTop: insets.top,
          paddingBottom: insets.bottom,
          paddingLeft: insets.left,
          paddingRight: insets.right,
        }]}>
          <View accessibilityLabel="Console navigation" role="navigation" testID="console-navbar"
            style={[styles.chrome, { backgroundColor: theme.colors.card, borderBottomColor: theme.colors.border }]}>
            <View style={[styles.chromeContent, {
              maxWidth: frame.maxWidth, paddingHorizontal: wide ? frame.inset + theme.spacing.md + 1 : theme.spacing.md,
            }]}>
              <View style={styles.top}>
                <Brand iconOnly={width < 560} />
                <StatusRail hasNotice={notice !== null} compact={!wide} health={overview.health} packet={latest.status} />
                {wide && notice !== null ? <ModeBanner compact message={notice} offline={overview.health === 'offline'} /> : null}
                {wide ? readings : null}
                <ThemeToggle />
              </View>
              {wide ? null : readings}
              {!wide && notice !== null ? <ModeBanner compact message={notice} offline={overview.health === 'offline'} /> : null}
              {getLinkUrl() === null ? (
                <Text style={[theme.typography.caption, { color: theme.colors.danger }]}>{LINK_CONFIG_ERROR}</Text>
              ) : null}
              {serviceUrl === null ? (
                <Text style={[theme.typography.caption, { color: theme.colors.danger }]}>{SERVICE_CONFIG_ERROR}</Text>
              ) : null}
              {snapshot.status === 'open' && snapshot.bridge?.historyStorage === 'degraded' ? (
                <Text accessibilityLiveRegion="polite" style={[theme.typography.caption, { color: theme.colors.danger }]}>
                  Bridge recording is unavailable. Recent telemetry is kept in memory.
                </Text>
              ) : null}
            </View>
          </View>
          <CommandWorkspace>
            <Stack screenOptions={{ animation: 'none', headerShown: false, contentStyle: { backgroundColor: theme.colors.canvas } }} />
          </CommandWorkspace>
        </View>
      </ControlsProvider>
    </NavigationTheme>
  );
}

const styles = StyleSheet.create({
  shell: { flex: 1 },
  chrome: { borderBottomWidth: 1, paddingVertical: 6 },
  chromeContent: { alignSelf: 'center', gap: 4, width: '100%' },
  top: { alignItems: 'center', flexDirection: 'row', gap: 8 },
});

/** The single-page web build has no +html template, so the global rules go in at run time. */
function useGlobalCss() {
  useEffect(() => {
    if (Platform.OS !== 'web') return undefined;
    const style = document.createElement('style');
    style.setAttribute('data-global-css', '');
    style.textContent = GLOBAL_CSS;
    document.head.appendChild(style);
    return () => style.remove();
  }, []);
}

export default function RootLayout() {
  useGlobalCss();

  return (
    <ThemeProvider>
      <TelemetryProvider>
        <RootNavigator />
      </TelemetryProvider>
    </ThemeProvider>
  );
}
