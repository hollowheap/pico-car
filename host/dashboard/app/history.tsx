import { useEffect, useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

import { ConsoleNav } from '../components/console/ConsoleNav';
import { Button } from '../components/ui/button';
import { HistoryPanel } from '../components/history/HistoryPanel';
import { SERVICE_CONFIG_ERROR, getServiceUrl } from '../lib/config';
import { fetchHistory, type HistoryRecord } from '../lib/history';
import { useTheme } from '../lib/theme-provider';

type HistoryState =
  | { status: 'loading' }
  | { status: 'ready'; records: HistoryRecord[] }
  | { status: 'error'; message: string };

export default function HistoryScreen() {
  const [state, setState] = useState<HistoryState>({ status: 'loading' });
  const [request, setRequest] = useState(0);
  const serviceUrl = getServiceUrl();
  const theme = useTheme();
  const insets = useSafeAreaInsets();

  useEffect(() => {
    let active = true;
    setState({ status: 'loading' });
    if (serviceUrl === null) {
      setState({ status: 'error', message: SERVICE_CONFIG_ERROR });
      return () => {
        active = false;
      };
    }
    fetchHistory(serviceUrl)
      .then((records) => {
        if (active) {
          setState({ status: 'ready', records });
        }
      })
      .catch((reason: Error) => {
        if (active) {
          setState({ status: 'error', message: reason.message });
        }
      });
    return () => {
      active = false;
    };
  }, [request, serviceUrl]);

  const retry = () => setRequest((value) => value + 1);

  return (
    <View style={[styles.page, { backgroundColor: theme.colors.canvas, paddingTop: insets.top + 16 }]}>
      <ConsoleNav active="history" />
      <Text style={[theme.typography.heading, { color: theme.colors.text }]}>
        Run history
      </Text>
      {state.status === 'loading' ? (
        <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
          Loading history...
        </Text>
      ) : null}
      {state.status === 'error' ? (
        <View
          style={[
            styles.error,
            {
              backgroundColor: theme.colors.dangerSurface,
              borderColor: theme.colors.border,
            },
          ]}
        >
          <Text style={[theme.typography.body, { color: theme.colors.danger }]}>
            {state.message}
          </Text>
          <Button variant="secondary" onPress={retry}>
            <Button.Text>Retry</Button.Text>
          </Button>
        </View>
      ) : null}
      {state.status === 'ready' && state.records.length === 0 ? (
        <Text style={[theme.typography.body, { color: theme.colors.muted }]}>
          No frames recorded yet.
        </Text>
      ) : null}
      {state.status === 'ready' && state.records.length > 0 ? (
        <HistoryPanel records={state.records} />
      ) : null}
    </View>
  );
}

const styles = StyleSheet.create({
  page: { flex: 1, gap: 16, padding: 24 },
  error: { borderWidth: 1, borderRadius: 8, gap: 10, padding: 12 },
});
