import { useEffect, useState } from 'react';
import { Modal, Pressable, StyleSheet, Text, TextInput, View } from 'react-native';
import { getServiceUrl, SERVICE_CONFIG_ERROR } from '../../lib/config';
import { useTheme } from '../../lib/theme-provider';

const actions = [
  ['stop', 'Stop'], ['left', 'Left'], ['right', 'Right'], ['straight', 'Straight'], ['uturn', 'U-turn'],
] as const;
type Action = typeof actions[number][0];
type MapReply = {
  bindings: Record<string, Action>;
  status: 'default' | 'pending' | 'applied' | 'unconfirmed';
};

async function mapRequest(bindings?: MapReply['bindings']): Promise<MapReply> {
  const url = getServiceUrl();
  if (!url) throw new Error(SERVICE_CONFIG_ERROR);
  const response = await fetch(`${url}/barcode-map`, bindings === undefined ? { cache: 'no-store' } : {
    method: 'PUT', headers: { 'content-type': 'application/json' }, body: JSON.stringify(bindings),
  });
  const body = await response.json() as MapReply & { error?: string };
  if (!response.ok) throw new Error(body.error ?? 'Could not reach the barcode settings');
  return body;
}

export function BarcodeMapSettings() {
  const theme = useTheme();
  const [open, setOpen] = useState(false);
  const [draft, setDraft] = useState<Partial<Record<Action, string>> | null>(null);
  const [status, setStatus] = useState<MapReply['status']>('default');
  const [error, setError] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);

  useEffect(() => {
    if (!open) return;
    let active = true;
    void mapRequest().then(({ bindings, status: next }) => {
      if (!active) return;
      const fields: Partial<Record<Action, string>> = {};
      for (const [letter, action] of Object.entries(bindings)) fields[action] = (fields[action] ?? '') + letter;
      setDraft(fields);
      setStatus(next);
    }).catch((reason: Error) => { if (active) setError(reason.message); });
    return () => { active = false; };
  }, [open]);

  useEffect(() => {
    if (!open || status !== 'pending') return;
    const timer = setInterval(() => { void mapRequest().then((reply) => setStatus(reply.status)).catch(() => {}); }, 1000);
    return () => clearInterval(timer);
  }, [open, status]);

  async function save() {
    if (!draft) return;
    setSaving(true);
    setError(null);
    try {
      const bindings: MapReply['bindings'] = {};
      for (const [action] of actions) {
        for (const letter of (draft[action] ?? '').replace(/[\s,]/g, '').toUpperCase()) {
          if (!/^[A-Z]$/.test(letter)) throw new Error('Use letters A–Z.');
          if (bindings[letter]) throw new Error(`Assign ${letter} to one command.`);
          bindings[letter] = action;
        }
      }
      setStatus((await mapRequest(bindings)).status);
    } catch (reason) {
      setError((reason as Error).message);
    } finally {
      setSaving(false);
    }
  }

  const text = { color: theme.colors.text };
  const border = { borderColor: theme.colors.border };
  return <>
    <Pressable accessibilityRole="button" onPress={() => { setError(null); setDraft(null); setOpen(true); }}>
      <Text style={[theme.typography.caption, { color: theme.colors.accent }]}>Edit barcode mapping</Text>
    </Pressable>
    <Modal visible={open} transparent animationType="none" onRequestClose={() => setOpen(false)}>
      <View style={styles.backdrop}>
        <View style={[styles.dialog, border, { backgroundColor: theme.colors.card }]} accessibilityViewIsModal>
          <Text style={[theme.typography.label, text]}>Barcode mapping</Text>
          {draft === null ? <Text style={text}>{error ?? 'Loading…'}</Text> : actions.map(([action, label]) => (
            <View key={action} style={styles.row}>
              <Text style={text}>{label}</Text>
              <TextInput accessibilityLabel={`${label} letters`} autoCapitalize="characters" autoCorrect={false}
                maxLength={52} style={[styles.input, text, border]} value={draft[action] ?? ''}
                onChangeText={(letters) => setDraft({ ...draft, [action]: letters })} />
            </View>
          ))}
          {draft !== null && (error || status !== 'default') ? <Text accessibilityLiveRegion="polite" style={text}>
            {error ?? (status === 'applied' ? 'Car confirmed the mapping.' : status === 'unconfirmed' ?
              'Saved. The car has yet to confirm.' : 'Saved. Waiting for the car.')}
          </Text> : null}
          <View style={styles.buttons}>
            <Pressable accessibilityRole="button" onPress={() => setOpen(false)} style={[styles.button, border]}>
              <Text style={text}>Close</Text>
            </Pressable>
            <Pressable accessibilityRole="button" disabled={!draft || saving} onPress={() => { void save(); }}
              style={[styles.button, border, (!draft || saving) && { opacity: 0.5 }]}>
              <Text style={text}>{saving ? 'Saving…' : 'Save'}</Text>
            </Pressable>
          </View>
        </View>
      </View>
    </Modal>
  </>;
}

const styles = StyleSheet.create({
  backdrop: { flex: 1, justifyContent: 'center', alignItems: 'center', padding: 20, backgroundColor: '#0008' },
  dialog: { width: '100%', maxWidth: 360, borderWidth: 1, borderRadius: 8, padding: 16, gap: 10 },
  row: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 16 },
  input: { borderWidth: 1, borderRadius: 4, minWidth: 130, paddingHorizontal: 8, paddingVertical: 5 },
  buttons: { flexDirection: 'row', justifyContent: 'flex-end', gap: 8 },
  button: { borderWidth: 1, borderRadius: 4, paddingHorizontal: 12, paddingVertical: 6 },
});
