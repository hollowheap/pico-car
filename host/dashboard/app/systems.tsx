import { Redirect } from 'expo-router';

export default function SystemsScreen() {
  return <Redirect href={{ pathname: '/', params: { diagnostics: 'open' } }} />;
}
